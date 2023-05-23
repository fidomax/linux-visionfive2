// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_fw_startstop.h"
#include "pvr_power.h"
#include "pvr_rogue_fwif.h"

#include <drm/drm_managed.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define POWER_SYNC_TIMEOUT_US (1000000) /* 1s */

#define POWER_IDLE_DELAY_JIFFIES (1)

#define WATCHDOG_TIME_MS (500)

static void
pvr_device_lost(struct pvr_device *pvr_dev)
{
	if (!pvr_dev->lost) {
		pvr_dev->lost = true;
		pvr_context_cancel_active_jobs(pvr_dev);
	}
}

static int
pvr_power_send_command(struct pvr_device *pvr_dev, struct rogue_fwif_kccb_cmd *pow_cmd)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	u32 slot_nr;
	u32 value;
	int err;

	WRITE_ONCE(*fw_dev->power_sync, 0);

	err = pvr_kccb_send_cmd_power_locked(pvr_dev, pow_cmd, &slot_nr);
	if (err)
		goto err_out;

	/* Wait for FW to acknowledge. */
	err = readl_poll_timeout(pvr_dev->fw_dev.power_sync, value, value != 0, 100,
				 POWER_SYNC_TIMEOUT_US);
	if (err)
		goto err_out;

	return 0;

err_out:
	return err;
}

int
pvr_power_request_idle(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd pow_cmd;

	/* Send FORCED_IDLE request to FW. */
	pow_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_POW;
	pow_cmd.cmd_data.pow_data.pow_type = ROGUE_FWIF_POW_FORCED_IDLE_REQ;
	pow_cmd.cmd_data.pow_data.power_req_data.pow_request_type = ROGUE_FWIF_POWER_FORCE_IDLE;

	return pvr_power_send_command(pvr_dev, &pow_cmd);
}

int
pvr_power_request_pwr_off(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd pow_cmd;

	/* Send POW_OFF request to firmware. */
	pow_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_POW;
	pow_cmd.cmd_data.pow_data.pow_type = ROGUE_FWIF_POW_OFF_REQ;
	pow_cmd.cmd_data.pow_data.power_req_data.forced = true;

	return pvr_power_send_command(pvr_dev, &pow_cmd);
}

static int
pvr_power_fw_disable(struct pvr_device *pvr_dev)
{
	int err;

	cancel_delayed_work_sync(&pvr_dev->watchdog.work);

	err = pvr_power_request_idle(pvr_dev);
	if (err)
		return err;

	err = pvr_power_request_pwr_off(pvr_dev);
	if (err)
		return err;

	return pvr_fw_stop(pvr_dev);
}

static int
pvr_power_fw_enable(struct pvr_device *pvr_dev)
{
	int err;

	err = pvr_fw_start(pvr_dev);
	if (err)
		return err;

	err = pvr_wait_for_fw_boot(pvr_dev);
	if (err) {
		drm_err(from_pvr_device(pvr_dev), "Firmware failed to boot\n");
		pvr_fw_stop(pvr_dev);
		return err;
	}

	return 0;
}

bool
pvr_power_is_idle(struct pvr_device *pvr_dev)
{
	/*
	 * FW power state can be out of date if a KCCB command has been submitted but the FW hasn't
	 * started processing it yet. So also check the KCCB status.
	 */
	enum rogue_fwif_pow_state pow_state = READ_ONCE(pvr_dev->fw_dev.fwif_sysdata->pow_state);
	bool kccb_idle = pvr_kccb_is_idle(pvr_dev);

	return (pow_state == ROGUE_FWIF_POW_IDLE) && kccb_idle;
}

/**
 * pvr_power_check_idle() - Check for GPU idle, and schedule power off if required
 * @pvr_dev: Target PowerVR device
 *
 * The actual power off is performed by a delayed work item. This implements hysteresis.
 */
void
pvr_power_check_idle(struct pvr_device *pvr_dev)
{
	enum rogue_fwif_pow_state pow_state = READ_ONCE(pvr_dev->fw_dev.fwif_sysdata->pow_state);

	if (pow_state == ROGUE_FWIF_POW_IDLE &&
	    !delayed_work_pending(&pvr_dev->delayed_idle_work)) {
		queue_delayed_work(pvr_dev->irq_wq, &pvr_dev->delayed_idle_work,
				   POWER_IDLE_DELAY_JIFFIES);
	} else if (pow_state != ROGUE_FWIF_POW_IDLE &&
		   delayed_work_pending(&pvr_dev->delayed_idle_work)) {
		cancel_delayed_work(&pvr_dev->delayed_idle_work);
	}
}

static bool
pvr_watchdog_kccb_stalled(struct pvr_device *pvr_dev)
{
	/* Check KCCB commands are progressing. */
	u32 kccb_cmds_executed = pvr_dev->fw_dev.fwif_osdata->kccb_cmds_executed;
	bool kccb_is_idle = pvr_kccb_is_idle(pvr_dev);

	if (pvr_dev->watchdog.old_kccb_cmds_executed == kccb_cmds_executed && !kccb_is_idle) {
		pvr_dev->watchdog.kccb_stall_count++;

		/*
		 * If we have commands pending with no progress for 2 consecutive polls then
		 * consider KCCB command processing stalled.
		 */
		if (pvr_dev->watchdog.kccb_stall_count == 2)
			return true;
	} else if (pvr_dev->watchdog.old_kccb_cmds_executed == kccb_cmds_executed) {
		bool has_active_contexts;

		spin_lock(&pvr_dev->active_contexts.lock);
		has_active_contexts = list_empty(&pvr_dev->active_contexts.list);
		spin_unlock(&pvr_dev->active_contexts.lock);

		if (has_active_contexts) {
			/* Send a HEALTH_CHECK command so we can verify FW is still alive. */
			struct rogue_fwif_kccb_cmd health_check_cmd;

			health_check_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_HEALTH_CHECK;

			pvr_kccb_send_cmd_power_locked(pvr_dev, &health_check_cmd, NULL);
		}

		pvr_dev->watchdog.kccb_stall_count = 0;
	} else {
		pvr_dev->watchdog.old_kccb_cmds_executed = kccb_cmds_executed;
		pvr_dev->watchdog.kccb_stall_count = 0;
	}

	return false;
}

static void
pvr_watchdog_worker(struct work_struct *work)
{
	struct pvr_device *pvr_dev = container_of(work, struct pvr_device,
						  watchdog.work.work);
	bool stalled;

	if (pvr_dev->lost)
		return;

	if (pm_runtime_get_if_in_use(from_pvr_device(pvr_dev)->dev) <= 0)
		return;

	if (!pvr_dev->fw_dev.booted)
		goto out_requeue;

	stalled = pvr_watchdog_kccb_stalled(pvr_dev);

	if (stalled) {
		drm_err(from_pvr_device(pvr_dev), "GPU device lost");
		pvr_device_lost(pvr_dev);
	}

out_requeue:
	if (!pvr_dev->lost) {
		queue_delayed_work(pvr_dev->irq_wq, &pvr_dev->watchdog.work,
				   msecs_to_jiffies(WATCHDOG_TIME_MS));
	}

	pm_runtime_put(from_pvr_device(pvr_dev)->dev);
}

/**
 * pvr_power_init() - Initialise power management for device
 * @pvr_dev: Target PowerVR device.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%ENOMEM on out of memory.
 */
int
pvr_power_init(struct pvr_device *pvr_dev)
{
	INIT_DELAYED_WORK(&pvr_dev->watchdog.work, pvr_watchdog_worker);

	return 0;
}

int
pvr_power_device_suspend(struct device *dev)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct drm_device *drm_dev = platform_get_drvdata(plat_dev);
	struct pvr_device *pvr_dev = to_pvr_device(drm_dev);
	int err = 0;

	if (pvr_dev->fw_dev.booted) {
		err = pvr_power_fw_disable(pvr_dev);
		if (err)
			goto err_out;
	}

	if (pvr_dev->vendor.callbacks &&
	    pvr_dev->vendor.callbacks->power_disable) {
		err = pvr_dev->vendor.callbacks->power_disable(pvr_dev);
		if (err)
			goto err_out;
	}

	clk_disable(pvr_dev->mem_clk);
	clk_disable(pvr_dev->sys_clk);
	clk_disable(pvr_dev->core_clk);

	if (pvr_dev->regulator)
		regulator_disable(pvr_dev->regulator);

err_out:
	return err;
}


int
pvr_power_device_resume(struct device *dev)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct drm_device *drm_dev = platform_get_drvdata(plat_dev);
	struct pvr_device *pvr_dev = to_pvr_device(drm_dev);
	int err;

	if (pvr_dev->regulator) {
		err = regulator_enable(pvr_dev->regulator);
		if (err)
			goto err_out;
	}

	clk_enable(pvr_dev->core_clk);
	clk_enable(pvr_dev->sys_clk);
	clk_enable(pvr_dev->mem_clk);

	if (pvr_dev->vendor.callbacks &&
	    pvr_dev->vendor.callbacks->power_enable) {
		err = pvr_dev->vendor.callbacks->power_enable(pvr_dev);
		if (err)
			goto err_clk_disable;
	}

	if (pvr_dev->fw_dev.booted) {
		err = pvr_power_fw_enable(pvr_dev);
		if (err)
			goto err_power_disable;
	}

	return 0;

err_power_disable:
	if (pvr_dev->vendor.callbacks &&
	    pvr_dev->vendor.callbacks->power_disable) {
		err = pvr_dev->vendor.callbacks->power_disable(pvr_dev);
		if (err)
			goto err_out;
	}

err_clk_disable:
	clk_disable(pvr_dev->mem_clk);
	clk_disable(pvr_dev->sys_clk);
	clk_disable(pvr_dev->core_clk);

err_out:
	return err;
}

int
pvr_power_device_idle(struct device *dev)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct drm_device *drm_dev = platform_get_drvdata(plat_dev);
	struct pvr_device *pvr_dev = to_pvr_device(drm_dev);

	return pvr_power_is_idle(pvr_dev) ? 0 : -EBUSY;
}

int
pvr_power_reset(struct pvr_device *pvr_dev)
{
	int err;

	/*
	 * Take a power reference during the reset. This should prevent any interference with the
	 * power state during reset.
	 */
	err = pvr_power_get(pvr_dev);
	if (err)
		goto err_out;

	err = pvr_power_fw_disable(pvr_dev);
	if (err)
		goto err_power_put;

	/* Clear the FW faulted flags. */
	pvr_dev->fw_dev.fwif_sysdata->hwr_state_flags &= ~(ROGUE_FWIF_HWR_FW_FAULT |
							   ROGUE_FWIF_HWR_RESTART_REQUESTED);

	err = pvr_power_fw_enable(pvr_dev);

err_power_put:
	pvr_power_put(pvr_dev);

err_out:
	return err;
}

/**
 * pvr_power_fini() - Shutdown power management for device
 * @pvr_dev: Target PowerVR device.
 */
void
pvr_power_fini(struct pvr_device *pvr_dev)
{
	cancel_delayed_work_sync(&pvr_dev->watchdog.work);
}
