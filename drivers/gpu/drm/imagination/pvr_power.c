// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_fw_startstop.h"
#include "pvr_power.h"
#include "pvr_rogue_fwif.h"

#include <drm/drm_managed.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
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

static int
pvr_power_request_idle(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd pow_cmd;

	/* Send FORCED_IDLE request to FW. */
	pow_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_POW;
	pow_cmd.cmd_data.pow_data.pow_type = ROGUE_FWIF_POW_FORCED_IDLE_REQ;
	pow_cmd.cmd_data.pow_data.power_req_data.pow_request_type = ROGUE_FWIF_POWER_FORCE_IDLE;

	return pvr_power_send_command(pvr_dev, &pow_cmd);
}

static int
pvr_power_request_pwr_off(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd pow_cmd;

	/* Send POW_OFF request to firmware. */
	pow_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_POW;
	pow_cmd.cmd_data.pow_data.pow_type = ROGUE_FWIF_POW_OFF_REQ;
	pow_cmd.cmd_data.pow_data.power_req_data.forced = true;

	return pvr_power_send_command(pvr_dev, &pow_cmd);
}

/**
 * pvr_power_set_state() - Change GPU power state
 * @pvr_dev: Target PowerVR device.
 * @new_state: Desired power state.
 *
 * If the GPU is already in the desired power state then this function is a no-op.
 *
 * Caller must hold the device's power lock.
 *
 * Returns:
 *  * 0 on success, or
 *  * Any appropriate error on failure.
 */
int
pvr_power_set_state(struct pvr_device *pvr_dev, enum pvr_power_state new_state)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct device *dev = drm_dev->dev;
	int err;

	lockdep_assert_held(&pvr_dev->power_lock);

	cancel_delayed_work(&pvr_dev->delayed_idle_work);

	if (pvr_dev->power_state == new_state) {
		err = 0;
		goto err_out;
	}

	switch (new_state) {
	case PVR_POWER_STATE_OFF:
		/*
		 * Allow power off if the device is lost. If the device is lost then we expect the
		 * firmware power requests to fail, but we should still proceed with (forcibly)
		 * powering off the device.
		 */
		cancel_delayed_work(&pvr_dev->watchdog.work);

		/* Force idle */
		err = pvr_power_request_idle(pvr_dev);
		if (err == -ETIMEDOUT) {
			drm_err(drm_dev, "Firmware idle timed out\n");
			pvr_device_lost(pvr_dev);
		}

		err = pvr_power_request_pwr_off(pvr_dev);
		if (err == -ETIMEDOUT) {
			drm_err(drm_dev, "Firmware power off timed out\n");
			pvr_device_lost(pvr_dev);
		}

		err = pvr_fw_stop(pvr_dev);
		if (err == -ETIMEDOUT) {
			drm_err(drm_dev, "Failed to stop firmware\n");
			pvr_device_lost(pvr_dev);
		}

		pm_runtime_put_sync_suspend(dev);
		break;

	case PVR_POWER_STATE_ON:
		/* Don't try to power on if the device is lost. */
		if (pvr_dev->lost) {
			err = -EIO;
			goto err_out;
		}

		err = pm_runtime_resume_and_get(dev);
		if (err)
			goto err_out;

		/* Restart FW */
		err = pvr_fw_start(pvr_dev);
		if (err) {
			drm_err(drm_dev, "Firmware failed to boot\n");
			pvr_device_lost(pvr_dev);
			goto err_out;
		}

		err = pvr_wait_for_fw_boot(pvr_dev);
		if (err) {
			drm_err(drm_dev, "Firmware failed to boot\n");
			pvr_device_lost(pvr_dev);
			goto err_out;
		}

		pvr_dev->watchdog.kccb_stall_count = 0;
		queue_delayed_work(pvr_dev->irq_wq, &pvr_dev->watchdog.work,
				   msecs_to_jiffies(WATCHDOG_TIME_MS));
		break;

	default:
		WARN_ON(true);
		err = -EINVAL;
		goto err_out;
	}

	/* Set power state */
	pvr_dev->power_state = new_state;

	return 0;

err_out:
	return err;
}

static bool
pvr_device_is_idle(struct pvr_device *pvr_dev)
{
	/*
	 * FW power state can be out of date if a KCCB command has been submitted but the FW hasn't
	 * started processing it yet. So also check the KCCB status.
	 */
	enum rogue_fwif_pow_state pow_state = READ_ONCE(pvr_dev->fw_dev.fwif_sysdata->pow_state);
	bool kccb_idle = pvr_kccb_is_idle(pvr_dev);

	return (pow_state == ROGUE_FWIF_POW_IDLE) && kccb_idle;
}

static void
pvr_delayed_idle_worker(struct work_struct *work)
{
	struct pvr_device *pvr_dev = container_of(work, struct pvr_device,
						  delayed_idle_work.work);

	mutex_lock(&pvr_dev->power_lock);

	if (pvr_device_is_idle(pvr_dev))
		pvr_power_set_state(pvr_dev, PVR_POWER_STATE_OFF);

	mutex_unlock(&pvr_dev->power_lock);
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

	mutex_lock(&pvr_dev->power_lock);

	if (pvr_dev->lost)
		goto out_unlock;

	if (pvr_dev->power_state != PVR_POWER_STATE_ON)
		goto out_unlock;

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

out_unlock:
	mutex_unlock(&pvr_dev->power_lock);
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
	int err;

	err = drmm_mutex_init(from_pvr_device(pvr_dev), &pvr_dev->power_lock);
	if (err)
		goto err_out;

	pvr_dev->power_state = PVR_POWER_STATE_OFF;
	INIT_DELAYED_WORK(&pvr_dev->delayed_idle_work, pvr_delayed_idle_worker);
	INIT_DELAYED_WORK(&pvr_dev->watchdog.work, pvr_watchdog_worker);

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
