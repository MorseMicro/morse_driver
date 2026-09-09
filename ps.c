/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/pm_wakeup.h>
#include <linux/jiffies.h>

#include "morse.h"
#include "debug.h"
#include "morse_commands.h"
#include "skbq.h"
#include "mac.h"
#include "bus.h"
#include "hw.h"
#include "ps.h"
#include "trace.h"
#include "dot11ah/s1g_ieee80211.h"

#define MORSE_PS_DBG(_m, _f, _a...)		morse_dbg(FEATURE_ID_POWERSAVE, _m, _f, ##_a)
#define MORSE_PS_WARN(_m, _f, _a...)		morse_warn(FEATURE_ID_POWERSAVE, _m, _f, ##_a)

enum dot11ah_powersave_mode {
	POWERSAVE_MODE_DISABLED = 0x00,
	POWERSAVE_MODE_PROTOCOL_ENABLED = 0x01,
	POWERSAVE_MODE_FULLY_ENABLED = 0x02,
	POWERSAVE_MODE_UNKNOWN = 0xFF
};

/* Enable/Disable Powersave */
static enum dot11ah_powersave_mode enable_ps __read_mostly = CONFIG_MORSE_POWERSAVE_MODE;
module_param(enable_ps, uint, 0644);
MODULE_PARM_DESC(enable_ps, "Enable PS");

/* Enable/Disable Powersave */
static bool enable_dynamic_ps_offload __read_mostly = true;
module_param(enable_dynamic_ps_offload, bool, 0644);
MODULE_PARM_DESC(enable_dynamic_ps_offload, "Enable dynamic PS firmware offload");

static bool morse_ps_is_busy_pin_asserted(struct morse *mors)
{
	bool active_high = !(mors->firmware_flags & MORSE_FW_FLAGS_BUSY_ACTIVE_LOW);

	if (!morse_hw_ps_gpios_are_supported(&mors->cfg->gpios))
		return false;

	return (!!gpio_get_value(mors->cfg->gpios.busy) == active_high);
}

static u8 morse_ps_get_warm_boot_time_ms(struct morse *mors)
{
	return mors->cfg->get_warm_boot_time_ms(mors->chip_id);
}

static void morse_ps_set_wake_gpio(struct morse *mors, bool raise)
{
	int ret;

	if (!morse_hw_ps_gpios_are_supported(&mors->cfg->gpios))
		return;

	ret = gpio_direction_output(mors->cfg->gpios.wake, (raise) ? 1 : 0);

	if (ret) {
		MORSE_ERR(mors, "%s: Failed to %s wake pin (ret:%d)", __func__,
			  (raise) ? "raise" : "lower", ret);
		return;
	}

	trace_ps_wake_gpio(raise);
	MORSE_PS_DBG(mors, "%s: %s wake up pin", __func__, (raise) ? "set" : "cleared");
}

static void morse_ps_wait_after_wake_pin_raise(struct morse *mors)
{
	unsigned long rem;
	unsigned long timeout;
	unsigned int max_boot_time_ms;
	unsigned int warm_boot_time_ms = morse_ps_get_warm_boot_time_ms(mors);
	bool hw_signals_wake = !!(mors->firmware_flags &
				  MORSE_FW_FLAGS_TOGGLES_BUSY_PIN_ON_WAKE_PIN);

	if (!morse_hw_ps_gpios_are_supported(&mors->cfg->gpios))
		return;

	if (morse_ps_is_busy_pin_asserted(mors))
		return; /* device already indicating busy - don't wait the delay */

	/* Wake time includes time for driver to warm boot (multiplied to give some buffer) and the
	 * theoretical max time that the chip could be in RX
	 */
	if (hw_signals_wake) {
		max_boot_time_ms = 2 * warm_boot_time_ms +
				   MORSE_USECS_TO_MSECS_CEIL(S1G_MAX_PACKET_AIR_TIME_USECS);
	} else {
		max_boot_time_ms = warm_boot_time_ms;
	}

	timeout = msecs_to_jiffies(max_boot_time_ms);
	rem = wait_for_completion_timeout(mors->ps.awake, timeout);
	MORSE_PS_DBG(mors, "%s: took %dms to wake\n", __func__, jiffies_to_msecs(timeout - rem));

	if (hw_signals_wake && rem > 0 && jiffies_to_msecs(rem) <= warm_boot_time_ms) {
		trace_ps_chip_wake_error(warm_boot_time_ms);
		MORSE_PS_WARN(mors, "%s: HW took longer than expected to signal wake, continuing\n",
			   __func__);
	}

	/* Warn if HW did not signal within the expected boot period - continue anyway */
	if (hw_signals_wake && rem == 0) {
		trace_ps_chip_wake_error(timeout);
		MORSE_PS_WARN(mors, "%s: HW did not signal wake within expected boot period\n",
			      __func__);
	}
}

static int morse_ps_wakeup(struct morse_ps *mps)
{
	DECLARE_COMPLETION_ONSTACK(awake);
	struct morse *mors = container_of(mps, struct morse, ps);

	if (!mps->is_drv_ps_allowed)
		return -EPERM;

	if (!mps->suspended)
		return -EALREADY;

	trace_ps_wake_start(0);
	WRITE_ONCE(mors->ps.awake, &awake);
	morse_ps_set_wake_gpio(mors, true);
	morse_ps_wait_after_wake_pin_raise(mors);
	WRITE_ONCE(mors->ps.awake, NULL);
	trace_ps_wake_end(0);

	morse_set_bus_enable(mors, true);
	mps->suspended = false;
	return 0;
}

static int morse_ps_sleep(struct morse_ps *mps)
{
	struct morse *mors = container_of(mps, struct morse, ps);

	if (!mps->is_drv_ps_allowed)
		return 0;

	if (mps->suspended)
		return 0;

	trace_ps_sleep(0);
	mps->suspended = true;
	morse_set_bus_enable(mors, false);
	morse_ps_set_wake_gpio(mors, false);
	return 0;
}

static irqreturn_t morse_ps_irq_handle(int irq, void *arg)
{
	struct morse_ps *mps = (struct morse_ps *)arg;
	struct morse *mors = container_of(mps, struct morse, ps);
	struct completion *awake = READ_ONCE(mps->awake);

	if (awake)
		complete(awake);
	else
		queue_work(mors->chip_wq, &mps->async_wake_work);

	trace_ps_to_host_busy_irq((awake) ? 1 : 0);
	return IRQ_HANDLED;
}

static void morse_ps_async_wake_work(struct work_struct *work)
{
	int ret;
	struct morse_ps *mps = container_of(work, struct morse_ps, async_wake_work);
	struct morse *mors = container_of(mps, struct morse, ps);

	if (test_bit(MORSE_STATE_FLAG_SYSTEM_IN_SUSPEND, &mors->state_flags) ||
	    test_bit(MORSE_STATE_FLAG_STOPPING, &mors->state_flags))
		return;

	mutex_lock(&mps->lock);
	ret = morse_ps_wakeup(mps);
	mutex_unlock(&mps->lock);

	if (ret == 0) {
		/*
		 * Forcibly check for any pending hostsync bits on chip->host wake. Some host
		 * platforms will not trigger an interrupt for bits that were set while
		 * interrupts were masked.
		 */
		morse_claim_bus(mors);
		morse_hw_irq_handle(mors);
		morse_release_bus(mors);
	}

	/*
	 * In some cases the busy line can be asserted without the chip having anything in YAPS.
	 * This ensures that we deassert the wake line eventually in those cases.
	 */
	morse_ps_queue_eval(mors);
}

void morse_ps_bus_activity(struct morse *mors, int timeout_ms)
{
	mutex_lock(&mors->ps.lock);
	mors->ps.bus_ps_timeout = jiffies + msecs_to_jiffies(timeout_ms);
	mutex_unlock(&mors->ps.lock);
}

static bool morse_ps_is_interface_enabled_nolock(struct morse *mors)
{
	struct morse_ps *mps = &mors->ps;

	lockdep_assert_held(&mps->lock);
	return mps->mors_vif && mps->mors_vif->is_ps_enabled;
}

bool morse_ps_is_interface_enabled(struct morse *mors)
{
	bool enabled;
	struct morse_ps *mps = &mors->ps;

	mutex_lock(&mps->lock);
	enabled = morse_ps_is_interface_enabled_nolock(mors);
	mutex_unlock(&mps->lock);
	return enabled;
}

static bool is_interface_same_nolock(struct morse *mors, struct morse_vif *mors_vif)
{
	lockdep_assert_held(&mors->ps.lock);
	return mors->ps.mors_vif == mors_vif;
}

int morse_ps_get_iface_id(struct morse *mors)
{
	struct morse_ps *mps = &mors->ps;
	int id = -1;

	mutex_lock(&mps->lock);
	if (mps->mors_vif)
		id = mps->mors_vif->id;
	mutex_unlock(&mps->lock);

	return id;
}

bool morse_ps_is_interface_same(struct morse *mors, struct morse_vif *mors_vif)
{
	struct morse_ps *mps = &mors->ps;
	bool ifaces_are_same;

	mutex_lock(&mps->lock);
	ifaces_are_same = is_interface_same_nolock(mors, mors_vif);
	mutex_unlock(&mps->lock);
	return ifaces_are_same;
}

static int morse_ps_evaluate(struct morse_ps *mps)
{
	struct morse *mors = container_of(mps, struct morse, ps);
	bool needs_wake = false;
	bool eval_later = false;
	unsigned long flags_on_entry = READ_ONCE(mors->chip_if->event_flags);

	flags_on_entry &= ~BIT(MORSE_DATA_TRAFFIC_PAUSE_PEND);

	needs_wake = (mps->wakers > 0);
	needs_wake |= (morse_ps_is_interface_enabled_nolock(mors) == false);
	needs_wake |= (flags_on_entry > 0);
	needs_wake |= (mors->cfg->ops->skbq_get_tx_buffered_count(mors) > 0);

	if (!needs_wake &&
	    mps->dynamic_ps_en &&
	    morse_is_data_tx_allowed(mors) && time_before(jiffies, mps->bus_ps_timeout)) {
		/*
		 * Eval later if there is nothing explicitly holding the bus awake,
		 * but the bus ps timeout has been set to some time in the future
		 * (i.e. network traffic has recently occurred).
		 *
		 * In TWT, the device may go into TWT sleep immediately without
		 * caring about recent network traffic.
		 */
		needs_wake = true;
		eval_later = true;
	}

	if (needs_wake) {
		morse_ps_wakeup(mps);
	} else if (morse_ps_is_busy_pin_asserted(mors)) {
		/* Chip has something to send across the bus, re-evaluate later */
		eval_later = true;
	} else {
		MORSE_WARN_ON(FEATURE_ID_POWERSAVE, eval_later);
		morse_ps_sleep(mps);
	}

	if (eval_later) {
		static const int default_bus_timeout_ms = 5;
		unsigned long expire = jiffies + msecs_to_jiffies(default_bus_timeout_ms);

		if (mps->dynamic_ps_en && time_after(mps->bus_ps_timeout, expire))
			expire = mps->bus_ps_timeout;

		expire = expire - jiffies;
		MORSE_PS_DBG(mors, "%s: Delaying eval work by %d ms\n",
			     __func__, jiffies_to_msecs(expire));

		cancel_delayed_work(&mps->delayed_eval_work);
		queue_delayed_work(mors->chip_wq, &mps->delayed_eval_work, expire);
	}

	return 0;
}

void morse_ps_force_eval(struct morse *mors)
{
	if (!mors->ps.is_drv_ps_allowed)
		return;

	mutex_lock(&mors->ps.lock);
	morse_ps_evaluate(&mors->ps);
	mutex_unlock(&mors->ps.lock);
}

void morse_ps_queue_eval(struct morse *mors)
{
	if (!mors->ps.is_drv_ps_allowed)
		return;
	if (test_bit(MORSE_STATE_FLAG_SYSTEM_IN_SUSPEND, &mors->state_flags) ||
	    test_bit(MORSE_STATE_FLAG_STOPPING, &mors->state_flags))
		return;

	queue_delayed_work(mors->chip_wq, &mors->ps.delayed_eval_work, 0);
}

static void morse_ps_evaluate_work(struct work_struct *work)
{
	struct morse_ps *mps = container_of(work, struct morse_ps, delayed_eval_work.work);
	struct morse *mors = container_of(mps, struct morse, ps);

	if (test_bit(MORSE_STATE_FLAG_SYSTEM_IN_SUSPEND, &mors->state_flags) ||
	    test_bit(MORSE_STATE_FLAG_STOPPING, &mors->state_flags))
		return;

	morse_ps_force_eval(mors);
}

int morse_ps_wakers_inc(struct morse *mors)
{
	struct morse_ps *mps = &mors->ps;

	if (!mps->is_drv_ps_allowed)
		return 0;

	mutex_lock(&mps->lock);
	mps->wakers++;
	mutex_unlock(&mps->lock);
	return 0;
}

int morse_ps_wakers_dec(struct morse *mors)
{
	int ret;
	struct morse_ps *mps = &mors->ps;

	if (!mps->is_drv_ps_allowed)
		return 0;

	mutex_lock(&mps->lock);
	if (mps->wakers == 0) {
		MORSE_PS_WARN(mors, "%s: number of wakers = 0\n", __func__);
		ret = -EFAULT;
	} else {
		mps->wakers--;
		ret = 0;
	}
	mutex_unlock(&mps->lock);
	return ret;
}

static void update_interface_state_nolock(struct morse *mors, struct morse_vif *mors_vif,
					  bool enabled)
{
	struct morse_ps *mps = &mors->ps;

	mps->mors_vif = mors_vif;
	if (mps->mors_vif)
		mps->mors_vif->is_ps_enabled = enabled;
}

void morse_ps_update_interface_state(struct morse *mors, struct morse_vif *mors_vif, bool enabled)
{
	struct morse_ps *mps = &mors->ps;

	if (!mps->is_drv_ps_allowed)
		return;

	mutex_lock(&mps->lock);
	update_interface_state_nolock(mors, mors_vif, enabled);
	mutex_unlock(&mps->lock);

	morse_ps_queue_eval(mors);
}

void morse_ps_iface_down_notify(struct morse *mors, struct morse_vif *mors_vif)
{
	struct morse_ps *mps = &mors->ps;
	bool updated = false;

	mutex_lock(&mps->lock);
	if (is_interface_same_nolock(mors, mors_vif)) {
		update_interface_state_nolock(mors, NULL, false);
		updated = true;
	}
	mutex_unlock(&mps->lock);

	if (updated)
		morse_ps_force_eval(mors);
}

int mors_ps_get_net_timeout_ms(struct morse *mors)
{
	int timeout_ms;
	struct morse_ps *mps = &mors->ps;
	static const int default_uapsd_ps_net_timeout_ms = 5;

	mutex_lock(&mps->lock);
	timeout_ms = mps->ps_net_timeout_ms;
	if (mors->uapsd_per_ac) /* U-APSD will override default setting when active */
		timeout_ms = min(timeout_ms, default_uapsd_ps_net_timeout_ms);
	mutex_unlock(&mps->lock);

	return timeout_ms;
}

void mors_ps_set_net_timeout_ms(struct morse *mors, int timeout_ms)
{
	struct morse_ps *mps = &mors->ps;

	mutex_lock(&mps->lock);
	mps->ps_net_timeout_ms = timeout_ms;
	mutex_unlock(&mps->lock);
}

int morse_ps_system_suspend(struct morse *mors)
{
	int ret;
	int irq;
	struct morse_ps *mps = &mors->ps;

	MORSE_WARN_ON(FEATURE_ID_POWERSAVE, !morse_hw_ps_gpios_are_supported(&mors->cfg->gpios));

	/* Disable bus interrupts before entering suspend. This will prevent
	 * misfires upon wake.
	 */
	morse_bus_set_irq(mors, false);
	/* Decrement waker set in calling site */
	morse_ps_wakers_dec(mors);

	irq = gpio_to_irq(mors->cfg->gpios.busy);
	if (irq < 0) {
		ret = -ENOTSUPP;
		goto exit;
	}

	/* Disable the async interrupt and cancel any pending work */
	disable_irq(irq);
	cancel_work_sync(&mps->async_wake_work);
	cancel_delayed_work_sync(&mps->delayed_eval_work);

	/* Make the async wake gpio the system wake-from-suspend IRQ */
	ret = enable_irq_wake(irq);
	if (ret) {
		/* The system will not suspend, restore the irq */
		enable_irq(irq);
		goto exit;
	}

	mutex_lock(&mps->lock);
	if (mps->wakers)
		MORSE_PS_WARN(mors, "%s: wakers in suspend:%d\n", __func__, mps->wakers);

	morse_ps_sleep(mps);
	MORSE_WARN_ON(FEATURE_ID_POWERSAVE, !mps->suspended);
	mutex_unlock(&mps->lock);

exit:
	if (ret)
		MORSE_ERR(mors, "%s: failed to enable suspend wake irq (ret:%d)\n", __func__, ret);
	else
		MORSE_INFO(mors, "%s: complete\n", __func__);

	return ret;
}

void morse_ps_system_resume(struct morse *mors)
{
	int irq;
	struct morse_ps *mps = &mors->ps;

	irq = gpio_to_irq(mors->cfg->gpios.busy);
	if (irq >= 0) {
		disable_irq_wake(irq);

		/* If the chip is asserted busy while resuming, it was the wakeup source.
		 * Tell the PM core we caused the wake event.
		 */
		if (morse_ps_is_busy_pin_asserted(mors))
			pm_wakeup_event(mors->dev, 0);

		enable_irq(irq);
	} else {
		MORSE_PS_WARN(mors, "%s: invalid irq for system resume\n", __func__);
	}

	mutex_lock(&mps->lock);
	morse_ps_wakeup(mps);
	mutex_unlock(&mps->lock);
	morse_bus_set_irq(mors, true);

	MORSE_INFO(mors, "%s: complete\n", __func__);
}

int morse_ps_init(struct morse *mors)
{
	int ret;
	int irq;
	struct morse_ps *mps = &mors->ps;
	static const int default_ps_net_timeout_ms = 90;

	if (is_virtual_sta_test_mode()) {
		if (!morse_ps_is_disabled()) {
			MORSE_ERR(mors, "%s: Disabling power save in virtual STA test mode\n",
				  __func__);
			enable_ps = POWERSAVE_MODE_DISABLED;
		}
	}

	mps->ps_net_timeout_ms = default_ps_net_timeout_ms;
	mps->is_drv_ps_allowed = enable_ps != POWERSAVE_MODE_DISABLED;
	mps->bus_ps_timeout = jiffies;
	mps->dynamic_ps_en = enable_dynamic_ps_offload;
	mps->suspended = false;
	mps->wakers = 0;

	mutex_init(&mps->lock);

	if (mps->is_drv_ps_allowed) {
		INIT_WORK(&mps->async_wake_work, morse_ps_async_wake_work);
		INIT_DELAYED_WORK(&mps->delayed_eval_work, morse_ps_evaluate_work);

		morse_ps_update_interface_state(mors, NULL, false);

		if (!morse_hw_ps_gpios_are_supported(&mors->cfg->gpios)) {
			/* The rest of the code is GPIO related, we need to bail */
			return 0;
		}

		irq = gpio_to_irq(mors->cfg->gpios.busy);

		/**
		 * SW-1674: Should be the following, but issues observed.
		 * gpio_request_one(mors->cfg->gpios.wake, GPIOF_OPEN_DRAIN, NULL);
		 */
		/* Default to allow chip to wakeup */
		ret = gpio_request(mors->cfg->gpios.wake, "morse-wakeup-ctrl");
		if (ret < 0) {
			MORSE_PR_ERR(FEATURE_ID_POWERSAVE, "Failed to acquire wakeup gpio.\n");
			return ret;
		}
		gpio_direction_output(mors->cfg->gpios.wake, 1);

		gpio_request(mors->cfg->gpios.busy, "morse-async-wakeup-ctrl");

		/* The following input gpio must be configured with pull-down */
		gpio_direction_input(mors->cfg->gpios.busy);

		ret = request_irq(irq, (irq_handler_t)morse_ps_irq_handle,
				  (mors->firmware_flags & MORSE_FW_FLAGS_BUSY_ACTIVE_LOW) ?
					IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING,
					"async_wakeup_from_chip", mps);

		MORSE_WARN_ON(FEATURE_ID_POWERSAVE, ret);

		device_init_wakeup(mors->dev, true);
	}

	if (!morse_ps_is_fully_enabled())
		morse_ps_wakers_inc(mors);

	return 0;
}

void morse_ps_finish(struct morse *mors)
{
	struct morse_ps *mps = &mors->ps;

	if (mps->is_drv_ps_allowed) {
		mps->is_drv_ps_allowed = false;
		mps->dynamic_ps_en = false;

		if (morse_hw_ps_gpios_are_supported(&mors->cfg->gpios)) {
			device_init_wakeup(mors->dev, false);
			free_irq(gpio_to_irq(mors->cfg->gpios.busy), mps);
			gpio_free(mors->cfg->gpios.busy);
			gpio_free(mors->cfg->gpios.wake);
		}

		cancel_work_sync(&mps->async_wake_work);
		cancel_delayed_work_sync(&mps->delayed_eval_work);
	}
}

bool morse_ps_is_dynamic_offload_enabled(void)
{
	return enable_dynamic_ps_offload;
}

bool morse_ps_is_disabled(void)
{
	return (enable_ps == POWERSAVE_MODE_DISABLED);
}

bool morse_ps_is_fully_enabled(void)
{
	return (enable_ps == POWERSAVE_MODE_FULLY_ENABLED);
}

bool morse_ps_is_supported(struct morse *mors)
{
	if (mors->bus_type == MORSE_HOST_BUS_TYPE_USB)
		return !morse_ps_is_disabled();

	return (!morse_ps_is_disabled()) &&
	       morse_hw_ps_gpios_are_supported(&mors->cfg->gpios);
}
