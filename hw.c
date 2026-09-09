/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <linux/types.h>
#include <linux/gpio.h>
#include "morse.h"
#include "debug.h"
#include "hw.h"
#include "bus.h"
#include "firmware.h"
#include "ps.h"
#include "trace.h"

#define MORSE_HWCLOCK_DBG(_m, _f, _a...)	morse_dbg(FEATURE_ID_HWCLOCK, _m, _f, ##_a)
#define MORSE_HWCLOCK_INFO(_m, _f, _a...)	morse_info(FEATURE_ID_HWCLOCK, _m, _f, ##_a)
#define MORSE_HWCLOCK_WARN(_m, _f, _a...)	morse_warn(FEATURE_ID_HWCLOCK, _m, _f, ##_a)
#define MORSE_HWCLOCK_ERR(_m, _f, _a...)	morse_err(FEATURE_ID_HWCLOCK, _m, _f, ##_a)

#define MORSE_HEADLESS_DBG(_m, _f, _a...)	morse_dbg(FEATURE_ID_HEADLESS, _m, _f, ##_a)
#define MORSE_HEADLESS_INFO(_m, _f, _a...)	morse_info(FEATURE_ID_HEADLESS, _m, _f, ##_a)
#define MORSE_HEADLESS_WARN(_m, _f, _a...)	morse_warn(FEATURE_ID_HEADLESS, _m, _f, ##_a)
#define MORSE_HEADLESS_ERR(_m, _f, _a...)	morse_err(FEATURE_ID_HEADLESS, _m, _f, ##_a)

/* Current FW takes about 1350 ms to boot in the worst case. Attach/Detach time is a
 * fraction of that, so use this value as the baseline for timeout.
 */
#define HEADLESS_TIMEOUT_MS 1350

static int hw_reload_after_stop __read_mostly = 5;
module_param(hw_reload_after_stop, int, 0644);
MODULE_PARM_DESC(hw_reload_after_stop,
"Reload HW after a stop notification. Abort if stop events are less than this seconds apart (-1 to disable)");

/* Re-attach to a running hardware */
bool reattach_hw __read_mostly = REATTACH_HW_DEFAULT;
module_param(reattach_hw, bool, 0644);
MODULE_PARM_DESC(reattach_hw,
	"Do not reset hardware state during module exit, attempt to reattach during module init");

static const struct morse_chip_series *chip_series[] = {
	&mm61xx_chip_series,
	&mm81xx_chip_series,
};

static int hw_enable_irq(struct morse *mors, u32 irq, bool enable)
{
	u32 irq_en, irq_en_addr = irq < 32 ? MORSE_REG_INT1_EN(mors) : MORSE_REG_INT2_EN(mors);
	u32 irq_clr_addr = irq < 32 ? MORSE_REG_INT1_CLR(mors) : MORSE_REG_INT2_CLR(mors);
	u32 mask = irq < 32 ? (1 << irq) : (1 << (irq - 32));

	morse_reg32_read(mors, irq_en_addr, &irq_en);
	if (enable)
		irq_en |= (mask);
	else
		irq_en &= ~(mask);
	morse_reg32_write(mors, irq_clr_addr, mask);
	morse_reg32_write(mors, irq_en_addr, irq_en);

	return 0;
}

int morse_hw_irq_enable(struct morse *mors, u32 irq, bool enable)
{
	morse_claim_bus(mors);
	hw_enable_irq(mors, irq, enable);
	morse_release_bus(mors);

	return 0;
}

void morse_hw_stop_work(struct work_struct *work)
{
	bool is_already_stopped;
	time64_t now;
	struct morse *mors = container_of(work, struct morse, recovery.hw_stop);

	mutex_lock(&mors->lock);
	is_already_stopped = morse_hw_is_stopped(mors);
	trace_hw_stop(is_already_stopped);
	if (!is_already_stopped)
		morse_hw_set_state(mors, MORSE_HW_STATE_STOPPED);
	mutex_unlock(&mors->lock);

	if (is_already_stopped) {
		dev_err(mors->dev, "HW already stopped\n");
		return;
	}

	dev_err(mors->dev, "HW has stopped%s\n", (hw_reload_after_stop < 0) ? " (ignoring)" : "");
	if (hw_reload_after_stop < 0)
		return;

	if (!mors->recovery.is_ready) {
		dev_err(mors->dev, "HW recovery flow is not ready\n");
		return;
	}

	now = ktime_get_seconds();
	if (hw_reload_after_stop > 0 &&
	    mors->last_hw_stop &&
	    (now - mors->last_hw_stop) < hw_reload_after_stop) {
		/* HW reload was attempted twice in rapid succession - abort to prevent thrashing */
		dev_err(mors->dev,
			"Automatic HW reload aborted due to retry in < %ds\n",
			hw_reload_after_stop);
		return;
	}

	mutex_lock(&mors->lock);
	if (!morse_coredump_new(mors, MORSE_COREDUMP_REASON_CHIP_INDICATED_STOP))
		set_bit(MORSE_STATE_FLAG_DO_COREDUMP, &mors->state_flags);
	mors->last_hw_stop = now;
	mutex_unlock(&mors->lock);
	schedule_work(&mors->recovery.driver_restart);
}

void morse_hw_restart(struct morse *mors)
{
	bool is_already_stopped;

	mutex_lock(&mors->lock);
	is_already_stopped = morse_hw_is_stopped(mors);
	if (!is_already_stopped)
		morse_hw_set_state(mors, MORSE_HW_STATE_STOPPED);

	mors->last_hw_stop = ktime_get_seconds();
	mutex_unlock(&mors->lock);
	schedule_work(&mors->recovery.driver_restart);
}

static void to_host_hw_stop_irq_handle(struct morse *mors)
{
	schedule_work(&mors->recovery.hw_stop);
}

static void morse_hw_headless_done_irq_handle(struct morse *mors)
{
	queue_work(mors->chip_wq, &mors->headless.work);
}

int morse_hw_irq_handle(struct morse *mors)
{
	u32 status1 = 0;
#if defined(CONFIG_MORSE_DEBUG_IRQ)
	int i;
#endif

	morse_reg32_read(mors, MORSE_REG_INT1_STS(mors), &status1);
	trace_hw_irq(status1);
	if (!status1)
		goto exit;

	if (status1 & MORSE_INT_BEACON_VIF_MASK_ALL)
		morse_beacon_irq_handle(mors, status1);
	if (status1 & MORSE_CHIP_IF_IRQ_MASK_ALL)
		mors->cfg->ops->chip_if_handle_irq(mors, status1);
	if (status1 & MORSE_INT_HW_STOP_NOTIFICATION)
		to_host_hw_stop_irq_handle(mors);
	if (status1 & MORSE_HW_HEADLESS_DONE)
		morse_hw_headless_done_irq_handle(mors);
#if defined(CONFIG_MORSE_ENABLE_TEST_MODES)
	if (status1 & MORSE_INT_BUS_IRQ_SELF_TEST)
		morse_bus_interrupt_profiler_irq(mors);
#endif

	morse_reg32_write(mors, MORSE_REG_INT1_CLR(mors), status1);

exit:
#if defined(CONFIG_MORSE_DEBUG_IRQ)
	mors->debug.hostsync_stats.irq++;
	for (i = 0; i < ARRAY_SIZE(mors->debug.hostsync_stats.irq_bits); i++) {
		if (status1 & BIT(i))
			mors->debug.hostsync_stats.irq_bits[i]++;
	}
#endif

	return status1 ? 1 : 0;
}

int morse_hw_irq_clear(struct morse *mors)
{
	morse_claim_bus(mors);
	morse_reg32_write(mors, MORSE_REG_INT1_CLR(mors), 0xFFFFFFFF);
	morse_reg32_write(mors, MORSE_REG_INT2_CLR(mors), 0xFFFFFFFF);
	morse_release_bus(mors);
	return 0;
}

int morse_hw_toggle_aon_latch(struct morse *mors)
{
	u32 address = MORSE_REG_AON_LATCH_ADDR(mors);
	u32 mask = MORSE_REG_AON_LATCH_MASK(mors);
	u32 latch = 0;

	if (address) {
		/* invoke AON latch procedure */
		morse_reg32_read(mors, address, &latch);
		morse_reg32_write(mors, address, latch & ~(mask));
		mdelay(5);
		morse_reg32_write(mors, address, latch | mask);
		mdelay(5);
		morse_reg32_write(mors, address, latch & ~(mask));
		mdelay(5);
	}

	return 0;
}

int morse_hw_reset(int reset_pin)
{
	int ret = gpio_request(reset_pin, "morse-reset-ctrl");

	if (ret < 0) {
		MORSE_PR_ERR(FEATURE_ID_DEFAULT, "Failed to acquire reset gpio. Skipping reset.\n");
		return ret;
	}

	pr_info("Resetting Morse Chip\n");
	gpio_direction_output(reset_pin, 0);
	mdelay(20);
	/* setting gpio as float to avoid forcing 3.3V High */
	gpio_direction_input(reset_pin);
	pr_info("Done\n");

	gpio_free(reset_pin);

	return ret;
}

int morse_hw_clock_now(const struct morse *mors, u64 *now)
{
	int ret;
	const struct morse_hw_clock *local;
	u64 clock_val;
	ktime_t reference;

	rcu_read_lock();
	local = rcu_dereference(mors->hw_clock.clock);
	if (local) {
		ret = 0;
		clock_val = local->val;
		reference = local->reference;
	} else {
		/* Clock reference yet to be taken */
		ret = -EFAULT;
	}
	rcu_read_unlock();

	if (ret)
		MORSE_HWCLOCK_ERR(mors, "%s: failed (ret:%d)", __func__, ret);
	else
		*now = clock_val + ktime_to_us(ktime_sub(ktime_get_boottime(), reference));

	return ret;
}

static bool hw_supports_clock_interaction(const struct morse *mors)
{
	return MORSE_REG_MTIME_LOWER(mors) > 0 && MORSE_REG_MTIME_UPPER(mors) > 0;
}


static int hw_read_clock(struct morse *mors, u64 *out)
{
	int ret;
	u32 lower1;
	u32 lower2;
	u32 upper;

	ret = morse_reg32_read(mors, MORSE_REG_MTIME_LOWER(mors), &lower1);
	if (ret)
		return ret;

	ret = morse_reg32_read(mors, MORSE_REG_MTIME_UPPER(mors), &upper);
	if (ret)
		return ret;

	ret = morse_reg32_read(mors, MORSE_REG_MTIME_LOWER(mors), &lower2);
	if (ret)
		return ret;

	/* If lower has wrapped, upper will have incremented */
	if (lower2 < lower1)
		upper++;

	*out = ((u64)upper << 32) | lower2;
	return 0;
}

int morse_hw_clock_update(struct morse *mors)
{
	int ret;
	u64 now = 0;
	unsigned long flags;
	struct morse_hw_clock *new;
	struct morse_hw_clock *old;
	ktime_t reference;

	if (!hw_supports_clock_interaction(mors)) {
		ret = -ENOTSUPP;
		MORSE_WARN_ON_ONCE(FEATURE_ID_HWCLOCK, 1);
		goto exit;
	}

	/* This code is time-critical - disable interrupts */
	local_irq_save(flags);
	ret = hw_read_clock(mors, &now);
	reference = ktime_get_boottime();
	local_irq_restore(flags);

	if (ret)
		goto exit;

	MORSE_HWCLOCK_DBG(mors, "%s: %lld us\n", __func__, now);
	new = kmalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		ret = -ENOMEM;
		goto exit;
	}
	new->val = now;
	new->reference = reference;

	/* Old may be NULL if this is the first update operation */
	old = rcu_dereference(mors->hw_clock.clock);
	rcu_assign_pointer(mors->hw_clock.clock, new);
	kfree_rcu(old, rcu);

exit:
	mutex_lock(&mors->hw_clock.update_wait_lock);
	if (mors->hw_clock.update)
		complete(mors->hw_clock.update);
	mutex_unlock(&mors->hw_clock.update_wait_lock);

	if (ret)
		MORSE_HWCLOCK_ERR(mors, "%s: failed to update clock (ret:%d)\n", __func__, ret);

	return ret;
}

int morse_hw_clock_trigger_update(struct morse *mors, bool wait)
{
	int ret = 0;
	unsigned int timeout_ms;
	const int overhead_ms = 10;
	DECLARE_COMPLETION_ONSTACK(hw_clock_update);

	if (!hw_supports_clock_interaction(mors)) {
		ret = -EOPNOTSUPP;
		goto exit;
	}

	/* Derive timeout to wait for clock update */
	timeout_ms = mors->cfg->get_warm_boot_time_ms(mors->chip_id) + overhead_ms;

	if (wait) {
		mutex_lock(&mors->hw_clock.update_wait_lock);
		/* WARN if HW clock completion already set/waiting */
		MORSE_WARN_ON(FEATURE_ID_HWCLOCK, mors->hw_clock.update);
		mors->hw_clock.update = &hw_clock_update;
		mutex_unlock(&mors->hw_clock.update_wait_lock);
	}

	set_bit(MORSE_UPDATE_HW_CLOCK_REFERENCE, &mors->chip_if->event_flags);
	queue_work(mors->chip_wq, &mors->chip_if_work);

	if (wait) {
		unsigned long rem = wait_for_completion_timeout(mors->hw_clock.update,
								msecs_to_jiffies(timeout_ms));

		/* Clear completion */
		mutex_lock(&mors->hw_clock.update_wait_lock);
		mors->hw_clock.update = NULL;
		mutex_unlock(&mors->hw_clock.update_wait_lock);
		ret = (rem) ? 0 : -ETIMEDOUT;
	}

exit:
	if (ret)
		MORSE_HWCLOCK_ERR(mors, "%s: failed to trigger update (ret:%d)\n", __func__, ret);

	return ret;
}

bool is_otp_xtal_wait_supported(struct morse *mors)
{
	int ret;
	u32 otp_word2;
	u32 otp_xtal_wait;

	if (MORSE_REG_OTP_DATA_WORD(mors, 0) == 0)
		/* Device doesn't support OTP (probably an FPGA) */
		return true;

	if (MORSE_REG_OTP_DATA_WORD(mors, 2) != 0) {
		morse_claim_bus(mors);
		ret = morse_reg32_read(mors, MORSE_REG_OTP_DATA_WORD(mors, 2), &otp_word2);
		morse_release_bus(mors);
		if (ret < 0) {
			MORSE_ERR(mors, "OTP data2 value read failed: %d\n", ret);
			return false;
		}
		otp_xtal_wait = (otp_word2 & MM610X_OTP_DATA2_XTAL_WAIT_POS);
		if (!otp_xtal_wait) {
			ret = -1;
			MORSE_ERR(mors, "OTP xtal wait bits not set\n");
			return false;
		}
		return true;
	}
	return false;
}

int morse_hw_enable_stop_notifications(struct morse *mors, bool enable)
{
	return morse_hw_irq_enable(mors, MORSE_INT_HW_STOP_NOTIFICATION_NUM, enable);
}

int morse_chip_cfg_set_and_validate(struct morse *mors, struct morse_chip_series *mors_chip_series)
{
	int ret = 0;
	u32 chip_id = 0;

	/* cfg must be available on mors object prior to initiating any bus operations */
	mors->cfg = mors_chip_series->cfg;

	morse_claim_bus(mors);
	ret = morse_reg32_read(mors, MORSE_REG_CHIP_ID(mors), &chip_id);
	morse_release_bus(mors);
	if (ret < 0) {
		MORSE_ERR(mors, "%s: Failed to access HW (errno:%d)", __func__, ret);
		return ret;
	}

	if (!mors_chip_series->chip_id_matches(chip_id))
		return -ENODEV;

	mors->chip_id = chip_id;

	return ret;
}

int morse_chip_cfg_init(struct morse *mors, u32 chip_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(chip_series); i++) {
		if (chip_series[i]->chip_id_matches(chip_id)) {
			mors->cfg = chip_series[i]->cfg;
			mors->chip_id = chip_id;
			return 0;
		}
	}

	return -ENODEV;
}

bool morse_hw_is_already_loaded(struct morse *mors)
{
	if ((morse_firmware_get_host_table_ptr(mors) != 0) ||
	    (morse_firmware_magic_verify(mors) != 0) ||
	    (morse_firmware_get_fw_flags(mors) != 0))
		return false;

	if (mors->firmware_flags & MORSE_FW_FLAGS_FAILSAFE_MODE) {
		MORSE_INFO(mors, "Firmware is in failsafe mode; forcing reload\n");
		return false;
	}

	return true;
}

static const u8 *hw_headless_state_to_str(enum headless_state state)
{
	switch (state) {
	case HEADLESS_OFF:
		return "off";
	case HEADLESS_ON_REQUEST:
		return "on (requested)";
	case HEADLESS_ON_PENDING:
		return "on (pending)";
	case HEADLESS_ON:
		return "on";
	case HEADLESS_OFF_REQUEST:
		return "off (requested)";
	case HEADLESS_OFF_PENDING:
		return "off (pending)";
	default:
		return "unknown";
	}
}

static int hw_headless_done_irq_enable(struct morse *mors, bool enable)
{
	return hw_enable_irq(mors, MORSE_HW_HEADLESS_DONE_NUM, enable);
}

int morse_hw_headless_done_irq_enable(struct morse *mors, bool enable)
{
	int ret;

	morse_claim_bus(mors);
	ret = hw_headless_done_irq_enable(mors, enable);
	morse_release_bus(mors);

	return ret;
}

/**
 * hw_trigger_attach - Send attach interrupt to hardware
 *
 * @mors: morse struct
 *
 * @return 0 on success
 */
static int hw_trigger_attach(struct morse *mors)
{
	return morse_reg32_write(mors, MORSE_HW_ATTACH_TRGR_SET(mors), MORSE_HW_ATTACH_IRQ_BIT);
}

/**
 * hw_trigger_detach - Send detach interrupt to hardware
 *
 * @mors: morse struct
 *
 * @return 0 on success
 */
static int hw_trigger_detach(struct morse *mors)
{
	return morse_reg32_write(mors, MORSE_HW_DETACH_TRGR_SET(mors), MORSE_HW_DETACH_IRQ_BIT);
}

int morse_hw_trigger_detach(struct morse *mors)
{
	int ret = 0;

	morse_claim_bus(mors);
	ret = hw_trigger_detach(mors);
	morse_release_bus(mors);

	return ret;
}

bool morse_hw_headless_is_off(struct morse *mors)
{
	return mors->headless.state == HEADLESS_OFF;
}

void morse_hw_headless_work(struct work_struct *work)
{
	struct morse *mors = container_of(work, struct morse, headless.work);
	struct completion *drv_work;
	enum headless_state now, after;

	/* Necessary to wake the chip from power save before interacting
	 * with the hostsync register
	 */
	morse_ps_wakers_inc(mors);
	morse_ps_force_eval(mors);

	/* Claim headless lock before claiming the whole bus */
	mutex_lock(&mors->headless.lock);
	morse_claim_bus(mors);

	drv_work = READ_ONCE(mors->headless.wait);
	now = mors->headless.state;
	switch (mors->headless.state) {
	case HEADLESS_ON_REQUEST:
		(void)morse_reg32_write(mors,
					mors->chip_if->headless_cfg_addr,
					mors->headless.pending_cfg);
		hw_headless_done_irq_enable(mors, true);
		mors->headless.state = HEADLESS_ON_PENDING;
		hw_trigger_detach(mors);
		break;
	case HEADLESS_ON_PENDING:
		hw_headless_done_irq_enable(mors, false);
		mors->headless.state = HEADLESS_ON;
		morse_watchdog_pause(mors);

		if (mors->cfg->ops->flush_cache)
			mors->cfg->ops->flush_cache(mors);

		if (drv_work)
			complete(drv_work);
		break;
	case HEADLESS_OFF_REQUEST:
		(void)morse_reg32_write(mors,
					mors->chip_if->headless_cfg_addr,
					mors->headless.pending_cfg);
		hw_headless_done_irq_enable(mors, true);
		mors->headless.state = HEADLESS_OFF_PENDING;
		hw_trigger_attach(mors);
		break;
	case HEADLESS_OFF_PENDING:
		hw_headless_done_irq_enable(mors, false);
		mors->headless.state = HEADLESS_OFF;
		morse_watchdog_resume(mors);

		if (drv_work)
			complete(drv_work);
		break;
	default:
		MORSE_HEADLESS_WARN(mors, "%s: ignoring event for %s", __func__,
				    hw_headless_state_to_str(now));
	}
	after = mors->headless.state;
	mutex_unlock(&mors->headless.lock);

	morse_release_bus(mors);
	/* ps will be evaluated by the initiator once the operation is complete */
	morse_ps_wakers_dec(mors);

	trace_headless_work(hw_headless_state_to_str(now), hw_headless_state_to_str(after));
	MORSE_HEADLESS_DBG(mors,
			   "%s: %s -> %s\n",
			   __func__,
			   hw_headless_state_to_str(now),
			   hw_headless_state_to_str(after));
}

void morse_hw_headless_reset(struct morse *mors)
{
	enum headless_state now = mors->headless.state;

	mors->headless.state = HEADLESS_OFF;
	MORSE_HEADLESS_DBG(mors,
		"%s: %s -> %s\n",
		__func__,
		hw_headless_state_to_str(now),
		hw_headless_state_to_str(mors->headless.state));
}

int morse_hw_detach(struct morse *mors, int cfg)
{
	int ret = 0;
	bool triggered = false;
	unsigned int rem;
	DECLARE_COMPLETION_ONSTACK(drv_work);

	trace_headless_detach(cfg);
	mutex_lock(&mors->headless.lock);

	if (!(mors->firmware_flags & MORSE_FW_FLAGS_SUPPORT_HW_REATTACH)) {
		ret = -ENOTSUPP;
	} else if (mors->headless.wait) {
		ret = -EALREADY;
	} else if (mors->headless.state == HEADLESS_ON) {
		ret = 0; /* Already detached - go straight to exit */
		goto exit;
	} else if (mors->headless.state != HEADLESS_OFF) {
		ret = -EINVAL;
	}

	if (ret)
		goto exit;

	morse_ps_wakers_inc(mors);

	WRITE_ONCE(mors->headless.wait, &drv_work);
	mors->headless.pending_cfg = cfg;
	mors->headless.state = HEADLESS_ON_REQUEST;
	MORSE_HEADLESS_DBG(mors, "%s: headless on requested\n", __func__);
	/* Queue headless ON request and wait for its completion */
	queue_work(mors->chip_wq, &mors->headless.work);
	mutex_unlock(&mors->headless.lock);

	rem = wait_for_completion_timeout(mors->headless.wait,
		msecs_to_jiffies(HEADLESS_TIMEOUT_MS));
	mutex_lock(&mors->headless.lock);
	WRITE_ONCE(mors->headless.wait, NULL);

	morse_ps_wakers_dec(mors);
	morse_ps_queue_eval(mors);

	triggered = true;
	if (rem == 0)
		ret = -ETIMEDOUT;
	else
		ret = 0;

exit:
	mutex_unlock(&mors->headless.lock);
	if (ret)
		MORSE_HEADLESS_ERR(mors, "%s: failed (ret:%d)\n", __func__, ret);
	else if (triggered)
		MORSE_HEADLESS_INFO(mors, "%s: success\n", __func__);

	trace_headless_detach_return(ret);
	return ret;
}

int morse_hw_attach(struct morse *mors, int cfg)
{
	int ret = 0;
	unsigned int rem;
	bool triggered = false;
	DECLARE_COMPLETION_ONSTACK(drv_work);

	trace_headless_attach(cfg);
	mutex_lock(&mors->headless.lock);

	if (!(mors->firmware_flags & MORSE_FW_FLAGS_SUPPORT_HW_REATTACH)) {
		ret = -ENOTSUPP;
	} else if (mors->headless.wait) {
		ret = -EALREADY;
	} else if (mors->headless.state == HEADLESS_OFF) {
		ret = 0; /* Already attached - go straight to exit */
		goto exit;
	} else if (mors->headless.state != HEADLESS_ON) {
		ret = -EINVAL;
	}

	if (ret)
		goto exit;

	morse_ps_wakers_inc(mors);

	WRITE_ONCE(mors->headless.wait, &drv_work);
	mors->headless.pending_cfg = cfg;
	mors->headless.state = HEADLESS_OFF_REQUEST;
	MORSE_HEADLESS_DBG(mors, "%s: headless off requested\n", __func__);
	/* Queue headless OFF request and wait its completion */
	queue_work(mors->chip_wq, &mors->headless.work);
	mutex_unlock(&mors->headless.lock);

	rem = wait_for_completion_timeout(mors->headless.wait,
		msecs_to_jiffies(HEADLESS_TIMEOUT_MS));
	mutex_lock(&mors->headless.lock);
	WRITE_ONCE(mors->headless.wait, NULL);

	morse_ps_wakers_dec(mors);
	morse_ps_queue_eval(mors);

	triggered = true;
	if (rem == 0)
		ret = -ETIMEDOUT;
	else
		ret = 0;

exit:
	mutex_unlock(&mors->headless.lock);
	if (ret)
		MORSE_HEADLESS_ERR(mors, "%s: failed (ret:%d)\n", __func__, ret);
	else if (triggered)
		MORSE_HEADLESS_INFO(mors, "%s: success\n", __func__);

	trace_headless_attach_return(ret);
	return ret;
}

void morse_hw_headless_init(struct morse *mors)
{
	INIT_WORK(&mors->headless.work, morse_hw_headless_work);
	mutex_init(&mors->headless.lock);

	/* Assume initially that headless is ON until hardware is succesfully
	 * loaded or attached.
	 */
	mors->headless.state = HEADLESS_ON;
}

bool morse_hw_is_stopped_notification_set(struct morse *mors)
{
	u32 status1 = 0;
	int ret = 0;

	morse_claim_bus(mors);
	ret = morse_reg32_read(mors, MORSE_REG_INT1_STS(mors), &status1);
	morse_release_bus(mors);

	return (ret == 0) && (status1 & MORSE_INT_HW_STOP_NOTIFICATION);
}

bool morse_hw_should_reattach(void)
{
	return reattach_hw;
}

bool morse_hw_can_detach(const struct morse *mors)
{
	return reattach_hw && !morse_hw_is_stopped(mors) && !mors->recovery.ignore_detach;
}

static const u8 *hw_state_to_str(enum morse_hw_state state)
{
	switch (state) {
	case MORSE_HW_STATE_OFF:
		return "off";
	case MORSE_HW_STATE_ON:
		return "on";
	case MORSE_HW_STATE_STOPPED:
		return "stopped";
	case MORSE_HW_STATE_RESTARTING:
		return "restarting";
	default:
		return "unknown";
	}
}

bool morse_hw_is_restarting(const struct morse *mors)
{
	return mors->hw_state == MORSE_HW_STATE_RESTARTING;
}

bool morse_hw_is_on(const struct morse *mors)
{
	return mors->hw_state == MORSE_HW_STATE_ON;
}

bool morse_hw_is_stopped(const struct morse *mors)
{
	return mors->hw_state == MORSE_HW_STATE_STOPPED;
}

void morse_hw_set_state(struct morse *mors, enum morse_hw_state next)
{
	bool transition_okay;
	enum morse_hw_state state;

	lockdep_assert_held(&mors->lock);

	state = mors->hw_state;
	switch (state) {
	case MORSE_HW_STATE_OFF:
		transition_okay = (next == MORSE_HW_STATE_ON);
		break;
	case MORSE_HW_STATE_ON:
		transition_okay = (next == MORSE_HW_STATE_STOPPED ||
				   next == MORSE_HW_STATE_RESTARTING ||
				   next == MORSE_HW_STATE_OFF);
		break;
	case MORSE_HW_STATE_STOPPED:
		transition_okay = (next == MORSE_HW_STATE_RESTARTING ||
				   next == MORSE_HW_STATE_OFF);
		break;
	case MORSE_HW_STATE_RESTARTING:
		transition_okay = (next == MORSE_HW_STATE_ON || next == MORSE_HW_STATE_STOPPED);
		break;
	default:
		transition_okay = false;
		break;
	}

	MORSE_INFO(mors, "%s: %s -> %s", __func__, hw_state_to_str(state), hw_state_to_str(next));
	trace_hw_state(hw_state_to_str(state), hw_state_to_str(next));
	mors->hw_state = next;
	MORSE_WARN_ON(FEATURE_ID_DEFAULT, !transition_okay);
}
