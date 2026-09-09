/*
 * Copyright 2017-2022 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#if !defined(__TRACE_MORSE_H) || defined(TRACE_HEADER_MULTI_READ)
#define __TRACE_MORSE_H

#include <linux/tracepoint.h>
#include "morse.h"
#include "debug.h"
#include "skb_header.h"

#undef TRACE_SYSTEM
#define TRACE_SYSTEM morse

#define SKB_CHAN_ENTRY __array(char, skb_chan_name, 16)
#define SKB_CHAN_ASSIGN strscpy(__entry->skb_chan_name, morse_skb_channel_name(channel), 16)
#define SKB_CHAN_PR_FMT "[%s]"
#define SKB_CHAN_PR_ARG __entry->skb_chan_name

DECLARE_EVENT_CLASS(morse_u32_evt,
	TP_PROTO(u32 value),
	TP_ARGS(value),
	TP_STRUCT__entry(__field(u32, value)),
	TP_fast_assign(__entry->value = value;),
	TP_printk("%d", __entry->value)
);

DECLARE_EVENT_CLASS(morse_u32_hex_evt,
	TP_PROTO(u32 value),
	TP_ARGS(value),
	TP_STRUCT__entry(__field(u32, value)),
	TP_fast_assign(__entry->value = value;),
	TP_printk("0x%08x", __entry->value)
);

DECLARE_EVENT_CLASS(morse_bus_error,
	TP_PROTO(const char *op, uint fn, u32 address, uint len, u32 reg_base, u32 reg_bulk,
		 int ret),
	TP_ARGS(op, fn, address, len, reg_base, reg_bulk, ret),
	TP_STRUCT__entry(__string(operation, op)
			 __field(uint, fn)
			 __field(u32, address)
			 __field(uint, len)
			 __field(u32, reg_base)
			 __field(u32, reg_bulk)
			 __field(int, ret)
	),
#if KERNEL_VERSION(6, 10, 0) > LINUX_VERSION_CODE
	TP_fast_assign(__assign_str(operation, op);
#else
	TP_fast_assign(__assign_str(operation);
#endif
		       __entry->fn = fn;
		       __entry->address = address;
		       __entry->len = len;
		       __entry->reg_base = reg_base;
		       __entry->reg_bulk = reg_bulk;
		       __entry->ret = ret;
	),
	TP_printk("%s fn[%d] 0x%08x:%d r:0x%08x b:0x%08x (ret:%d)",
		  __get_str(operation),
		  __entry->fn, __entry->address, __entry->len, __entry->reg_base,
		  __entry->reg_bulk, __entry->ret)
);

DECLARE_EVENT_CLASS(morse_state_change,
	TP_PROTO(const char *old, const char *new),
	TP_ARGS(old, new),
	TP_STRUCT__entry(__string(old, old)
			 __string(new, new)
	),
#if KERNEL_VERSION(6, 10, 0) > LINUX_VERSION_CODE
	TP_fast_assign(__assign_str(old, old);
		       __assign_str(new, new);
#else
	TP_fast_assign(__assign_str(old);
		       __assign_str(new);
#endif
	),
	TP_printk("%s -> %s", __get_str(old), __get_str(new))
);

DEFINE_EVENT(morse_bus_error, sdio_err,
	TP_PROTO(const char *op, uint fn, u32 address, uint len, u32 reg_base, u32 reg_bulk,
		 int ret),
	TP_ARGS(op, fn, address, len, reg_base, reg_bulk, ret)
);

DEFINE_EVENT(morse_u32_evt, beacon_tasklet_enter,
	TP_PROTO(u32 value), TP_ARGS(value)
);

DEFINE_EVENT(morse_u32_hex_evt, beacon_tasklet_exit,
	TP_PROTO(u32 value), TP_ARGS(value)
);

DEFINE_EVENT(morse_u32_evt, hw_stop,
	TP_PROTO(u32 value), TP_ARGS(value)
);

DEFINE_EVENT(morse_state_change, hw_state,
	TP_PROTO(const char *old, const char *new), TP_ARGS(old, new)
);

#ifdef CONFIG_MORSE_TRACE_LOG_MSG
#define MORSE_MSG_MAX 200

DECLARE_EVENT_CLASS(morse_log_event,
	TP_PROTO(const struct morse *mors, struct va_format *vaf),
	TP_ARGS(mors, vaf),
	TP_STRUCT__entry(__string(device,
			 dev_name(mors->dev))
			 __string(driver, dev_driver_string(mors->dev))
			 __dynamic_array(char, msg, MORSE_MSG_MAX)),
#if KERNEL_VERSION(6, 10, 0) > LINUX_VERSION_CODE
	TP_fast_assign(__assign_str(device, dev_name(mors->dev));
		       __assign_str(driver, dev_driver_string(mors->dev));
#else
	TP_fast_assign(__assign_str(device);
		       __assign_str(driver);
#endif
		       WARN_ON_ONCE(vsnprintf(__get_dynamic_array(msg),
					      MORSE_MSG_MAX,
					      vaf->fmt, *vaf->va) >= MORSE_MSG_MAX);),
	TP_printk("%s %s %s", __get_str(driver), __get_str(device), __get_str(msg))
);

DEFINE_EVENT(morse_log_event, morse_err,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_warn,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_info,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_dbg,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_err_ratelimited,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_warn_ratelimited,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_info_ratelimited,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

DEFINE_EVENT(morse_log_event, morse_dbg_ratelimited,
	TP_PROTO(const struct morse *mors, struct va_format *vaf), TP_ARGS(mors, vaf)
);

#else
#define trace_morse_err(...)
#define trace_morse_warn(...)
#define trace_morse_info(...)
#define trace_morse_dbg(...)
#define trace_morse_err_ratelimited(...)
#define trace_morse_warn_ratelimited(...)
#define trace_morse_info_ratelimited(...)
#define trace_morse_dbg_ratelimited(...)
#endif /* CONFIG_MORSE_TRACE_LOG_MSG */

#ifdef CONFIG_MORSE_TRACE_BUS
DECLARE_EVENT_CLASS(morse_bus_evt,
	TP_PROTO(uint fn, u32 address, uint len),
	TP_ARGS(fn, address, len),
	TP_STRUCT__entry(__field(uint, fn)
			 __field(u32, address)
			 __field(uint, len)
	),
	TP_fast_assign(__entry->fn = fn;
		       __entry->address = address;
		       __entry->len = len;
	),
	TP_printk("fn[%d] 0x%08x (%d)", __entry->fn, __entry->address, __entry->len)
);

DEFINE_EVENT(morse_bus_evt, bus_set_reg_base,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_set_bulk_base,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_en_irq,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_reset_base,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_reg_write,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_reg_read,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_bulk_write,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_bulk_read,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);

DEFINE_EVENT(morse_bus_evt, bus_en,
	TP_PROTO(uint fn, u32 address, uint len), TP_ARGS(fn, address, len)
);
#else
#define trace_bus_en_irq(...)
#define trace_bus_reset_base(...)
#define trace_bus_reg_write(...)
#define trace_bus_reg_read(...)
#define trace_bus_bulk_write(...)
#define trace_bus_bulk_read(...)
#define trace_bus_en(...)
#endif /* CONFIG_MORSE_TRACE_BUS */

#ifdef CONFIG_MORSE_TRACE_PS
DEFINE_EVENT(morse_u32_evt, ps_wake_start,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, ps_wake_end,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, ps_sleep,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, ps_wake_gpio,
	TP_PROTO(u32 raise), TP_ARGS(raise)
);
DEFINE_EVENT(morse_u32_evt, ps_to_host_busy_irq,
	TP_PROTO(u32 work_waiting), TP_ARGS(work_waiting)
);
DEFINE_EVENT(morse_u32_evt, ps_chip_wake_error,
	TP_PROTO(u32 timeout), TP_ARGS(timeout)
);
#else
#define trace_ps_wake_start(...)
#define trace_ps_wake_end(...)
#define trace_ps_sleep(...)
#define trace_ps_wake_gpio(...)
#define trace_ps_to_host_busy_irq(...)
#define trace_ps_chip_wake_error(...)
#endif /* CONFIG_MORSE_TRACE_PS */

#ifdef CONFIG_MORSE_TRACE_PAGER_HW
#include "pager_hw.h"

#define PAGER_ENTRY __array(char, pager_name, 32)
#define PAGER_ASSIGN strscpy(__entry->pager_name, morse_pager_hw_name(pager->flags), 32)
#define PAGER_PR_FMT "[%s]"
#define PAGER_PR_ARG __entry->pager_name

DECLARE_EVENT_CLASS(morse_pager_hw_evt,
	TP_PROTO(const struct morse_pager *pager, uint arg),
	TP_ARGS(pager, arg),
	TP_STRUCT__entry(PAGER_ENTRY
			 __field(uint, arg)
	),
	TP_fast_assign(PAGER_ASSIGN;
		       __entry->arg = arg;
	),
	TP_printk(PAGER_PR_FMT " 0x%08x", PAGER_PR_ARG, __entry->arg)
);

DEFINE_EVENT(morse_pager_hw_evt, pager_hw_cache_put_pages,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_pop,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_cache_get,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_put,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_store_bit_bulk,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_notify,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_write_page,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
DEFINE_EVENT(morse_pager_hw_evt, pager_hw_read_page,
	TP_PROTO(const struct morse_pager *pager, uint arg), TP_ARGS(pager, arg)
);
#else
#define trace_pager_hw_cache_put_pages(...)
#define trace_pager_hw_pop(...)
#define trace_pager_hw_cache_get(...)
#define trace_pager_hw_put(...)
#define trace_pager_hw_store_bit_bulk(...)
#define trace_pager_hw_notify(...)
#define trace_pager_hw_write_page(...)
#define trace_pager_hw_read_page(...)
#endif /* CONFIG_MORSE_TRACE_PAGER_HW */

#ifdef CONFIG_MORSE_TRACE_PAGESET
DECLARE_EVENT_CLASS(morse_pagesets_channel,
	TP_PROTO(enum morse_skb_channel channel, uint count),
	TP_ARGS(channel, count),
	TP_STRUCT__entry(SKB_CHAN_ENTRY
			 __field(uint, count)
	),
	TP_fast_assign(SKB_CHAN_ASSIGN;
		       __entry->count = count;
	),
	TP_printk(SKB_CHAN_PR_FMT " %d", SKB_CHAN_PR_ARG, __entry->count)
);

DEFINE_EVENT(morse_u32_hex_evt, pagesets_work_enter,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_hex_evt, pagesets_work_exit,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, pagesets_stopped_rx_for_beacon_tx,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, pagesets_rx,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, pagesets_stale_tx_flushed,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_pagesets_channel, pagesets_tx,
	TP_PROTO(enum morse_skb_channel channel, uint count), TP_ARGS(channel, count)
);
DEFINE_EVENT(morse_pagesets_channel, pagesets_tx_remaining,
	TP_PROTO(enum morse_skb_channel channel, uint count), TP_ARGS(channel, count)
);
DEFINE_EVENT(morse_u32_evt, pagesets_tx_buffers_avail,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_pagesets_channel, pagesets_rx_skbq_enqueue_fail,
	TP_PROTO(enum morse_skb_channel channel, uint count), TP_ARGS(channel, count)
);
#else
#define trace_pagesets_work_enter(...)
#define trace_pagesets_work_exit(...)
#define trace_pagesets_stopped_rx_for_beacon_tx(...)
#define trace_pagesets_rx(...)
#define trace_pagesets_stale_tx_flushed(...)
#define trace_pagesets_tx(...)
#define trace_pagesets_tx_remaining(...)
#define trace_pagesets_tx_buffers_avail(...)
#define trace_pagesets_rx_skbq_enqueue_fail(...)
#endif /* CONFIG_MORSE_TRACE_PAGESET */

#ifdef CONFIG_MORSE_TRACE_HW_IRQ
DEFINE_EVENT(morse_u32_hex_evt, hw_irq,
	TP_PROTO(u32 status), TP_ARGS(status)
);
#else
#define trace_hw_irq(...)
#endif /* CONFIG_MORSE_TRACE_HW_IRQ */

#ifdef CONFIG_MORSE_TRACE_RX
DECLARE_EVENT_CLASS(morse_rx,
	TP_PROTO(enum morse_skb_channel channel),
	TP_ARGS(channel),
	TP_STRUCT__entry(SKB_CHAN_ENTRY),
	TP_fast_assign(SKB_CHAN_ASSIGN;),
	TP_printk(SKB_CHAN_PR_FMT, SKB_CHAN_PR_ARG)
);
DEFINE_EVENT(morse_rx, rx_processed,
	TP_PROTO(enum morse_skb_channel channel), TP_ARGS(channel)
);
#else
#define trace_rx_processed(...)
#endif /* CONFIG_MORSE_TRACE_RX */

#ifdef CONFIG_MORSE_TRACE_HEADLESS
DEFINE_EVENT(morse_state_change, headless_work,
	TP_PROTO(const char *old, const char *new), TP_ARGS(old, new)
);
DEFINE_EVENT(morse_u32_evt, headless_detach,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, headless_detach_return,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, headless_attach,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, headless_attach_return,
	TP_PROTO(u32 value), TP_ARGS(value)
);
#else
#define trace_headless_work(...)
#define trace_headless_detach(...)
#define trace_headless_detach_return(...)
#define trace_headless_attach(...)
#define trace_headless_attach_return(...)
#endif /* CONFIG_MORSE_TRACE_HEADLESS */

#ifdef CONFIG_MORSE_TRACE_SUSPEND
DEFINE_EVENT(morse_u32_evt, wowlan_suspend,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, wowlan_suspend_return,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, wowlan_resume,
	TP_PROTO(u32 value), TP_ARGS(value)
);
DEFINE_EVENT(morse_u32_evt, wowlan_resume_return,
	TP_PROTO(u32 value), TP_ARGS(value)
);
#else
#define trace_wowlan_suspend(...)
#define trace_wowlan_suspend_return(...)
#define trace_wowlan_resume(...)
#define trace_wowlan_resume_return(...)
#endif /* CONFIG_MORSE_TRACE_SUSPEND */

#ifdef CONFIG_MORSE_TRACE_SKBQ
DECLARE_EVENT_CLASS(morse_skbq_channel_event,
	TP_PROTO(enum morse_skb_channel channel, uint event),
	TP_ARGS(channel, event),
	TP_STRUCT__entry(SKB_CHAN_ENTRY
			 __field(uint, event)
	),
	TP_fast_assign(SKB_CHAN_ASSIGN;
		       __entry->event = event;
	),
	TP_printk(SKB_CHAN_PR_FMT " %d", SKB_CHAN_PR_ARG, __entry->event)
);

DEFINE_EVENT(morse_skbq_channel_event, skbq_new_tx,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
DEFINE_EVENT(morse_skbq_channel_event, skbq_tx_complete,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
DEFINE_EVENT(morse_skbq_channel_event, skbq_await_tx_status,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
DEFINE_EVENT(morse_skbq_channel_event, skbq_drop_pending_skb,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
DEFINE_EVENT(morse_skbq_channel_event, skbq_drop_stale_tx_status,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
DEFINE_EVENT(morse_skbq_channel_event, skbq_tx_status_finish,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
DEFINE_EVENT(morse_skbq_channel_event, skbq_tx_status_unknown_channel,
	TP_PROTO(enum morse_skb_channel channel, uint event), TP_ARGS(channel, event)
);
#else
#define trace_skbq_new_tx(...)
#define trace_skbq_tx_complete(...)
#define trace_skbq_await_tx_status(...)
#define trace_skbq_drop_pending_skb(...)
#define trace_skbq_drop_stale_tx_status(...)
#define trace_skbq_tx_status_finish(...)
#define trace_skbq_tx_status_unknown_channel(...)
#endif /* CONFIG_MORSE_TRACE_SKBQ */
#endif

/* we don't want to use include/trace/events */
#undef TRACE_INCLUDE_PATH
#ifndef MORSE_TRACE_PATH
#error "MORSE_TRACE_PATH must be defined"
#endif
#define TRACE_INCLUDE_PATH	MORSE_TRACE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE	trace

/* This part must be outside protection */
#include <trace/define_trace.h>
