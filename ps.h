#ifndef _MORSE_PS_H_
#define _MORSE_PS_H_

/*
 * Copyright 2017-2022 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "morse.h"

/**
 * mors_ps_get_net_timeout_ms() - Get power save timeout after network activity.
 *
 * @mors: Morse chip instance
 *
 * Return: timeout in milliseconds
 */
int mors_ps_get_net_timeout_ms(struct morse *mors);

/**
 * mors_ps_set_net_timeout_ms() - Set power save timeout after network activity.
 *
 * @mors: Morse chip instance
 * @timeout_ms: Timeout in milliseconds
 */
void mors_ps_set_net_timeout_ms(struct morse *mors, int timeout_ms);

/**
 * morse_ps_force_eval() - Force an evaluation of power save requirements.
 * @mors: Morse chip instance
 */
void morse_ps_force_eval(struct morse *mors);

/**
 * morse_ps_queue_eval() - Queue evaluation of power save requirements to happen
 *                         at a later time.
 * @mors: Morse chip instance
 */
void morse_ps_queue_eval(struct morse *mors);

/**
 * morse_ps_wakers_inc() - Increase number of wakers holding the chip awake.
 * @mors: Morse chip instance
 *
 * Invoke this function to keep the chip awake. Calls to inc/dec are typically balanced.
 *
 * Return: 0 if success else error code
 */
int morse_ps_wakers_inc(struct morse *mors);

/**
 * morse_ps_wakers_dec() - Decrease number of wakers holding the chip awake.
 * @mors: Morse chip instance
 *
 * Invoke this function to remove chip wake requirement. Calls to inc/dec are typically balanced.
 *
 * Return: 0 if success else error code
 */
int morse_ps_wakers_dec(struct morse *mors);

/**
 * morse_ps_bus_activity() - Call this function when there is activity on the
 *                           bus that should delay the driver in disabling the bus.
 *
 * @mors: Morse chip instance
 * @timeout_ms: The timeout from now to add (ms)
 */
void morse_ps_bus_activity(struct morse *mors, int timeout_ms);

/**
 * morse_ps_get_iface_id() - Get interface ID of PS interface.
 *
 * @mors: Morse chip instance
 *
 * @return id or -1 if not set
 */
int morse_ps_get_iface_id(struct morse *mors);

/**
 * morse_ps_iface_down_notify() - Notify power save logic that an interface is going down.
 * @mors: Morse chip instance
 * @mors_vif: Virtual interface being disabled
 *
 * Updates internal state to reflect that the given virtual interface is no
 * longer active.
 */
void morse_ps_iface_down_notify(struct morse *mors, struct morse_vif *mors_vif);

/**
 * morse_ps_is_interface_enabled() - Check if any interface is currently enabled.
 * @mors: Morse chip instance
 *
 * Return: if the interface associated with the morse_ps object is currently enabled
 */
bool morse_ps_is_interface_enabled(struct morse *mors);

/**
 * morse_ps_is_interface_same() - Compare given interface against current active one.
 * @mors: Morse chip instance
 * @mors_vif: Virtual interface to compare
 *
 * Return: true if the given virtual interface matches the one currently in use.
 */
bool morse_ps_is_interface_same(struct morse *mors, struct morse_vif *mors_vif);

/**
 * morse_ps_update_interface_state() - Update interface state in power save logic.
 * @mors: Morse chip instance
 * @mors_vif: New interface to set as the owner of the ps object
 * @enabled: New state of power save
 *
 * Update the interface and the power save state of the morse_ps object
 */
void morse_ps_update_interface_state(struct morse *mors, struct morse_vif *mors_vif, bool enabled);

int morse_ps_init(struct morse *mors);

void morse_ps_finish(struct morse *mors);

/**
 * morse_ps_suspend() - Invoked on host system suspend
 *
 * @mors: Morse chip instance
 */
int morse_ps_system_suspend(struct morse *mors);

/**
 * morse_ps_resume() - Invoked on host system resume
 *
 * @mors: Morse chip instance
 */
void morse_ps_system_resume(struct morse *mors);

/**
 * morse_ps_is_dynamic_offload_enabled() - Check if dynamic_ps_offload is enabled or not.
 *
 * Return: true if enabled, false if not.
 */
bool morse_ps_is_dynamic_offload_enabled(void);

/**
 * morse_ps_is_disabled() - Check if ps is disabled.
 *
 * Return: true if ps is disabled, false if not.
 */
bool morse_ps_is_disabled(void);

/**
 * morse_ps_is_fully_enabled() - Check if ps is fully enabled.
 *
 * Return: true if ps if fully enabled, false if not.
 */
bool morse_ps_is_fully_enabled(void);

/**
 * morse_ps_is_supported() - Check if the device have capability to support ps.
 *
 * Return: true if the device meet all conditions to support PS, false if not.
 */
bool morse_ps_is_supported(struct morse *mors);

#endif /* !_MORSE_PS_H_ */
