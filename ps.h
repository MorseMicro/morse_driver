#ifndef _MORSE_PS_H_
#define _MORSE_PS_H_

/*
 * Copyright 2017-2022 Morse Micro
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see
 * <https://www.gnu.org/licenses/>.
 *
 */
#include "morse.h"

/** This should be nominally <= the dynamic ps timeout */
#define NETWORK_BUS_TIMEOUT_MS (90)
#define UAPSD_NETWORK_BUS_TIMEOUT_MS (5)
/** The default period of time to wait to re-evaluate powersave */
#define DEFAULT_BUS_TIMEOUT_MS (5)

static inline int morse_network_bus_timeout(struct morse *mors)
{
	return mors->uapsd_per_ac ? UAPSD_NETWORK_BUS_TIMEOUT_MS : NETWORK_BUS_TIMEOUT_MS;
}

int morse_ps_enable(struct morse *mors);

int morse_ps_disable(struct morse *mors);

/**
 * Call this function when there is activity on the bus that should
 * delay the driver in disabling the bus.
 *
 * @mors: Morse chip instance
 * @timeout_ms: The timeout from now to add (ms)
 */
void morse_ps_bus_activity(struct morse *mors, int timeout_ms);

int morse_ps_init(struct morse *mors, bool enable, bool enable_dynamic_ps);

void morse_ps_finish(struct morse *mors);

#endif /* !_MORSE_PS_H_ */
