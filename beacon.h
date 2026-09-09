/*
 * Copyright 2025 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef _MORSE_BEACON_H_
#define _MORSE_BEACON_H_

#include "morse.h"

#define MORSE_BEACON_DBG(_m, _f, _a...)   morse_dbg(FEATURE_ID_BEACON, _m, _f, ##_a)
#define MORSE_BEACON_INFO(_m, _f, _a...)  morse_info(FEATURE_ID_BEACON, _m, _f, ##_a)
#define MORSE_BEACON_WARN(_m, _f, _a...)  morse_warn(FEATURE_ID_BEACON, _m, _f, ##_a)
#define MORSE_BEACON_ERR(_m, _f, _a...)   morse_err(FEATURE_ID_BEACON, _m, _f, ##_a)

#define MORSE_BEACON_HEXDUMP_DBG(prefix, buf, len)						\
	do {											\
		if (morse_log_is_enabled(FEATURE_ID_BEACON, MORSE_MSG_DEBUG))			\
			print_hex_dump_bytes(prefix, DUMP_PREFIX_OFFSET, buf, len);		\
	} while (0)

#define MORSE_BEACON_ERR_RATELIMITED(_m, _f, _a...) \
	morse_err_ratelimited(FEATURE_ID_BEACON, _m, _f, ##_a)
#define MORSE_BEACON_WARN_RATELIMITED(_m, _f, _a...) \
	morse_warn_ratelimited(FEATURE_ID_BEACON, _m, _f, ##_a)
#define MORSE_BEACON_DBG_RATELIMITED(_m, _f, _a...) \
	morse_dbg_ratelimited(FEATURE_ID_BEACON, _m, _f, ##_a)

#if KERNEL_VERSION(6, 0, 0) > MAC80211_VERSION_CODE
#define MORSE_IEEE_BEACON_GET(mors, vif) ieee80211_beacon_get((mors)->hw, (vif))
#define MORSE_IEEE_BEACON_GET_TEMPLATE(mors, vif, offs) \
	ieee80211_beacon_get_template((mors)->hw, (vif), (offs))
#else
#define MORSE_IEEE_BEACON_GET(mors, vif) ieee80211_beacon_get((mors)->hw, (vif), 0)
#define MORSE_IEEE_BEACON_GET_TEMPLATE(mors, vif, offs) \
	ieee80211_beacon_get_template((mors)->hw, (vif), (offs), 0)

#endif

/** Options for inserting RSN IE into beacon */
enum morse_mac_rsn_beacon_mode {
	/** Don't insert RSN into beacon (default) */
	RSN_BEACON_DISABLED = 0x00,
	/** Insert RSN into long beacon only */
	RSN_BEACON_LONG = 0x01,
	/** Insert RSN into all beacons */
	RSN_BEACON_ALL = 0x02
};

/**
 * morse_beacon_generate - convert a list of IEs and insert them into an SKB
 * @mors_vif: morse VIF structure
 * @bcn_skb: SKB to populate with the modified IEs
 * @ies_mask: IEs mask to convert to S1G IEs and then populate bcn_skb with
 * @ie_mask_len: length of the IE mask
 * @short_beacon: true if a short beacon is to be generated
 *
 * Convert a beacon SKB from an 11n frame to an S1g frame and populate it with modified s1g IEs.
 * The bcn_skb will be resized to accommodate new IEs it is the callers responsibility to free it.
 *
 * All calls inside this function must be atomic as it will be called by the beacon tasklet.
 *
 * Return: TRUE if beacon IEs updated successfully, else error code
 */
int morse_beacon_convert_ies_and_populate_skb(struct morse_vif *mors_vif,
					      struct sk_buff **bcn_skb,
					      struct dot11ah_ies_mask *ies_mask,
					      int *ie_mask_len,
					      const bool short_beacon);

/**
 * morse_beacon_insert_ies - insert relevent beacon IEs into the IE mask
 * @mors_vif: morse VIF structure
 * @ies_mask: IEs mask to insert beacon IEs
 * @frame_control: Beacon frame control
 * @short_beacon: true if a short beacon is to be generated
 *
 * Insert relevent beacon IEs not done so by mac80211
 *
 */
void morse_beacon_insert_ies(struct morse_vif *mors_vif,
			     struct dot11ah_ies_mask *ies_mask,
			     __le16 frame_control, bool short_beacon);
/**
 * morse_beacon_get_rsn_mode - return the rsn_beacon_mode mod param
 */
enum morse_mac_rsn_beacon_mode morse_beacon_get_rsn_mode(void);

/**
 * morse_beacon_get_short_bcn_dtim_override - return the enable_short_bcn_as_dtim_override mod param
 */
int morse_beacon_get_short_bcn_dtim_override(void);

#endif
