#ifndef _MORSE_WIPHY_H_
#define _MORSE_WIPHY_H_

/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include <net/mac80211.h>

#include "debug.h"
#include "morse.h"

#ifdef CONFIG_MORSE_FULLMAC

/**
 * morse_wiphy_connect_insert_tlvs() - Pack connect TLVs into request buffer.
 * @buf: Buffer to be packed with TLVs. This is the variable portion of the request structure.
 * @params: Connection request parameters to be converted to TLVs.
 *
 * Use morse_wiphy_connect_get_command_size() to size the request buffer appropriately.
 */
void morse_wiphy_connect_insert_tlvs(u8 *buf, const struct morse_wiphy_connect_params *params);

/**
 * morse_wiphy_connect_get_command_size() - Calculate required length for connect request.
 * @params: Connection request parameters.
 *
 * Return: total length of the request, including command header.
 */
size_t morse_wiphy_connect_get_command_size(const struct morse_wiphy_connect_params *params);

/**
 * morse_wiphy_get_sta_vif() - Return global STA vif
 * @morse: morse device instance
 *
 * Return: STA vif, or NULL.
 */
struct morse_vif *morse_wiphy_get_sta_vif(struct morse *mors);

/**
 * morse_wiphy_to_morse -  Look up &struct mors inside &struct wiphy
 *
 * Return: pointer to &struct mors
 */
struct morse *morse_wiphy_to_morse(struct wiphy *wiphy);

/**
 * morse_wdev_to_morse_vif -  Look up &morse_vif inside &struct wireless_dev
 *
 * Return: pointer to &struct morse_vif, NULL when we're in sniffer mode
 */
struct morse_vif *morse_wdev_to_morse_vif(struct wireless_dev *wdev);

/**
 * morse_wiphy_init() -  Init wiphy device
 * @morse: morse device instance
 *
 * Initilise wiphy device
 *
 * Return: 0 success, else error.
 */
int morse_wiphy_init(struct morse *mors);

/**
 * morse_wiphy_register() -  Register wiphy device
 * @morse: morse device instance
 *
 * Register wiphy device
 *
 * Return: 0 success, else error.
 */
int morse_wiphy_register(struct morse *mors);

/**
 * morse_wiphy_create() -  Create wiphy device
 * @priv_size: extra size per structure to allocate
 * @dev: Bus device structure
 *
 * Allocate memory for wiphy device and do basic initialisation.
 *
 * Return: morse device struct, else NULL.
 */
struct morse *morse_wiphy_create(size_t priv_size, struct device *dev);

/**
 * morse_wiphy_stop() -  Stop wiphy device in preparation for chip restart
 * @mors: morse device instance
 */
void morse_wiphy_stop(struct morse *mors);

/**
 * morse_wiphy_cleanup() -  Clean up cfg80211 state on chip shutdown.
 * @morse: morse device instance
 */
void morse_wiphy_cleanup(struct morse *mors);

/**
 * morse_wiphy_restarted() -  Notify wiphy device that chip restarted
 * @mors: morse device instance
 *
 * Device state will be reset and userspace will be informed that the connection was lost.
 */
void morse_wiphy_restarted(struct morse *mors);

/**
 * morse_wiphy_deinit() -  Deinit wiphy device
 * @morse: morse device instance
 *
 * Deinitilise wiphy device
 *
 * Return: None.
 */
void morse_wiphy_deinit(struct morse *mors);

/**
 * morse_wiphy_destroy() -  Destroy wiphy device
 * @morse: morse device instance
 *
 * Free wiphy device
 * Acquires and releases the rtnl lock.
 *
 * Return: None.
 */
void morse_wiphy_destroy(struct morse *mors);

/**
 * morse_wiphy_rx() -  Receive WIPHY (802.3) packet
 * @mors: Morse state struct
 * @skb: SKB packet
 *
 * Passes the packet to upper layers.
 * Must be invoked from process context.
 */
void morse_wiphy_rx(struct morse *mors, struct sk_buff *skb);

/**
 * morse_wiphy_scan_result() -  Process a result from an in-progress scan
 * @mors: morse device instance
 * @result: scan result event data
 *
 * Return: 0 on success, else error.
 */
int morse_wiphy_scan_result(struct morse *mors, struct morse_cmd_evt_scan_result *result);

/**
 * morse_wiphy_scan_done() -  Mark scan as complete
 * @mors: morse device instance
 * @aborted: true if the scan terminated without scanning all channels
 */
void morse_wiphy_scan_done(struct morse *mors, bool aborted);

/**
 * morse_wiphy_connected() -  Mark connection as established
 * @mors: morse device instance
 * @bssid: BSSID which the chip is connected to
 */
void morse_wiphy_connected(struct morse *mors, const u8 *bssid, const u8 *assoc_resp_ies,
			   u16 assoc_resp_ies_len);

/**
 * morse_wiphy_disconnected() -  Mark connection as lost
 * @mors: morse device instance
 */
void morse_wiphy_disconnected(struct morse *mors);

/**
 * morse_wiphy_disconnected_work_nolock() - Notify the kernel that the connection has been lost
 *
 * The caller of this function must hold mors->lock.
 *
 * @mors: Morse device instance.
 * @mors_vif: Morse interface.
 */
void morse_wiphy_disconnected_work_nolock(struct morse *mors, struct morse_vif *mors_vif);

/**
 * morse_wiphy_traffic_control() -  Pause or resume traffic
 * @mors: morse device instance
 * @pause_data_traffic: true to pause, false to resume
 * @sources: source of traffic control
 */
int morse_wiphy_traffic_control(struct morse *mors, bool pause_data_traffic, int sources);

/**
 * morse_wiphy_rx_mgmt() -	Receive 802.11 management frame
 * @mors: morse device instance
 * @skb: SKB packet
 * @hdr_rx_status: rx status header
 */
void
morse_wiphy_rx_mgmt(struct morse *mors,
		    struct sk_buff *skb,
		    struct morse_skb_rx_status *hdr_rx_status);

/**
 * morse_wiphy_stop_tx_queues() - Request the network stack to stop enqueuing packets for tx.
 */
void morse_wiphy_stop_tx_queues(struct morse *mors);

/**
 * morse_wiphy_wake_tx_queues() - Inform the network stack it may resume enqueuing packets for tx.
 */
void morse_wiphy_wake_tx_queues(struct morse *mors);

/**
 * morse_wiphy_is_idle() - FullMAC VIF is currently in IDLE state.
 */
bool morse_wiphy_is_idle(const struct morse_vif *mors_vif);

/**
 * morse_wiphy_is_connecting() - FullMAC VIF is connecting to AP.
 */
bool morse_wiphy_is_connecting(const struct morse_vif *mors_vif);

/**
 * morse_wiphy_is_connected() - FullMAC VIF is connected to AP.
 */
bool morse_wiphy_is_connected(const struct morse_vif *mors_vif);

/**
 * morse_wiphy_is_disconnecting() - FullMAC VIF is disconnecting from AP.
 */
bool morse_wiphy_is_disconnecting(const struct morse_vif *mors_vif);

/**
 * morse_wiphy_is_roaming() - FullMAC VIF is currently roaming.
 */
bool morse_wiphy_is_roaming(const struct morse_vif *mors_vif);

#else

static inline void
morse_wiphy_connect_insert_tlvs(u8 *buf, const struct morse_wiphy_connect_params *params)
{
}

static inline size_t
morse_wiphy_connect_get_command_size(const struct morse_wiphy_connect_params *params)
{
	return 0;
}

static inline struct morse_vif *morse_wiphy_get_sta_vif(struct morse *mors)
{
	return NULL;
}

static inline struct morse *morse_wiphy_to_morse(struct wiphy *wiphy)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);

	return hw->priv;
}

static inline struct morse_vif *morse_wdev_to_morse_vif(struct wireless_dev *wdev)
{
	struct ieee80211_vif *vif = wdev_to_ieee80211_vif(wdev);

	if (vif)
		return ieee80211_vif_to_morse_vif(vif);

	return NULL;
}

static inline int morse_wiphy_init(struct morse *mors)
{
	MORSE_ERR(mors, "FullMAC support disabled at build time\n");
	return -ENOTSUPP;
}

static inline int morse_wiphy_register(struct morse *mors)
{
	return -ENOTSUPP;
}

static inline struct morse *morse_wiphy_create(size_t priv_size, struct device *dev)
{
	return NULL;
}

static inline void morse_wiphy_stop(struct morse *mors)
{
}

static inline void morse_wiphy_cleanup(struct morse *mors)
{
}

static inline void morse_wiphy_restarted(struct morse *mors)
{
}

static inline void morse_wiphy_deinit(struct morse *mors)
{
}

static inline void morse_wiphy_destroy(struct morse *mors)
{
}

static inline void morse_wiphy_rx(struct morse *mors, struct sk_buff *skb)
{
}

static inline int
morse_wiphy_scan_result(struct morse *mors, struct morse_cmd_evt_scan_result *result)
{
	return -ENOTSUPP;
}

static inline void morse_wiphy_scan_done(struct morse *mors, bool aborted)
{
}

static inline void
morse_wiphy_connected(struct morse *mors, const u8 *bssid, const u8 *assoc_resp_ies,
		      u16 assoc_resp_ies_len)
{
}

static inline void morse_wiphy_disconnected(struct morse *mors)
{
}

static inline void
morse_wiphy_disconnected_work_nolock(struct morse *mors, struct morse_vif *mors_vif)
{
}

static inline int
morse_wiphy_traffic_control(struct morse *mors, bool pause_data_traffic, int sources)
{
	return -ENOTSUPP;
}

static inline void morse_wiphy_rx_mgmt(struct morse *mors,
				       struct sk_buff *skb,
				       struct morse_skb_rx_status *hdr_rx_status)
{
}

static inline void morse_wiphy_stop_tx_queues(struct morse *mors)
{
}

static inline void morse_wiphy_wake_tx_queues(struct morse *mors)
{
}

static inline bool
morse_wiphy_is_idle(const struct morse_vif *mors_vif)
{
	return false;
}

static inline bool
morse_wiphy_is_connecting(const struct morse_vif *mors_vif)
{
	return false;
}

static inline bool
morse_wiphy_is_connected(const struct morse_vif *mors_vif)
{
	return false;
}

static inline bool
morse_wiphy_is_disconnecting(const struct morse_vif *mors_vif)
{
	return false;
}

static inline bool
morse_wiphy_is_roaming(const struct morse_vif *mors_vif)
{
	return false;
}

#endif

#endif /* !_MORSE_WIPHY_H_ */
