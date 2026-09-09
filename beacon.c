/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include <linux/interrupt.h>
#include <linux/ieee80211.h>

#include "mac.h"
#include "bus.h"
#include "raw.h"
#include "debug.h"
#include "dot11ah/dot11ah.h"
#include "skb_header.h"
#include "skbq.h"
#include "vendor.h"
#include "vendor_ie.h"
#include "utils.h"
#include "mbssid.h"
#include "mesh.h"
#include "beacon.h"
#include "trace.h"

#define FRAGMENTATION_OVERHEAD			(36)

/**
 * Max beacon length limit for 1MHz, MCS0. If the beacon is larger than this it may get
 * fragmented by the FW, which is not permitted by the 802.11 protocol.
 */
#define DOT11AH_1MHZ_MCS0_MAX_BEACON_LENGTH			(764 - FRAGMENTATION_OVERHEAD)

/* Set RSN IE in beacon mode */
static enum morse_mac_rsn_beacon_mode
rsn_beacon_mode __read_mostly = RSN_BEACON_DISABLED;
module_param(rsn_beacon_mode, uint, 0644);
MODULE_PARM_DESC(rsn_beacon_mode,
	"Mode for insertion of RSN IEs in beacons (none, long beacons only, or all beacons)");

static int enable_short_bcn_as_dtim_override = -1;
module_param(enable_short_bcn_as_dtim_override, int, 0644);
MODULE_PARM_DESC(enable_short_bcn_as_dtim_override,
		 "Override enable for short beacon to be the DTIM beacon (experimental)");

static unsigned long beacon_irqs_enabled;
static bool enable_short_bcn_as_dtim;

enum morse_mac_rsn_beacon_mode morse_beacon_get_rsn_mode(void)
{
	return rsn_beacon_mode;
}

int morse_beacon_get_short_bcn_dtim_override(void)
{
	return enable_short_bcn_as_dtim_override;
}

bool morse_mac_is_s1g_long_beacon(struct morse *mors, struct sk_buff *skb)
{
	bool ret = false;
	struct ieee80211_ext *s1g_beacon = (struct ieee80211_ext *)skb->data;
	u32 frame_len = skb->len;
	const u8 *s1g_ies = s1g_beacon->u.s1g_beacon.variable;
	u32 s1g_header_length = s1g_ies - skb->data;
	u32 s1g_ies_len = frame_len - s1g_header_length;
	u16 fc = le16_to_cpu(s1g_beacon->frame_control);

	if (fc & IEEE80211_FC_NEXT_TBTT) {
		s1g_ies += IEEE80211_NEXT_TBTT_SIZE;
		s1g_ies_len -= IEEE80211_NEXT_TBTT_SIZE;
	}

	if (fc & IEEE80211_FC_COMPRESS_SSID) {
		s1g_ies += IEEE80211_COMPRESS_SSID_SIZE;
		s1g_ies_len -= IEEE80211_COMPRESS_SSID_SIZE;
	}

	if (fc & IEEE80211_FC_ANO) {
		s1g_ies += IEEE80211_ANO_SIZE;
		s1g_ies_len -= IEEE80211_ANO_SIZE;
	}

	if (!s1g_ies_len)
		return ret;

	if (cfg80211_find_ie(WLAN_EID_S1G_BCN_COMPAT, s1g_ies, s1g_ies_len))
		ret = true;

	return ret;
}

static bool skip_beacon_tx_if_offload_active(struct morse_vif *mors_vif)
{
	bool skip;

	spin_lock(&mors_vif->beacon_offload.lock);
	if (mors_vif->beacon_offload.state > MORSE_BEACON_OFFLOAD_STATE_DISABLED)
		mors_vif->beacon_offload.skipped_tasklet_events++;
	skip = mors_vif->beacon_offload.skipped_tasklet_events;
	spin_unlock(&mors_vif->beacon_offload.lock);
	return skip;
}

void morse_insert_beacon_timing_element(struct morse_vif *mors_vif,
					struct dot11ah_ies_mask *ies_mask)
{
	struct beacon_timing_element *bcn_timing_ie;
	u16 beacon_timing_element_size;
	u8 no_of_mesh_neighbors;
	struct morse *mors = morse_vif_to_morse(mors_vif);
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_vif);
	struct morse_mesh_config *mesh_conf = mors_vif->mesh->conf;

	/* Find number of mesh neighbors available */
	no_of_mesh_neighbors =
	    min(morse_dot11ah_find_no_of_mesh_neighbors(vif->bss_conf.beacon_int),
		MORSE_MESH_MAX_BEACON_INFO_ENTRIES);

	if (no_of_mesh_neighbors > 0) {
		struct ie_element *element;

		beacon_timing_element_size = sizeof(struct beacon_timing_element) +
		    (no_of_mesh_neighbors * sizeof(struct mesh_neighbor_beacon_info));

		if (mesh_conf) {
			MORSE_BEACON_DBG(mors, "%s: neighbors: %d, ie len: %d, bcn count: %d\n",
				__func__, no_of_mesh_neighbors, beacon_timing_element_size,
				mesh_conf->mbca.beacon_count);
		}

		element = morse_dot11_ies_create_ie_element(ies_mask, WLAN_EID_BEACON_TIMING,
							    beacon_timing_element_size, true, true);
		if (element) {
			bcn_timing_ie = (struct beacon_timing_element *)element->ptr;
			bcn_timing_ie->report_control_field = 0;
		}
	}
}

static void morse_beacon_fill_tx_info(struct morse *mors, struct morse_skb_tx_info *tx_info,
				      struct sk_buff *skb, struct morse_vif *mors_vif,
				      int tx_bw_mhz)
{
	enum dot11_bandwidth bw_idx = morse_ratecode_bw_mhz_to_bw_index(tx_bw_mhz);
	enum morse_rate_preamble pream = MORSE_RATE_PREAMBLE_S1G_SHORT;
	const u8 nss_index = 0;	/* TODO */
	const u8 mcs_index = 0;
	(void)mors;
	(void)skb;

	morse_skbq_set_mac80211_owned(skb, true);
	tx_info->flags |= cpu_to_le32(MORSE_TX_CONF_FLAGS_VIF_ID_SET(mors_vif->id));

	if (bw_idx == DOT11_BANDWIDTH_1MHZ)
		pream = MORSE_RATE_PREAMBLE_S1G_1M;

	tx_info->rates[0].morse_ratecode = morse_ratecode_init(bw_idx, nss_index, mcs_index, pream);
	tx_info->rates[0].count = 1;
	mors->debug.mcs_stats_tbl.mcs0.tx_beacons++;
	mors->debug.mcs_stats_tbl.mcs0.tx_success++;
	tx_info->rates[1].count = 0;
	/* Enable immediate report flag if fw reports tx completion status */
	if (mors->firmware_flags & MORSE_FW_FLAGS_REPORTS_TX_BEACON_COMPLETION)
		tx_info->flags |= cpu_to_le32(MORSE_TX_CONF_FLAGS_IMMEDIATE_REPORT);
}

int morse_beacon_convert_ies_and_populate_skb(struct morse_vif *mors_vif,
					      struct sk_buff **bcn_skb,
					      struct dot11ah_ies_mask *ies_mask,
					      int *ie_mask_len,
					      const bool short_beacon)
{
	struct sk_buff *beacon;
	struct ieee80211_vif *vif;
	struct morse *mors;
	struct ieee80211_mgmt *beacon_mgmt;
	struct ieee80211_ext *s1g_beacon;
	u8 *s1g_beacon_ies;
	u8 *s1g_ordered_ies_buff;
	int s1g_ies_length;
	int s1g_hdr_length;

	beacon = *bcn_skb;
	s1g_ies_length = *ie_mask_len;
	beacon_mgmt = (struct ieee80211_mgmt *)beacon->data;

	mors = morse_vif_to_morse(mors_vif);
	vif = morse_vif_to_ieee80211_vif(mors_vif);

	s1g_beacon_ies = morse_mac_get_ie_pos(beacon, &s1g_ies_length, &s1g_hdr_length, false);

	morse_dot11ah_11n_to_s1g_tx_packet(vif, beacon, s1g_ies_length, short_beacon, ies_mask);

	/* To evaluate the required skb size, we need to get the new IE start position
	 * of the new S1G beacon. At this point s1g_ies_length is not right because
	 * the IEs are in ies_mask at this point
	 */
	s1g_beacon_ies = morse_mac_get_ie_pos(beacon, &s1g_ies_length, &s1g_hdr_length, true);
	if (!s1g_beacon_ies) {
		*bcn_skb = NULL;
		MORSE_BEACON_WARN_RATELIMITED(mors, "%s: failed to locate beacon IEs\n", __func__);
		return -EAGAIN;
	}

	s1g_beacon = (struct ieee80211_ext *)beacon->data;

	/* Lower 32 bits Get inserted into the timestamp field here */
	s1g_beacon->u.s1g_beacon.timestamp =
	    cpu_to_le32(LOWER_32_BITS(morse_mac_generate_timestamp_for_frame(mors_vif)));

	morse_mac_update_custom_s1g_capab(mors_vif, ies_mask, vif->type);

	/* Need to calculate the IEs length from the ies_mask */
	s1g_ies_length = morse_dot11_insert_ordered_ies_from_ies_mask(beacon,
								      NULL,
								      ies_mask,
								      beacon_mgmt->frame_control);

	/* allocate new buffer s1g_pkt and reorder all ies_mask and copy */
	s1g_ordered_ies_buff = kmalloc(s1g_ies_length, GFP_ATOMIC);
	beacon_mgmt = (struct ieee80211_mgmt *)beacon->data;

	morse_dot11_insert_ordered_ies_from_ies_mask(beacon,
						     s1g_ordered_ies_buff,
						     ies_mask, beacon_mgmt->frame_control);

	if ((beacon->len + skb_tailroom(beacon)) < (s1g_hdr_length + s1g_ies_length)) {
		struct sk_buff *skb2;

		skb2 = skb_copy_expand(beacon, skb_headroom(beacon),
				       (s1g_hdr_length + s1g_ies_length) - beacon->len, GFP_ATOMIC);

		if (!skb2) {
			kfree(s1g_ordered_ies_buff);
			*bcn_skb = NULL;
			return -ENOMEM;
		}

		/* Just say we transmitted it */
		MORSE_IEEE80211_TX_STATUS(mors->hw, beacon);
		beacon = skb2;
	}

	skb_trim(beacon, s1g_hdr_length);
	s1g_beacon_ies = skb_put(beacon, s1g_ies_length);
	memcpy(s1g_beacon_ies, s1g_ordered_ies_buff, s1g_ies_length);
	kfree(s1g_ordered_ies_buff);
	*ie_mask_len = s1g_ies_length;

	if (beacon->len >= DOT11AH_1MHZ_MCS0_MAX_BEACON_LENGTH &&
	    mors_vif->custom_configs->channel_info.pri_bw_mhz == 1) {
		MORSE_BEACON_ERR_RATELIMITED(mors, "%s: S1G beacon too big for 1MHz TX: %u\n",
									__func__, beacon->len);
		*bcn_skb = NULL;
		return -EAGAIN;
	}

	*bcn_skb = beacon;
	return 0;
}

void morse_beacon_insert_ies(struct morse_vif *mors_vif,
			     struct dot11ah_ies_mask *ies_mask,
			     __le16 frame_control, bool short_beacon)
{
	struct morse *mors;
	struct ieee80211_vif *vif;
	struct morse_mesh *mesh;
	u8 rps_ie_size;
	u8 page_slice_no = S1G_TIM_PAGE_SLICE_ENTIRE_PAGE;
	u8 page_index = 0;

	mors = morse_vif_to_morse(mors_vif);
	vif = morse_vif_to_ieee80211_vif(mors_vif);

	/* Insert RPS IE if RAW is enabled. We will place it at the end and it
	 * will be reordered by the 11n to s1g layer.
	 */
	rps_ie_size = morse_raw_get_rps_ie_size(mors_vif);
	if (rps_ie_size != 0)
		morse_dot11ah_insert_element(ies_mask,
					     WLAN_EID_S1G_RPS,
					     morse_raw_get_rps_ie(mors_vif), rps_ie_size);

	morse_cac_insert_ie(ies_mask, vif, frame_control);

	if (ies_mask->ies[WLAN_EID_TIM].ptr) {
		/* If page slicing is enabled then it will schedule the TIM into different
		 * TIM slices and updates TIM element to point to the (11n)TIM slice to serve
		 * after out going beacon.
		 */
		if (mors_vif->page_slicing_info.enabled)
			morse_page_slicing_process_tim_element(vif,
							       ies_mask,
							       &page_slice_no,
							       &page_index);

		/* Convert 11n TIM (TIM slice if page slicing is enabled) to S1G TIM */
		morse_dot11ah_insert_s1g_tim(vif, ies_mask, page_slice_no, page_index);
	}

	/* Mesh points need RSN IE in beacons for MPM (Mesh Peering Management) */
	if (!ieee80211_vif_is_mesh(vif) &&
	    (rsn_beacon_mode == RSN_BEACON_DISABLED ||
	     (rsn_beacon_mode == RSN_BEACON_LONG && short_beacon))) {
		morse_dot11_clear_eid_from_ies_mask(ies_mask, WLAN_EID_RSN);
		morse_dot11_clear_eid_from_ies_mask(ies_mask, WLAN_EID_RSNX);
	}

	morse_mbssid_insert_ie(mors_vif, mors, ies_mask);
	spin_lock_bh(&mors_vif->vendor_ie.lock);
	morse_vendor_ie_add_ies(mors_vif, ies_mask, MORSE_VENDOR_IE_TYPE_BEACON);

	mesh = mors_vif->mesh;
	if (ieee80211_vif_is_mesh(vif) && mesh->conf && mesh->conf->mbca.config != 0) {
		struct morse_mesh_config *mesh_conf = mesh->conf;
		bool add_beacon_timing_elem = !(mesh_conf->mbca.beacon_count %
						mesh_conf->mbca.beacon_timing_report_interval);

		if (ies_mask->ies[WLAN_EID_MESH_CONFIG].ptr)
			morse_enable_mbca_capability(ies_mask->ies[WLAN_EID_MESH_CONFIG].ptr);

		if (add_beacon_timing_elem) {
			mesh_conf->mbca.beacon_count = 0;
			morse_insert_beacon_timing_element(mors_vif, ies_mask);
		}
		mesh_conf->mbca.beacon_count++;
	}

	spin_unlock_bh(&mors_vif->vendor_ie.lock);
}

/**
 * morse_beacon_generate - generate a morse beacon
 * @mors_vif: morse VIF structure
 * @bcn_skb: SKB to populate with the beacon
 * @long_beacon_dtim_count: Long beacon DTIM offset count
 *
 * Generate a beacon,callers of this function are responsible for freeing the beacon if successfully
 * generated.
 *
 * All calls inside this function must be atomic as it will be called by the beacon tasklet.
 *
 * @returns TRUE if beacon generated successfully, -ENOENT if no beacon available, else error code
 */
static int morse_beacon_generate(struct morse_vif *mors_vif, struct sk_buff **bcn_skb,
			  int long_beacon_dtim_count)
{
	struct sk_buff *beacon = NULL;
	struct ieee80211_vif *vif;
	struct morse *mors;
	struct dot11ah_ies_mask *ies_mask = NULL;
	struct ieee80211_mgmt *beacon_mgmt;
	const u8 *tim_ie;
	bool short_beacon;
	u8 *s1g_beacon_ies;
	int s1g_ies_length;
	int s1g_hdr_length;
	int ret = 0;

	mors = morse_vif_to_morse(mors_vif);
	vif = morse_vif_to_ieee80211_vif(mors_vif);

	if (!morse_mac_is_iface_ap_type(vif))
		return -EINVAL;

	ies_mask = morse_dot11ah_ies_mask_alloc();
	if (!ies_mask)
		return -ENOMEM;

	short_beacon = (mors_vif->dtim_count != long_beacon_dtim_count);

	beacon = MORSE_IEEE_BEACON_GET(mors, vif);

	if (!beacon) {
		/* Per ieee80211_beacon_get() contract, NULL is valid: interface
		 * not running, no beacon assigned, or mid-teardown.
		 * Returned via -ENOENT so real parse failures below
		 * are still surfaced as errors.
		 */
		morse_dot11ah_ies_mask_free(ies_mask);
		return -ENOENT;
	}

	*bcn_skb = beacon;

	beacon_mgmt = (struct ieee80211_mgmt *)beacon->data;

	tim_ie = cfg80211_find_ie(WLAN_EID_TIM, beacon_mgmt->u.beacon.variable,
			beacon->len - (ieee80211_hdrlen(beacon_mgmt->frame_control)
				+ 12));

	if (tim_ie) {
		short_beacon = (tim_ie[2] != long_beacon_dtim_count);
		if (tim_ie[2] == 0)
			mors_vif->dtim_count = 0;
	}

	if (mors_vif->ecsa_chan_configured) {
		short_beacon = false;
		MORSE_BEACON_DBG(mors, "%s: tx long beacon, dtim count: %d\n",
					__func__,
					((mors_vif->dtim_count + 1) % vif->bss_conf.dtim_period));
	}

	/* IBSS does not support short beacons */
	if (vif->type == NL80211_IFTYPE_ADHOC)
		short_beacon = false;

	s1g_beacon_ies = morse_mac_get_ie_pos(beacon, &s1g_ies_length, &s1g_hdr_length, false);

	/* Parse out the original IEs so we can mess with them */
	if (morse_dot11ah_parse_ies(s1g_beacon_ies, s1g_ies_length, ies_mask) < 0) {
		MORSE_BEACON_WARN_RATELIMITED(mors, "%s: failed to parse beacon IEs\n", __func__);
		ret = -EAGAIN;
		goto exit;
	}

	morse_beacon_insert_ies(mors_vif, ies_mask, beacon_mgmt->frame_control, short_beacon);

	*bcn_skb = beacon;
	ret = morse_beacon_convert_ies_and_populate_skb(mors_vif, bcn_skb, ies_mask,
							&s1g_ies_length, short_beacon);
	if (ret)
		goto exit;

	if (vif->bss_conf.dtim_period)
		mors_vif->dtim_count =
			(mors_vif->dtim_count + 1) % vif->bss_conf.dtim_period;
	else
		mors_vif->dtim_count = 0;

exit:
	if (ret) {
		kfree_skb(beacon);
		*bcn_skb = NULL;
	}

	morse_dot11ah_ies_mask_free(ies_mask);
	return ret;
}

static void morse_beacon_tasklet(unsigned long data)
{
	struct morse_skbq *mq;
	struct morse_vif *mors_vif = (struct morse_vif *)data;
	struct ieee80211_vif *vif;
	struct morse *mors;
	struct morse_skb_tx_info tx_info = { 0 };
	int tx_bw_mhz;
	const struct chip_if_ops *chip_if_ops;
	bool fw_reports_tx_beacon_comp;
	int num_bcn_vifs;
	uint long_beacon_dtim_count;
	struct sk_buff *beacon = NULL;
	int ret;

	if (!mors_vif || !mors_vif->custom_configs)
		return;

	trace_beacon_tasklet_enter(mors_vif->id);
	mors = morse_vif_to_morse(mors_vif);

	if (!mors->cfg)
		return;

	vif = morse_vif_to_ieee80211_vif(mors_vif);

	if (!morse_mac_is_iface_ap_type(vif))
		return;

	if (skip_beacon_tx_if_offload_active(mors_vif)) {
		MORSE_BEACON_DBG(mors, "%s: tasklet skipped due to beacon offload\n",
					__func__);
		return;
	}

	/* Set the long beacon index to 0 if long beacon is the DTIM beacon
	 * otherwise shift the long beacon to be the beacon immediately after the DTIM beacon
	 */
	long_beacon_dtim_count = enable_short_bcn_as_dtim ? (vif->bss_conf.dtim_period - 1) : 0;
	fw_reports_tx_beacon_comp = mors->firmware_flags &
		MORSE_FW_FLAGS_REPORTS_TX_BEACON_COMPLETION;
	num_bcn_vifs = atomic_read(&mors->num_bcn_vifs);

	chip_if_ops = mors->cfg->ops;

	mq = chip_if_ops->skbq_bcn_tc_q(mors);
	if (!mq) {
		MORSE_BEACON_ERR_RATELIMITED(mors, "%s: no matching Q found\n", __func__);
		return;
	}

	if (morse_skbq_count(mq) >= num_bcn_vifs) {
		MORSE_BEACON_ERR_RATELIMITED(mors,
			"%s: previous beacon not consumed, dropping req [id:%d]\n",
			__func__, mors_vif->id);
		return;
	}

	/* The following can occur if the TX status reporting the beacon completion
	 * gets lost. A stale timer in skbq will eventually flush the pending frame.
	 */
	if (fw_reports_tx_beacon_comp && morse_skbq_pending_count(mq) && num_bcn_vifs == 1)
		MORSE_BEACON_DBG(mors, "%s: number of beacons awaiting tx status: %u\n",
						__func__, morse_skbq_pending_count(mq));

	ret = morse_beacon_generate(mors_vif, &beacon, long_beacon_dtim_count);
	if (!beacon || ret) {
		if (ret == -ENOENT) {
			/* mac80211 had no beacon, skip */
			MORSE_BEACON_DBG_RATELIMITED(mors,
					 "%s: no beacon from mac80211, skipping\n",
					 __func__);
			return;
		}
		MORSE_BEACON_ERR_RATELIMITED(mors, "%s: failed to generate the beacon, ret: %d\n",
					     __func__, ret);
		return;
	}

	/* Use full operating BW if subbands are disabled */
	tx_bw_mhz = (mors->enable_subbands == SUBBANDS_MODE_DISABLED) ?
	    mors->custom_configs.channel_info.op_bw_mhz :
	    mors->custom_configs.channel_info.pri_bw_mhz;
	morse_beacon_fill_tx_info(mors, &tx_info, beacon, mors_vif, tx_bw_mhz);
	morse_skbq_skb_tx(mq, &beacon, &tx_info, MORSE_SKB_CHAN_BEACON);

	/* Wake up pageset handler */
	trace_beacon_tasklet_exit(mors->chip_if->event_flags);

	/* TODO: currently due to the way we implement firmware beaconing,
	 * these might still get sent before the DTIM beacon.
	 */
	if (!test_bit(MORSE_STATE_FLAG_DATA_QS_STOPPED, &mors->state_flags))
		morse_mac_send_buffered_bc(vif);

	morse_raw_beacon_sent(mors_vif);
}

void morse_beacon_irq_handle(struct morse *mors, u32 status)
{
	struct morse_vif *mors_vif;
	struct ieee80211_vif *vif;
	int count, masked_status;

	count = 0;
	masked_status = (status & beacon_irqs_enabled) >> MORSE_INT_BEACON_BASE_NUM;
	spin_lock_bh(&mors->vif_list_lock);
	while (masked_status && (count < mors->max_vifs)) {
		if (masked_status & 1) {
			vif = __morse_get_vif_from_vif_id(mors, count);
			mors_vif = ieee80211_vif_to_morse_vif(vif);

			tasklet_schedule(&mors_vif->beacon_tasklet);
		}
		masked_status >>= 1;
		count++;
	}
	spin_unlock_bh(&mors->vif_list_lock);
}

static int morse_beacon_irq_enable(struct morse_vif *mors_vif, bool enable)
{
	struct morse *mors = morse_vif_to_morse(mors_vif);
	u8 beacon_irq_num = MORSE_INT_BEACON_BASE_NUM + mors_vif->id;

	if (mors_vif->id > mors->max_vifs) {
		MORSE_BEACON_ERR(mors, "%s: invalid interface id:%d\n", __func__, mors_vif->id);
		return -1;
	}

	if (enable)
		set_bit(beacon_irq_num, &beacon_irqs_enabled);
	else
		clear_bit(beacon_irq_num, &beacon_irqs_enabled);

	MORSE_BEACON_DBG(mors, "%s: irq:%lx id:%d\n",
					__func__, beacon_irqs_enabled, mors_vif->id);

	return morse_hw_irq_enable(mors, beacon_irq_num, enable);
}

static void morse_beacon_offload_init(struct morse_vif *mors_vif)
{
	spin_lock_init(&mors_vif->beacon_offload.lock);
	spin_lock_bh(&mors_vif->beacon_offload.lock);
	mors_vif->beacon_offload.state = MORSE_BEACON_OFFLOAD_STATE_DISABLED;
	mors_vif->beacon_offload.skipped_tasklet_events = 0;
	spin_unlock_bh(&mors_vif->beacon_offload.lock);
}

int morse_beacon_init(struct morse_vif *mors_vif)
{
	struct morse *mors = morse_vif_to_morse(mors_vif);
	int ret;

	if (enable_short_bcn_as_dtim_override >= 0) {
		MORSE_WARN(mors, "%s: overriding default enable_short_bcn_as_dtim: %s to %s\n",
			   __func__,
			   mors->cfg->enable_short_bcn_as_dtim ? "true" : "false",
			   enable_short_bcn_as_dtim_override ? "true" : "false");
		enable_short_bcn_as_dtim = enable_short_bcn_as_dtim_override;
	} else {
		enable_short_bcn_as_dtim = mors->cfg->enable_short_bcn_as_dtim;
	}

	morse_beacon_offload_init(mors_vif);
	tasklet_init(&mors_vif->beacon_tasklet, morse_beacon_tasklet, (unsigned long)mors_vif);

	ret = morse_beacon_irq_enable(mors_vif, true);
	if (ret == MORSE_RET_SUCCESS)
		atomic_inc(&mors->num_bcn_vifs);
	return ret;
}

void morse_beacon_finish(struct morse_vif *mors_vif)
{
	struct morse *mors = morse_vif_to_morse(mors_vif);

	morse_beacon_irq_enable(mors_vif, false);
	tasklet_kill(&mors_vif->beacon_tasklet);
	atomic_dec(&mors->num_bcn_vifs);
}
