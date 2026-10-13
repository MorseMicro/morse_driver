/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include <net/xfrm.h>
#include <net/mac80211.h>
#include <net/ieee80211_radiotap.h>
#include <linux/unaligned.h>
#include <linux/if_arp.h>
#include <linux/rtnetlink.h>

#include "morse.h"
#include "debug.h"
#include "mac.h"
#include "monitor.h"
#include "s1g_radiotap.h"
#include "skb_header.h"
#include "skbq.h"
#include "vendor.h"
#include "dot11ah/dot11ah.h"

#define RT_ZERO_LEN_PSDU_DATA			0x2
#define RT_ZERO_LEN_PSDU_VENDOR_SPECIFIC	0xff

#ifndef IEEE80211_RADIOTAP_AMPDU_EOF
#define IEEE80211_RADIOTAP_AMPDU_EOF 0x0040
#endif

/* This should later be supported by Linux */
enum morse_ieee80211_radiotap_type {
	IEEE80211_RADIOTAP_HALOW = 22,
};

struct morse_radiotap_hdr {
	struct ieee80211_radiotap_header hdr;
	__le64 rt_tsft;
	u8 rt_flags;
	/* Rate for PSDU with length, type for zero len PSDU */
	u8 rt_rate_or_zl_psdu;
	__le16 rt_channel;
	__le16 rt_chbitmask;
	s8 rt_dbm_antsignal;
} __packed;

/** Radiotap header with optional timestamp field used for signal field errors. */
struct morse_collision_radiotap_hdr {
	struct ieee80211_radiotap_header hdr;
	__le64 rt_tsft;
	/* Timestamp field */
	struct {
		__le64 timestamp;
		__le16 accuracy;
		u8 unit_position;
		u8 flags;
	} timestamp;
} __packed;

struct zero_length_psdu {
	u8 psdu_type;
	u8 ndp_type;
	__le64 ndp[];
} __packed;

struct ampdu_header {
	__le32 ref_num;
	__le16 flags;
	u8 eof_value;
	u8 reserved;
} __packed;

struct padding {
	u8 padding;
} __packed;

static struct net_device *morse_mon;	/* global monitor netdev */

/* Preserve the receive-only behavior until an operator explicitly enables TX. */
static bool enable_monitor_injection;
module_param(enable_monitor_injection, bool, 0644);
MODULE_PARM_DESC(enable_monitor_injection,
		 "Allow raw 802.11ah management/data TX through the monitor netdev");

/* The monitor netdev is separate from the mac80211 virtual interface. */
struct morse_mon_priv {
	struct morse *mors;
};

struct morse_mon_tx_params {
	u16 rt_len;
	u16 channel_mhz;
	u32 freq_khz;
	u8 mcs;
	u8 bw;
	u8 attempts;
	u8 ppdu_format;
	bool has_channel;
	bool has_freq;
	bool has_mcs;
	bool has_bw;
	bool has_ppdu_format;
	bool has_s1g;
	bool short_gi;
	bool strip_fcs;
};

#define MORSE_MON_TX_FIELDS (BIT(IEEE80211_RADIOTAP_TSFT) | \
	BIT(IEEE80211_RADIOTAP_FLAGS) | BIT(IEEE80211_RADIOTAP_RATE) | \
	BIT(IEEE80211_RADIOTAP_CHANNEL) | \
	BIT(IEEE80211_RADIOTAP_DBM_ANTSIGNAL) | \
	BIT(IEEE80211_RADIOTAP_TX_FLAGS) | \
	BIT(IEEE80211_RADIOTAP_DATA_RETRIES) | \
	BIT(IEEE80211_RADIOTAP_AMPDU_STATUS) | \
	BIT(IEEE80211_RADIOTAP_TLV))

static int morse_mon_parse_tlvs(const u8 *data, u16 rt_len, size_t offset,
				struct morse_mon_tx_params *tx)
{
	while (offset < rt_len) {
		u16 type, length;
		size_t padded;

		if (rt_len - offset < 4)
			return -EINVAL;
		type = get_unaligned_le16(data + offset);
		length = get_unaligned_le16(data + offset + 2);
		if (length > rt_len - offset - 4)
			return -EINVAL;
		padded = ALIGN((size_t)length + 4, 4);
		if (padded > rt_len - offset)
			return -EINVAL;

		if (type == DOT11_RT_TLV_S1G_TYPE) {
			u16 known, data1;
			u8 nss;

			if (length != DOT11_RT_TLV_S1G_LENGTH || tx->has_s1g)
				return -EINVAL;
			known = get_unaligned_le16(data + offset + 4);
			data1 = get_unaligned_le16(data + offset + 6);
			if (known & ~(DOT11_RT_S1G_KNOWN_PPDU_FMT |
				      DOT11_RT_S1G_KNOWN_GI |
				      DOT11_RT_S1G_KNOWN_NSS |
				      DOT11_RT_S1G_KNOWN_BW |
				      DOT11_RT_S1G_KNOWN_MCS))
				return -EOPNOTSUPP;
			tx->has_s1g = true;
			if (known & DOT11_RT_S1G_KNOWN_NSS) {
				nss = (data1 >> 6) & 0x3;
				if (nss != 0) /* MM6108 has one spatial stream. */
					return -EOPNOTSUPP;
			}
			if (known & DOT11_RT_S1G_KNOWN_BW) {
				tx->bw = (data1 >> 8) & 0xf;
				tx->has_bw = true;
			}
			if (known & DOT11_RT_S1G_KNOWN_MCS) {
				u8 mcs = (data1 >> 12) & 0xf;

				if (tx->has_mcs && tx->mcs != mcs)
					return -EINVAL;
				tx->mcs = mcs;
				tx->has_mcs = true;
			}
			if (known & DOT11_RT_S1G_KNOWN_GI)
				tx->short_gi = !!(data1 & BIT(5));
			if (known & DOT11_RT_S1G_KNOWN_PPDU_FMT) {
				tx->ppdu_format = data1 & 0x3;
				tx->has_ppdu_format = true;
			}
		} else if (type == IEEE80211_RADIOTAP_VENDOR_NAMESPACE &&
			   length == MORSE_VENDOR_TLV_FREQ_KHZ_SIZE && !tx->has_freq) {
			const u8 *vendor = data + offset + 4;

			if (vendor[0] == (MORSE_OUI >> 16) &&
			    vendor[1] == ((MORSE_OUI >> 8) & 0xff) &&
			    vendor[2] == (MORSE_OUI & 0xff) &&
			    vendor[3] == MORSE_VENDOR_TLV_SUBNS_0 &&
			    get_unaligned_le16(vendor + 4) == MORSE_VENDOR_TLV_FREQ_KHZ_TYPE) {
				tx->freq_khz = get_unaligned_le32(vendor + 8);
				tx->has_freq = true;
			} else {
				return -EOPNOTSUPP;
			}
		} else {
			return -EOPNOTSUPP;
		}
		offset += padded;
	}
	return 0;
}

static int morse_mon_parse_radiotap(const u8 *data, size_t length,
				    struct morse_mon_tx_params *tx)
{
	const struct ieee80211_radiotap_header *rt = (const void *)data;
	u32 fields;
	size_t offset = sizeof(*rt);
	int bit;

	if (length < sizeof(*rt) || rt->it_version)
		return -EINVAL;
	tx->rt_len = get_unaligned_le16(&rt->it_len);
	if (tx->rt_len < sizeof(*rt) || tx->rt_len > length)
		return -EINVAL;
	fields = get_unaligned_le32(&rt->it_present);
	/* Reject extended or unsupported fields instead of silently ignoring TX controls. */
	if (fields & ~MORSE_MON_TX_FIELDS)
		return -EOPNOTSUPP;
	tx->attempts = 1;

	for (bit = 0; bit <= IEEE80211_RADIOTAP_TLV; bit++) {
		size_t align = 1, size = 0;

		if (!(fields & BIT(bit)))
			continue;
		switch (bit) {
		case IEEE80211_RADIOTAP_TSFT:
			align = 8;
			size = 8;
			break;
		case IEEE80211_RADIOTAP_FLAGS:
		case IEEE80211_RADIOTAP_RATE:
		case IEEE80211_RADIOTAP_DBM_ANTSIGNAL:
		case IEEE80211_RADIOTAP_DATA_RETRIES:
			size = 1;
			break;
		case IEEE80211_RADIOTAP_CHANNEL:
			align = 2;
			size = 4;
			break;
		case IEEE80211_RADIOTAP_TX_FLAGS:
			align = 2;
			size = 2;
			break;
		case IEEE80211_RADIOTAP_AMPDU_STATUS:
			align = 4;
			size = 8;
			break;
		case IEEE80211_RADIOTAP_TLV:
			offset = ALIGN(offset, 4);
			if (offset > tx->rt_len)
				return -EINVAL;
			return morse_mon_parse_tlvs(data, tx->rt_len, offset, tx);
		default:
			return -EOPNOTSUPP;
		}
		offset = ALIGN(offset, align);
		if (offset > tx->rt_len || size > tx->rt_len - offset)
			return -EINVAL;
		switch (bit) {
		case IEEE80211_RADIOTAP_FLAGS:
			/* Encryption and fragmentation requests need a normal VIF. */
			if (data[offset] & ~IEEE80211_RADIOTAP_F_FCS)
				return -EOPNOTSUPP;
			tx->strip_fcs = !!(data[offset] & IEEE80211_RADIOTAP_F_FCS);
			break;
		case IEEE80211_RADIOTAP_RATE:
			/* Morse's capture format uses the high bit plus the S1G MCS. */
			if (!(data[offset] & BIT(7)))
				return -EOPNOTSUPP;
			tx->mcs = data[offset] & 0x7f;
			tx->has_mcs = true;
			break;
		case IEEE80211_RADIOTAP_CHANNEL:
			tx->channel_mhz = get_unaligned_le16(data + offset);
			tx->has_channel = true;
			break;
		case IEEE80211_RADIOTAP_TX_FLAGS:
			/* NOACK/RTS/CTS are not exposed by this firmware TX API. */
			if (get_unaligned_le16(data + offset))
				return -EOPNOTSUPP;
			break;
		case IEEE80211_RADIOTAP_DATA_RETRIES:
			if (data[offset] > 3)
				return -ERANGE;
			tx->attempts = data[offset] + 1;
			break;
		default:
			break;
		}
		offset += size;
	}
	return offset == tx->rt_len ? 0 : -EINVAL;
}

static bool morse_mon_regdom_allows_channel(const char *region, u32 center_khz, u8 bw_mhz)
{
	const struct morse_reg_rule *rule = morse_regdom_get_rule_for_freq(region, center_khz);
	const struct ieee80211_freq_range *range;
	u32 half_bw_khz = bw_mhz * 500;

	if (!rule || center_khz < half_bw_khz)
		return false;
	range = &rule->dot11_reg.freq_range;
	return !(rule->dot11_reg.flags & NL80211_RRF_NO_IR) &&
		bw_mhz * 1000 <= range->max_bandwidth_khz &&
		center_khz - half_bw_khz >= range->start_freq_khz &&
		center_khz + half_bw_khz <= range->end_freq_khz;
}

static netdev_tx_t morse_mon_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct morse *mors = ((struct morse_mon_priv *)netdev_priv(dev))->mors;
	struct morse_mon_tx_params tx = { 0 };
	struct morse_skb_tx_info tx_info = { 0 };
	struct morse_skbq *mq;
	struct sk_buff *fw_skb;
	struct morse_channel_info *channel;
	const u8 *frame;
	__le16 fc;
	size_t frame_len;
	u32 op_freq_khz, tx_freq_khz;
	u8 bw_mhz;
	int ret;

	/* This path has only been validated with MM6108 SoftMAC firmware. */
	if (!READ_ONCE(enable_monitor_injection) || is_fullmac_mode() ||
	    (mors->chip_id != MM6108A0_ID && mors->chip_id != MM6108A1_ID &&
	     mors->chip_id != MM6108A2_ID) ||
	    !READ_ONCE(mors->monitor_mode) ||
	    READ_ONCE(mors->mon_if.id) == INVALID_VIF_ID ||
	    !morse_mlme_is_started(mors) ||
	    test_bit(MORSE_STATE_FLAG_STOPPING, &mors->state_flags))
		goto drop;
	if (skb_is_nonlinear(skb) && skb_linearize(skb))
		goto drop;
	ret = morse_mon_parse_radiotap(skb->data, skb->len, &tx);
	if (ret)
		goto drop;
	frame_len = skb->len - tx.rt_len;
	if (tx.strip_fcs) {
		if (frame_len < 4)
			goto drop;
		frame_len -= 4; /* Firmware generates its own FCS. */
	}
	if (frame_len < 24 || frame_len > IEEE80211_MAX_FRAME_LEN)
		goto drop;
	frame = skb->data + tx.rt_len;
	fc = cpu_to_le16(get_unaligned_le16(frame));
	if (!ieee80211_is_mgmt(fc) && !ieee80211_is_data(fc))
		goto drop;
	if (frame_len < ieee80211_hdrlen(fc))
		goto drop;

	channel = &mors->custom_configs.channel_info;
	if (!channel->op_chan_freq_hz ||
	    (channel->op_bw_mhz != 1 && channel->op_bw_mhz != 2 &&
	     channel->op_bw_mhz != 4 && channel->op_bw_mhz != 8) ||
	    channel->pri_1mhz_chan_idx >= channel->op_bw_mhz)
		goto drop;

	if (tx.has_bw && tx.bw > DOT11_RT_S1G_BW_8MHZ)
		goto drop;
	bw_mhz = tx.has_bw ? (1U << tx.bw) : channel->pri_bw_mhz;
	if (!bw_mhz || bw_mhz > channel->op_bw_mhz || bw_mhz > 8 ||
	    (bw_mhz != 1 && bw_mhz != 2 && bw_mhz != 4 && bw_mhz != 8))
		goto drop;
	/* The chip sends narrow PPDUs in the block containing the primary 1 MHz channel. */
	op_freq_khz = channel->op_chan_freq_hz / 1000;
	if (op_freq_khz < channel->op_bw_mhz * 500)
		goto drop;
	tx_freq_khz = op_freq_khz - channel->op_bw_mhz * 500 +
		(channel->pri_1mhz_chan_idx / bw_mhz) * bw_mhz * 1000 +
		bw_mhz * 500;
	if (!morse_mon_regdom_allows_channel(morse_dot11ah_get_region_str(), op_freq_khz,
					      channel->op_bw_mhz) ||
	    !morse_mon_regdom_allows_channel(morse_dot11ah_get_region_str(), tx_freq_khz,
					      bw_mhz))
		goto drop;
	if (tx.has_channel && tx.channel_mhz != tx_freq_khz / 1000)
		goto drop;
	if (tx.has_freq && tx.freq_khz != tx_freq_khz)
		goto drop;
	if (tx.has_ppdu_format &&
	    ((tx.ppdu_format == DOT11_RT_S1G_PPDU_S1G_1M && bw_mhz != 1) ||
	     (tx.ppdu_format == DOT11_RT_S1G_PPDU_S1G_SHORT && bw_mhz == 1) ||
	     tx.ppdu_format > DOT11_RT_S1G_PPDU_S1G_SHORT))
		goto drop;
	if (tx.mcs > 10 || (tx.mcs == 10 && bw_mhz != 1))
		goto drop;

	tx_info.flags = cpu_to_le32(MORSE_TX_CONF_FLAGS_VIF_ID_SET(mors->mon_if.id));
	if (ieee80211_is_mgmt(fc)) {
		tx_info.tid = MORSE_QOS_TID_UP_HIGHEST;
		tx_info.flags |= cpu_to_le32(MORSE_TX_CONF_NO_PS_BUFFER);
	} else if (ieee80211_is_data_qos(fc)) {
		tx_info.tid = frame[ieee80211_hdrlen(fc) - 2] & IEEE80211_QOS_CTL_TID_MASK;
	}
	mq = ieee80211_is_mgmt(fc) ? mors->cfg->ops->skbq_mgmt_tc_q(mors) :
		mors->cfg->ops->skbq_tc_q_from_aci(mors, dot11_tid_to_ac(tx_info.tid));
	if (!mq)
		goto drop;
	fw_skb = morse_skbq_alloc_skb(mq, frame_len);
	if (!fw_skb)
		goto drop;
	memcpy(fw_skb->data, frame, frame_len);
	morse_skbq_set_mac80211_owned(fw_skb, false);
	tx_info.rates[0].morse_ratecode = morse_ratecode_init(
		morse_ratecode_bw_mhz_to_bw_index(bw_mhz), 0, tx.mcs,
		bw_mhz == 1 ? MORSE_RATE_PREAMBLE_S1G_1M : MORSE_RATE_PREAMBLE_S1G_SHORT);
	if (tx.short_gi)
		morse_ratecode_enable_sgi(&tx_info.rates[0].morse_ratecode);
	tx_info.rates[0].count = tx.attempts;
	ret = morse_skbq_skb_tx(mq, &fw_skb, &tx_info,
		ieee80211_is_mgmt(fc) ? MORSE_SKB_CHAN_MGMT : MORSE_SKB_CHAN_DATA);
	if (ret)
		goto drop;

	/* These counters describe firmware queue acceptance, not on-air delivery. */
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += frame_len;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;

drop:
	dev->stats.tx_dropped++;
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops morse_mon_ops = {
	.ndo_start_xmit = morse_mon_xmit,
#if KERNEL_VERSION(5, 10, 11) > LINUX_VERSION_CODE
	.ndo_change_mtu = eth_change_mtu,
#endif
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
};

static void morse_mon_setup(struct net_device *dev)
{
#if KERNEL_VERSION(5, 17, 0) <= LINUX_VERSION_CODE
	u8 monitor_mac[ETH_ALEN] = {0x12, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif

	dev->netdev_ops = &morse_mon_ops;
	ether_setup(dev);
	dev->priv_flags |= IFF_NO_QUEUE;
	dev->type = ARPHRD_IEEE80211_RADIOTAP;

#if KERNEL_VERSION(5, 17, 0) <= LINUX_VERSION_CODE
	dev_addr_set(dev, monitor_mac);
#else
	eth_zero_addr(dev->dev_addr);
	dev->dev_addr[0] = 0x12;
#endif
}

/* Convers an integer bandwidth (in MHz) to an S1G radiotap header bandwidth */
static enum dot11_rt_s1g_bandwidth int_bw_to_radiotap_bw_enum(int bw_mhz)
{
	int bw;

	switch (bw_mhz) {
	case 1:
		bw = DOT11_RT_S1G_BW_1MHZ;
		break;
	case 2:
		bw = DOT11_RT_S1G_BW_2MHZ;
		break;
	case 4:
		bw = DOT11_RT_S1G_BW_4MHZ;
		break;
	case 8:
		bw = DOT11_RT_S1G_BW_8MHZ;
		break;
	case 16:
		bw = DOT11_RT_S1G_BW_16MHZ;
		break;
	default:
		bw = DOT11_RT_S1G_BW_INVALID;
		break;
	}
	return bw;
}

/**
 * morse_mon_add_freq_khz_tlv() - add a frequency (kHz) TLV to a radiotap header
 * @skb: socket buffer to prepend the TLV data to
 * @hdr_rx_status: Morse rx status struct
 */
static void morse_mon_add_freq_khz_tlv(struct sk_buff *skb,
					  struct morse_skb_rx_status *hdr_rx_status)
{
	struct radiotap_morse_freq_khz *freq_tlv;

	freq_tlv = (struct radiotap_morse_freq_khz *)skb_push(skb, sizeof(*freq_tlv));
	freq_tlv->hdr.type = cpu_to_le16(IEEE80211_RADIOTAP_VENDOR_NAMESPACE);
	freq_tlv->hdr.length = cpu_to_le16(MORSE_VENDOR_TLV_FREQ_KHZ_SIZE);
	freq_tlv->hdr.oui[0] = (MORSE_OUI >> 16) & 0xFF;
	freq_tlv->hdr.oui[1] = (MORSE_OUI >> 8) & 0xFF;
	freq_tlv->hdr.oui[2] = MORSE_OUI & 0xFF;
	freq_tlv->hdr.subtype = MORSE_VENDOR_TLV_SUBNS_0;
	freq_tlv->hdr.vendor_type = cpu_to_le16(MORSE_VENDOR_TLV_FREQ_KHZ_TYPE);
	freq_tlv->hdr.reserved = 0;
	freq_tlv->freq_khz = cpu_to_le32(KHZ100_TO_KHZ(le16_to_cpu(hdr_rx_status->freq_100khz)));
}

/**
 * morse_mon_add_vendor_tlvs() - add all required vendor TLVs to a radiotap header
 * @skb: socket buffer to prepend TLV data to
 * @hdr_rx_status: Morse rx status struct
 *
 * Assumes the skb size requirements have already been accounted for
 */
static void morse_mon_add_vendor_tlvs(struct sk_buff *skb,
				      struct morse_skb_rx_status *hdr_rx_status)
{
	morse_mon_add_freq_khz_tlv(skb, hdr_rx_status);
}

/**
 * morse_mon_vendor_tlv_size() - Get the size of all vendor TLVs
 *
 * @returns the total length in bytes of the vendor TLVs to be added by morse_mon_add_vendor_tlvs().
 *         This is the total length including all padding.
 */
static int morse_mon_vendor_tlv_size(void)
{
	return sizeof(struct radiotap_morse_freq_khz);
}

void morse_mon_rx(struct morse *mors, struct sk_buff *rx_skb,
		  struct morse_skb_rx_status *hdr_rx_status)
{
	struct sk_buff *skb;
	u16 flags;
	struct morse_radiotap_hdr *hdr;
	struct zero_length_psdu *psdu;
	struct ampdu_header *ampdu_hdr = NULL;
	struct radiotap_s1g_tlv *s1g_info_hdr = NULL;
	struct padding *align_padding;
	enum dot11_rt_s1g_bandwidth bw;
	int ndp_sub_type;
	u32 bw_mhz;
	u8 mcs_index;
	int morse_vendor_tlv_size = 0;
	enum dot11_bandwidth bw_idx =
		morse_ratecode_bw_index_get(hdr_rx_status->morse_ratecode);
	u32 status_flags = le32_to_cpu(hdr_rx_status->flags);
	bool is_in_ampdu;

	if (status_flags & MORSE_RX_STATUS_FLAGS_NDP) {
		/* Null Data Packets contain no data, therefore no
		 * mcs encoding. The STF/LTF are usually BPSK, therefore
		 * the NDP mcs rate can always be considered as 0.
		 */
		morse_ratecode_mcs_index_set(&hdr_rx_status->morse_ratecode, 0);
		morse_ratecode_nss_index_set(&hdr_rx_status->morse_ratecode, NSS_TO_NSS_IDX(1));

		/**
		 * BSS COLOR is not present in NDP frames.
		 */
		hdr_rx_status->bss_color = 0;
	}

	if (!netif_running(morse_mon))
		return;

	/* Do not report S-MPDUs as an A-MPDU */
	is_in_ampdu = (status_flags & MORSE_RX_STATUS_FLAGS_AMPDU) &&
		      !(status_flags & MORSE_RX_STATUS_FLAGS_EOF);

	/* There are specific radiotap fields we need
	 * to append to our skb depending on the packet type
	 */
	if (status_flags & MORSE_RX_STATUS_FLAGS_NDP) {
		skb = skb_copy_expand(rx_skb, sizeof(*hdr) + sizeof(*psdu), 0, GFP_KERNEL);
		if (!skb)
			return;
		psdu = (struct zero_length_psdu *)skb_push(skb, sizeof(*psdu));

		/* Set bits for 0 length PSDU radiotap field */
		psdu->psdu_type = IEEE80211_RADIOTAP_HALOW_FLAGS_S1G_NDP_CMAC;

		ndp_sub_type =
			MORSE_RX_STATUS_FLAGS_NDP_TYPE_GET(status_flags);
		psdu->ndp_type = (ndp_sub_type == IEEE80211_NDP_FTYPE_PREQ) ?
		    IEEE80211_RADIOTAP_HALOW_FLAGS_S1G_NDP_MANAGEMENT :
		    IEEE80211_RADIOTAP_HALOW_FLAGS_S1G_NDP_CONTROL;

		if (bw_idx == DOT11_BANDWIDTH_2MHZ) {
			psdu->ndp[0] &= cpu_to_le64(IEEE80211_RADIOTAP_HALOW_MASK_NDP_2MHZ);
			psdu->ndp[0] |= cpu_to_le64(IEEE80211_RADIOTAP_HALOW_MASK_NDP_BW_2MHZ);
		} else {
			psdu->ndp[0] &= cpu_to_le64(IEEE80211_RADIOTAP_HALOW_MASK_NDP_1MHZ);
		}
	} else {
		int len = sizeof(*hdr) +
		    sizeof(*ampdu_hdr) + sizeof(*s1g_info_hdr) + sizeof(*align_padding);

		morse_vendor_tlv_size = morse_mon_vendor_tlv_size();
		len += morse_vendor_tlv_size;
		skb = skb_copy_expand(rx_skb, len, 0, GFP_KERNEL);
		if (!skb)
			return;

		s1g_info_hdr = (struct radiotap_s1g_tlv *)skb_push(skb, sizeof(*s1g_info_hdr));

		morse_mon_add_vendor_tlvs(skb, hdr_rx_status);

		if (is_in_ampdu)
			ampdu_hdr = (struct ampdu_header *)skb_push(skb, sizeof(*ampdu_hdr));

		/* Add padding to keep the radiotap alignment.
		 * This is required for most packets, except ndps.
		 */
		align_padding = (struct padding *)skb_push(skb, sizeof(*align_padding));
		align_padding->padding = 0;
	}
	bw_mhz = morse_ratecode_bw_index_to_s1g_bw_mhz(bw_idx);
	mcs_index = morse_ratecode_mcs_index_get(hdr_rx_status->morse_ratecode);
	bw = int_bw_to_radiotap_bw_enum(bw_mhz);
	if (bw == DOT11_RT_S1G_BW_INVALID)
		MORSE_ERR(mors, "Packet with invalid BW '%d' received\n", bw_mhz);

	hdr = (struct morse_radiotap_hdr *)skb_push(skb, sizeof(*hdr));
	/* No flags set for now */
	hdr->rt_flags = 0;
	hdr->hdr.it_len = cpu_to_le16(sizeof(*hdr));
	hdr->rt_tsft = hdr_rx_status->rx_timestamp_us;
	hdr->hdr.it_version = PKTHDR_RADIOTAP_VERSION;
	hdr->hdr.it_pad = 0;
	hdr->hdr.it_present = cpu_to_le32(BIT(IEEE80211_RADIOTAP_FLAGS) |
					  BIT(IEEE80211_RADIOTAP_CHANNEL) |
					  BIT(IEEE80211_RADIOTAP_TSFT) |
					  BIT(IEEE80211_RADIOTAP_DBM_ANTSIGNAL));

	/* Size and flag rtp data conditionally */
	if (status_flags & MORSE_RX_STATUS_FLAGS_NDP) {
		hdr->hdr.it_len = cpu_to_le16(le16_to_cpu(hdr->hdr.it_len) + sizeof(*psdu));
		hdr->hdr.it_present |= cpu_to_le32(BIT(IEEE80211_RADIOTAP_ZERO_LEN_PSDU));
		hdr->rt_rate_or_zl_psdu = RT_ZERO_LEN_PSDU_DATA;
	} else {
		enum morse_rate_preamble pream =
			morse_ratecode_preamble_get(hdr_rx_status->morse_ratecode);
		enum dot11_rt_s1g_ppdu_format ppdu_format = DOT11_RT_S1G_PPDU_S1G_SHORT;

		if (pream == MORSE_RATE_PREAMBLE_S1G_LONG)
			ppdu_format = DOT11_RT_S1G_PPDU_S1G_LONG;
		else if (pream == MORSE_RATE_PREAMBLE_S1G_1M)
			ppdu_format = DOT11_RT_S1G_PPDU_S1G_1M;

		hdr->hdr.it_present |= cpu_to_le32(BIT(IEEE80211_RADIOTAP_RATE) |
						   BIT(IEEE80211_RADIOTAP_TLVS));

		/* Set MSB of rate so it is interpreted as an MCS index */
		hdr->rt_rate_or_zl_psdu =
		    BIT(7) | morse_ratecode_mcs_index_get(hdr_rx_status->morse_ratecode);

		hdr->hdr.it_len = cpu_to_le16(le16_to_cpu(hdr->hdr.it_len)
						+ sizeof(*s1g_info_hdr)
						+ sizeof(*align_padding)
						+ morse_vendor_tlv_size);

		if (status_flags & MORSE_RX_STATUS_FLAGS_FCS_INCLUDED)
			hdr->rt_flags |= IEEE80211_RADIOTAP_F_FCS;

		if (status_flags & MORSE_RX_STATUS_FLAGS_CRC_ERROR)
			hdr->rt_flags |= IEEE80211_RADIOTAP_F_BADFCS;

		/* Populate the s1g info rdtp header */
		/* TODO Parse in RX status info e.g. MCS type */
		s1g_info_hdr->type = cpu_to_le16(DOT11_RT_TLV_S1G_TYPE);
		s1g_info_hdr->length = cpu_to_le16(DOT11_RT_TLV_S1G_LENGTH);

		s1g_info_hdr->known = cpu_to_le16(DOT11_RT_S1G_KNOWN_PPDU_FMT |
						  DOT11_RT_S1G_KNOWN_GI |
						  DOT11_RT_S1G_KNOWN_BW |
						  DOT11_RT_S1G_KNOWN_MCS |
						  DOT11_RT_S1G_KNOWN_RES_IND |
						  DOT11_RT_S1G_KNOWN_COLOR |
						  DOT11_RT_S1G_KNOWN_UPL_IND);

		s1g_info_hdr->data1 = cpu_to_le16(DOT11_RT_S1G_DAT1_PPDU_FMT_SET(ppdu_format) |
						  DOT11_RT_S1G_DAT1_GI_SET(morse_ratecode_sgi_get
								   (hdr_rx_status->morse_ratecode))
						  | DOT11_RT_S1G_DAT1_BW_SET(bw) |
						  DOT11_RT_S1G_DAT1_MCS_SET(mcs_index) |
						  DOT11_RT_S1G_DAT1_RES_IND_SET
						  (MORSE_RX_STATUS_FLAGS_RI_GET
						   (status_flags)));

		s1g_info_hdr->data2 =
		    cpu_to_le16(DOT11_RT_S1G_DAT2_RSSI_SET((s8)le16_to_cpu(hdr_rx_status->rssi)) |
				DOT11_RT_S1G_DAT2_COLOR_SET(hdr_rx_status->bss_color) |
				DOT11_RT_S1G_DAT2_UPL_IND_SET(MORSE_RX_STATUS_FLAGS_UPL_IND_GET
							      (status_flags)));
		memset(s1g_info_hdr->__padding, 0, sizeof(s1g_info_hdr->__padding));
	}

	if (is_in_ampdu) {
		static __le64 ampdu_ts;
		static uint reference;

		/* Individual MPDUs in an AMPDUs are reported with the same timestamp,
		 * use this to group them.
		 */
		if (ampdu_ts != hdr_rx_status->rx_timestamp_us) {
			reference++; /* New A-MPDU */
			ampdu_ts = hdr_rx_status->rx_timestamp_us;
		}

		ampdu_hdr->ref_num = cpu_to_le32(reference);
		ampdu_hdr->flags = 0;
		hdr->hdr.it_present |= cpu_to_le32(BIT(IEEE80211_RADIOTAP_AMPDU_STATUS));
		hdr->hdr.it_len = cpu_to_le16(le16_to_cpu(hdr->hdr.it_len) + sizeof(*ampdu_hdr));
	}

	hdr->rt_dbm_antsignal = (s8)le16_to_cpu(hdr_rx_status->rssi);

	/* Need to truncate the 100kHz reported frequency to MHz for radiotap hdr compatibility */
	hdr->rt_channel = cpu_to_le16(KHZ100_TO_MHZ(le16_to_cpu(hdr_rx_status->freq_100khz)));
	if (le16_to_cpu(hdr->rt_channel) <= 700)
		flags = IEEE80211_CHAN_700MHZ;
	else if (le16_to_cpu(hdr->rt_channel) <= 800)
		flags = IEEE80211_CHAN_800MHZ;
	else
		flags = IEEE80211_CHAN_900MHZ;
	hdr->rt_chbitmask = cpu_to_le16(flags);

	/* Populate skb headers */
	skb->dev = morse_mon;
	skb_reset_mac_header(skb);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = htons(ETH_P_802_2);
	memset(skb->cb, 0, sizeof(skb->cb));
	/* Push to network interface */
#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
	netif_rx(skb);
#else
	netif_rx_ni(skb);
#endif
}

void morse_mon_sig_field_error(const struct morse_cmd_evt_sig_field_error *sig_field_error_evt)
{
	struct sk_buff *skb;
	struct morse_collision_radiotap_hdr *hdr;

	if (!netif_running(morse_mon))
		return;

	skb = dev_alloc_skb(sizeof(*hdr));
	if (!skb)
		return;

	hdr = (struct morse_collision_radiotap_hdr *)skb_put(skb, sizeof(*hdr));
	hdr->hdr.it_version = PKTHDR_RADIOTAP_VERSION;
	hdr->hdr.it_pad = 0;
	hdr->hdr.it_len = cpu_to_le16(sizeof(*hdr));
	hdr->hdr.it_present =
	    cpu_to_le32(BIT(IEEE80211_RADIOTAP_TSFT) | BIT(IEEE80211_RADIOTAP_TIMESTAMP));

	hdr->rt_tsft = sig_field_error_evt->start_timestamp;
	hdr->timestamp.timestamp = sig_field_error_evt->end_timestamp;
	hdr->timestamp.accuracy = 0;
	hdr->timestamp.unit_position =
	    IEEE80211_RADIOTAP_TIMESTAMP_UNIT_US | IEEE80211_RADIOTAP_TIMESTAMP_SPOS_EO_PPDU;
	hdr->timestamp.flags = IEEE80211_RADIOTAP_TIMESTAMP_FLAG_64BIT;

	/* Populate skb headers */
	skb->dev = morse_mon;
	skb_reset_mac_header(skb);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = htons(ETH_P_802_2);
	memset(skb->cb, 0, sizeof(skb->cb));
	/* Push to network interface */
#if KERNEL_VERSION(5, 18, 0) <= LINUX_VERSION_CODE
	netif_rx(skb);
#else
	netif_rx_ni(skb);
#endif
}

int morse_mon_init(struct morse *mors)
{
	int err = 0;

	morse_mon = alloc_netdev(sizeof(struct morse_mon_priv), "morse%d", NET_NAME_UNKNOWN,
			       morse_mon_setup);
	if (!morse_mon) {
		err = -ENOMEM;
		goto alloc_netdev_err;
	}
	((struct morse_mon_priv *)netdev_priv(morse_mon))->mors = mors;

	rtnl_lock();
	err = dev_alloc_name(morse_mon, morse_mon->name);
	if (err < 0) {
		rtnl_unlock();
		goto alloc_name_err;
	}

	err = register_netdevice(morse_mon);
	if (err < 0) {
		rtnl_unlock();
		goto alloc_name_err;
	}
	rtnl_unlock();

	return 0;

alloc_name_err:
	free_netdev(morse_mon);
alloc_netdev_err:
	return err;
}

void morse_mon_free(struct morse *mors)
{
	unregister_netdev(morse_mon);
	free_netdev(morse_mon);
}
