#ifndef _MORSE_S1G_IES_H_
#define _MORSE_S1G_IES_H_

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
#if KERNEL_VERSION(5, 10, 0) > MAC80211_VERSION_CODE
#define S1G_CAP0_S1G_LONG			BIT(0)
#define S1G_CAP0_SGI_1MHZ			BIT(1)
#define S1G_CAP0_SGI_2MHZ			BIT(2)
#define S1G_CAP0_SGI_4MHZ			BIT(3)
#define S1G_CAP0_SGI_8MHZ			BIT(4)
#define S1G_CAP0_SGI_16MHZ			BIT(5)
#define S1G_CAP0_SUPP_CH_WIDTH			GENMASK(7, 6)

#define S1G_SUPP_CH_WIDTH_2			0
#define S1G_SUPP_CH_WIDTH_4			1
#define S1G_SUPP_CH_WIDTH_8			2
#define S1G_SUPP_CH_WIDTH_16			3
#define S1G_SUPP_CH_WIDTH_MAX(cap) \
	((1 << FIELD_GET(S1G_CAP0_SUPP_CH_WIDTH, cap[0])) << 1)

#define S1G_CAP1_RX_LDPC			BIT(0)
#define S1G_CAP1_TX_STBC			BIT(1)
#define S1G_CAP1_RX_STBC			BIT(2)
#define S1G_CAP1_SU_BFER			BIT(3)
#define S1G_CAP1_SU_BFEE			BIT(4)
#define S1G_CAP1_BFEE_STS			GENMASK(7, 5)

#define S1G_CAP2_SOUNDING_DIMENSIONS		GENMASK(2, 0)
#define S1G_CAP2_MU_BFER			BIT(3)
#define S1G_CAP2_MU_BFEE			BIT(4)
#define S1G_CAP2_PLUS_HTC_VHT			BIT(5)
#define S1G_CAP2_TRAVELING_PILOT		GENMASK(7, 6)

#define S1G_CAP3_RD_RESPONDER			BIT(0)
#define S1G_CAP3_HT_DELAYED_BA			BIT(1)
#define S1G_CAP3_MAX_MPDU_LEN			BIT(2)
#define S1G_CAP3_MAX_AMPDU_LEN_EXP		GENMASK(4, 3)
#define S1G_CAP3_MIN_MPDU_START			GENMASK(7, 5)

#define S1G_CAP4_UPLINK_SYNC			BIT(0)
#define S1G_CAP4_DYNAMIC_AID			BIT(1)
#define S1G_CAP4_BAT				BIT(2)
#define S1G_CAP4_TIME_ADE			BIT(3)
#define S1G_CAP4_NON_TIM			BIT(4)
#define S1G_CAP4_GROUP_AID			BIT(5)
#define S1G_CAP4_STA_TYPE			GENMASK(7, 6)

#define S1G_CAP5_CENT_AUTH_CONTROL		BIT(0)
#define S1G_CAP5_DIST_AUTH_CONTROL		BIT(1)
#define S1G_CAP5_AMSDU				BIT(2)
#define S1G_CAP5_AMPDU				BIT(3)
#define S1G_CAP5_ASYMMETRIC_BA			BIT(4)
#define S1G_CAP5_FLOW_CONTROL			BIT(5)
#define S1G_CAP5_SECTORIZED_BEAM		GENMASK(7, 6)

#define S1G_CAP6_OBSS_MITIGATION		BIT(0)
#define S1G_CAP6_FRAGMENT_BA			BIT(1)
#define S1G_CAP6_NDP_PS_POLL			BIT(2)
#define S1G_CAP6_RAW_OPERATION			BIT(3)
#define S1G_CAP6_PAGE_SLICING			BIT(4)
#define S1G_CAP6_TXOP_SHARING_IMP_ACK		BIT(5)
#define S1G_CAP6_VHT_LINK_ADAPT			GENMASK(7, 6)

#define S1G_CAP7_TACK_AS_PS_POLL		BIT(0)
#define S1G_CAP7_DUP_1MHZ			BIT(1)
#define S1G_CAP7_MCS_NEGOTIATION		BIT(2)
#define S1G_CAP7_1MHZ_CTL_RESPONSE_PREAMBLE	BIT(3)
#define S1G_CAP7_NDP_BFING_REPORT_POLL		BIT(4)
#define S1G_CAP7_UNSOLICITED_DYN_AID		BIT(5)
#define S1G_CAP7_SECTOR_TRAINING_OPERATION	BIT(6)
#define S1G_CAP7_TEMP_PS_MODE_SWITCH		BIT(7)

#define S1G_CAP8_TWT_GROUPING			BIT(0)
#define S1G_CAP8_BDT				BIT(1)
#define S1G_CAP8_COLOR				GENMASK(4, 2)
#define S1G_CAP8_TWT_REQUEST			BIT(5)
#define S1G_CAP8_TWT_RESPOND			BIT(6)
#define S1G_CAP8_PV1_FRAME			BIT(7)

#define S1G_CAP9_LINK_ADAPT_PER_CONTROL_RESPONSE BIT(0)
#endif

/* Offsets and values for 9.4.2.200.2 S1G Capabilities Information field */
/* Note these are 0 indexed in code, 1 indexed in the standard */
/* Octet 1 */
#define S1G_CAP0_SUPP_WIDTH_OFFSET	6
#define S1G_CAP0_SET_SUPP_WIDTH(x) (((x) << S1G_CAP0_SUPP_WIDTH_OFFSET) & S1G_CAP0_SUPP_CH_WIDTH)
#define S1G_CAP0_GET_SUPP_WIDTH(x) (((x) & S1G_CAP0_SUPP_CH_WIDTH) >> S1G_CAP0_SUPP_WIDTH_OFFSET)
#define S1G_CAP0_SUPP_2MHZ	S1G_CAP0_SET_SUPP_WIDTH(0)
#define S1G_CAP0_SUPP_4MHZ	S1G_CAP0_SET_SUPP_WIDTH(1)
#define S1G_CAP0_SUPP_8MHZ	S1G_CAP0_SET_SUPP_WIDTH(2)
#define S1G_CAP0_SUPP_16MHZ	S1G_CAP0_SET_SUPP_WIDTH(3)

/* Octet 2 */
#define S1G_CAP1_BFEE_STS_OFFSET	(5)
#define S1G_CAP1_SET_BFEE_STS(x)	(((x) << S1G_CAP1_BFEE_STS_OFFSET) & S1G_CAP1_BFEE_STS)
#define S1G_CAP1_GET_BFEE_STS(x)	(((x) & S1G_CAP1_BFEE_STS) >> S1G_CAP1_BFEE_STS_OFFSET)

/* Octet 3 */
/**
 * 802.11me Table 9-300 (Subfields of the S1G Capabilities Information field).
 *
 * - Set to 0 if RX of travelling pilots is not supported
 * - Set to 1 if RX of 1NSS travelling pilots is supported with STBC
 * - 2 is reserved
 * - Set to 3 if RX of 1NSS & 2NSS is supported with STBC
 */
#define S1G_CAP2_GET_TRAV_PILOT(byte2)		(((byte2) >> 6) & 0x03)
#define S1G_CAP2_SET_TRAV_PILOT(trav_pilot)	(((trav_pilot) & 0x3) << 6)

enum trav_pilot_support {
	TRAV_PILOT_RX_NOT_SUPPORTED = 0,
	TRAV_PILOT_RX_1NSS = 1,
	TRAV_PILOT_RESERVED1 = 2,
	TRAV_PILOT_RX_1_2_NSS = 3
};

#define S1G_CAP2_SOUNDING_DIMENSIONS_OFFSET	(0)
#define S1G_CAP2_SET_SOUNDING_DIMENSIONS(x)	\
	(((x) << S1G_CAP2_SOUNDING_DIMENSIONS_OFFSET) & S1G_CAP2_SOUNDING_DIMENSIONS)
#define S1G_CAP2_GET_SOUNDING_DIMENSIONS(x)	\
	(((x) & S1G_CAP2_SOUNDING_DIMENSIONS) >> S1G_CAP2_SOUNDING_DIMENSIONS_OFFSET)

/* Octet 4 */
#define S1G_CAP3_MPDU_MAX_LEN_OFFSET		2
#define S1G_CAP3_SET_MPDU_MAX_LEN(x)		\
		(((x) << S1G_CAP3_MPDU_MAX_LEN_OFFSET) & S1G_CAP3_MAX_MPDU_LEN)
#define S1G_CAP3_MPDU_MAX_LEN_3895		S1G_CAP3_SET_MPDU_MAX_LEN(0)
#define S1G_CAP3_MPDU_MAX_LEN_7991		S1G_CAP3_SET_MPDU_MAX_LEN(1)

#define S1G_CAP3_MAX_AMPDU_LEN_EXP_OFFSET	3
#define S1G_CAP3_SET_MAX_AMPDU_LEN_EXP(x)	\
		(((x) << S1G_CAP3_MAX_AMPDU_LEN_EXP_OFFSET) & S1G_CAP3_MAX_AMPDU_LEN_EXP)
#define S1G_CAP3_MIN_AMPDU_START_SPC_OFFSET	5
#define S1G_CAP3_SET_MIN_AMPDU_START_SPC(x)	\
		(((x) << S1G_CAP3_MIN_AMPDU_START_SPC_OFFSET) & S1G_CAP3_MIN_MPDU_START)
#define S1G_CAP3_GET_MIN_AMPDU_START_SPC(x)	\
		(((x) & S1G_CAP3_MIN_MPDU_START) >> S1G_CAP3_MIN_AMPDU_START_SPC_OFFSET)
/* Octet 5 */
#define S1G_CAP4_STA_TYPE_OFFSET		6
#define S1G_CAP4_SET_STA_TYPE(x)		\
(((x) << S1G_CAP4_STA_TYPE_OFFSET) & S1G_CAP4_STA_TYPE)
#define S1G_CAP4_CLEAR_AND_SET_STA_TYPE(x)	\
		(((x) & ~S1G_CAP4_GET_STA_TYPE(3)) | S1G_CAP4_SET_STA_TYPE(x))
#define S1G_CAP4_GET_STA_TYPE(x)		\
		(((x) & S1G_CAP4_STA_TYPE) >> S1G_CAP4_STA_TYPE_OFFSET)
#define S1G_CAP4_STA_TYPE_BOTH			S1G_CAP4_SET_STA_TYPE(0)
#define S1G_CAP4_STA_TYPE_SENSOR		S1G_CAP4_SET_STA_TYPE(1)
#define S1G_CAP4_STA_TYPE_NON_SENSOR		S1G_CAP4_SET_STA_TYPE(2)
/* Octet 6 */
/* Octet 7 */
/* Octet 8 */
/* Octet 9 */
#define S1G_CAP8_COLOR_OFFSET			2
#define S1G_CAP8_SET_COLOR(x)			(((x) << S1G_CAP8_COLOR_OFFSET) & S1G_CAP8_COLOR)
#define S1G_CAP8_GET_COLOR(x)			(((x) & S1G_CAP8_COLOR) >> S1G_CAP8_COLOR_OFFSET)
/* Octet 10 */

#define NL80211_S1G_NSS_MAX			(4)
#define S1G_CAP_BITS_PER_MCS_NSS		(2)

#endif /* !_MORSE_S1G_IES_H_ */
