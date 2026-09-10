#ifndef _MORSE_MORSE_H_
#define _MORSE_MORSE_H_

/*
 * Copyright 2017-2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */

#include <net/mac80211.h>
#include <linux/workqueue.h>
#include <linux/ieee80211.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/crc32.h>
#include <linux/notifier.h>
#if KERNEL_VERSION(4, 9, 81) < LINUX_VERSION_CODE
#include <linux/nospec.h>
#endif
#include "compat.h"
#include "hw.h"
#include "skbq.h"
#include "skb_header.h"
#include "firmware.h"
#include "dot11ah/dot11ah.h"
#include "dot11ah/tim.h"
#include "s1g_ies.h"
#include "watchdog.h"
#include "raw.h"
#include "chip_if.h"
#include "operations.h"
#include "hw_scan.h"
#include "utils.h"
#ifdef CONFIG_MORSE_RC
#include "rc.h"
#endif
#include "cac.h"
#include "pv1.h"
#include "coredump.h"
#include "bss_stats.h"
#include "twt.h"

#ifdef CONFIG_MORSE_USER_ACCESS
#include "uaccess.h"
#endif
#include "page_slicing.h"

#ifdef MAC80211_BACKPORT_VERSION_CODE
#define MAC80211_VERSION_CODE MAC80211_BACKPORT_VERSION_CODE
#else
#define MAC80211_VERSION_CODE LINUX_VERSION_CODE
#endif

/* Redefine IEEE80211_CHAN_IGNORE as a NOOP if it wasn't defined by a Morse Micro cfg80211 patch */
#ifndef HAS_IEEE80211_CHAN_IGNORE
#define IEEE80211_CHAN_IGNORE	0
#endif

#define MORSE_SEMVER_GET_MAJOR(x) (((x) >> 22) & 0x3FF)
#define MORSE_SEMVER_GET_MINOR(x) (((x) >> 10) & 0xFFF)
#define MORSE_SEMVER_GET_PATCH(x) ((x) & 0x3FF)

#define DRV_VERSION __stringify(MORSE_VERSION)

#define TOTAL_HALOW_CHANNELS 52

#define STA_PRIV_BACKUP_NUM (10)

#define SERIAL_SIZE_MAX  32
#define BCF_SIZE_MAX     48

/** Size in bytes of an OUI */
#define OUI_SIZE				(3)

/** Max number of OUIs supported in vendor IE OUI filter. Must match the define in the firmware */
#define MAX_NUM_OUI_FILTERS			(5)

/**
 * AID limit, currently limited to non-s1g for compatibility.
 *
 * TODO: Increase limit to 8192 (S1G) and support pages. See 802.11-2020 Section
 *	 9.4.1.8 AID Field.
 */
#define AID_LIMIT	(2007)

#define INVALID_BCN_CHANGE_SEQ_NUM 0xFFFF

#define INVALID_VIF_ID 0xFFFF

/**
 * From firmware: Time to trigger chswitch_timer in AP mode after sending
 * last beacon data to firmware in the current channel.
 */
#define BEACON_REQUEST_GRACE_PERIOD_MS (10)

/**
 * Value to use for overriding sk_pacing_shift. This influences the kernel's TCP queuing behaviour
 * and improves TCP throughput.
 */
#define SK_PACING_SHIFT (3)

/* Generate a device ID from chip ID, revision and chip type */
#define MORSE_DEVICE_ID(chip_id, chip_rev, chip_type) \
	((chip_id) | ((chip_rev) << 8) | ((chip_type) << 12))

/* Get constituents of the device id */
#define MORSE_DEVICE_GET_CHIP_ID(device_id) \
	((device_id) & 0xff)
#define MORSE_DEVICE_GET_CHIP_REV(device_id) \
	((((device_id) >> 8) & 0xf))
#define MORSE_DEVICE_GET_CHIP_TYPE(device_id) \
	((((device_id) >> 12) & 0xf))

#define MORSE_DEVICE_TYPE_IS_FPGA(device_id) \
	(MORSE_DEVICE_GET_CHIP_TYPE(device_id) == CHIP_TYPE_FPGA)

#define HZ_TO_KHZ(x) ((x) / 1000)
#define KHZ_TO_HZ(x) ((x) * 1000)
#define MHZ_TO_HZ(x) ((x) * 1000000)
#define HZ_TO_MHZ(x) ((x) / 1000000)
#define KHZ100_TO_MHZ(x) ((x) / 10)
#define KHZ100_TO_KHZ(freq) ((freq) * 100)
#define KHZ100_TO_HZ(freq) ((freq) * 100000)

#define DBM_TO_MBM(gain)  ((gain) * 100)
#define QDBM_TO_MBM(gain) (((gain) * 100) >> 2)
#define MBM_TO_QDBM(gain) (((gain) << 2) / 100)
#define QDBM_TO_DBM(gain) ((gain) / 4)

#define BPS_TO_KBPS(x) ((x) / 1000)

/**
 * enum morse_config_test_mode - test mode
 * @MORSE_CONFIG_TEST_MODE_DISABLED: normal operation
 * @MORSE_CONFIG_TEST_MODE_DOWNLOAD_ONLY: download only (no verification)
 * @MORSE_CONFIG_TEST_MODE_DOWNLOAD_AND_GET_HOST_TBL_PTR: download and get host ptr only
 * @MORSE_CONFIG_TEST_MODE_GET_HOST_TBL_PTR_ONLY: get host ptr only (no download or verification)
 * @MORSE_CONFIG_TEST_MODE_RESET: reset only (no download or verification)
 * @MORSE_CONFIG_TEST_MODE_BUS: write/read block via the bus
 * @MORSE_CONFIG_TEST_MODE_BUS_PROFILE: measure time to perform bus operations
 */
enum morse_config_test_mode {
	MORSE_CONFIG_TEST_MODE_DISABLED,
	MORSE_CONFIG_TEST_MODE_DOWNLOAD_ONLY,
	MORSE_CONFIG_TEST_MODE_DOWNLOAD_AND_GET_HOST_TBL_PTR,
	MORSE_CONFIG_TEST_MODE_GET_HOST_TBL_PTR_ONLY,
	MORSE_CONFIG_TEST_MODE_RESET,
	MORSE_CONFIG_TEST_MODE_BUS,
	MORSE_CONFIG_TEST_MODE_BUS_PROFILE,

	/* Add more test modes before this line */
	MORSE_CONFIG_TEST_MODE_INVALID,
};

struct morse_buff;
struct morse_bus_ops;

/**
 * modparam variables
 */
extern uint test_mode;
extern char serial[];
extern char board_config_file[];
extern u8 macaddr_octet;
extern bool enable_otp_check;
extern bool enable_ext_xtal_init;
extern u8 macaddr[ETH_ALEN];
extern bool enable_ibss_probe_filtering;
extern uint ocs_type;
extern uint sdio_reset_time;
extern enum morse_cmd_duty_cycle_mode duty_cycle_mode;
extern bool enable_secureboot;

/**
 * enum morse_mac_subbands_mode - flags to describe sub-bands handling
 *
 * @SUBBANDS_MODE_DISABLED : sub-bands disabled all packets are sent in operating bandwidth
 * @SUBBANDS_MODE_MANAGEMENT : sub-bands are set only for know management packets
 *	(e.g. beacons, probe requests/responses, etc.)
 * @SUBBANDS_MODE_ENABLED : [default] sub-bands fully enabled, data packets will follow sub-band
 *	signaling from the RC algorithm (ministrel)
 */
enum morse_mac_subbands_mode {
	SUBBANDS_MODE_DISABLED = 0x00,
	SUBBANDS_MODE_MANAGEMENT = 0x01,
	SUBBANDS_MODE_ENABLED = 0x02,
	SUBBANDS_MODE_UNKNOWN = 0xFF
};

struct morse_mbssid_info {
	u8 max_bssid_indicator;
	u8 transmitter_vif_id;
};

struct morse_custom_configs {
	u8 sta_type;
	u8 enc_mode;
	bool enable_ampdu;
	bool enable_trav_pilot;
	bool enable_airtime_fairness;
	bool enable_sgi_rc;
	bool listen_interval_ovr;
	u16 listen_interval;
	enum morse_mac_subbands_mode enable_subbands;
	struct morse_channel_info channel_info;
	struct morse_channel_info default_bw_info;
	bool enable_arp_offload;
	bool enable_legacy_amsdu;
	bool enable_dhcpc_offload;
	bool enable_sta_cac;
	char *dhcpc_lease_update_script;
	u32 duty_cycle;
	u32 dynamic_ps_timeout_ms;
};

struct morse_ps {
	/* Number of clients requesting to talk to chip */
	u32 wakers;
	/* Is the driver permitted to signal power save to the chip */
	bool is_drv_ps_allowed;

	/* The interface in control of power save decisions. NULL if not enabled */
	struct morse_vif *mors_vif;

	bool suspended;
	bool dynamic_ps_en;
	/* Host enforced remain awake time after network activity (ms) */
	int ps_net_timeout_ms;
	unsigned long bus_ps_timeout;
	/* Serialise access to the PS structure */
	struct mutex lock;
	struct work_struct async_wake_work;
	struct delayed_work delayed_eval_work;
	struct completion *awake;
};

/* Morse ACI map for page metadata */
enum morse_page_aci {
	MORSE_ACI_BE = 0,
	MORSE_ACI_BK = 1,
	MORSE_ACI_VI = 2,
	MORSE_ACI_VO = 3,
};

/* Taken from 802.11me Table 10-1.
 * Encodes the user priority (UP) section of a TID
 */
enum qos_tid_up_index {
	MORSE_QOS_TID_UP_BK = 1,
	MORSE_QOS_TID_UP_xx = 2,	/* Not specified */
	MORSE_QOS_TID_UP_BE = 0,
	MORSE_QOS_TID_UP_EE = 3,
	MORSE_QOS_TID_UP_CL = 4,
	MORSE_QOS_TID_UP_VI = 5,
	MORSE_QOS_TID_UP_VO = 6,
	MORSE_QOS_TID_UP_NC = 7,

	MORSE_QOS_TID_UP_LOWEST = MORSE_QOS_TID_UP_BK,
	MORSE_QOS_TID_UP_HIGHEST = MORSE_QOS_TID_UP_NC
};

struct morse_sw_version {
	u8 major;
	u8 minor;
	u8 patch;
};

/* Rate control method in use */
enum morse_rc_method {
	MORSE_RC_METHOD_UNKNOWN = 0,
	MORSE_RC_METHOD_MINSTREL = 1,
	MORSE_RC_METHOD_MMRC = 2,
};

/* non-TIM mode status */
enum morse_non_tim_mode {
	/** Indicates that non-TIM mode is disabled */
	NON_TIM_MODE_DISABLED,
	/** Used in AP mode to keep track STAs requesting non-TIM mode */
	NON_TIM_MODE_REQUESTED,
	/** Indicates that AP has allowed STA to enter non-TIM mode */
	NON_TIM_MODE_ENABLED
};

/* beacon offload state */
enum morse_beacon_offload_state {
	/* Beacon offload disabled all beacon info generated by mac80211 */
	MORSE_BEACON_OFFLOAD_STATE_DISABLED,
	/* Beacon offload enabled */
	MORSE_BEACON_OFFLOAD_STATE_ENABLED,
	/* Beacon offload stopping */
	MORSE_BEACON_OFFLOAD_STATE_STOPPING
};

/**
 * struct morse_vendor_info - filled from mm vendor IE
 */
struct morse_vendor_info {
	/** Indicates if vendor info is valid (and has been filled) */
	bool valid;
	/** Vendor capability mmss offset signalled by Morse devices */
	u8 morse_mmss_offset;
	/** Identifies underlying hardware of device */
	u32 chip_id;
	/** Identifies underlying software ver of device */
	struct morse_sw_version sw_ver;
	/** Operational features in use on device */
	struct morse_ops operations;
	/** Device supports short 'additional ack timeout delay' */
	bool supports_short_ack_timeout;
	/** Device supports PV1 Data Frames only */
	bool pv1_data_frame_only_support;
	/**
	 * Device support exclusive page slicing (all stations in the BSS should support
	 * page slicing)
	 */
	bool page_slicing_exclusive_support;
	/** Rate control method in use */
	enum morse_rc_method rc_method;
};

/** Morse Private STA record */
struct morse_sta {
	/** virtual interface this sta is on */
	const struct ieee80211_vif *vif;
	/** Count of how many association requests we have received while associating */
	int assoc_req_count;
	/** Count of how many reassociation requests we have to process */
	int reassoc_req_count;
	/** When to timeout this record (used in backup) */
	unsigned long timeout;
	/** The address of this sta */
	u8 addr[ETH_ALEN];
	/** Current state of the station (this is not part of the ieee80211_sta struct) */
	enum ieee80211_sta_state state;
	/** Whether A-MPDU is supported on this STA */
	bool ampdu_supported;
	/** STA's required Minimum MPDU start spacing as reported by s1g capabs */
	u8 ampdu_mmss;
	/** Whether we have a TX A-MPDU on this TID */
	bool tid_tx[IEEE80211_NUM_TIDS];
	/** Whether we have tried to start a TX A-MPDU on this TID */
	bool tid_start_tx[IEEE80211_NUM_TIDS];
	/** Whether travelling pilots is supported */
	enum trav_pilot_support trav_pilot_support;
	/** Per-TID parameters */
	u8 tid_params[IEEE80211_NUM_TIDS];
	/** STA's max tx bw as reported in s1g capabilities */
	int max_bw_mhz;
	/** Vendor information - filled from mm vendor IE */
	struct morse_vendor_info vendor_info;
#ifdef CONFIG_MORSE_RC
	/** Morse Micro RC info*/
	struct morse_rc_sta rc;
	struct mmrc_rate last_sta_tx_rate;
	struct mmrc_rate last_sta_rx_rate;
#endif
	/** RX status of last rx skb */
	struct {
		bool is_data_set;
		bool is_mgmt_set;
		struct morse_skb_rx_status data_status;
		struct morse_skb_rx_status mgmt_status;
	} last_rx;

	/** average rssi of rx packets */
	s16 avg_rssi;
	/** When set, frames destined for this STA must be returned to mac80211
	 *  for rescheduling. Cleared after frame destined for STA has
	 *  IEEE80211_TX_CTL_CLEAR_PS_FILT set.
	 */
	bool tx_ps_filter_en;

	/** Counts the number of packets passed from the kernel to the driver */
	u64 tx_pkt_count;

	/** number of peerings established and valid only if it is mesh peer */
	u8 mesh_no_of_peerings;

	/** Set when PV1 capability is advertised in S1G capabilities of peer STA */
	bool pv1_frame_support;

	/** Save stored status of peer STA from Header Compression Response at TX */
	struct morse_sta_pv1 tx_pv1_ctx;

	/** Save stored status of peer STA from Header Compression Request on RX*/
	struct morse_sta_pv1 rx_pv1_ctx;

	/** Last received S1G protected action PN */
	u64 last_rx_mgmt_pn;

	/** non-TIM mode negotiated between AP & STA */
	enum morse_non_tim_mode non_tim_mode_status;

	/** This is the 1st byte of S1G capabilities IE, stored to retrieve STA's short GI
	 *  capabilities and supported channel width.
	 */
	u8 s1g_cap0;

	/** RAW Priority, extracted from QoS traffic capability IE */
	u8 raw_priority;

	/** STA entry is in BSS statistics module */
	struct morse_bss_stats_sta bss_stats_sta;
};

/** Number of bits in AID bitmap.
 * +1 as AID 0 is reserved, and the AID_LIMIT is the 'max' AID
 */
#define MORSE_AP_AID_BITMAP_SIZE		(AID_LIMIT + 1)

/** AP specific information */
struct morse_ap {
	/** back pointer to interface */
	struct morse_vif *mors_vif;
	/** Number of stas currently associated */
	u16 num_stas;
	/** Largest AID currently in use */
	u16 largest_aid;
	/** RAW state */
	struct morse_raw raw;
	/** BSS statistics */
	struct morse_bss_stats_context bss_stats;
	/**
	 * Bitmap of AIDs currently in use. Bit position corresponds to the AID.
	 */
	DECLARE_BITMAP(aid_bitmap, MORSE_AP_AID_BITMAP_SIZE);
};

struct morse_mbca_config {
	/**
	 * Configuration to enable or disable MBCA TBTT selection and adjustment.
	 */
	u8 config;
	/**
	 * Interval at which beacon timing elements are included in beacons.
	 */
	u8 beacon_timing_report_interval;
	/**
	 * To keep track number of beacons sent for beacon timing report interval.
	 */
	u8 beacon_count;
	/**
	 * Minimum gap between our beacons and neighbor beacons for TBTT Selection.
	 */
	u8 min_beacon_gap_ms;
	/**
	 * Initial scan to find peers in the MBSS
	 */
	u16 mbss_start_scan_duration_ms;
	/**
	 * TBTT adjustment timer interval in target LMAC firmware.
	 */
	u16 tbtt_adj_interval_ms;
};

/** Mesh specific information */
struct morse_mesh {
	/** back pointer */
	struct morse_vif *mors_vif;
	/** mesh periodic probe timer */
	struct timer_list mesh_probe_timer;
	/** rx status of probe req */
	struct ieee80211_rx_status probe_rx_status;
	/** probe resp template */
	struct sk_buff *mesh_probe_resp_skb;
	/** address of the peer kicked out */
	u8 kickout_peer_addr[ETH_ALEN];
	/** Timestamp when peer is kicked out */
	unsigned long kickout_ts;
	/** Mesh configuration */
	struct morse_mesh_config *conf;
	/** List head for delayed mgmt tx */
	struct list_head delayed_tx_list;
	/** Spinlock to protect access to delayed list */
	spinlock_t delayed_tx_lock;
};

struct morse_mesh_config {
	/** Mesh ID of the network */
	u8 mesh_id[IEEE80211_MAX_SSID_LEN];
	/** dynamic peering mode */
	bool dynamic_peering;
	/** Mesh active */
	bool is_mesh_active;
	/** RSSI margin to consider while selecting a peer to kick out */
	u8 rssi_margin;
	/** Length of the Mesh ID */
	u8 mesh_id_len;
	/** Mode of mesh beaconless operation */
	u8 mesh_beaconless_mode;
	/** Maximum number of peer links */
	u8 max_plinks;
	/** Duration in seconds, a blacklisted peer is not allowed peering */
	u32 blacklist_timeout;
	/** Mesh Beacon Collision Avoidance state */
	struct morse_mbca_config mbca;
} __packed;

struct morse_vendor_ie {
	/** List of run-time configurable vendor IEs to insert into management frames */
	struct list_head ie_list;

	/** List of vendor IE OUIs for which to generate a netlink event if seen in a mgmt
	 * frame.
	 */
	struct list_head oui_filter_list;

	/** Number of elements in the OUI filter list */
	u8 n_oui_filters;
};

/**
 * enum morse_scan_state_flags - Scan state flags in fullmac mode.
 */
enum morse_scan_state_flags {
	/** @MORSE_SCAN_STATE_SCANNING: Scan is in progress. */
	MORSE_SCAN_STATE_SCANNING,
	/** @MORSE_SCAN_STATE_ABORTED: An error occurred during the scan. */
	MORSE_SCAN_STATE_ABORTED,
};

/**
 * enum morse_sme_state - VIF state in fullmac mode.
 */
enum morse_sme_state {
	/** @MORSE_SME_STATE_IDLE: Connection not requested. */
	MORSE_SME_STATE_IDLE,
	/** @MORSE_SME_STATE_CONNECTING: Connection request is in progress. */
	MORSE_SME_STATE_CONNECTING,
	/** @MORSE_SME_STATE_CONNECTED: Connection is established. */
	MORSE_SME_STATE_CONNECTED,
	/** @MORSE_SME_STATE_DISCONNECTING: Disconnect request is in progress. */
	MORSE_SME_STATE_DISCONNECTING,
	/** @MORSE_SME_STATE_DISCONNECTING: Roaming request is in progress. */
	MORSE_SME_STATE_ROAMING,
};

/**
 * struct morse_wiphy_connect_params - Parameters for connection request within the driver.
 * @roam: Request roam instead of fresh connection (reassociate within the same ESS).
 * @auth_type: Authentication type (open, OWE, SAE).
 * @ssid: SSID to connect to.
 * @ssid_len: Length of @ssid.
 * @sae_pwd: Password for SAE authentication. Ignored for other authentication types.
 * @sae_pwd_len: Length of @sae_pwd.
 * @extra_assoc_ies: IEs appended to Association Request frame, in addition to
 *                   firmware-generated IEs. Buffer is heap-allocated.
 * @extra_assoc_ies_len: Length of @extra_assoc_ies.
 * @bssid: Optional BSSID to connect to. If zero, firmware chooses any BSS.
 * @bg_scan_period: Background scan period in seconds, or -1 to use default.
 * @use_4addr: True if Linux "4-address mode" compatibility should be enabled.
 */
struct morse_wiphy_connect_params {
	bool roam;
	enum morse_cmd_connect_auth_type auth_type;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
	size_t ssid_len;
	u8 sae_pwd[SAE_PASSWORD_MAX_LEN];
	size_t sae_pwd_len;
	u8 *extra_assoc_ies;
	size_t extra_assoc_ies_len;
	u8 bssid[ETH_ALEN];
	int bg_scan_period;
	bool use_4addr;
};

struct morse_connected_evt_params {
	u8 bssid[ETH_ALEN];
	u16 assoc_resp_ies_len;
	u8 assoc_resp_ies[];
};

struct morse_vif {
	u16 id;			/* interface ID from chip */

	struct morse_vif_sysfs {
		/* @registered: true once vif registers with sysfs */
		bool registered;
		/* @attr_g: top level handle for sysfs group / node */
		struct attribute_group attr_g;
		/* @attrs: Group attributes (NULL terminated list) */
		struct attribute *attrs[2];
		/* @aid: attribute - AID of VIF */
		struct device_attribute aid;
	} sysfs;

	u16 dtim_count;

	/* Is power save enabled for this interface */
	bool is_ps_enabled;

	/**
	 * Used to keep track of time for beacons
	 * and probe responses. This is an approximation!
	 * Proper solution is to use timing from PHY.
	 *
	 * This field is in Linux jiffies.
	 **/
	u64 epoch;

	/**
	 * Pointer to current custom configuration
	 * for the chip.
	 */
	struct morse_custom_configs *custom_configs;

	/**
	 * Signals that when we send control response frames we will
	 * send on 1MHz. Our S1G capabilities field must be updated accordingly.
	 */
	bool ctrl_resp_out_1mhz_en;

	/**
	 * Signals that for frames we send we can expect the control
	 * response frames (primarily ACKs) to be on 1MHz. This is sent
	 * to the firmware so that it can adjust timeouts as necessary.
	 *
	 * To be strictly standards compliant with ack times and
	 * avoid uneccesary performance degradation, this should be
	 * tracked per MAC address + vif as individual STAs may or may
	 * not opt into it.
	 */
	bool ctrl_resp_in_1mhz_en;

	/**
	 * CAC (Centralized Authentication Control)
	 */
	struct morse_cac cac;

	/**
	 * AP's required Minimum MPDU start spacing as communicated by S1G
	 * capabilities field. Only valid after association response
	 * is received for STAs.
	 */
	u8 bss_ampdu_mmss;

	/**
	 * TWT state information.
	 */
	struct morse_twt twt;

	/**
	 * AP mode specific information
	 * NULL if not an AP
	 */
	struct morse_ap *ap;

	/**
	 * Mesh mode specific information. NULL if vif is not a Mesh
	 */
	struct morse_mesh *mesh;

	struct {
		/** List of run-time configurable vendor IEs to insert into or filter
		 * from management frames - stored in persistent vif config
		 */
		struct morse_vendor_ie *vie_config;

		/** Spinlock to protect access to these fields */
		spinlock_t lock;
	} vendor_ie;

	/** SW-3908 unveiled a race condition, so sometimes we have to store a backup
	 * of our private data when a device reassociates so that S1G information
	 * is persisted
	 */
	struct {
		/** Spinlock to protect access to these fields */
		spinlock_t lock;
		/** Backup entries for STA private data */
		struct morse_sta backups[STA_PRIV_BACKUP_NUM];
	} sta_backups;

	struct morse_caps capabilities;
	struct morse_ops operations;

	/**
	 *  Custom features obtained from the associated AP, filled via
	 *  vendor IE. Only valid after association response is received for STAs.
	 */
	struct morse_vendor_info bss_vendor_info;

	struct ieee80211_s1g_cap s1g_cap_ie;

	/**
	 * Beacon Tasklet
	 */
	struct tasklet_struct beacon_tasklet;

	/**
	 *  Keeping track of beacon change sequence number for both AP and STA
	 */
	u16 s1g_bcn_change_seq;

	/**
	 * To keep track of channel switch in progress and to restrict
	 * the updating of s1g_bcn_change_seq to only once
	 */
	bool chan_switch_in_progress;

	/**
	 * CRC of EDCA parameter set either from EDCA parameter IE or WMM IE
	 */
	u32 edca_param_crc;

	/**
	 * CRC of S1G operation Parameter IE
	 */
	u32 s1g_oper_param_crc;

	/**
	 * Template Buffer of probe request unicast/directed packet for sending
	 * to connected AP only. This will be populated in bss_info_change event
	 * handler and used in other places, ex: to trigger probe on detecting
	 * update on change sequence number
	 */
	struct sk_buff *probe_req_buf;

	/**
	 * Buffer that saves beacon from mac80211 for this BSS
	 */
	struct sk_buff *beacon_buf;

	/**
	 * Pointer to SSID IE buffer got from beacon.
	 */
	const u8 *ssid_ie;

	/**
	 * Flag to check if STA is in asscociated state. This gets populated in
	 * bss_info_changed event handler. Any other place to access vif->bss_conf
	 * may not be safe. This flag saves the snapshot of connection status.
	 * valid only for STA mode.
	 */
	bool is_sta_assoc;

	/**
	 * Flag to check if ibss node joined/created a network. This flag gets
	 * updated in morse_mac_join_ibss and morse_mac_leave_ibss.
	 */
	bool is_ibss_node_joined;

	/**
	 * Flag to keep track if unicast/directed probe req needs to be sent.
	 */
	bool waiting_for_probe_req_sched;

	/**
	 * Flag to indicate if this VIF supports hw PS filtering
	 * (see IEEE80211_TX_STAT_TX_FILTERED). If support is not enabled,
	 * frames returned from the LMAC with this flag set will be dropped.
	 */
	bool supports_ps_filter;

	/**
	 * Stores the station channel info after association
	 */
	struct morse_channel_info assoc_sta_channel_info;

	/**
	 * Stores the ECSA channel info
	 */
	struct morse_channel_info ecsa_channel_info;

	/**
	 * Stores MBSSID IE info to select transmitting/non-transmitting BSS
	 */
	struct morse_mbssid_info mbssid_info;

	/**
	 * Channel Switch Timer for station mode
	 */
	struct timer_list chswitch_timer;

	/**
	 * Flag to check if ECSA Info IEs needs to be masked in beacon & probe response.
	 */
	bool mask_ecsa_info_in_beacon;

	/**
	 * Flag to check if new channel configured to send full beacon in new channel
	 * irrespective of short beacon interval.
	 */
	bool ecsa_chan_configured;

	/**
	 * Work queue to configure ecsa channel
	 */
	struct delayed_work ecsa_chswitch_work;

	/**
	 * Save PV1 status for this VIF
	 */
	struct morse_pv1 pv1;

	/**
	 * Flag to check if PV1 support is allowed for this vif
	 */
	bool enable_pv1;

	/**
	 *  Tasklet for sending unicast directed probe request from Morse driver
	 */
	struct tasklet_struct send_probe_req;

	/**
	 * Holds page slicing information like page period, page slice control and bitmap.
	 */
	struct page_slicing page_slicing_info;

	/**
	 * Flag to keep track of beaconing enabled
	 */
	bool beaconing_enabled;

	/**
	 * Flag to indicate beacon generation has been offloaded to the chip
	 */
	struct {
		/** Beacon offload state **/
		enum morse_beacon_offload_state state;
		/**
		 * Number of beacon tasklet events skipped while starting / stopping
		 * beacon offload
		 */
		unsigned int skipped_tasklet_events;
		/** Spinlock to protect access to these fields */
		spinlock_t lock;
	} beacon_offload;

	/**
	 * Flag to check if multicast rate control is not enabled or not.
	 */
	bool enable_multicast_rate_control;

	/**
	 * Tx rate to be used for multicast traffic.
	 */
	struct mmrc_rate mcast_tx_rate;

	/**
	 * Throughput calculated using the above mcast_tx_rate.
	 */
	u32 mcast_tx_rate_throughput;

	/**
	 * User configuration of non-TIM mode.
	 */
	bool enable_non_tim_mode;

	/**
	 *  Work to evaluate the tx rate to use for multicast packets transmission.
	 */
	struct delayed_work mcast_tx_rate_work;

	/**
	 * FullMAC device parameters.
	 */
	struct wireless_dev wdev;
	struct net_device *ndev;

	/**
	 * @sme_state: Current state of the chip SME in fullmac mode.
	 */
	enum morse_sme_state sme_state;

	/** @connect_params: Connection parameters when connecting/connected in fullmac mode. */
	struct morse_wiphy_connect_params connect_params;

	/** @connected_bss: The BSS we are connected to, when connected in fullmac mode. */
	struct cfg80211_bss *connected_bss;

	/** @connected_params: Connected event params, consumed in a work item (fullmac only). */
	struct morse_connected_evt_params *connected_params;

	/** @connected_work: Work item for handling connected events (fullmac only). */
	struct work_struct connected_work;

	/** @disconnected_work: Work item for handling disconnected events (fullmac only). */
	struct work_struct disconnected_work;

	/** @autoconnect: Automatically reconnect to given SSID upon disconnect (fullmac only). */
	bool autoconnect;

	/* ARP filtering related fields (fullmac only) */
	struct {
		/**
		 * @arp_filter.ifa_notifier: Notifier for IPv4 address changes.
		 */
		struct notifier_block ifa_notifier;
		/**
		 * @arp_filter.addr_list: List of IPv4 addresses for ARP filtering/offload.
		 *                        Equivalent to mac80211's arp_addr_list on
		 *                        &struct ieee80211_vif.
		 */
		__be32 addr_list[IEEE80211_BSS_ARP_ADDR_LIST_LEN];
		/**
		 * @arp_filter.addr_cnt: Number of IPv4 addresses for ARP filtering/offload.
		 *                       Note this may be longer than the length of
		 *                       %arp_filter.addr_list. Equivalent to mac80211's
		 *                       arp_addr_cnt on &struct ieee80211_vif.
		 */
		int addr_cnt;
	} arp_filter;

	/** @stypes: Registered frame subtypes for this interface (fullmac only). */
	u32 stypes;

#ifdef CONFIG_ANDROID
	struct {
		/**
		 * Flag to indicate whether APF is enabled or not.
		 */
		bool enabled;

		/**
		 * Maximum length of the APF memory in bytes.
		 */
		u32 max_length;
	} apf;
#endif
};

struct morse_debug {
	struct dentry *debugfs_phy;
#ifdef CONFIG_MORSE_DEBUG_TXSTATUS
	 DECLARE_KFIFO(tx_status_entries, struct morse_skb_tx_status, 1024);
#endif
	struct {
		struct {
			unsigned int tx_beacons;
			unsigned int tx_ndpprobes;
			unsigned int tx_count;
			unsigned int tx_success;
			unsigned int tx_fail;
			unsigned int rx_count;
		} mcs0;
		struct {
			unsigned int tx_count;
			unsigned int tx_success;
			unsigned int tx_fail;
			unsigned int rx_count;
		} mcs10;
	} mcs_stats_tbl;
	struct {
		unsigned int cmd_tx;
		unsigned int bcn_tx;
		unsigned int mgmt_tx;
		unsigned int data_tx;
		unsigned int write_fail;
		unsigned int no_page;
		unsigned int cmd_no_page;
		unsigned int bcn_no_page;
		unsigned int queue_stop;
		unsigned int page_owned_by_chip;
		unsigned int tx_aged_out;
		unsigned int tx_ps_filtered;
		unsigned int tx_status_flushed;
		unsigned int tx_status_page_invalid;
		unsigned int tx_status_duty_cycle_cant_send;
		unsigned int tx_duty_cycle_retry_disabled;
		unsigned int tx_status_dropped;
		unsigned int rx_empty;
		unsigned int rx_split;
		unsigned int rx_invalid_count;
		unsigned int invalid_checksum;
		unsigned int invalid_tx_status_checksum;
	} page_stats;
#if defined(CONFIG_MORSE_DEBUG_IRQ)
	struct {
		unsigned int irq;
		unsigned int irq_bits[32];
	} hostsync_stats;
#endif
#ifdef CONFIG_MORSE_DEBUGFS
	struct {
		/* Serialise manipulation of the log queue */
		struct mutex lock;
		wait_queue_head_t waitqueue;
		int active_clients;
		struct list_head items;
		int enabled_channel_mask;
	} hostif_log;
#endif
	struct dentry *debugfs_logging;
};

/**
 * struct morse_channel_survey - RF/traffic characteristics of a channel
 * @time_listen: total time spent receiving, usecs
 * @time_rx: duration of time spent receiving, usecs
 * @freq_hz: the center frequency of the channel
 * @bw_mhz: The bandwidth of the channel
 * @noise: channel noise, dBm
 */
struct morse_survey_rx_usage_record {
	u64 time_listen;
	u64 time_rx;
	u32 freq_hz;
	u8 bw_mhz;
	s8 noise;
};

struct morse_channel_survey {
	bool first_channel_in_scan;
	int num_records;
	struct morse_survey_rx_usage_record *records;
};

struct morse_watchdog {
	struct hrtimer timer;
	uint interval_secs;
	watchdog_callback_t ping;
	int consumers;
	/* Serialise use of watchdog functions */
	struct mutex lock;
	int paused;
};

struct morse_stale_tx_status {
	/* Protect stale Tx status queue handling and timer functions */
	spinlock_t lock;
	struct timer_list timer;
	bool enabled;
};

struct mcast_filter {
	u8 count;
	/* Integer representation of the last four bytes of a multicast MAC address.
	 * The first two bytes are always 0x0100 (IPv4) or 0x3333 (IPv6).
	 */
	__le32 addr_list[];
};

/** State flags for managing state of mors object */
enum morse_state_flags {
	/** Pushing/Pulling from the mac80211 DATA Qs have been stopped */
	MORSE_STATE_FLAG_DATA_QS_STOPPED,
	/** Sending TX data to the chip has been stopped */
	MORSE_STATE_FLAG_DATA_TX_STOPPED,
	/** Regulatory domain has been set by the user using `iw reg set` */
	MORSE_STATE_FLAG_REGDOM_SET_BY_USER,
	/** Regulatory domain is set in the OTP */
	MORSE_STATE_FLAG_REGDOM_SET_BY_OTP,
	/** An action on initialisation requires a FW reload. (eg. regdom change) */
	MORSE_STATE_FLAG_RELOAD_FW_AFTER_START,
	/** Perform a core dump on next mac restart */
	MORSE_STATE_FLAG_DO_COREDUMP,
	/** TX from host to firmware is blocked */
	MORSE_STATE_FLAG_HOST_TO_CHIP_TX_BLOCKED,
	/** TX CMD from host to firmware is blocked */
	MORSE_STATE_FLAG_HOST_TO_CHIP_CMD_BLOCKED,
	/** Host system is in, or in the process of, entering suspend */
	MORSE_STATE_FLAG_SYSTEM_IN_SUSPEND,
	/** Driver is being removed or probe failed */
	MORSE_STATE_FLAG_STOPPING,
};

/**
 * State flags are cleared on .start(). Bits specified here will not be cleared
 */
#define MORSE_STATE_FLAG_KEEP_ON_START_MASK	(BIT(MORSE_STATE_FLAG_REGDOM_SET_BY_USER))

#define MORSE_MAX_IF (2)
#define MORSE_COUNTRY_LEN (3)
#define INVALID_VIF_INDEX 0xFF

struct morse_persistent_vif_configs {
	/** VIF mac address */
	u8 addr[ETH_ALEN];
	/** BSS stats config */
	struct morse_bss_stats_config bss_stats_conf;
	/** TWT config with active agreement */
	struct morse_twt_config twt_conf;
	/** CAC configuration and rules */
	struct morse_cac_config cac_conf;
	/** Mesh configs */
	struct morse_mesh_config mesh_conf;
	/** List of RAW configs */
	struct list_head raw_config_list;
	/** Vendor IEs list */
	struct morse_vendor_ie vie_config;
	/**
	 * The Max Away Duration field indicates the maximum duration that the AP
	 * can be out of reach for the STA (operating in other channels, enter
	 * power save mode, or operating in other RAWs).
	 *
	 * The value of the Max Away Duration field is expressed in units of TU. (AP iface only)
	 */
	u16 bss_max_away_duration;
	/** mlme allows power save for AP interfaces (AP iface only) */
	bool ap_ps_enabled;
	/**
	 * Configured BSS color.
	 * Only valid after association response is received for STAs
	 */
	u8 bss_color;
};

struct morse {
	u32 chip_id;

	/** Refer to enum morse_host_bus_type */
	u32 bus_type;

	/* Parsed from the release tag, which should be in the format
	 * 'rel_<major>_<minor>_<patch>' or
	 * 'rel_<chip>_<major>_<minor>_<patch>', where <chip> is a 6-character
	 * identifier such as mm6108. If the tag is not in one of these formats
	 * then the corresponding version field will be 0.
	 */
	struct morse_sw_version sw_ver;
	u8 macaddr[ETH_ALEN];
	u8 country[MORSE_COUNTRY_LEN];

	/* mask of type \enum host_table_firmware_flags */
	u32 firmware_flags;
	struct morse_caps capabilities;
	enum morse_hw_state hw_state;

	/** @in_scan: whether the chip has been configured for scan mode (softmac only). */
	bool in_scan;
	bool chip_was_reset;
	enum morse_cmd_boot_code boot_code;

	/* wiphy device registered with cfg80211 */
	struct wiphy *wiphy;

	/** @workqueue: Workqueue for fullmac work items. */
	struct workqueue_struct *wiphy_wq;

	/** true when wiphy has started */
	bool wiphy_started;

	/** true when reusing vif after reattach (fullmac only). */
	bool reuse_vif;

	/** @scan_req: pointer to the current scan request which is in progress,
	 * or NULL if no scan is in progress (fullmac only).
	 */
	struct cfg80211_scan_request *scan_req;

	/** @scan_state: Bit field of state flags for scanning in fullmac mode.
	 *               See &enum morse_scan_state_flags for bit numbers.
	 */
	unsigned long scan_state;

	/** @scan_done_work: Work item for handling scan_done events (fullmac only). */
	struct work_struct scan_done_work;

	/**
	 * @rts_allowed: Whether RTS can be enabled (fullmac only).
	 */
	bool rts_allowed;

	/**
	 * @orig_rts_threshold: If RTS is forcibly disabled, this saves the previously configured
	 *                      RTS threshold so it can be restored later (fullmac only).
	 */
	u32 orig_rts_threshold;

	/* Extra padding to insert at the start of each tx packet */
	u8 extra_tx_offset;

	struct morse_hw_scan hw_scan;

	struct morse_channel_survey *channel_survey;

	struct ieee80211_hw *hw;

	struct {
		/**
		 * @restriction_is_enforced: Whether memory access restriction is enforced or not
		 *
		 */
		bool restriction_is_enforced;

		/**
		 * @restriction_is_overridden: Whether memory access restriction is overridden
		 *
		 */
		bool restriction_is_overridden;

		/**
		 * @configured_region Memory access region that is configured through requests
		 *
		 */
		struct morse_hw_memory configured_region;
	} mem_access;

	/** softmac state tracking of mlme */
	struct {
		/** Has the MLME started? */
		bool started;
		/** Is the MLME reconfiguring the HW? */
		bool in_reconfig;
	} mlme;

	/* Array of vif pointers, indexed by vif ID. Allocated based on max interfaces supported.
	 * Do not access directly. Use morse_get_vif_* functions.
	 */
	struct ieee80211_vif **vif;
	/* Size of the above array */
	u16 max_vifs;

	/* Maximum number of STAs reported by firmware */
	u32 max_ap_num_sta;

	/* spinlock to protect vif array */
	spinlock_t vif_list_lock;

	struct device *dev;
	/** See morse_state_flags */
	unsigned long state_flags;

	/* Command sequence counter */
	u16 cmd_seq;
	struct completion *cmd_comp;
	/* Mutex to martial command completion and retries */
	struct mutex cmd_lock;
	/* Allow only one command at a time */
	struct mutex cmd_wait;

	/** User-initiated coredump complete signal mechanism */
	struct completion *user_coredump_comp;

	struct {
		/** Address in hardware to write the BCF file */
		u32 address;
		/** Size of the firmware BCF memory region */
		u32 size;
	} bcf;

	struct tasklet_struct tasklet_txq;
	/* Serialise high-level operations to the morse structure */
	struct mutex lock;
	/**
	 * 80211n channel number - may or may not map to currently selected
	 * s1g channel.
	 */
	int channel_num_80211n;
	int enable_subbands;

	/* Deprecated, required for platform support */
	int rb_cnt;
	struct morse_rb *rb;

	/* Chip interface variables */
	struct morse_chip_if_state *chip_if;
	/* Work queue used by code directly talking to the chip */
	struct workqueue_struct *chip_wq;
	struct work_struct chip_if_work;
	struct work_struct usb_irq_work;

	/* Used to periodically check for stale tx skbs */
	struct morse_stale_tx_status stale_status;

#ifdef CONFIG_MORSE_USER_ACCESS
	struct uaccess_device udev;
#endif

	struct morse_ps ps;

	/* U-APSD status per Access Category (bitfield) */
	u8 uapsd_per_ac;

	/* Actual Tx power currently applied by the firmware */
	s32 tx_power_mbm;
	s32 tx_max_power_mbm;

	bool enable_mbssid_ie;
	/* Hardware scan is enabled/disabled */
	bool enable_hw_scan;
	bool enable_sched_scan;

	/* Type of rate control method in use */
	enum morse_rc_method rc_method;
#ifdef CONFIG_MORSE_RC
	struct morse_rc mrc;
	int rts_threshold;
#endif
	struct morse_vif mon_if;	/* monitor interface */
	/**
	 * @monitor_mode: Whether the device is operating in monitor mode.
	 */
	bool monitor_mode;

	/**
	 * @cfg: hardware config to be used. cfg must be set before initiating any bus operations
	 */
	struct morse_hw_cfg *cfg;
	const struct morse_bus_ops *bus_ops;

	/* Work queue used by code copying between Linux and local buffers */
	struct workqueue_struct *net_wq;

	/* Work items responsible for HW recovery */
	struct {
		/** @hw_stop: Scheduled on HW IRQ MORSE_INT_HW_STOP_NOTIFICATION */
		struct work_struct hw_stop;
		/** @driver_restart: Scheduled by hw_stop work to restart and recover HW */
		struct work_struct driver_restart;
		/** @bus_reset: Scheduled upon failure to restart during normal recovery flow */
		struct work_struct bus_reset;
		/** @health_check: Work is periodically scheduled to determine HW accesibility */
		struct work_struct health_check;
		/** @is_ready: true once restart recovery flow is available for use */
		bool is_ready;
		/** @ignore_detach: ignore detach logic on driver removal */
		bool ignore_detach;
	} recovery;

	/** @tx_stale_work: Work to periodically recover / flush outstanding TX SKBs */
	struct work_struct tx_stale_work;

	struct morse_debug debug;

	char *board_serial;
	int board_id;

	struct {
		int count;
		char (*list)[3];
	} regdoms;

	/* Stored Channel Information, sta_type, enc_mode, RAW */
	struct morse_custom_configs custom_configs;

	/* Stored configurations per VIF */
	struct {
		/* Persistent configurations for different features */
		struct morse_persistent_vif_configs configs[MORSE_MAX_IF];
		/* Lock held for the VIF config array */
		struct mutex lock;
	} persistent_vif_config;

	/* watchdog */
	struct morse_watchdog watchdog;

	/** Multicast filter list */
	struct mcast_filter *mcast_filter;

	/* reset stats */
	unsigned int restart_counter;

	/** Extra timeout applied to wait for ctrl-resp frames */
	int extra_ack_timeout_us;

	/** Num of beacon enabled VIFs (AP/Mesh/IBSS) */
	atomic_t num_bcn_vifs;

	/** Current Duty Cycle in 100ths of a percent. E.g. 10000 = 100% */
	u32 duty_cycle;

	struct {
		/* read from the FW at runtime, used for coredump metadata filling */
		const char *fw_ver_str;
		/* firmware binary name, used for coredump metadata filling */
		const char *fw_binary_str;
		/* coredump crash info */
		struct morse_coredump_data crash;
		/* lock for accessing / modifying crash data */
		struct mutex lock;
	} coredump;

	/* Kernel time of last HW stop event */
	time64_t last_hw_stop;

	/* Number of AP interfaces */
	u8 num_of_ap_interfaces;

	/** Tracking of STAs yet to join the BSS (if ap-type interfaces are active) */
	struct {
		/** See @ref morse_pre_assoc_peer */
		struct list_head list;
		/** Protect access to list */
		spinlock_t lock;
		/**
		 * A counter tracking the number of ifaces using this list
		 * (used for station record clearing).
		 */
		int n_ifaces_using;
	} pre_assoc_peers;

	struct {
		/* Used to protect modification of clock update completion */
		struct mutex update_wait_lock;
		/* Used to wait on clock updates */
		struct completion *update;
		/* RCU protected clock reference. Do not access directly, go through
		 * morse_hw_clock_xxx API.
		 */
		struct morse_hw_clock __rcu *clock;
	} hw_clock;

	struct {
		enum headless_state {
			HEADLESS_OFF,
			HEADLESS_ON_REQUEST,
			HEADLESS_ON_PENDING,
			HEADLESS_ON,
			HEADLESS_OFF_REQUEST,
			HEADLESS_OFF_PENDING
		} state;
		/* Pending configuration to write into hw upon detach */
		u32 pending_cfg;
		/* Completion to signal headless attach / detach (from cmd_q context) */
		struct completion *wait;
		/* Work items are executed on the Chip WQ context */
		struct work_struct work;
		/* Used to protect headless state update */
		struct mutex lock;
	} headless;

	struct {
		/* List of MM-SKB + probe response cache entries */
		struct sk_buff_head cache;
		/* Enabled state is initialised once on boot */
		bool is_enabled;
	} scan_res_cache;

	/* must be last */
	u8 drv_priv[] __aligned(sizeof(void *));

};

/* Map from mac80211 queue to Morse ACI value for page metadata */
static inline u8 map_mac80211q_2_morse_aci(u16 mac80211queue)
{
	switch (mac80211queue) {
	case IEEE80211_AC_VO:
		return MORSE_ACI_VO;
	case IEEE80211_AC_VI:
		return MORSE_ACI_VI;
	case IEEE80211_AC_BK:
		return MORSE_ACI_BK;
	default:
		return MORSE_ACI_BE;
	}
}

/**
 * Convert dot11 traffic ID (TID) to WMM access category (AC)
 *
 * @param TID 4-bit TID value
 *
 * @return QoS AC index
 */
static inline enum morse_page_aci dot11_tid_to_ac(enum qos_tid_up_index tid)
{
	switch (tid) {
	case MORSE_QOS_TID_UP_BK:
	case MORSE_QOS_TID_UP_xx:
		return MORSE_ACI_BK;
	case MORSE_QOS_TID_UP_CL:
	case MORSE_QOS_TID_UP_VI:
		return MORSE_ACI_VI;
	case MORSE_QOS_TID_UP_VO:
	case MORSE_QOS_TID_UP_NC:
		return MORSE_ACI_VO;
	case MORSE_QOS_TID_UP_BE:
	case MORSE_QOS_TID_UP_EE:
	default:
		return MORSE_ACI_BE;
	}
}

#ifdef CONFIG_MORSE_SDIO
int __init morse_sdio_init(void);
void __exit morse_sdio_exit(void);
#endif

#ifdef CONFIG_MORSE_SPI
int __init morse_spi_init(void);
void __exit morse_spi_exit(void);
#endif

#ifdef CONFIG_MORSE_USB
int __init morse_usb_init(void);
void __exit morse_usb_exit(void);
#endif

static inline bool morse_is_data_tx_allowed(struct morse *mors)
{
	return !test_bit(MORSE_STATE_FLAG_DATA_TX_STOPPED, &mors->state_flags) &&
	    !test_bit(MORSE_DATA_TRAFFIC_PAUSE_PEND, &mors->chip_if->event_flags);
}

static inline struct ieee80211_vif *morse_vif_to_ieee80211_vif(struct morse_vif *mors_vif)
{
	return container_of((void *)mors_vif, struct ieee80211_vif, drv_priv);
}

static inline struct morse_vif *ieee80211_vif_to_morse_vif(struct ieee80211_vif *vif)
{
	return (struct morse_vif *)vif->drv_priv;
}

static inline struct morse *morse_vif_to_morse(struct morse_vif *mors_vif)
{
	return container_of(mors_vif->custom_configs, struct morse, custom_configs);
}

static inline struct ieee80211_sta *morse_sta_to_ieee80211_sta(struct morse_sta *msta)
{
	return container_of((void *)msta, struct ieee80211_sta, drv_priv);
}

static inline struct morse_persistent_vif_configs *morse_get_vif_conf_from_id(struct morse *mors,
		int vif_id)
{
	return &mors->persistent_vif_config.configs[vif_id];
}

static inline bool morse_test_mode_enabled(uint test_mode)
{
	if (test_mode == MORSE_CONFIG_TEST_MODE_DISABLED ||
	    test_mode == MORSE_CONFIG_TEST_MODE_DOWNLOAD_AND_GET_HOST_TBL_PTR ||
	    test_mode == MORSE_CONFIG_TEST_MODE_GET_HOST_TBL_PTR_ONLY)
		return false;

	return true;
}

/**
 * @brief Get CSSID from SSID/MESH ID and its length
 *
 * @param ssid SSID/MESH ID pointer
 * @param len Length of SSID/MESH ID
 *
 * @returns Derived CSSID
 */
static inline u32 morse_generate_cssid(const u8 *ssid, u8 len)
{
	return ~crc32(~0, ssid, len);
}

/**
 * mac2uint64 - Convert a MAC address to an integer
 *
 * @bssid: MAC address
 *
 * @return An unsigned integer representing the MAC address
 */
static inline u64 mac2uint64(const u8 *bssid)
{
	return ((u64)(bssid[0]) << 40) | ((u64)(bssid[1]) << 32) | ((u64)(bssid[2]) << 24) |
	       ((u64)(bssid[3]) << 16) | ((u64)(bssid[4]) << 8) | ((u64)(bssid[5]));
}

static inline bool is_assoc_frame(__le16 frame_control)
{
	return (ieee80211_is_assoc_req(frame_control) ||
			ieee80211_is_reassoc_req(frame_control) ||
			ieee80211_is_assoc_resp(frame_control) ||
			ieee80211_is_reassoc_resp(frame_control));
}

int morse_beacon_init(struct morse_vif *mors_vif);
void morse_beacon_finish(struct morse_vif *mors_vif);
void morse_beacon_irq_handle(struct morse *mors, u32 status);

/**
 * morse_ndp_probe_req_rx - Generate a probe response in response to an NDP probe request
 *
 * @mors:	Global morse struct
 * @vif:	The vif that the probe was received on
 * @bw:		Channel BW that the probe was received on
 * @is_pv1:	Whether the frame is PV1 or PV0
 */
int morse_ndp_probe_req_rx(struct morse *mors, struct ieee80211_vif *vif,
					u8 bw, u8 is_pv1);
int morse_send_probe_req_enable(struct ieee80211_vif *vif, bool enable);
int morse_send_probe_req_init(struct ieee80211_vif *vif);
void morse_send_probe_req_finish(struct ieee80211_vif *vif);
void morse_mac_schedule_probe_req(struct ieee80211_vif *vif);

/**
 * morse_mac_is_s1g_long_beacon - Checks if the given beacon is long(TBTT) or not.
 *
 * @note: A short beacon won’t have a S1G Beacon Compatibility element according to the spec,
 * so if the beacon contains S1G Beacon Compatibility IE then it is long Beacon. Please
 * check https://morsemicro.atlassian.net/browse/SW-10802 for more details.
 *
 * @mors: pointer to morse struct
 * @skb: pointer to beacon buffer
 *
 * Return: True if the s1g beacon contains beacon compatibility IE.
 */
bool morse_mac_is_s1g_long_beacon(struct morse *mors, struct sk_buff *skb);

/**
 * morse_usb_ndr_reset - Performs non-destructive reset through Morse USB.
 *
 * @note: A non-destructive reset is the same as a digital reset except that USB
 * connection is maintained, meaning that re-enumeration is not required.
 * USB must be already enumerated and able to communicate as this sends a custom
 * USB message to the chip.
 *
 * @mors: Global morse struct
 *
 * Return: 0 on success, else error code
 */
int morse_usb_ndr_reset(struct morse *mors);
int morse_survey_add_channel_usage(struct morse *mors, struct morse_survey_rx_usage_record *record);
int morse_survey_init_usage_records(struct morse *mors);

#endif	/* !_MORSE_MORSE_H_ */
