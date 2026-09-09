/*
 * Copyright 2023 Morse Micro
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include <linux/timer.h>
#include <linux/bitfield.h>

#include "morse.h"
#include "mac.h"
#include "mesh.h"
#include "beacon.h"
#include "vendor.h"
#include "debug.h"

/**
 * Periodic Time to trigger probe req in Mesh mode for discovery of new neighbor peers
 */
#define MESH_DISCOVERY_PROBE_PERIOD_S (60)
/**
 * Initial delay to trigger probe req in Mesh mode for discovery of new neighbor peers
 */
#define MESH_INITIAL_DISCOVERY_PROBE_DELAY_S (5)

/**
 * If the RSSI of new peer is better than existing weak(lowest RSSI) peer by RSSI margin
 * then mesh STA will kick out the peer with lowest RSSI.
 */
#define DEFAULT_MESH_RSSI_MARGIN	5

/** Duration in seconds, a kicked out peer is blacklisted */
#define DEFAULT_MESH_BLACKLIST_TIMEOUT 30

/** Inject +/- 20% jitter for probe request interval */
#define MESH_PROBE_JITTER_PCT		(20)

/** Mesh Beaconless mode probe resp delay in msecs */
#define MESH_BCNLESS_PROBE_RESP_DELAY_MS	(50)

/** Context to transmit the delayed probe response */
struct morse_mesh_delayed_tx {
	/** Work item scheduled to transmit */
	struct delayed_work work;

	/** Pointer to morse struct */
	struct morse *mors;

	/** Cloned SKB */
	struct sk_buff *skb;

	/** Pointer to the associated morse_vif */
	struct morse_vif *mors_vif;

	/** List node in vif's delayed_tx_list */
	struct list_head node;

	/** reference count */
	refcount_t ref;
};

static int jitter_delay(int base_ms)
{
	u32 jitter = base_ms * MESH_PROBE_JITTER_PCT / 100;

	return base_ms + (int)morse_random_u32_max(2 * jitter + 1) - (int)jitter;
}

static void morse_mesh_delayed_tx_release(struct morse_mesh_delayed_tx *ctx)
{
	if (ctx->skb)
		dev_kfree_skb_any(ctx->skb);
	kfree(ctx);
}

static void morse_mesh_delayed_tx_get(struct morse_mesh_delayed_tx *ctx)
{
	refcount_inc(&ctx->ref);
}

static void morse_mesh_delayed_tx_put(struct morse_mesh_delayed_tx *ctx)
{
	if (refcount_dec_and_test(&ctx->ref))
		morse_mesh_delayed_tx_release(ctx);
}

static void morse_schedule_mesh_probe_timer(struct morse_mesh *mesh, int delay)
{
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(jitter_delay(delay * 1000));

	mod_timer(&mesh->mesh_probe_timer, timeout);
}

int morse_cmd_process_mbca_conf(struct morse_vif *mors_vif,
				struct morse_cmd_req_set_mcba_conf *mbca)
{
	struct morse_mesh_config *mesh_conf;

	if (!mors_vif || !mors_vif->mesh || !mors_vif->mesh->conf)
		return -EFAULT;

	mesh_conf = mors_vif->mesh->conf;
	mesh_conf->mbca.config = mbca->mbca_config;
	mesh_conf->mbca.beacon_timing_report_interval = mbca->beacon_timing_report_interval;
	mesh_conf->mbca.min_beacon_gap_ms = mbca->min_beacon_gap_ms;
	mesh_conf->mbca.tbtt_adj_interval_ms = le16_to_cpu(mbca->tbtt_adj_interval_ms);
	mesh_conf->mbca.mbss_start_scan_duration_ms =
				le16_to_cpu(mbca->mbss_start_scan_duration_ms);

	return 0;
}

int morse_cmd_process_dynamic_peering_conf(struct morse_vif *mors_vif,
					   struct morse_cmd_req_dynamic_peering_config *req)
{
	struct morse *mors;
	struct morse_mesh_config *mesh_conf;

	if (!mors_vif || !mors_vif->mesh || !mors_vif->mesh->conf)
		return -EFAULT;

	mors = morse_vif_to_morse(mors_vif);

	mesh_conf = mors_vif->mesh->conf;
	mesh_conf->dynamic_peering = req->enabled;
	mesh_conf->rssi_margin = req->rssi_margin;
	mesh_conf->blacklist_timeout = le32_to_cpu(req->blacklist_timeout);

	MORSE_MESH_INFO(mors, "dynamic_peering=%u, rssi_margin=%u, timeout=%u\n",
			mesh_conf->dynamic_peering, mesh_conf->rssi_margin,
			mesh_conf->blacklist_timeout);

	return 0;
}

int morse_cmd_cfg_mesh_bss(struct morse_vif *mors_vif, bool stop_mesh)
{
	struct morse_mesh_config *mesh_conf;
	struct morse *mors = morse_vif_to_morse(mors_vif);
	int ret;

	if (!mors_vif->mesh || !mors_vif->mesh->conf)
		return -ENXIO;

	mesh_conf = mors_vif->mesh->conf;

	/* Reset MBCA configuration parameter if beaconing is not enabled */
	if (mesh_conf->mesh_beaconless_mode)
		mesh_conf->mbca.config = 0;

	ret = morse_cmd_cfg_mesh(mors, mors_vif, stop_mesh, !mesh_conf->mesh_beaconless_mode);
	if (!ret)
		MORSE_MESH_INFO(mors, "%s: beaconless:%u stop:%u mbca.config:0x%02x\n",
				__func__, mesh_conf->mesh_beaconless_mode, stop_mesh,
				mesh_conf->mbca.config);

	mesh_conf->mbca.beacon_count = 0;

	return ret;
}

int morse_cmd_set_mesh_config(struct morse_vif *mors_vif,
			      struct morse_cmd_req_set_mesh_config *mesh_req,
				  bool in_reconfig)
{
	struct ieee80211_vif *vif;
	struct morse_mesh *mesh;
	struct morse_mesh_config *mesh_config;

	if (!mors_vif)
		return -EFAULT;

	vif = morse_vif_to_ieee80211_vif(mors_vif);
	mesh = mors_vif->mesh;

	mesh_config = mors_vif->mesh->conf;
	if (!mesh_config)
		return -ENXIO;

	if (!ieee80211_vif_is_mesh(vif) ||
	    (mesh_config->is_mesh_active && !in_reconfig) ||
	    (!mesh_config->is_mesh_active && in_reconfig))
		return -ENOENT;

	if (mesh_req) {
		if (mesh_req->mesh_id_len  > IEEE80211_MAX_SSID_LEN)
			return -EINVAL;

		memcpy(mesh_config->mesh_id, mesh_req->mesh_id, mesh_req->mesh_id_len);
		mesh_config->mesh_id_len = mesh_req->mesh_id_len;
		mesh_config->mesh_beaconless_mode = mesh_req->mesh_beaconless_mode;
		mesh_config->max_plinks = mesh_req->max_plinks;
	}

	if (morse_cmd_cfg_mesh_bss(mors_vif, false))
		return -EPERM;

	if (mesh_config->mesh_beaconless_mode)
		morse_schedule_mesh_probe_timer(mesh, 0);
	mesh_config->is_mesh_active = true;

	return 0;
}

#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
static void morse_mesh_probe_timer_cb(unsigned long addr)
#else
static void morse_mesh_probe_timer_cb(struct timer_list *t)
#endif
{
#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
	struct morse_mesh *mesh = (struct morse_mesh *)addr;
#else
	struct morse_mesh *mesh = TIMER_TO_OBJ(mesh, t, mesh_probe_timer);
#endif
	struct morse_vif *mors_vif = mesh->mors_vif;
	struct ieee80211_vif *vif;
	u8 bcast_addr[ETH_ALEN];
	u32 next_probe_delay;

	if (!mors_vif) {
		pr_info("Mesh probe timer: ERROR! mors_vif NULL\n");
		MORSE_WARN_ON(FEATURE_ID_MESH, 1);
		return;
	}
	vif = morse_vif_to_ieee80211_vif(mors_vif);

	if (!ieee80211_vif_is_mesh(vif))
		return;

	eth_broadcast_addr(bcast_addr);
	morse_mac_tx_mesh_probe_req(mors_vif, bcast_addr);

	if (mors_vif->ap && mors_vif->ap->num_stas)
		next_probe_delay = MESH_DISCOVERY_PROBE_PERIOD_S;
	else
		next_probe_delay = MESH_INITIAL_DISCOVERY_PROBE_DELAY_S;

	morse_schedule_mesh_probe_timer(mesh, next_probe_delay);
}

int morse_mac_tx_mesh_probe_req(struct morse_vif *mors_vif, const u8 *dest_addr)
{
	struct ieee80211_vif *vif;
	struct morse *mors;
	struct morse_mesh_config *mesh_conf;
	struct sk_buff *skb = NULL;
	struct ieee80211_hdr *hdr;

	if (!mors_vif)
		return -EFAULT;

	vif = morse_vif_to_ieee80211_vif(mors_vif);
	mors = morse_vif_to_morse(mors_vif);

	if (!ieee80211_vif_is_mesh(vif) || !mors_vif->mesh ||
	    !mors_vif->mesh->conf || !mors_vif->mesh->conf->mesh_id_len) {
		MORSE_MESH_ERR(mors, "Failed to send mesh probe req\n");
		return -ENOENT;
	}

	mesh_conf = mors_vif->mesh->conf;
	skb = ieee80211_probereq_get(mors->hw, vif->addr, mesh_conf->mesh_id,
			mesh_conf->mesh_id_len, 0);
	if (!skb) {
		MORSE_MESH_ERR(mors, "Failed to allocate mesh probe req\n");
		return -ENOMEM;
	}
	hdr = (struct ieee80211_hdr *)skb->data;
	memcpy(hdr->addr1, dest_addr, ETH_ALEN);

	if (morse_mac_tx_mgmt_frame(vif, skb)) {
		MORSE_MESH_ERR(mors, "Failed to send mesh probe req\n");
		dev_kfree_skb_any(skb);
		return -EPERM;
	}

	return 0;
}

static int morse_mesh_fill_probe_resp(struct morse_mesh *mesh)
{
	struct morse_vif *mors_if = mesh->mors_vif;
	struct morse *mors = morse_vif_to_morse(mors_if);
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_if);
	struct sk_buff *probe_resp;
	struct ieee80211_mgmt *hdr;

	probe_resp = MORSE_IEEE_BEACON_GET(mors, vif);
	if (!probe_resp) {
		MORSE_MESH_ERR(mors, "%s ieee80211_beacon_get failed\n", __func__);
		return -ENOMEM;
	}

	hdr = (struct ieee80211_mgmt *)probe_resp->data;
	hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					 IEEE80211_STYPE_PROBE_RESP);
	memset(hdr->da, 0, sizeof(hdr->da));
	mesh->mesh_probe_resp_skb = probe_resp;

	return 0;
}

/**
 * morse_mesh_generate_probe_resp() - Simulate a Rx probe resp frame from a peer.
 * In beacon-less mode, both peers will need to have information about each other peer.
 *
 * @mesh: pointer to mesh interface
 * @src_addr: peer mac address from which probe resp to simulate
 *
 * Return: 0 on success and error code on failure
 */
static int morse_mesh_generate_probe_resp(struct morse_mesh *mesh, const u8 *src_addr)
{
	struct ieee80211_mgmt *mgt_probe_resp;
	struct morse_vif *mors_if = mesh->mors_vif;
	struct morse *mors = morse_vif_to_morse(mors_if);
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_if);
	struct sk_buff *skb_probe_resp;
	struct ieee80211_rx_status *rx_status;

	rx_status = &mesh->probe_rx_status;
	if (!mesh->mesh_probe_resp_skb &&
	    morse_mesh_fill_probe_resp(mesh)) {
		MORSE_MESH_ERR(mors, "%s: Don't have probe resp populated yet\n", __func__);
		return -EPERM;
	}
	skb_probe_resp = skb_copy(mesh->mesh_probe_resp_skb, GFP_ATOMIC);

	if (!skb_probe_resp) {
		MORSE_MESH_ERR(mors, "%s: SKB for probe resp failed\n", __func__);
		return -ENOMEM;
	}
	mgt_probe_resp = (struct ieee80211_mgmt *)skb_probe_resp->data;
	memcpy(IEEE80211_SKB_RXCB(skb_probe_resp), rx_status, sizeof(*rx_status));
	memcpy(mgt_probe_resp->sa, src_addr, ETH_ALEN);
	memcpy(mgt_probe_resp->bssid, src_addr, ETH_ALEN);
	memcpy(mgt_probe_resp->da, vif->addr, ETH_ALEN);

	if (skb_probe_resp->len > 0 && rx_status->band == NL80211_BAND_5GHZ) {
		MORSE_MESH_DBG(mors, "%s: Indicating SKB for probe resp\n", __func__);
		ieee80211_rx_irqsafe(mors->hw, skb_probe_resp);
	} else {
		MORSE_MESH_ERR(mors, "%s: Failed to simulate Mesh probe resp band:%d len:%d\n",
						__func__, rx_status->band, skb_probe_resp->len);
		dev_kfree_skb_any(skb_probe_resp);
		return -EPERM;
	}

	return 0;
}

static int process_mesh_rx_mgmt_beaconless(struct morse_vif *mors_vif,
					struct sk_buff *skb,
					struct dot11ah_ies_mask *ies_mask,
					const struct ieee80211_rx_status *rx_status)
{
	struct ieee80211_vif *vif;
	struct ie_element *mesh_id_ie = &ies_mask->ies[WLAN_EID_MESH_ID];
	struct morse_mesh *mesh;
	struct morse_mesh_config *mesh_conf;
	struct morse *mors;
	struct ieee80211_sta *sta;

	struct ieee80211_mgmt *mgmt;
	struct ieee80211_hdr *hdr;
	const u8 *src_addr;
	bool is_mesh_auth_frame;
	int auth_alg;
	__le16 fctl;
	u16 stype;

	if (!mors_vif)
		return -EFAULT;

	vif = morse_vif_to_ieee80211_vif(mors_vif);
	mors = morse_vif_to_morse(mors_vif);
	mesh = mors_vif->mesh;

	if (!ieee80211_vif_is_mesh(vif))
		return -ENOENT;

	if (!mesh || !mesh->conf)
		return -ENXIO;

	mesh_conf = mesh->conf;

	if (!ieee80211_vif_is_mesh(vif) || !mesh_conf->is_mesh_active)
		return -ENOENT;

	hdr = (struct ieee80211_hdr *)skb->data;
	src_addr = hdr->addr2;
	fctl = hdr->frame_control;
	mgmt = (struct ieee80211_mgmt *)skb->data;
	auth_alg = le16_to_cpu(mgmt->u.auth.auth_alg);
	stype = le16_to_cpu(mgmt->frame_control) & IEEE80211_FCTL_STYPE;
	is_mesh_auth_frame = ieee80211_is_auth(fctl) && (auth_alg == WLAN_AUTH_SAE);

	rcu_read_lock();
	sta = ieee80211_find_sta_by_ifaddr(mors->hw, src_addr, vif->addr);

	switch (stype) {
	case IEEE80211_STYPE_PROBE_REQ:
		/* known peer. drop discovery probe req */
		if (sta) {
			rcu_read_unlock();
			return -EACCES;
		}

		/* Only present in CSSID list, allow to pass up */
		if (morse_dot11ah_is_mesh_peer_known(src_addr)) {
			rcu_read_unlock();
			return 0;
		}
		if (mesh_id_ie->len != 0 && (mesh_id_ie->len == mesh_conf->mesh_id_len &&
		    !memcmp(mesh_id_ie->ptr, mesh_conf->mesh_id, mesh_conf->mesh_id_len)))
			memcpy(&mesh->probe_rx_status, rx_status, sizeof(mesh->probe_rx_status));
		break;
	case IEEE80211_STYPE_AUTH:
		if (!sta && is_mesh_auth_frame) {
			/* Peer not known yet. Indicate simulated probe resp and drop this frame
			 * Peer will retry the auth frame and by then supplicant will have peer info
			 */
			MORSE_MESH_ERR(mors, "%s: Rx of Mesh Auth from unknown peer [%pM]\n",
				__func__, src_addr);
			memcpy(&mesh->probe_rx_status, rx_status, sizeof(mesh->probe_rx_status));
			morse_mesh_generate_probe_resp(mesh, src_addr);
			rcu_read_unlock();
			return -EACCES;
		}
		break;
	}
	rcu_read_unlock();
	return 0;
}

int morse_mac_add_meshid_ie(struct morse_vif *mors_vif, struct sk_buff *skb,
			    struct dot11ah_ies_mask *ies_mask)
{
	u8 mesh_id[IEEE80211_MAX_SSID_LEN];
	struct ie_element *ssid_ie = &ies_mask->ies[WLAN_EID_SSID];
	struct ie_element *mesh_id_ie = &ies_mask->ies[WLAN_EID_MESH_ID];
	struct morse_mesh *mesh;
	struct morse_mesh_config *mesh_conf;

	if (!mors_vif || !ies_mask)
		return -EFAULT;

	mesh = mors_vif->mesh;

	if (!mesh || !mesh->conf || !mesh->conf->mesh_id_len)
		return -1;

	mesh_conf = mesh->conf;
	memcpy(mesh_id, mesh_conf->mesh_id, mesh_conf->mesh_id_len);

	mesh_id_ie->ptr = NULL;

	morse_dot11ah_insert_element(ies_mask, WLAN_EID_MESH_ID, (u8 *)mesh_id,
			mesh_conf->mesh_id_len);

	ssid_ie->len = 0;

	return 0;
}

static void morse_mesh_delayed_probe_tx_worker(struct work_struct *work)
{
	struct morse_mesh_delayed_tx *ctx =
		container_of(to_delayed_work(work), struct morse_mesh_delayed_tx, work);
	struct morse *mors;
	struct morse_vif *mors_vif;
	struct morse_skbq *mq;
	struct sk_buff *skb;
	struct morse_skb_tx_info tx_info = { 0 };
	struct ieee80211_sta *sta = NULL;
	struct ieee80211_vif *vif;
	struct ieee80211_mgmt *mgmt;
	struct morse_mesh *mesh;
	u8 tx_bw_mhz;

	mors = ctx->mors;
	skb = ctx->skb;
	mors_vif = ctx->mors_vif;

	if (!skb || !mors || !mors_vif)
		goto out;

	vif = morse_vif_to_ieee80211_vif(mors_vif);
	mgmt = (struct ieee80211_mgmt *)skb->data;
	mesh = mors_vif->mesh;
	if (!mesh)
		goto out;

	mq = mors->cfg->ops->skbq_mgmt_tc_q(mors);
	tx_bw_mhz = mors->custom_configs.channel_info.pri_bw_mhz;
	rcu_read_lock();
	sta = ieee80211_find_sta_by_ifaddr(mors->hw, mgmt->da, vif->addr);
	morse_mac_fill_tx_info(mors, &tx_info, skb, vif, tx_bw_mhz, sta);
	rcu_read_unlock();

	spin_lock_bh(&mesh->delayed_tx_lock);
	list_del_init(&ctx->node);
	spin_unlock_bh(&mesh->delayed_tx_lock);

	morse_skbq_skb_tx(mq, &skb, &tx_info, MORSE_SKB_CHAN_MGMT);
	/* skb is now owned by tx path or freed on error */
	ctx->skb = NULL;

out:
	morse_mesh_delayed_tx_put(ctx);
}

int morse_mesh_bcnless_delay_probe_resp(struct morse_vif *mors_vif,
					struct sk_buff *skb)
{
	struct morse_mesh_delayed_tx *ctx;
	struct morse *mors = morse_vif_to_morse(mors_vif);
	struct morse_mesh *mesh;
	struct sk_buff *cloned;
	u32 delay;

	cloned = skb_copy(skb, GFP_ATOMIC);
	if (!cloned) {
		MORSE_MESH_ERR(mors, "Failed to clone mesh beaconless probe resp\n");
		return 0;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		MORSE_MESH_ERR(mors, "Failed to alloc mesh beaconless delayed ctx\n");
		dev_kfree_skb_any(cloned);
		return 0;
	}

	ctx->mors = mors;
	ctx->skb = cloned;
	ctx->mors_vif = mors_vif;
	mesh = mors_vif->mesh;
	INIT_LIST_HEAD(&ctx->node);
	refcount_set(&ctx->ref, 1);
	INIT_DELAYED_WORK(&ctx->work, morse_mesh_delayed_probe_tx_worker);

	spin_lock_bh(&mesh->delayed_tx_lock);
	list_add_tail(&ctx->node, &mesh->delayed_tx_list);
	spin_unlock_bh(&mesh->delayed_tx_lock);
	delay = morse_random_u32_max(MESH_BCNLESS_PROBE_RESP_DELAY_MS);
	queue_delayed_work(system_wq, &ctx->work, msecs_to_jiffies(delay));

	return -EACCES;
}

int morse_mac_process_mesh_tx_mgmt(struct morse_vif *mors_vif,
				   struct sk_buff *skb, struct dot11ah_ies_mask *ies_mask)
{
	struct ieee80211_hdr *hdr;
	struct morse_mesh *mesh;
	struct morse_mesh_config *mesh_conf;

	if (!mors_vif || !skb || !ies_mask)
		return -EFAULT;

	hdr = (struct ieee80211_hdr *)skb->data;
	mesh = mors_vif->mesh;

	if (!mesh || !mesh->conf)
		return -ENXIO;

	mesh_conf = mesh->conf;

	if (ieee80211_is_probe_resp(hdr->frame_control)) {
		/* Probe resp processing */
		if (mesh_conf->mesh_beaconless_mode) {
			struct ieee80211_mgmt *mgt_probe_resp = (struct ieee80211_mgmt *)skb->data;

			morse_mesh_generate_probe_resp(mesh, hdr->addr1);
			/* Add this mesh peer into cssid list */
			morse_dot11ah_add_mesh_peer(ies_mask,
					le16_to_cpu(mgt_probe_resp->u.probe_resp.capab_info),
					hdr->addr1);
		} else if (mesh_conf->mbca.config != 0) {
			u8 *ptr = ies_mask->ies[WLAN_EID_MESH_CONFIG].ptr;

			/* Enable MBCA Capability */
			if (ptr)
				morse_enable_mbca_capability(ptr);
		}
	} else if (ieee80211_is_probe_req(hdr->frame_control)) {
		/* Add Mesh ID to probe req in beaconless mode */
		if (mesh_conf->mesh_beaconless_mode)
			morse_mac_add_meshid_ie(mors_vif, skb, ies_mask);
	}

	return 0;
}

struct lowest_peer_rssi_iter {
	const struct ieee80211_vif *on_vif;
	bool is_set;
	s16 rssi;
	u8 peer[ETH_ALEN];
};

static void peer_with_lowest_rssi(void *data, struct ieee80211_sta *sta)
{
	struct lowest_peer_rssi_iter *iter_data = data;
	struct morse_vif *mors_vif =
		ieee80211_vif_to_morse_vif((struct ieee80211_vif *)iter_data->on_vif);
	const struct morse *mors = morse_vif_to_morse(mors_vif);
	struct morse_sta *msta = (struct morse_sta *)sta->drv_priv;

	if (!msta) {
		MORSE_WARN_ON(FEATURE_ID_MESH, 1);
		return;
	}

	if (msta->vif != iter_data->on_vif)
		return;

	MORSE_MESH_DBG(mors, "msta %pM with rssi %d and peerings=%u\n",
		       msta->addr, msta->avg_rssi, msta->mesh_no_of_peerings);

	/* Ignore if number of peerings is 1 */
	if (msta->mesh_no_of_peerings == 1)
		return;

	if (!iter_data->is_set || iter_data->rssi > msta->avg_rssi) {
		iter_data->is_set = true;
		iter_data->rssi = msta->avg_rssi;
		memcpy(iter_data->peer, msta->addr, sizeof(iter_data->peer));
	}
}

/**
 * morse_mac_check_for_dynamic_peering() - Checks if a link can be established with the
 * new peer by kicking out one of existing peer (with low signal strength).
 *
 * @mors_vif: pointer to morse interface
 * @sa: address of the new peer
 * @rssi: RSSI of the new peer
 * @ies_mask: pointer to information elements.
 */
static void morse_mac_check_for_dynamic_peering(struct morse_vif *mors_vif, u8 *sa,
						s16 rssi, struct dot11ah_ies_mask *ies_mask)
{
	struct morse_mesh *mesh = mors_vif->mesh;
	struct morse *mors = morse_vif_to_morse(mors_vif);
	struct ie_element *mesh_id_ie = &ies_mask->ies[WLAN_EID_MESH_ID];
	struct ie_element *mesh_conf_ie = &ies_mask->ies[WLAN_EID_MESH_CONFIG];
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_vif);
	struct morse_mesh_config *mesh_conf;
	struct lowest_peer_rssi_iter data = {
		.is_set = false,
		.on_vif = vif
	};
	bool accept_additional_peer;

	if (!mesh || !mesh->conf)
		return;

	mesh_conf = mesh->conf;

	/* Check if number of peers reached the limit */
	if (mors_vif->ap && mors_vif->ap->num_stas < mesh_conf->max_plinks)
		return;

	/* process frames from same mesh BSS */
	if (!mesh_id_ie || mesh_id_ie->len != mesh_conf->mesh_id_len ||
	    memcmp(mesh_id_ie->ptr, mesh_conf->mesh_id, mesh_id_ie->len)) {
		return;
	}

	/* Ignore the frame if mesh config element is not present */
	if (!mesh_conf_ie)
		return;

	accept_additional_peer =
	    !!(mesh_conf_ie->ptr[MESH_CONF_IE_CAPABILITY_FLAG_BYTE_OFFSET] &
	       MESH_CAP_ACCEPT_ADDITIONAL_PEER);

	/* Consider new peer only if it accepts additional peering */
	if (!accept_additional_peer)
		return;

	ieee80211_iterate_stations_atomic(mors->hw, peer_with_lowest_rssi, &data);

	/* Check if the new peer has better signal than existing peer */
	if (data.is_set && (data.rssi + mesh_conf->rssi_margin) < rssi) {
		struct morse_mesh_peer_addr_vendor_evt event;
		int ret;

		memcpy(event.addr, data.peer, ETH_ALEN);

		/* New peer has better rssi - indicate peer to supplicant to kick out */
		ret = morse_vendor_send_peer_addr_event(vif, &event);
		if (!ret) {
			memcpy(mesh->kickout_peer_addr, data.peer, ETH_ALEN);
			mesh->kickout_ts = jiffies;
		}
		MORSE_MESH_INFO(mors, "Kickout Peer %pM rssi %d, new peer %pM rssi %d, ret=%d\n",
				mesh->kickout_peer_addr, data.rssi, sa, rssi, ret);
	}
}

/**
 * morse_dynamic_peering_is_frame_allowed() - Checks if a given frame is allowed to
 * indicate to upper stack or not.
 *
 * @mesh: pointer to mesh interface
 * @mgmt: pointer to management frame
 *
 * Returns: True if the packet can be indicated to upper stack.
 */
static bool morse_dynamic_peering_is_frame_allowed(struct morse_mesh *mesh,
						   struct ieee80211_mgmt *mgmt)
{
	u16 stype = le16_to_cpu(mgmt->frame_control) & IEEE80211_FCTL_STYPE;
	bool allowed = true;

	switch (stype) {
	case IEEE80211_STYPE_S1G_BEACON:
	case IEEE80211_STYPE_PROBE_RESP:
		/* Drop beacon and probe response frames as it will trigger a new peering */
		allowed = false;
		break;
	case IEEE80211_STYPE_ACTION:
		if (mgmt->u.action.category == WLAN_ACTION_SELF_PROTECTED &&
		    !ieee80211_has_protected(mgmt->frame_control) &&
		    (MORSE_MPM_ACTION_CODE(mgmt) == PLINK_OPEN ||
		     MORSE_MPM_ACTION_CODE(mgmt) == PLINK_CONFIRM ||
		     MORSE_MPM_ACTION_CODE(mgmt) == PLINK_CLOSE)) {
			/* Drop peering open, confirm and close frames */
			allowed = false;
		}
		break;
	default:
		break;
	}
	return allowed;
}

int morse_mac_process_mesh_rx_mgmt(struct morse_vif *mors_vif, struct sk_buff *skb,
				   struct dot11ah_ies_mask *ies_mask,
				   const struct ieee80211_rx_status *rx_status)
{
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_vif);
	struct morse_mesh *mesh = mors_vif->mesh;
	struct morse *mors = morse_vif_to_morse(mors_vif);
	struct ieee80211_mgmt *mgmt = (struct ieee80211_mgmt *)skb->data;
	struct ieee80211_sta *sta;
	struct morse_mesh_config *mesh_conf;
	u8 *src_addr;
	u16 stype;

	if (!mesh || !mesh->conf || !mesh->conf->is_mesh_active)
		return -ENOENT;

	mesh_conf = mesh->conf;

	if (ieee80211_is_s1g_beacon(mgmt->frame_control))
		src_addr = mgmt->da;
	else
		src_addr = mgmt->sa;
	stype = le16_to_cpu(mgmt->frame_control) & IEEE80211_FCTL_STYPE;

	if (mesh_conf->mesh_beaconless_mode)
		return process_mesh_rx_mgmt_beaconless(mors_vif, skb, ies_mask, rx_status);

	if (mesh_conf->dynamic_peering &&
	    (ieee80211_is_s1g_beacon(mgmt->frame_control) ||
	     stype == IEEE80211_STYPE_ACTION || stype == IEEE80211_STYPE_PROBE_RESP)) {
		struct ie_element *mesh_conf_ie = &ies_mask->ies[WLAN_EID_MESH_CONFIG];
		u8 no_of_peerings = 0;

		/* check if timeout has expired */
		if (mesh->kickout_ts && (jiffies_to_msecs(jiffies - mesh->kickout_ts) >=
					 MORSE_SECS_TO_MSECS(mesh_conf->blacklist_timeout))) {
			MORSE_MESH_INFO(mors, "Reset blacklisted peer=%pM, kickout_ts=%lu\n",
				       mesh->kickout_peer_addr, mesh->kickout_ts);
			memset(mesh->kickout_peer_addr, 0, ETH_ALEN);
			mesh->kickout_ts = 0;
		}

		/* check if the frame is from kicked out peer */
		if (!memcmp(mesh->kickout_peer_addr, src_addr, ETH_ALEN) &&
		    !morse_dynamic_peering_is_frame_allowed(mesh, mgmt)) {
			return -EACCES;
		}

		if (mesh_conf_ie->ptr)
			no_of_peerings =
			    MESH_PARSE_NO_OF_PEERINGS(mesh_conf_ie->ptr
						      [MESH_CONF_IE_FORMATION_INFO_BYTE_OFFSET]);

		rcu_read_lock();
		sta = ieee80211_find_sta_by_ifaddr(mors->hw, src_addr, vif->addr);
		if (sta && mesh_conf_ie->ptr) {
			struct morse_sta *msta = (struct morse_sta *)sta->drv_priv;

			msta->mesh_no_of_peerings = no_of_peerings;
		}
		rcu_read_unlock();

		/* check for dynamic peering if the frame is from a new peer and
		 * not part of the network already. Also check if kick out period timed out.
		 */
		if (!sta && !mesh->kickout_ts && !no_of_peerings)
			morse_mac_check_for_dynamic_peering(mors_vif, src_addr, rx_status->signal,
							    ies_mask);
	}
	return 0;
}

void morse_mesh_reconfig(struct morse *mors, struct morse_vif *mors_vif)
{
	struct morse_mesh *mesh = mors_vif->mesh;
	struct morse_persistent_vif_configs *mors_vif_conf =
					morse_get_vif_conf_from_id(mors, mors_vif->id);
	struct morse_mesh_peer_addr_vendor_evt event;
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_vif);
	int ret;

	lockdep_assert_held(&mors->persistent_vif_config.lock);

	if (!mesh)
		return;

	mesh->conf = &mors_vif_conf->mesh_conf;
	morse_cmd_set_mesh_config(mors_vif, NULL, morse_mlme_in_reconfig(mors));

	eth_broadcast_addr(event.addr);

	ret = morse_vendor_send_peer_addr_event(vif, &event);
	MORSE_MESH_INFO(mors, "Close all Peer links %pM, ret=%d\n", event.addr, ret);
}

int morse_mesh_deinit(struct morse_vif *mors_vif)
{
	struct morse_mesh *mesh = mors_vif->mesh;
	struct morse_mesh_delayed_tx *ctx, *tmp;

	DEL_TIMER_SYNC(&mesh->mesh_probe_timer);
	if (mesh->mesh_probe_resp_skb)
		dev_kfree_skb_any(mesh->mesh_probe_resp_skb);

	spin_lock_bh(&mesh->delayed_tx_lock);
	list_for_each_entry_safe(ctx, tmp, &mesh->delayed_tx_list, node) {
		list_del_init(&ctx->node);
		morse_mesh_delayed_tx_get(ctx);
		spin_unlock_bh(&mesh->delayed_tx_lock);

		cancel_delayed_work_sync(&ctx->work);
		morse_mesh_delayed_tx_put(ctx);

		spin_lock_bh(&mesh->delayed_tx_lock);
	}
	spin_unlock_bh(&mesh->delayed_tx_lock);

	mors_vif->mesh->conf = NULL;
	kfree(mors_vif->mesh);
	mors_vif->mesh = NULL;

	return 0;
}

int morse_mesh_init(struct morse_vif *mors_vif, bool in_reconfig)
{
	struct morse_mesh *mesh;
	struct morse_mesh_config *mesh_conf;
	struct morse_persistent_vif_configs *mors_vif_conf;
	struct morse *mors = morse_vif_to_morse(mors_vif);
	struct ieee80211_vif *vif = morse_vif_to_ieee80211_vif(mors_vif);
	int ret = 0;

	mors_vif->mesh = kzalloc(sizeof(*mors_vif->mesh), GFP_KERNEL);
	if (!mors_vif->mesh)
		return -ENOMEM;

	mesh = mors_vif->mesh;
	mesh->mors_vif = mors_vif;
#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
	init_timer(&mesh->mesh_probe_timer);
	mesh->mesh_probe_timer.data = (unsigned long)mesh;
	mesh->mesh_probe_timer.function = morse_mesh_probe_timer_cb;
	add_timer(&mesh->mesh_probe_timer);
#else
	timer_setup(&mesh->mesh_probe_timer, morse_mesh_probe_timer_cb, 0);
#endif
	morse_set_max_skb_txq_len(MORSE_MESH_MAX_TXQ_LENGTH);

	INIT_LIST_HEAD(&mesh->delayed_tx_list);
	spin_lock_init(&mesh->delayed_tx_lock);

	mutex_lock(&mors->persistent_vif_config.lock);

	mors_vif_conf = morse_get_vif_conf_from_addr(mors, vif->addr);
	if (!mors_vif_conf) {
		ret = -ENXIO;
		goto exit;
	}

	mesh->conf = &mors_vif_conf->mesh_conf;

	if (in_reconfig)
		goto exit;

	mesh_conf = mesh->conf;
	mesh_conf->mesh_id_len = 0;
	/* Assign default values for Mesh Beacon Collision Avoidance configuration
	 * parameters. It will be overwritten by supplicant configuration before interface
	 * start.
	 */
	mesh_conf->mbca.config = MESH_MBCA_CFG_TBTT_SEL_ENABLE;
	mesh_conf->mbca.beacon_timing_report_interval = DEFAULT_MESH_BCN_TIMING_REPORT_INT;
	mesh_conf->mbca.mbss_start_scan_duration_ms = DEFAULT_MBSS_START_SCAN_DURATION_MS;
	mesh_conf->mbca.min_beacon_gap_ms = DEFAULT_MBCA_MIN_BEACON_GAP_MS;
	mesh_conf->mbca.tbtt_adj_interval_ms = DEFAULT_TBTT_ADJ_INTERVAL_MSEC;
	mesh_conf->dynamic_peering = DEFAULT_DYNAMIC_MESH_PEERING;
	mesh_conf->rssi_margin = DEFAULT_MESH_RSSI_MARGIN;
	mesh_conf->blacklist_timeout = DEFAULT_MESH_BLACKLIST_TIMEOUT;

exit:
	mutex_unlock(&mors->persistent_vif_config.lock);
	return ret;
}
