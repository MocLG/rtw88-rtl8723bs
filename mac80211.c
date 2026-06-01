// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* Copyright(c) 2018-2019  Realtek Corporation
 */

#include "main.h"
#include "sec.h"
#include "tx.h"
#include "fw.h"
#include "mac.h"
#include "coex.h"
#include "ps.h"
#include "reg.h"
#include "bf.h"
#include "debug.h"
#include "wow.h"
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
#include "sar.h"
#endif

#define RTW8723BS_JOIN_RETRY_LIMIT 0x30
#define RTW8723BS_AUTH_SYNC_WAIT_FALLBACK_MS 120
#define RTW8723BS_AUTH_SYNC_WAIT_MIN_MS 80
#define RTW8723BS_AUTH_SYNC_WAIT_MAX_MS 160
#define RTW8723BS_MGMT_DUMP_BYTES 192
#define RTW8723BS_ACK_PREAMBLE_SHORT BIT(7)
#define RTW8723BS_SHORT_SLOT_TIME 9
#define RTW8723BS_LONG_SLOT_TIME 20

static bool rtw8723bs_sdio(struct rtw_dev *rtwdev)
{
	return rtwdev->chip->id == RTW_CHIP_TYPE_8723B &&
	       rtw_hci_type(rtwdev) == RTW_HCI_TYPE_SDIO;
}

static bool rtw8723bs_mgd_prepare_skip_fresh_rfk(struct rtw_dev *rtwdev,
						 bool rfk_pending)
{
	struct rtw_hal *hal = &rtwdev->hal;

	if (!rtw8723bs_sdio(rtwdev) || !rfk_pending)
		return false;

	/* Staging runs 8723BS SDIO IQK during power-on. Its join path requests
	 * HW_VAR_DO_IQK before set_channel_bwmode(), but that flag is not
	 * consumed into another immediate IQK before auth. Keep the auth window
	 * on that power-on calibration result, then reapply the connect channel
	 * and PTA antenna state that staging does program before auth.
	 */
	rtw_set_channel(rtwdev);
	rtwdev->need_rfk = false;

	rtw_coex_connect_notify(rtwdev, COEX_ASSOCIATE_START);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: mgd_prepare_tx skip_fresh_rfk ch=%u bw=%u pri_idx=%u need_rfk=%d RXIGI_A=0x%08x RRSR2=0x%02x SLOT=0x%02x\n",
		 hal->current_channel, hal->current_band_width,
		 hal->current_primary_channel_index, rtwdev->need_rfk,
		 rtw_read32(rtwdev, REG_RXIGI_A),
		 rtw_read8(rtwdev, REG_RRSR + 2),
		 rtw_read8(rtwdev, REG_SLOT));

	return true;
}

static void rtw8723bs_auth_rx_filter(struct rtw_dev *rtwdev,
				     const char *where, bool accept_all)
{
	u32 before = rtw_read32(rtwdev, REG_RCR);

	/* Match the legacy rtl8723bs staging driver: keep the chip on
	 * target-only RCR throughout. Staging's hw_var_set_opmode() for
	 * STATION_STATE leaves REG_RCR at the _InitWMACSetting() default
	 * (CBSSID_DATA | CBSSID_BCN | AMF set, AAP clear) and never opens
	 * it during the auth/assoc window. The earlier 'accept_all'
	 * variant (caa16f1) cleared CBSSID_DATA | CBSSID_BCN and set AAP
	 * for the connect window on the assumption that AAP would let
	 * unicast AP responses through while NO_LINK; in practice on this
	 * 8723bs SDIO target the AP at -50 dBm still never sends a real
	 * Auth/Assoc response with that filter, even with byte-identical
	 * staging TX descriptor, the PTA antenna routing scan uses, and
	 * the staging-equivalent RESP_SIFS values programmed at MAC init.
	 *
	 * AMF=1 + APM=1 (already in WLAN_RCR_CFG = 0x700060CE | BIT_AMF)
	 * is sufficient to receive auth/assoc responses addressed to our
	 * STA MAC: AMF gates mgmt frames, APM gates frames whose addr1
	 * matches REG_MACID, and CBSSID_DATA/CBSSID_BCN do not filter
	 * mgmt at all (they only narrow data and beacon RX to the BSSID
	 * already programmed via PORT_SET_BSSID). So leaving the filter
	 * at target-only across the whole connect window matches staging
	 * exactly without losing the ability to ACK unicast AP responses.
	 *
	 * Both the accept_all=true (join_prepare) and accept_all=false
	 * (assoc / disassoc) call sites end up here so the RCR converges
	 * to target-only state on every transition; the parameter is
	 * preserved so the trace tag distinguishes the call site.
	 */
	rtwdev->hal.rcr |= BIT_AMF | BIT_CBSSID_DATA | BIT_CBSSID_BCN;
	rtwdev->hal.rcr &= ~BIT_AAP;

	rtw_write32(rtwdev, REG_RCR, rtwdev->hal.rcr);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s_rx %s RCR 0x%08x->0x%08x hal=0x%08x\n",
		 where, accept_all ? "join_target_only" : "target_only", before,
		 rtw_read32(rtwdev, REG_RCR), rtwdev->hal.rcr);
}

static void rtw8723bs_config_sec_cfg(struct rtw_dev *rtwdev,
				     const char *where)
{
	u16 before = rtw_read16(rtwdev, RTW_SEC_CONFIG);
	u16 sec = before;

	sec |= RTW_SEC_CHK_KEYID | RTW_SEC_TX_DEC_EN | RTW_SEC_RX_DEC_EN;

	rtw_write16(rtwdev, RTW_SEC_CONFIG, sec);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s sec_cfg SEC 0x%04x->0x%04x\n",
		 where, before,
		 rtw_read16(rtwdev, RTW_SEC_CONFIG));
}

static void rtw8723bs_config_default_key_search(struct rtw_dev *rtwdev,
						const char *where,
						bool enable)
{
	u16 before = rtw_read16(rtwdev, RTW_SEC_CONFIG);
	u16 sec = before;

	if (enable)
		sec |= RTW_SEC_TX_BC_USE_DK | RTW_SEC_TX_UNI_USE_DK |
		       RTW_SEC_RX_UNI_USE_DK;
	else
		sec &= ~(RTW_SEC_TX_UNI_USE_DK | RTW_SEC_RX_UNI_USE_DK |
			 RTW_SEC_TX_BC_USE_DK | RTW_SEC_RX_BC_USE_DK);

	rtw_write16(rtwdev, RTW_SEC_CONFIG, sec);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s dk_cfg enable=%d SEC 0x%04x->0x%04x\n",
		 where, enable, before,
		 rtw_read16(rtwdev, RTW_SEC_CONFIG));
}

static void rtw8723bs_enable_tsf_update(struct rtw_dev *rtwdev,
					const char *where)
{
	u8 before = rtw_read8(rtwdev, REG_BCN_CTRL);

	rtw_write8_clr(rtwdev, REG_BCN_CTRL, BIT_DIS_TSF_UDT);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s tsf_update BCN_CTRL 0x%02x->0x%02x\n",
		 where, before, rtw_read8(rtwdev, REG_BCN_CTRL));
}

static void rtw8723bs_set_ack_preamble(struct rtw_dev *rtwdev,
				       const char *where,
				       bool short_preamble)
{
	u8 before = rtw_read8(rtwdev, REG_RRSR + 2);
	u8 after;

	after = before & ~RTW8723BS_ACK_PREAMBLE_SHORT;
	if (short_preamble)
		after |= RTW8723BS_ACK_PREAMBLE_SHORT;

	rtw_write8(rtwdev, REG_RRSR + 2, after);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s ack_preamble short=%d RRSR2 0x%02x->0x%02x\n",
		 where, short_preamble, before, rtw_read8(rtwdev, REG_RRSR + 2));
}

static void rtw8723bs_set_slot_time(struct rtw_dev *rtwdev,
				    const char *where,
				    bool short_slot)
{
	u8 slot_time = short_slot ? RTW8723BS_SHORT_SLOT_TIME :
				    RTW8723BS_LONG_SLOT_TIME;
	u8 before = rtw_read8(rtwdev, REG_SLOT);

	rtw_write8(rtwdev, REG_SLOT, slot_time);

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s slot_time short=%d SLOT 0x%02x->0x%02x\n",
		 where, short_slot, before, rtw_read8(rtwdev, REG_SLOT));
}

static void rtw8723bs_apply_bss_cap(struct rtw_dev *rtwdev,
				    struct ieee80211_vif *vif,
				    const u8 *bssid,
				    const char *where)
{
	struct ieee80211_bss_conf *conf = &vif->bss_conf;
	struct cfg80211_bss *lookup_bss = NULL;
	struct cfg80211_bss *bss = NULL;
	const char *source = "conf";
	bool short_preamble;
	bool short_slot;
	bool have_cap = false;
	u16 cap = 0;

	if (!rtw8723bs_sdio(rtwdev) ||
	    vif->type != NL80211_IFTYPE_STATION)
		return;

	/* rtl8723bs staging runs update_capinfo() before auth. That programs
	 * ACK preamble and slot time from the target AP capability bits before
	 * sending auth/deauth, not only after association. At mgd_prepare_tx()
	 * time mac80211 has not yet called ieee80211_handle_bss_capability(),
	 * so bss_conf.use_short_* still carries the disconnected defaults even
	 * though the selected scan result already has the AP capabilities.
	 */
	if (conf->bss) {
		bss = conf->bss;
		source = "bss_conf";
	} else if (bssid && is_valid_ether_addr(bssid)) {
		lookup_bss = cfg80211_get_bss(rtwdev->hw->wiphy, NULL, bssid,
					      NULL, 0, IEEE80211_BSS_TYPE_ESS,
					      IEEE80211_PRIVACY_ANY);
		if (lookup_bss) {
			bss = lookup_bss;
			source = "scan_bss";
		}
	}

	if (bss) {
		cap = bss->capability;
		have_cap = true;
	} else if (conf->assoc_capability) {
		cap = conf->assoc_capability;
		source = "assoc_cap";
		have_cap = true;
	}

	if (have_cap) {
		short_preamble = !!(cap & WLAN_CAPABILITY_SHORT_PREAMBLE);
		short_slot = !!(cap & WLAN_CAPABILITY_SHORT_SLOT_TIME);
	} else {
		short_preamble = conf->use_short_preamble;
		short_slot = conf->use_short_slot;
	}

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: %s bss_cap source=%s valid=%d cap=0x%04x conf_short_preamble=%d conf_short_slot=%d\n",
		 where, source, have_cap, cap, conf->use_short_preamble,
		 conf->use_short_slot);

	rtw8723bs_set_ack_preamble(rtwdev, where, short_preamble);
	rtw8723bs_set_slot_time(rtwdev, where, short_slot);

	if (lookup_bss)
		cfg80211_put_bss(rtwdev->hw->wiphy, lookup_bss);
}

static const char *rtw_ops_tx_mgmt_stype_name(__le16 fc)
{
	switch (le16_to_cpu(fc) & IEEE80211_FCTL_STYPE) {
	case IEEE80211_STYPE_ASSOC_REQ:
		return "assoc_req";
	case IEEE80211_STYPE_REASSOC_REQ:
		return "reassoc_req";
	case IEEE80211_STYPE_AUTH:
		return "auth";
	default:
		return "mgmt";
	}
}

static bool rtw_ops_tx_trace_mgmt_needed(__le16 fc)
{
	switch (le16_to_cpu(fc) & IEEE80211_FCTL_STYPE) {
	case IEEE80211_STYPE_ASSOC_REQ:
	case IEEE80211_STYPE_REASSOC_REQ:
	case IEEE80211_STYPE_AUTH:
		return true;
	default:
		return false;
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
static bool rtw8723bs_mgd_prepare_is_auth(struct rtw_dev *rtwdev,
					  struct ieee80211_prep_tx_info *info)
{
	return rtw8723bs_sdio(rtwdev) && info &&
	       info->subtype == IEEE80211_STYPE_AUTH;
}

static unsigned int rtw8723bs_auth_sync_wait_ms(struct ieee80211_vif *vif)
{
	unsigned int wait_ms;
	u16 beacon_int = vif->bss_conf.beacon_int;

	if (!beacon_int)
		return RTW8723BS_AUTH_SYNC_WAIT_FALLBACK_MS;

	wait_ms = DIV_ROUND_UP(beacon_int * 1024, 1000);
	wait_ms += 20;

	return clamp_t(unsigned int, wait_ms, RTW8723BS_AUTH_SYNC_WAIT_MIN_MS,
		       RTW8723BS_AUTH_SYNC_WAIT_MAX_MS);
}

static void rtw8723bs_auth_sync_start(struct rtw_dev *rtwdev, const u8 *bssid)
{
	struct rtw_auth_sync *sync = &rtwdev->auth_sync;
	unsigned long flags;

	spin_lock_irqsave(&sync->lock, flags);
	ether_addr_copy(sync->bssid, bssid);
	sync->seen = false;
	sync->seen_count = 0;
	sync->active = true;
	spin_unlock_irqrestore(&sync->lock, flags);
}

static void rtw8723bs_auth_sync_stop(struct rtw_dev *rtwdev)
{
	struct rtw_auth_sync *sync = &rtwdev->auth_sync;
	unsigned long flags;

	spin_lock_irqsave(&sync->lock, flags);
	sync->active = false;
	spin_unlock_irqrestore(&sync->lock, flags);
}

static bool rtw8723bs_auth_sync_seen(struct rtw_dev *rtwdev)
{
	struct rtw_auth_sync *sync = &rtwdev->auth_sync;
	unsigned long flags;
	bool seen;

	spin_lock_irqsave(&sync->lock, flags);
	seen = sync->seen;
	spin_unlock_irqrestore(&sync->lock, flags);

	return seen;
}

static bool rtw8723bs_auth_sync_wait(struct rtw_dev *rtwdev,
				     unsigned int wait_ms)
{
	struct rtw_auth_sync *sync = &rtwdev->auth_sync;

	return wait_event_timeout(sync->wait,
				  rtw8723bs_auth_sync_seen(rtwdev),
				  msecs_to_jiffies(wait_ms)) > 0;
}

void rtw8723bs_auth_sync_rx(struct rtw_dev *rtwdev,
			    const struct ieee80211_hdr *hdr, u32 len,
			    const struct rtw_rx_pkt_stat *pkt_stat,
			    const struct ieee80211_rx_status *rx_status)
{
	struct rtw_auth_sync *sync = &rtwdev->auth_sync;
	const char *stype;
	unsigned long flags;
	bool matched = false;
	u32 seen_count = 0;
	__le16 fc = hdr->frame_control;

	if (rtwdev->chip->id != RTW_CHIP_TYPE_8723B ||
	    rtw_hci_type(rtwdev) != RTW_HCI_TYPE_SDIO ||
	    test_bit(RTW_FLAG_SCANNING, rtwdev->flags) ||
	    pkt_stat->crc_err || pkt_stat->icv_err)
		return;

	if (ieee80211_is_beacon(fc))
		stype = "beacon";
	else if (ieee80211_is_probe_resp(fc))
		stype = "probe_resp";
	else
		return;

	spin_lock_irqsave(&sync->lock, flags);
	if (sync->active && ether_addr_equal(hdr->addr3, sync->bssid)) {
		sync->seen = true;
		sync->seen_count++;
		seen_count = sync->seen_count;
		matched = true;
	}
	spin_unlock_irqrestore(&sync->lock, flags);

	if (!matched)
		return;

	rtw_info(rtwdev,
		 "MGMT_RX_DEBUG: auth_sync_target stype=%s len=%u fc=0x%04x seq_ctrl=0x%04x addr1=%pM addr2=%pM addr3=%pM count=%u rate=%u crc=%d icv=%d freq=%u signal=%d flags=0x%x drv_info=%u shift=%u\n",
		 stype, len, le16_to_cpu(fc), le16_to_cpu(hdr->seq_ctrl),
		 hdr->addr1, hdr->addr2, hdr->addr3, seen_count, pkt_stat->rate,
		 pkt_stat->crc_err, pkt_stat->icv_err, rx_status->freq,
		 rx_status->signal, (unsigned int)rx_status->flag,
		 pkt_stat->drv_info_sz, pkt_stat->shift);
	wake_up(&sync->wait);
}
EXPORT_SYMBOL(rtw8723bs_auth_sync_rx);

static bool rtw8723bs_mgd_prepare_join(struct rtw_dev *rtwdev,
				       struct ieee80211_vif *vif,
				       const u8 *bssid)
{
	struct rtw_vif *rtwvif = (struct rtw_vif *)vif->drv_priv;
	enum rtw_net_type old_net_type = rtwvif->net_type;
	bool fresh_join;
	u8 bcn_ctrl_before;
	u32 rcr_before;
	u16 rxfltmap2_before;
	u16 retry_before;
	u16 sec_before;
	u8 msr_before;
	u16 retry_limit;

	if (!is_valid_ether_addr(bssid)) {
		rtw_info(rtwdev,
			 "MGMT_TX_DEBUG: join_prepare skip invalid_bssid=%pM\n",
			 bssid);
		return false;
	}

	fresh_join = !ether_addr_equal(rtwvif->bssid, bssid);

	msr_before = rtw_read8(rtwdev, REG_CR + 2);
	bcn_ctrl_before = rtw_read8(rtwdev, REG_BCN_CTRL);
	rcr_before = rtw_read32(rtwdev, REG_RCR);
	rxfltmap2_before = rtw_read16(rtwdev, REG_RXFLTMAP2);
	retry_before = rtw_read16(rtwdev, REG_RETRY_LIMIT);
	sec_before = rtw_read16(rtwdev, RTW_SEC_CONFIG);

	ether_addr_copy(rtwvif->bssid, bssid);
	rtwvif->aid = 0;
	/* Set MSR to station state before auth, matching staging's
	 * set_msr(WIFI_FW_STATION_STATE) in start_clnt_join().  Staging
	 * sets MSR=0x02 before issuing auth and the same AP/hardware
	 * combination connects successfully.
	 *
	 * The rollback to NO_LINK is handled by rtw_vif_assoc_changed()
	 * when BSS_CHANGED_ASSOC fires with conf->assoc=false (after
	 * auth timeout or wpa_supplicant disconnect).
	 */
	rtwvif->net_type = RTW_NET_MGD_LINKED;
	rtw_vif_port_config(rtwdev, rtwvif,
			    PORT_SET_BSSID | PORT_SET_AID | PORT_SET_NET_TYPE);

	rtw8723bs_apply_bss_cap(rtwdev, vif, bssid, "join_prepare");

	rtw_fw_beacon_filter_config(rtwdev, false, vif);

	/* Match staging hw_var_set_opmode(_HW_STATE_STATION_):
	 * BCN_CTRL = DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_ATIM.
	 *
	 * eeffbe2 wrongly used DIS_BCNQ_SUB here on the assumption that
	 * staging's _InitBeaconParameters() / _BeaconFunctionEnable() also
	 * set DIS_BCNQ_SUB for STA mode.  In fact _BeaconFunctionEnable()
	 * is only called from rtl8723b_SetBeaconRelatedRegisters(), which
	 * staging invokes solely from the AP / IBSS join paths via
	 * beacon_timing_control().  For pure STA mode staging takes
	 * hw_var_set_opmode() which writes DIS_ATIM (BIT(0)), not
	 * DIS_BCNQ_SUB (BIT(1)).  Restoring DIS_ATIM keeps us byte-for-byte
	 * with the legacy rtl8723bs driver during the entire connect
	 * window.
	 */
	rtw_write8(rtwdev, REG_BCN_CTRL,
		   BIT_DIS_TSF_UDT | BIT_EN_BCN_FUNCTION | BIT_DIS_ATIM);

	rtw_write16(rtwdev, REG_RXFLTMAP0, 0xffff);
	rtw_write16(rtwdev, REG_RXFLTMAP2, 0xffff);
	rtw8723bs_auth_rx_filter(rtwdev, "join_prepare", true);

	retry_limit = (RTW8723BS_JOIN_RETRY_LIMIT << 8) |
		      RTW8723BS_JOIN_RETRY_LIMIT;
	rtw_write16(rtwdev, REG_RETRY_LIMIT, retry_limit);

	rtw8723bs_config_sec_cfg(rtwdev, "join_prepare");

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: join_prepare bssid=%pM fresh=%d net_type %u->%u media_status=defer MSR 0x%02x->0x%02x BCN_CTRL 0x%02x->0x%02x RCR 0x%08x->0x%08x hal=0x%08x RXFLTMAP2 0x%04x->0x%04x RETRY 0x%04x->0x%04x SEC 0x%04x->0x%04x\n",
		 bssid, fresh_join, old_net_type, rtwvif->net_type,
		 msr_before, rtw_read8(rtwdev, REG_CR + 2), bcn_ctrl_before,
		 rtw_read8(rtwdev, REG_BCN_CTRL), rcr_before,
		 rtw_read32(rtwdev, REG_RCR), rtwdev->hal.rcr,
		 rxfltmap2_before, rtw_read16(rtwdev, REG_RXFLTMAP2),
		 retry_before, rtw_read16(rtwdev, REG_RETRY_LIMIT),
		 sec_before, rtw_read16(rtwdev, RTW_SEC_CONFIG));

	return fresh_join;
}
#endif

static void rtw_trace_ops_tx_mgmt(struct rtw_dev *rtwdev,
				  struct ieee80211_tx_control *control,
				  struct sk_buff *skb, bool dropped)
{
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	const struct ieee80211_hdr *hdr;
	const u8 *sta_addr = zero_addr;
	const char *stype;
	__le16 fc;

	if (!rtw8723bs_sdio(rtwdev) ||
	    skb->len < sizeof(struct ieee80211_hdr_3addr))
		return;

	hdr = (const struct ieee80211_hdr *)skb->data;
	fc = hdr->frame_control;
	if (!ieee80211_is_mgmt(fc) || !rtw_ops_tx_trace_mgmt_needed(fc))
		return;

	if (control && control->sta)
		sta_addr = control->sta->addr;

	stype = rtw_ops_tx_mgmt_stype_name(fc);
	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: ops_tx stype=%s dropped=%d running=%d scanning=%d idle=%d len=%u fc=0x%04x seq_ctrl=0x%04x addr1=%pM addr2=%pM addr3=%pM sta=%pM\n",
		 stype, dropped, test_bit(RTW_FLAG_RUNNING, rtwdev->flags),
		 test_bit(RTW_FLAG_SCANNING, rtwdev->flags),
		 !!(rtwdev->hw->conf.flags & IEEE80211_CONF_IDLE), skb->len,
		 le16_to_cpu(fc), le16_to_cpu(hdr->seq_ctrl),
		 hdr->addr1, hdr->addr2, hdr->addr3, sta_addr);

	if (ieee80211_is_auth(fc) ||
	    ieee80211_is_assoc_req(fc) ||
	    ieee80211_is_reassoc_req(fc) ||
	    ieee80211_is_deauth(fc)) {
		int i, dump_len = min_t(int, skb->len,
					RTW8723BS_MGMT_DUMP_BYTES);
		char hex[RTW8723BS_MGMT_DUMP_BYTES * 3 + 1] = {};

		for (i = 0; i < dump_len; i++)
			sprintf(hex + i * 3, "%02x ", skb->data[i]);
		rtw_info(rtwdev,
			 "MGMT_TX_DEBUG: tx_data stype=%s (%d/%u bytes): %s\n",
			 stype, dump_len, skb->len, hex);
	}
}

static void rtw8723bs_tx_pre_auth_deauth(struct rtw_dev *rtwdev,
					 struct ieee80211_vif *vif,
					 const u8 *bssid)
{
	struct ieee80211_tx_control control = {};
	struct ieee80211_tx_info *info;
	struct ieee80211_mgmt *mgmt;
	struct sk_buff *skb;
	unsigned int frame_len;
	unsigned int headroom;

	frame_len = sizeof(struct ieee80211_hdr_3addr) + sizeof(mgmt->u.deauth);
	headroom = rtwdev->chip->tx_pkt_desc_sz + 8;

	skb = dev_alloc_skb(headroom + frame_len);
	if (!skb)
		return;

	skb_reserve(skb, headroom);
	mgmt = skb_put_zero(skb, frame_len);
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					  IEEE80211_STYPE_DEAUTH);
	memcpy(mgmt->da, bssid, ETH_ALEN);
	memcpy(mgmt->sa, vif->addr, ETH_ALEN);
	memcpy(mgmt->bssid, bssid, ETH_ALEN);
	mgmt->u.deauth.reason_code = cpu_to_le16(WLAN_REASON_DEAUTH_LEAVING);

	info = IEEE80211_SKB_CB(skb);
	memset(info, 0, sizeof(*info));
	info->control.vif = vif;

	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: pre_auth_deauth bssid=%pM wait_ms=100\n",
		 bssid);
	rtw_tx(rtwdev, &control, skb);
	msleep(100);
}

static void rtw_ops_tx(struct ieee80211_hw *hw,
		       struct ieee80211_tx_control *control,
		       struct sk_buff *skb)
{
	struct rtw_dev *rtwdev = hw->priv;

	if (!test_bit(RTW_FLAG_RUNNING, rtwdev->flags)) {
		rtw_trace_ops_tx_mgmt(rtwdev, control, skb, true);
		ieee80211_free_txskb(hw, skb);
		return;
	}

	rtw_trace_ops_tx_mgmt(rtwdev, control, skb, false);
	rtw_tx(rtwdev, control, skb);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
static void rtw8723bs_mgd_prepare_auth_join(struct rtw_dev *rtwdev,
					    struct ieee80211_vif *vif,
					    struct ieee80211_prep_tx_info *info)
{
	static const u8 zero_addr[ETH_ALEN] = { 0 };
	const u8 *bssid = NULL;

	if (!rtw8723bs_mgd_prepare_is_auth(rtwdev, info) ||
	    !vif ||
	    test_bit(RTW_FLAG_SCANNING, rtwdev->flags))
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
	if (!is_zero_ether_addr(vif->cfg.ap_addr))
		bssid = vif->cfg.ap_addr;
#endif
	if (!bssid && vif->bss_conf.bssid &&
	    !is_zero_ether_addr(vif->bss_conf.bssid))
		bssid = vif->bss_conf.bssid;

	if (!bssid) {
		rtw_info(rtwdev,
			 "MGMT_TX_DEBUG: mgd_prepare_auth_join skip invalid_bssid subtype=0x%04x ap_addr=%pM conf_bssid=%pM\n",
			 info->subtype,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0)
			 vif->cfg.ap_addr,
#else
			 zero_addr,
#endif
			 vif->bss_conf.bssid ? vif->bss_conf.bssid : zero_addr);
		return;
	}

	if (rtw8723bs_mgd_prepare_join(rtwdev, vif, bssid)) {
		unsigned int wait_ms = rtw8723bs_auth_sync_wait_ms(vif);
		bool beacon_seen;

		rtw8723bs_auth_sync_start(rtwdev, bssid);
		rtw8723bs_tx_pre_auth_deauth(rtwdev, vif, bssid);

		/* Staging waits for the target beacon before issuing auth.
		 * Wait until RX confirms a beacon/probe response from the target
		 * BSSID after BSSID/MSR/RCR have been programmed.
		 */
		rtw_info(rtwdev,
			 "MGMT_TX_DEBUG: join_prepare wait_for_beacon_sync bssid=%pM wait_ms=%u beacon_int=%u\n",
			 bssid, wait_ms, vif->bss_conf.beacon_int);
		beacon_seen = rtw8723bs_auth_sync_wait(rtwdev, wait_ms);
		rtw8723bs_auth_sync_stop(rtwdev);
		rtw_info(rtwdev,
			 "MGMT_TX_DEBUG: join_prepare beacon_sync_done bssid=%pM seen=%d wait_ms=%u\n",
			 bssid, beacon_seen, wait_ms);
	} else {
		rtw_info(rtwdev,
			 "MGMT_TX_DEBUG: join_prepare retry bssid=%pM reuse_join\n",
			 bssid);
	}
}
#endif

static void rtw_ops_wake_tx_queue(struct ieee80211_hw *hw,
				  struct ieee80211_txq *txq)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_txq *rtwtxq = (struct rtw_txq *)txq->drv_priv;

	if (!test_bit(RTW_FLAG_RUNNING, rtwdev->flags))
		return;

	spin_lock_bh(&rtwdev->txq_lock);
	if (list_empty(&rtwtxq->list))
		list_add_tail(&rtwtxq->list, &rtwdev->txqs);
	spin_unlock_bh(&rtwdev->txq_lock);

	/* ensure to dequeue EAPOL (4/4) at the right time */
	if (txq->ac == IEEE80211_AC_VO)
		__rtw_tx_work(rtwdev);
	else
		queue_work(rtwdev->tx_wq, &rtwdev->tx_work);
}

static int rtw_ops_start(struct ieee80211_hw *hw)
{
	struct rtw_dev *rtwdev = hw->priv;
	int ret;

	mutex_lock(&rtwdev->mutex);
	ret = rtw_core_start(rtwdev);
	mutex_unlock(&rtwdev->mutex);

	return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0))
static void rtw_ops_stop(struct ieee80211_hw *hw)
#else
static void rtw_ops_stop(struct ieee80211_hw *hw, bool suspend)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_core_stop(rtwdev);
	mutex_unlock(&rtwdev->mutex);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
static int rtw_ops_config(struct ieee80211_hw *hw, int radio_idx, u32 changed)
#else
static int rtw_ops_config(struct ieee80211_hw *hw, u32 changed)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;
	int ret = 0;

	/* let previous ips work finish to ensure we don't leave ips twice */
	cancel_work_sync(&rtwdev->ips_work);

	mutex_lock(&rtwdev->mutex);

	rtw_leave_lps_deep(rtwdev);

	if ((changed & IEEE80211_CONF_CHANGE_IDLE) &&
	    !(hw->conf.flags & IEEE80211_CONF_IDLE)) {
		ret = rtw_leave_ips(rtwdev);
		if (ret) {
			rtw_err(rtwdev, "failed to leave idle state\n");
			goto out;
		}
	}

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL)
		rtw_set_channel(rtwdev);

	if ((changed & IEEE80211_CONF_CHANGE_IDLE) &&
	    (hw->conf.flags & IEEE80211_CONF_IDLE) &&
	    !test_bit(RTW_FLAG_SCANNING, rtwdev->flags))
		rtw_enter_ips(rtwdev);

out:
	mutex_unlock(&rtwdev->mutex);
	return ret;
}

static const struct rtw_vif_port rtw_vif_port[] = {
	[0] = {
		.mac_addr	= {.addr = 0x0610},
		.bssid		= {.addr = 0x0618},
		.net_type	= {.addr = 0x0100, .mask = 0x30000},
		.aid		= {.addr = 0x06a8, .mask = 0x7ff},
		.bcn_ctrl	= {.addr = 0x0550, .mask = 0xff},
	},
	[1] = {
		.mac_addr	= {.addr = 0x0700},
		.bssid		= {.addr = 0x0708},
		.net_type	= {.addr = 0x0100, .mask = 0xc0000},
		.aid		= {.addr = 0x0710, .mask = 0x7ff},
		.bcn_ctrl	= {.addr = 0x0551, .mask = 0xff},
	},
	[2] = {
		.mac_addr	= {.addr = 0x1620},
		.bssid		= {.addr = 0x1628},
		.net_type	= {.addr = 0x1100, .mask = 0x3},
		.aid		= {.addr = 0x1600, .mask = 0x7ff},
		.bcn_ctrl	= {.addr = 0x0578, .mask = 0xff},
	},
	[3] = {
		.mac_addr	= {.addr = 0x1630},
		.bssid		= {.addr = 0x1638},
		.net_type	= {.addr = 0x1100, .mask = 0xc},
		.aid		= {.addr = 0x1604, .mask = 0x7ff},
		.bcn_ctrl	= {.addr = 0x0579, .mask = 0xff},
	},
	[4] = {
		.mac_addr	= {.addr = 0x1640},
		.bssid		= {.addr = 0x1648},
		.net_type	= {.addr = 0x1100, .mask = 0x30},
		.aid		= {.addr = 0x1608, .mask = 0x7ff},
		.bcn_ctrl	= {.addr = 0x057a, .mask = 0xff},
	},
};

static int rtw_ops_add_interface(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_vif *rtwvif = (struct rtw_vif *)vif->drv_priv;
	enum rtw_net_type net_type;
	u32 config = 0;
	u8 port;
	u8 bcn_ctrl = 0;

	if (rtw_fw_feature_check(&rtwdev->fw, FW_FEATURE_BCN_FILTER))
		vif->driver_flags |= IEEE80211_VIF_BEACON_FILTER |
				     IEEE80211_VIF_SUPPORTS_CQM_RSSI;
	rtwvif->stats.tx_unicast = 0;
	rtwvif->stats.rx_unicast = 0;
	rtwvif->stats.tx_cnt = 0;
	rtwvif->stats.rx_cnt = 0;
	rtwvif->scan_req = NULL;
	memset(&rtwvif->bfee, 0, sizeof(struct rtw_bfee));
	rtw_txq_init(rtwdev, vif->txq);
	INIT_LIST_HEAD(&rtwvif->rsvd_page_list);

	mutex_lock(&rtwdev->mutex);

	rtwvif->mac_id = rtw_acquire_macid(rtwdev);
	if (rtwvif->mac_id >= RTW_MAX_MAC_ID_NUM) {
		mutex_unlock(&rtwdev->mutex);
		return -ENOSPC;
	}

	port = find_first_zero_bit(rtwdev->hw_port, RTW_PORT_NUM);
	if (port >= RTW_PORT_NUM) {
		mutex_unlock(&rtwdev->mutex);
		return -EINVAL;
	}
	set_bit(port, rtwdev->hw_port);

	rtwvif->port = port;
	rtwvif->conf = &rtw_vif_port[port];
	rtw_leave_lps_deep(rtwdev);

	switch (vif->type) {
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_MESH_POINT:
		rtw_add_rsvd_page_bcn(rtwdev, rtwvif);
		net_type = RTW_NET_AP_MODE;
		bcn_ctrl = BIT_EN_BCN_FUNCTION | BIT_DIS_TSF_UDT;
		break;
	case NL80211_IFTYPE_ADHOC:
		rtw_add_rsvd_page_bcn(rtwdev, rtwvif);
		net_type = RTW_NET_AD_HOC;
		bcn_ctrl = BIT_EN_BCN_FUNCTION | BIT_DIS_TSF_UDT;
		break;
	case NL80211_IFTYPE_STATION:
		rtw_add_rsvd_page_sta(rtwdev, rtwvif);
		net_type = RTW_NET_NO_LINK;
		bcn_ctrl = BIT_EN_BCN_FUNCTION;
		break;
	default:
		WARN_ON(1);
		clear_bit(rtwvif->port, rtwdev->hw_port);
		mutex_unlock(&rtwdev->mutex);
		return -EINVAL;
	}

	ether_addr_copy(rtwvif->mac_addr, vif->addr);
	config |= PORT_SET_MAC_ADDR;
	rtwvif->net_type = net_type;
	config |= PORT_SET_NET_TYPE;
	rtwvif->bcn_ctrl = bcn_ctrl;
	config |= PORT_SET_BCN_CTRL;
	rtw_vif_port_config(rtwdev, rtwvif, config);
	rtw_core_port_switch(rtwdev, vif);
	rtw_recalc_lps(rtwdev, vif);

	mutex_unlock(&rtwdev->mutex);

	rtw_dbg(rtwdev, RTW_DBG_STATE, "start vif %pM mac_id %d on port %d\n",
		vif->addr, rtwvif->mac_id, rtwvif->port);
	return 0;
}

static void rtw_ops_remove_interface(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_vif *rtwvif = (struct rtw_vif *)vif->drv_priv;
	u32 config = 0;

	rtw_dbg(rtwdev, RTW_DBG_STATE, "stop vif %pM mac_id %d on port %d\n",
		vif->addr, rtwvif->mac_id, rtwvif->port);

	mutex_lock(&rtwdev->mutex);

	rtw_leave_lps_deep(rtwdev);

	rtw_txq_cleanup(rtwdev, vif->txq);
	rtw_remove_rsvd_page(rtwdev, rtwvif);

	eth_zero_addr(rtwvif->mac_addr);
	config |= PORT_SET_MAC_ADDR;
	rtwvif->net_type = RTW_NET_NO_LINK;
	config |= PORT_SET_NET_TYPE;
	rtwvif->bcn_ctrl = 0;
	config |= PORT_SET_BCN_CTRL;
	rtw_vif_port_config(rtwdev, rtwvif, config);
	clear_bit(rtwvif->port, rtwdev->hw_port);
	rtw_release_macid(rtwdev, rtwvif->mac_id);
	rtw_recalc_lps(rtwdev, NULL);

	mutex_unlock(&rtwdev->mutex);
}

static int rtw_ops_change_interface(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    enum nl80211_iftype type, bool p2p)
{
	struct rtw_dev *rtwdev = hw->priv;

	rtw_dbg(rtwdev, RTW_DBG_STATE, "change vif %pM (%d)->(%d), p2p (%d)->(%d)\n",
		vif->addr, vif->type, type, vif->p2p, p2p);

	rtw_ops_remove_interface(hw, vif);

	vif->type = type;
	vif->p2p = p2p;

	return rtw_ops_add_interface(hw, vif);
}

static void rtw_ops_configure_filter(struct ieee80211_hw *hw,
				     unsigned int changed_flags,
				     unsigned int *new_flags,
				     u64 multicast)
{
	struct rtw_dev *rtwdev = hw->priv;
	u32 rcr_before;

	*new_flags &= FIF_ALLMULTI | FIF_OTHER_BSS | FIF_FCSFAIL |
		      FIF_BCN_PRBRESP_PROMISC;

	mutex_lock(&rtwdev->mutex);

	rtw_leave_lps_deep(rtwdev);
	rcr_before = rtwdev->hal.rcr;

	if (changed_flags & FIF_ALLMULTI) {
		if (*new_flags & FIF_ALLMULTI)
			rtwdev->hal.rcr |= BIT_AM;
		else
			rtwdev->hal.rcr &= ~(BIT_AM);
	}
	if (changed_flags & FIF_FCSFAIL) {
		if (*new_flags & FIF_FCSFAIL)
			rtwdev->hal.rcr |= BIT_ACRC32;
		else
			rtwdev->hal.rcr &= ~(BIT_ACRC32);
	}
	if (changed_flags & FIF_OTHER_BSS) {
		if (*new_flags & FIF_OTHER_BSS)
			rtwdev->hal.rcr |= BIT_AAP;
		else
			rtwdev->hal.rcr &= ~(BIT_AAP);
	}
	if (changed_flags & FIF_BCN_PRBRESP_PROMISC) {
		if ((*new_flags & FIF_BCN_PRBRESP_PROMISC) ||
		    test_bit(RTW_FLAG_SCANNING, rtwdev->flags))
			rtwdev->hal.rcr &= ~BIT_CBSSID_BCN;
		else
			rtwdev->hal.rcr |= BIT_CBSSID_BCN;
	}

	rtw_dbg(rtwdev, RTW_DBG_RX,
		"config rx filter, changed=0x%08x, new=0x%08x, rcr=0x%08x\n",
		changed_flags, *new_flags, rtwdev->hal.rcr);
	rtw_info(rtwdev,
		 "SCAN_DEBUG: configure_filter scanning=%d changed=0x%08x new=0x%08x RCR 0x%08x->0x%08x\n",
		 test_bit(RTW_FLAG_SCANNING, rtwdev->flags), changed_flags,
		 *new_flags, rcr_before, rtwdev->hal.rcr);

	rtw_write32(rtwdev, REG_RCR, rtwdev->hal.rcr);

	mutex_unlock(&rtwdev->mutex);
}

/* Only have one group of EDCA parameters now */
static const u32 ac_to_edca_param[IEEE80211_NUM_ACS] = {
	[IEEE80211_AC_VO] = REG_EDCA_VO_PARAM,
	[IEEE80211_AC_VI] = REG_EDCA_VI_PARAM,
	[IEEE80211_AC_BE] = REG_EDCA_BE_PARAM,
	[IEEE80211_AC_BK] = REG_EDCA_BK_PARAM,
};

static u8 rtw_aifsn_to_aifs(struct rtw_dev *rtwdev,
			    struct rtw_vif *rtwvif, u8 aifsn)
{
	struct ieee80211_vif *vif = rtwvif_to_vif(rtwvif);
	u8 slot_time;
	u8 sifs;

	slot_time = vif->bss_conf.use_short_slot ? 9 : 20;
	sifs = rtwdev->hal.current_band_type == RTW_BAND_5G ? 16 : 10;

	return aifsn * slot_time + sifs;
}

static void __rtw_conf_tx(struct rtw_dev *rtwdev,
			  struct rtw_vif *rtwvif, u16 ac)
{
	struct ieee80211_tx_queue_params *params = &rtwvif->tx_params[ac];
	u32 edca_param = ac_to_edca_param[ac];
	u8 ecw_max, ecw_min;
	u8 aifs;

	/* 2^ecw - 1 = cw; ecw = log2(cw + 1) */
	ecw_max = ilog2(params->cw_max + 1);
	ecw_min = ilog2(params->cw_min + 1);
	aifs = rtw_aifsn_to_aifs(rtwdev, rtwvif, params->aifs);
	rtw_write32_mask(rtwdev, edca_param, BIT_MASK_TXOP_LMT, params->txop);
	rtw_write32_mask(rtwdev, edca_param, BIT_MASK_CWMAX, ecw_max);
	rtw_write32_mask(rtwdev, edca_param, BIT_MASK_CWMIN, ecw_min);
	rtw_write32_mask(rtwdev, edca_param, BIT_MASK_AIFS, aifs);
}

static void rtw_conf_tx(struct rtw_dev *rtwdev,
			struct rtw_vif *rtwvif)
{
	u16 ac;

	for (ac = 0; ac < IEEE80211_NUM_ACS; ac++)
		__rtw_conf_tx(rtwdev, rtwvif, ac);
}

static void rtw_ops_bss_info_changed(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     struct ieee80211_bss_conf *conf,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
				     u64 changed)
#else
				     u32 changed)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_vif *rtwvif = (struct rtw_vif *)vif->drv_priv;
	struct rtw_coex *coex = &rtwdev->coex;
	struct rtw_coex_stat *coex_stat = &coex->stat;
	u32 config = 0;

	mutex_lock(&rtwdev->mutex);

	rtw_leave_lps_deep(rtwdev);

	if (changed & BSS_CHANGED_ASSOC) {
		rtw_vif_assoc_changed(rtwvif, conf);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
		if (vif->cfg.assoc) {
#else
		if (conf->assoc) {
#endif
			if (rtw8723bs_sdio(rtwdev) &&
			    vif->type == NL80211_IFTYPE_STATION) {
				rtw8723bs_auth_rx_filter(rtwdev, "assoc",
							 false);
				rtw8723bs_apply_bss_cap(rtwdev, vif, NULL,
							"assoc");
				rtw8723bs_enable_tsf_update(rtwdev, "assoc");
				if (!rtwvif->fw_media_connected) {
					rtw_info(rtwdev,
						 "MGMT_TX_DEBUG: assoc media_status connect macid=%u bssid=%pM\n",
						 rtwvif->mac_id,
						 rtwvif->bssid);
					rtw_fw_media_status_report(rtwdev,
								   rtwvif->mac_id,
								   true);
					rtwvif->fw_media_connected = true;
				} else {
					rtw_info(rtwdev,
						 "MGMT_TX_DEBUG: assoc media_status already connected macid=%u bssid=%pM\n",
						 rtwvif->mac_id,
						 rtwvif->bssid);
				}
			}

			rtw_coex_connect_notify(rtwdev, COEX_ASSOCIATE_FINISH);
			rtw_fw_download_rsvd_page(rtwdev);
			rtw_send_rsvd_page_h2c(rtwdev);
			rtw_fw_default_port(rtwdev, rtwvif);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
			rtw_coex_media_status_notify(rtwdev, vif->cfg.assoc);
#else
			rtw_coex_media_status_notify(rtwdev, conf->assoc);
#endif
			if (rtw_bf_support)
				rtw_bf_assoc(rtwdev, vif, conf);

			rtw_set_ampdu_factor(rtwdev, vif, conf);

			rtw_fw_beacon_filter_config(rtwdev, true, vif);
		} else {
			rtw_leave_lps(rtwdev);
			rtw_bf_disassoc(rtwdev, vif, conf);
			/* Abort ongoing scan if cancel_scan isn't issued
			 * when disconnected by peer
			 */
			if (test_bit(RTW_FLAG_SCANNING, rtwdev->flags))
				rtw_hw_scan_abort(rtwdev);

			if (rtw8723bs_sdio(rtwdev) &&
			    vif->type == NL80211_IFTYPE_STATION)
				rtw8723bs_auth_rx_filter(rtwdev, "disassoc",
							 false);

		}

		config |= PORT_SET_NET_TYPE;
		config |= PORT_SET_AID;
	}

	if (changed & BSS_CHANGED_BSSID) {
		bool bssid_cleared = is_zero_ether_addr(conf->bssid);

		ether_addr_copy(rtwvif->bssid, conf->bssid);
		config |= PORT_SET_BSSID;
		if (rtwdev->chip->id == RTW_CHIP_TYPE_8723B &&
		    rtw_hci_type(rtwdev) == RTW_HCI_TYPE_SDIO &&
		    vif->type == NL80211_IFTYPE_STATION && bssid_cleared) {
			enum rtw_net_type old_net_type = rtwvif->net_type;
			u8 bcn_ctrl_before = rtw_read8(rtwdev, REG_BCN_CTRL);

			rtwvif->aid = 0;
			rtwvif->net_type = RTW_NET_NO_LINK;
			config |= PORT_SET_NET_TYPE | PORT_SET_AID;
			rtw_write8(rtwdev, REG_BCN_CTRL,
				   BIT_DIS_TSF_UDT | BIT_EN_BCN_FUNCTION |
				   BIT_DIS_ATIM);
			rtw_info(rtwdev,
				 "MGMT_TX_DEBUG: bssid_clear net_type %u->%u BCN_CTRL 0x%02x->0x%02x\n",
				 old_net_type, rtwvif->net_type, bcn_ctrl_before,
				 rtw_read8(rtwdev, REG_BCN_CTRL));
		}
		if (!rtw_core_check_sta_active(rtwdev))
			rtw_clear_op_chan(rtwdev);
		else
			rtw_store_op_chan(rtwdev, true);
	}

	if (changed & BSS_CHANGED_BEACON_INT) {
		if (ieee80211_vif_type_p2p(vif) == NL80211_IFTYPE_STATION)
			coex_stat->wl_beacon_interval = conf->beacon_int;
	}

	if (changed & BSS_CHANGED_BEACON) {
		rtw_set_dtim_period(rtwdev, conf->dtim_period);
		rtw_fw_download_rsvd_page(rtwdev);
		rtw_send_rsvd_page_h2c(rtwdev);
	}

	if (changed & BSS_CHANGED_BEACON_ENABLED) {
		if (conf->enable_beacon)
			rtw_write32_set(rtwdev, REG_FWHW_TXQ_CTRL,
					BIT_EN_BCNQ_DL);
		else
			rtw_write32_clr(rtwdev, REG_FWHW_TXQ_CTRL,
					BIT_EN_BCNQ_DL);
	}
	if (changed & BSS_CHANGED_CQM)
		rtw_fw_beacon_filter_config(rtwdev, true, vif);

	if (changed & BSS_CHANGED_MU_GROUPS)
		rtw_chip_set_gid_table(rtwdev, vif, conf);

	if (changed & BSS_CHANGED_ERP_PREAMBLE) {
		if (rtw8723bs_sdio(rtwdev) &&
		    vif->type == NL80211_IFTYPE_STATION)
			rtw8723bs_set_ack_preamble(rtwdev, "bss_info",
						   conf->use_short_preamble);
	}

	if (changed & BSS_CHANGED_ERP_SLOT) {
		if (rtw8723bs_sdio(rtwdev) &&
		    vif->type == NL80211_IFTYPE_STATION)
			rtw8723bs_set_slot_time(rtwdev, "bss_info",
						conf->use_short_slot);
		rtw_conf_tx(rtwdev, rtwvif);
	}

	if (changed & BSS_CHANGED_PS)
		rtw_recalc_lps(rtwdev, NULL);

	rtw_vif_port_config(rtwdev, rtwvif, config);

	mutex_unlock(&rtwdev->mutex);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
static int rtw_ops_start_ap(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif,
			    struct ieee80211_bss_conf *link_conf)
#else
static int rtw_ops_start_ap(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;
	const struct rtw_chip_info *chip = rtwdev->chip;

	mutex_lock(&rtwdev->mutex);
	rtw_write32_set(rtwdev, REG_TCR, BIT_TCR_UPDATE_HGQMD);
	rtwdev->ap_active = true;
	rtw_store_op_chan(rtwdev, true);
	chip->ops->phy_calibration(rtwdev);
	mutex_unlock(&rtwdev->mutex);

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
static void rtw_ops_stop_ap(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif,
			    struct ieee80211_bss_conf *link_conf)
#else
static void rtw_ops_stop_ap(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_write32_clr(rtwdev, REG_TCR, BIT_TCR_UPDATE_HGQMD);
	rtwdev->ap_active = false;
	if (!rtw_core_check_sta_active(rtwdev))
		rtw_clear_op_chan(rtwdev);
	mutex_unlock(&rtwdev->mutex);
}

static int rtw_ops_conf_tx(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 0, 0))
			   unsigned int link_id, u16 ac,
#else
			   u16 ac,
#endif
			   const struct ieee80211_tx_queue_params *params)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_vif *rtwvif = (struct rtw_vif *)vif->drv_priv;

	mutex_lock(&rtwdev->mutex);

	rtw_leave_lps_deep(rtwdev);

	rtwvif->tx_params[ac] = *params;
	__rtw_conf_tx(rtwdev, rtwvif, ac);

	mutex_unlock(&rtwdev->mutex);

	return 0;
}

static int rtw_ops_sta_add(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
			   struct ieee80211_sta *sta)
{
	struct rtw_dev *rtwdev = hw->priv;
	int ret = 0;

	mutex_lock(&rtwdev->mutex);
	ret = rtw_sta_add(rtwdev, sta, vif);
	mutex_unlock(&rtwdev->mutex);

	return ret;
}

static int rtw_ops_sta_remove(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta)
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_fw_beacon_filter_config(rtwdev, false, vif);
	rtw_sta_remove(rtwdev, sta, true);
	mutex_unlock(&rtwdev->mutex);

	return 0;
}

static int rtw_ops_set_tim(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
			   bool set)
{
	struct rtw_dev *rtwdev = hw->priv;

	ieee80211_queue_work(hw, &rtwdev->update_beacon_work);

	return 0;
}

static int rtw_ops_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			   struct ieee80211_vif *vif, struct ieee80211_sta *sta,
			   struct ieee80211_key_conf *key)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_sec_desc *sec = &rtwdev->sec;
	bool pairwise = key->flags & IEEE80211_KEY_FLAG_PAIRWISE;
	u8 hw_key_type;
	u8 hw_key_idx;
	int ret = 0;

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		hw_key_type = RTW_CAM_WEP40;
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		hw_key_type = RTW_CAM_WEP104;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		hw_key_type = RTW_CAM_TKIP;
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		hw_key_type = RTW_CAM_AES;
		key->flags |= IEEE80211_KEY_FLAG_SW_MGMT_TX;
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
	case WLAN_CIPHER_SUITE_CCMP_256:
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
		/* suppress error messages */
		return -EOPNOTSUPP;
	default:
		return -ENOTSUPP;
	}

	mutex_lock(&rtwdev->mutex);

	rtw_leave_lps_deep(rtwdev);

	if (pairwise) {
		hw_key_idx = rtw_sec_get_free_cam(sec);
	} else {
		/* multiple interfaces? */
		hw_key_idx = key->keyidx;
	}

	if (hw_key_idx > sec->total_cam_num) {
		ret = -ENOSPC;
		goto out;
	}

	switch (cmd) {
	case SET_KEY:
		/* need sw generated IV */
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
		key->hw_key_idx = hw_key_idx;
		rtw_sec_write_cam(rtwdev, sec, sta, key,
				  hw_key_type, hw_key_idx);
		if (rtw8723bs_sdio(rtwdev) &&
		    vif && vif->type == NL80211_IFTYPE_STATION && !pairwise)
			rtw8723bs_config_default_key_search(rtwdev, "set_key",
							    true);
		break;
	case DISABLE_KEY:
		rtw_hci_flush_all_queues(rtwdev, false);
		rtw_mac_flush_all_queues(rtwdev, false);
		rtw_sec_clear_cam(rtwdev, sec, key->hw_key_idx);
		if (rtw8723bs_sdio(rtwdev) &&
		    vif && vif->type == NL80211_IFTYPE_STATION && !pairwise)
			rtw8723bs_config_default_key_search(rtwdev,
							    "disable_key",
							    false);
		break;
	}

	/* download new cam settings for PG to backup */
	if (rtw_get_lps_deep_mode(rtwdev) == LPS_DEEP_MODE_PG)
		rtw_fw_download_rsvd_page(rtwdev);

out:
	mutex_unlock(&rtwdev->mutex);

	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
static int rtw_ops_ampdu_action(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_ampdu_params *params)
#else
static int rtw_ops_ampdu_action(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				enum ieee80211_ampdu_mlme_action action,
				struct ieee80211_sta *sta, u16 tid, u16 *ssn,
				u8 buf_size, bool amsdu)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	struct ieee80211_sta *sta = params->sta;
	u16 tid = params->tid;
	struct ieee80211_txq *txq = sta->txq[tid];
	struct rtw_txq *rtwtxq = (struct rtw_txq *)txq->drv_priv;

	switch (params->action) {
#else
	switch (action) {
#endif
	case IEEE80211_AMPDU_TX_START:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
		return IEEE80211_AMPDU_TX_START_IMMEDIATE;
#else
		ieee80211_start_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;
#endif
	case IEEE80211_AMPDU_TX_STOP_CONT:
	case IEEE80211_AMPDU_TX_STOP_FLUSH:
	case IEEE80211_AMPDU_TX_STOP_FLUSH_CONT:
		clear_bit(RTW_TXQ_AMPDU, &rtwtxq->flags);
		ieee80211_stop_tx_ba_cb_irqsafe(vif, sta->addr, tid);
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		set_bit(RTW_TXQ_AMPDU, &rtwtxq->flags);
		break;
	case IEEE80211_AMPDU_RX_START:
	case IEEE80211_AMPDU_RX_STOP:
		break;
	default:
		WARN_ON(1);
		return -ENOTSUPP;
	}

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
static bool rtw_ops_can_aggregate_in_amsdu(struct ieee80211_hw *hw,
					   struct sk_buff *head,
					   struct sk_buff *skb)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_hal *hal = &rtwdev->hal;

	/* we don't want to enable TX AMSDU on 2.4G */
	if (hal->current_band_type == RTW_BAND_2G)
		return false;

	return true;
}
#endif

static void rtw_ops_sw_scan_start(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
				  const u8 *mac_addr)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_vif *rtwvif = (struct rtw_vif *)vif->drv_priv;

	mutex_lock(&rtwdev->mutex);
	rtw_core_scan_start(rtwdev, rtwvif, mac_addr, false);
	mutex_unlock(&rtwdev->mutex);
}

static void rtw_ops_sw_scan_complete(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif)
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_core_scan_complete(rtwdev, vif, false);
	mutex_unlock(&rtwdev->mutex);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 18, 0)
static void rtw_ops_mgd_prepare_tx(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
    #if (LINUX_VERSION_CODE >= KERNEL_VERSION(5,14,0))
				   struct ieee80211_prep_tx_info *info)
    #else
				   u16 duration)
    #endif
#else
static void rtw_ops_mgd_prepare_tx(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;
	bool rfk_pending;
	int ret;

	mutex_lock(&rtwdev->mutex);
	rtw_leave_lps_deep(rtwdev);
	ret = rtw_leave_ips(rtwdev);
	if (ret) {
		rtw_err(rtwdev, "failed to leave idle state for mgd tx\n");
		goto out;
	}
	rtw_info(rtwdev,
		 "MGMT_TX_DEBUG: mgd_prepare_tx idle=%d poweron=%d running=%d\n",
		 !!(hw->conf.flags & IEEE80211_CONF_IDLE),
		 test_bit(RTW_FLAG_POWERON, rtwdev->flags),
		 test_bit(RTW_FLAG_RUNNING, rtwdev->flags));
	rtw_coex_connect_notify(rtwdev, COEX_ASSOCIATE_START);

	/* For 8723BS SDIO: after coex switches to the PTA antenna path
	 * (BB_SEL_BTG=0x200), re-apply the current channel so RF
	 * registers get correct values on the PTA routing.  During scan
	 * each dwell set_channel naturally runs after the PTA switch.
	 * During connect the earlier set_channel (in the IPS-leave
	 * rtw_power_on path) ran on the BT path (BB_SEL_BTG=0x280),
	 * where RF writes silently fail — leaving RF01 at 0x783 with
	 * the TX/RX data path gated.  Auth/deauth frames reach the
	 * SDIO FIFO and drain, but no RF energy reaches the antenna.
	 */
	if (rtw8723bs_sdio(rtwdev)) {
		rtw_set_channel(rtwdev);
		rtwdev->need_rfk = false;
		rtw_coex_connect_notify(rtwdev, COEX_ASSOCIATE_START);
	} else {
		rfk_pending = rtwdev->need_rfk;
		if (!rtw8723bs_mgd_prepare_skip_fresh_rfk(rtwdev, rfk_pending))
			rtw_chip_prepare_tx(rtwdev);
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 14, 0)
	rtw8723bs_mgd_prepare_auth_join(rtwdev, vif, info);
#endif
out:
	mutex_unlock(&rtwdev->mutex);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
static int rtw_ops_set_rts_threshold(struct ieee80211_hw *hw, int radio_idx,
				     u32 value)
#else
static int rtw_ops_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
#endif
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtwdev->rts_threshold = value;
	mutex_unlock(&rtwdev->mutex);

	return 0;
}

static void rtw_ops_sta_statistics(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				   struct ieee80211_sta *sta,
				   struct station_info *sinfo)
{
	struct rtw_sta_info *si = (struct rtw_sta_info *)sta->drv_priv;

	sinfo->txrate = si->ra_report.txrate;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
}



static void rtw_ops_flush(struct ieee80211_hw *hw,
			  struct ieee80211_vif *vif,
			  u32 queues, bool drop)
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_leave_lps_deep(rtwdev);

	rtw_hci_flush_queues(rtwdev, queues, drop);
	rtw_mac_flush_queues(rtwdev, queues, drop);
	mutex_unlock(&rtwdev->mutex);
}

struct rtw_iter_bitrate_mask_data {
	struct rtw_dev *rtwdev;
	struct ieee80211_vif *vif;
	const struct cfg80211_bitrate_mask *mask;
};

static void rtw_ra_mask_info_update_iter(void *data, struct ieee80211_sta *sta)
{
	struct rtw_iter_bitrate_mask_data *br_data = data;
	struct rtw_sta_info *si = (struct rtw_sta_info *)sta->drv_priv;

	if (si->vif != br_data->vif)
		return;

	/* free previous mask setting */
	kfree(si->mask);
	si->mask = kmemdup(br_data->mask, sizeof(struct cfg80211_bitrate_mask),
			   GFP_ATOMIC);
	if (!si->mask) {
		si->use_cfg_mask = false;
		return;
	}

	si->use_cfg_mask = true;
	rtw_update_sta_info(br_data->rtwdev, si, true);
}

static void rtw_ra_mask_info_update(struct rtw_dev *rtwdev,
				    struct ieee80211_vif *vif,
				    const struct cfg80211_bitrate_mask *mask)
{
	struct rtw_iter_bitrate_mask_data br_data;

	br_data.rtwdev = rtwdev;
	br_data.vif = vif;
	br_data.mask = mask;
	rtw_iterate_stas(rtwdev, rtw_ra_mask_info_update_iter, &br_data);
}

static int rtw_ops_set_bitrate_mask(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif,
				    const struct cfg80211_bitrate_mask *mask)
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_ra_mask_info_update(rtwdev, vif, mask);
	mutex_unlock(&rtwdev->mutex);

	return 0;
}


static int rtw_ops_set_antenna(struct ieee80211_hw *hw,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
			       int radio_idx,
#endif
			       u32 tx_antenna,
			       u32 rx_antenna)
{
	struct rtw_dev *rtwdev = hw->priv;
	const struct rtw_chip_info *chip = rtwdev->chip;
	int ret;

	if (!chip->ops->set_antenna)
		return -EOPNOTSUPP;

	mutex_lock(&rtwdev->mutex);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
	ret = chip->ops->set_antenna(rtwdev, -1, tx_antenna, rx_antenna);
#else
	ret = chip->ops->set_antenna(rtwdev, tx_antenna, rx_antenna);
#endif
	mutex_unlock(&rtwdev->mutex);

	return ret;
}

static int rtw_ops_get_antenna(struct ieee80211_hw *hw,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
			       int radio_idx,
#endif
			       u32 *tx_antenna,
			       u32 *rx_antenna)
{
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_hal *hal = &rtwdev->hal;

	*tx_antenna = hal->antenna_tx;
	*rx_antenna = hal->antenna_rx;

	return 0;
}

#ifdef CONFIG_PM
static int rtw_ops_suspend(struct ieee80211_hw *hw,
			   struct cfg80211_wowlan *wowlan)
{
	struct rtw_dev *rtwdev = hw->priv;
	int ret;

	mutex_lock(&rtwdev->mutex);
	ret = rtw_wow_suspend(rtwdev, wowlan);
	if (ret)
		rtw_err(rtwdev, "failed to suspend for wow %d\n", ret);
	mutex_unlock(&rtwdev->mutex);

	return ret ? 1 : 0;
}

static int rtw_ops_resume(struct ieee80211_hw *hw)
{
	struct rtw_dev *rtwdev = hw->priv;
	int ret;

	mutex_lock(&rtwdev->mutex);
	ret = rtw_wow_resume(rtwdev);
	if (ret)
		rtw_err(rtwdev, "failed to resume for wow %d\n", ret);
	mutex_unlock(&rtwdev->mutex);

	return ret ? 1 : 0;
}

static void rtw_ops_set_wakeup(struct ieee80211_hw *hw, bool enabled)
{
	struct rtw_dev *rtwdev = hw->priv;

	device_set_wakeup_enable(rtwdev->dev, enabled);
}
#endif

static void rtw_reconfig_complete(struct ieee80211_hw *hw,
				  enum ieee80211_reconfig_type reconfig_type)
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	if (reconfig_type == IEEE80211_RECONFIG_TYPE_RESTART)
		clear_bit(RTW_FLAG_RESTARTING, rtwdev->flags);
	mutex_unlock(&rtwdev->mutex);
}

static int rtw_ops_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			   struct ieee80211_scan_request *req)
{
	struct rtw_dev *rtwdev = hw->priv;
	int ret;

	if (!rtw_fw_feature_check(&rtwdev->fw, FW_FEATURE_SCAN_OFFLOAD))
		return 1;

	if (test_bit(RTW_FLAG_SCANNING, rtwdev->flags))
		return -EBUSY;

	mutex_lock(&rtwdev->mutex);
	rtw_hw_scan_start(rtwdev, vif, req);
	ret = rtw_hw_scan_offload(rtwdev, vif, true);
	if (ret) {
		rtw_hw_scan_abort(rtwdev);
		rtw_err(rtwdev, "HW scan failed with status: %d\n", ret);
	}
	mutex_unlock(&rtwdev->mutex);

	return ret;
}

static void rtw_ops_cancel_hw_scan(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif)
{
	struct rtw_dev *rtwdev = hw->priv;

	if (!rtw_fw_feature_check(&rtwdev->fw, FW_FEATURE_SCAN_OFFLOAD))
		return;

	if (!test_bit(RTW_FLAG_SCANNING, rtwdev->flags))
		return;

	mutex_lock(&rtwdev->mutex);
	rtw_hw_scan_abort(rtwdev);
	mutex_unlock(&rtwdev->mutex);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
static int rtw_ops_set_sar_specs(struct ieee80211_hw *hw,
				 const struct cfg80211_sar_specs *sar)
{
	struct rtw_dev *rtwdev = hw->priv;

	mutex_lock(&rtwdev->mutex);
	rtw_set_sar_specs(rtwdev, sar);
	mutex_unlock(&rtwdev->mutex);

	return 0;
}
#endif

static void rtw_ops_sta_rc_update(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 13, 0)
				  struct ieee80211_sta *sta, u32 changed)
#else
				  struct ieee80211_link_sta *link_sta,
				  u32 changed)
#endif
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
	struct ieee80211_sta *sta = link_sta->sta;
#endif
	struct rtw_dev *rtwdev = hw->priv;
	struct rtw_sta_info *si = (struct rtw_sta_info *)sta->drv_priv;

	if (changed & IEEE80211_RC_BW_CHANGED)
		ieee80211_queue_work(rtwdev->hw, &si->rc_work);
}

const struct ieee80211_ops rtw_ops = {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	.add_chanctx = ieee80211_emulate_add_chanctx,
	.remove_chanctx = ieee80211_emulate_remove_chanctx,
	.change_chanctx = ieee80211_emulate_change_chanctx,
	.switch_vif_chanctx = ieee80211_emulate_switch_vif_chanctx,
#endif
	.tx			= rtw_ops_tx,
	.wake_tx_queue		= rtw_ops_wake_tx_queue,
	.start			= rtw_ops_start,
	.stop			= rtw_ops_stop,
	.config			= rtw_ops_config,
	.add_interface		= rtw_ops_add_interface,
	.remove_interface	= rtw_ops_remove_interface,
	.change_interface	= rtw_ops_change_interface,
	.configure_filter	= rtw_ops_configure_filter,
	.bss_info_changed	= rtw_ops_bss_info_changed,
	.start_ap		= rtw_ops_start_ap,
	.stop_ap		= rtw_ops_stop_ap,
	.conf_tx		= rtw_ops_conf_tx,
	.sta_add		= rtw_ops_sta_add,
	.sta_remove		= rtw_ops_sta_remove,
	.set_tim		= rtw_ops_set_tim,
	.set_key		= rtw_ops_set_key,
	.ampdu_action		= rtw_ops_ampdu_action,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
	.can_aggregate_in_amsdu	= rtw_ops_can_aggregate_in_amsdu,
#endif
	.sw_scan_start		= rtw_ops_sw_scan_start,
	.sw_scan_complete	= rtw_ops_sw_scan_complete,
	.mgd_prepare_tx		= rtw_ops_mgd_prepare_tx,
	.set_rts_threshold	= rtw_ops_set_rts_threshold,
	.sta_statistics		= rtw_ops_sta_statistics,
	.flush			= rtw_ops_flush,
	.set_bitrate_mask	= rtw_ops_set_bitrate_mask,
	.set_antenna		= rtw_ops_set_antenna,
	.get_antenna		= rtw_ops_get_antenna,
	.reconfig_complete	= rtw_reconfig_complete,
	.hw_scan		= rtw_ops_hw_scan,
	.cancel_hw_scan		= rtw_ops_cancel_hw_scan,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 13, 0)
	.sta_rc_update		= rtw_ops_sta_rc_update,
#else
	.link_sta_rc_update	= rtw_ops_sta_rc_update,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 11, 0)
	.set_sar_specs          = rtw_ops_set_sar_specs,
#endif
#ifdef CONFIG_PM
	.suspend		= rtw_ops_suspend,
	.resume			= rtw_ops_resume,
	.set_wakeup		= rtw_ops_set_wakeup,
#endif
};
EXPORT_SYMBOL(rtw_ops);
