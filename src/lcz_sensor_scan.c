/**
 * @file lcz_sensor_scan.c
 * @brief Process BLE advertisements for Laird Connectivity sensors.
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(lcz_sensor_app_scan, CONFIG_LCZ_SENSOR_TELEM_APP_LOG_LEVEL);

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <zephyr.h>
#include <init.h>
#include <bluetooth/addr.h>

#include "attr.h"
#include "lcz_lwm2m_gateway_obj.h"
#include "lcz_lwm2m_util.h"
#include "lcz_bt_scan.h"
#include "lcz_sensor_event.h"
#include "lcz_sensor_adv_format.h"
#include "lcz_sensor_adv_match.h"
#if defined(CONFIG_LCZ_PKI_AUTH_SMP_CENTRAL)
#include "lcz_sensor_adv_enc.h"
#endif
#include "lcz_sensor_app.h"

#if defined(CONFIG_LCZ_LWM2M_GATEWAY_PROXY)
#include "lcz_lwm2m_gateway_proxy.h"
#endif

#if defined(CONFIG_LCZ_SENSOR_APP_LED)
#include "lcz_led.h"
#include "led_config.h"
#endif

/**************************************************************************************************/
/* Local Constant, Macro and Type Definitions                                                     */
/**************************************************************************************************/
/* Advertisement flag in DM advertisements indicating LwM2M data ready */
#define LWM2M_DATA_READY_FLAG BIT(0)

#define SCAN_RESTART_DELAY_SECONDS 2

#define MAX_INSTANCES CONFIG_LCZ_LWM2M_GATEWAY_MAX_INSTANCES

struct ble_sensor_data {
	uint8_t last_record_type;
	uint16_t last_event_id;
	int product_id;
};

#if defined(CONFIG_LCZ_SENSOR_APP_STATS)
struct stats {
	uint32_t raw_ads;
	uint32_t legacy_ads;
	uint32_t legacy_scan_rsps;
	uint32_t legacy_coded_ads;
	uint32_t dm_1m_ads;
	uint32_t dm_coded_unenc_ads;
	uint32_t dm_coded_enc_ads;
	uint32_t non_network_ads;
	uint32_t invalid_device_ads;
	uint32_t duplicate_ads;
	uint32_t dm_flag_set;
	uint32_t telem_sent;
	uint32_t name_updates;
	uint32_t decrypt_failed;
};
#define INCR_STAT(field) lbs.stats.field += 1
#else
#define INCR_STAT(field)
#endif

static struct bt_le_scan_param scan_parameters = BT_LE_SCAN_PARAM_INIT(
	BT_LE_SCAN_TYPE_ACTIVE, (BT_LE_SCAN_OPT_CODED | BT_LE_SCAN_OPT_FILTER_DUPLICATE),
	CONFIG_LCZ_BT_SCAN_DEFAULT_INTERVAL, CONFIG_LCZ_BT_SCAN_DEFAULT_WINDOW);

/**************************************************************************************************/
/* Local Data Definitions                                                                         */
/**************************************************************************************************/

static struct {
	int scan_user_id;
	struct k_work_delayable scan_restart_work;
	struct lwm2m_obj_agent agent;
	struct ble_sensor_data table[MAX_INSTANCES];
#if defined(CONFIG_LCZ_SENSOR_APP_STATS)
	struct stats stats;
#endif
} lbs;

/**************************************************************************************************/
/* Local Function Prototypes                                                                      */
/**************************************************************************************************/
static void ad_handler(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		       struct net_buf_simple *ad);

static void scan_resume(void);
static void scan_pause(void);
static void scan_restart_work_handler(struct k_work *work);

static bool valid_index(int idx);
static int get_index(const bt_addr_le_t *addr, bool add);

static uint16_t *get_flags(uint16_t protocol_id, AdHandle_t *handle);
static uint16_t *get_network_id(uint16_t protocol_id, AdHandle_t *handle);
static uint16_t *get_product_id(uint16_t protocol_id, AdHandle_t *handle);
static uint16_t *get_event_id(uint16_t protocol_id, AdHandle_t *handle);
static uint8_t *get_event_record_type(uint16_t protocol_id, AdHandle_t *handle);
static SensorEventData_t *get_event_data(uint16_t protocol_id, AdHandle_t *handle);

static bool is_in_network(uint16_t network_id);
static bool is_duplicate(int idx, uint16_t id, uint8_t record_type);
static void update_last_event(int idx, uint16_t id, uint8_t record_type);
static void name_handler(int idx, struct net_buf_simple *ad);
static int gw_obj_removed(int idx, void *context);

/**************************************************************************************************/
/* SYS INIT                                                                                       */
/**************************************************************************************************/
static int lcz_sensor_app_scan_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int idx;
	int r;

	for (idx = 0; idx < MAX_INSTANCES; idx++) {
		lbs.table[idx].product_id = INVALID_PRODUCT_ID;
	}

	/* Init delayed work to restart scanning if it fails */
	k_work_init_delayable(&(lbs.scan_restart_work), scan_restart_work_handler);

	if (!lcz_bt_scan_register(&lbs.scan_user_id, ad_handler)) {
		LOG_ERR("LWM2M sensor module failed to register with scan module");
	}

	r = lcz_bt_scan_update_parameters(lbs.scan_user_id, &scan_parameters);
	if (r < 0) {
		LOG_ERR("Unable to update scan parameters: %d", r);
	}
	r = lcz_bt_scan_start(lbs.scan_user_id);
	if (r < 0) {
		LOG_WRN("Scan start failed: %d. Retrying", r);
		k_work_reschedule(&(lbs.scan_restart_work), K_SECONDS(SCAN_RESTART_DELAY_SECONDS));
	}

	/* Find out when objects get deleted */
	lbs.agent.gw_obj_deleted = gw_obj_removed;
	lcz_lwm2m_util_register_agent(&lbs.agent);

	/* Register scan control functions with the CoAP proxy */
#if defined(CONFIG_LCZ_LWM2M_GATEWAY_PROXY)
	lcz_lwm2m_gateway_proxy_reg_scan_fns(scan_resume, scan_pause);
#endif

	return 0;
}
SYS_INIT(lcz_sensor_app_scan_init, APPLICATION, CONFIG_LCZ_SENSOR_APP_INIT_PRIORITY);

/**************************************************************************************************/
/* Occurs in BT RX Thread context                                                                 */
/**************************************************************************************************/
static void ad_handler(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		       struct net_buf_simple *ad)
{
	uint16_t protocol_id;
	int idx;
	AdHandle_t handle;
	uint16_t *flags_ptr;
	uint16_t *network_id_ptr;
	uint16_t *product_id_ptr;
	uint16_t *event_id_ptr;
	uint8_t *record_type_ptr;
	SensorEventData_t *sensor_data_ptr;
	int r = 0;

	INCR_STAT(raw_ads);
	handle = AdFind_Type(ad->data, ad->len, BT_DATA_MANUFACTURER_DATA, BT_DATA_INVALID);

	/* Determine the protocol ID for this advertisement */
	protocol_id = lcz_sensor_adv_match(ad, true, true);

	/* Update generic statistics */
	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		INCR_STAT(legacy_ads);
		break;

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		INCR_STAT(legacy_coded_ads);
		break;

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		INCR_STAT(legacy_scan_rsps);
		break;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
		INCR_STAT(dm_1m_ads);
		break;

	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		INCR_STAT(dm_coded_unenc_ads);
		break;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		INCR_STAT(dm_coded_enc_ads);
		break;

	case RESERVED_AD_PROTOCOL_ID:
	default:
		/* This isn't a supported protocol. Do nothing. */
		return;
	}

	/* Filter by network ID */
	network_id_ptr = get_network_id(protocol_id, &handle);
	if (network_id_ptr != NULL) {
		if (is_in_network(*network_id_ptr) == false) {
			INCR_STAT(non_network_ads);
			return;
		}
	}

	/* Get the device index from the database */
	if (protocol_id != BTXXX_1M_PHY_RSP_PROTOCOL_ID) {
		/* Allow the device to be created for an advertisement */
		idx = get_index(addr, true);
	} else {
		/* Do not allow the device to be created just for a scan response */
		idx = get_index(addr, false);
	}
	if (!valid_index(idx)) {
		INCR_STAT(invalid_device_ads);
		return;
	}

	/* Store the name and product ID (if present) */
	name_handler(idx, ad);
	product_id_ptr = get_product_id(protocol_id, &handle);
	if (product_id_ptr != NULL) {
		lbs.table[idx].product_id = *product_id_ptr;
	}

#if defined(CONFIG_LCZ_LWM2M_GATEWAY_PROXY)
	/* Handle DM flag (for DM protocol IDs) */
	flags_ptr = get_flags(protocol_id, &handle);
	if ((flags_ptr != NULL) && (protocol_id == BTXXX_DM_1M_PHY_AD_PROTOCOL_ID ||
				    protocol_id == BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID ||
				    protocol_id == BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID)) {
		if ((*flags_ptr & LWM2M_DATA_READY_FLAG) != 0) {
			INCR_STAT(dm_flag_set);
			if (protocol_id == BTXXX_DM_1M_PHY_AD_PROTOCOL_ID) {
				lcz_lwm2m_gateway_proxy_device_ready(addr, false);
			} else {
				lcz_lwm2m_gateway_proxy_device_ready(addr, true);
			}
		}
	}
#endif

	/* This device is good. Update the lifetime in the database */
	if (lcz_lwm2m_gw_obj_set_lifetime(idx, CONFIG_LCZ_SENSOR_APP_TIMEOUT_SECONDS) != 0) {
		LOG_ERR("Unable to set lifetime");
	}

	/* Handle the event (if present) */
	event_id_ptr = get_event_id(protocol_id, &handle);
	if (event_id_ptr != NULL) {
		/* If the data is encrypted, decrypt it */
		if (protocol_id == BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID) {
			if (lcz_sensor_adv_decrypt(addr, (LczSensorDMEncrAd_t *)handle.pPayload) !=
			    0) {
				/*
				 * Let this fail silently without log messages. Decryption can fail for
				 * a couple of reasons: (1) we just don't have the keys (wrong gateway?),
				 * or (2) we have just updated the keys, but this is old advertising data
				 * that hasn't been updated yet.
				 */
				INCR_STAT(decrypt_failed);
				return;
			}
		}

		/*
         * Fetch the event
         *
         * These two functions have the same operation as the get_event_id() function, so
         * if we were able to get an event ID pointer, these functions will return valid
         * pointers as well.
         */
		record_type_ptr = get_event_record_type(protocol_id, &handle);
		sensor_data_ptr = get_event_data(protocol_id, &handle);

		/* Filter duplicate events */
		if (is_duplicate(idx, *event_id_ptr, *record_type_ptr)) {
			INCR_STAT(duplicate_ads);
			return;
		}

		/* Send to telemetry handler */
		INCR_STAT(telem_sent);
#if defined(CONFIG_LCZ_SENSOR_TELEM_LOG_VERBOSE)
		LOG_INF("%s idx: %d RSSI: %d id: %u", lcz_sensor_event_get_string(*record_type_ptr),
			idx, rssi, *event_id_ptr);
#endif
#if defined(CONFIG_LCZ_SENSOR_TELEM_LWM2M)
		lcz_sensor_lwm2m_telemetry(idx, lbs.table[idx].product_id, *record_type_ptr,
					   *sensor_data_ptr);
#endif
#if defined(CONFIG_LCZ_SENSOR_TELEM_MQTT)
		/* Send the entire advertisement payload to the telemetry handler */
		r = lcz_sensor_mqtt_telemetry(idx, handle.pPayload, handle.size);
#endif

		/* If MQTT publish is busy, don't update event id. */
		if (r != -EAGAIN) {
			update_last_event(idx, *event_id_ptr, *record_type_ptr);
		}

#if defined(CONFIG_LCZ_SENSOR_APP_LED)
		lcz_led_blink(BLE_LED, &BLE_ACTIVITY_LED_PATTERN);
#endif
	}
}

/**************************************************************************************************/
/* Local Function Definitions                                                                     */
/**************************************************************************************************/
static void scan_resume(void)
{
	int r;
	if (lbs.scan_user_id >= 0 && lcz_bt_scan_active() == false) {
		r = lcz_bt_scan_restart(lbs.scan_user_id);
		if (r != 0) {
			LOG_WRN("Scan restart failed: %d. Retrying", r);
			k_work_reschedule(&(lbs.scan_restart_work),
					  K_SECONDS(SCAN_RESTART_DELAY_SECONDS));
		}
	}
}

static void scan_pause(void)
{
	if (lbs.scan_user_id >= 0 && lcz_bt_scan_active() == true) {
		k_work_cancel_delayable(&(lbs.scan_restart_work));
		lcz_bt_scan_stop(lbs.scan_user_id);
	}
}

static void scan_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	scan_resume();
}

static bool valid_index(int idx)
{
	return (idx >= 0 && idx < MAX_INSTANCES);
}

static int get_index(const bt_addr_le_t *addr, bool add)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int idx;

	/* Check for the device in the database */
	idx = lcz_lwm2m_gw_obj_lookup_ble(addr);

	/* If it wasn't there, and we're allowed to add, attempt to add it */
	if (!valid_index(idx) && add) {
		/* Create the device */
		idx = lcz_lwm2m_gw_obj_create(addr);

		bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

		/* Limit logging for blocked devices */
		if (idx < 0) {
			if (idx != -EPERM || IS_ENABLED(CONFIG_LCZ_SENSOR_CREATE_LOG_VERBOSE)) {
				LOG_ERR("Gateway object create for %s failed: %d", addr_str, idx);
			}
		} else if (idx >= 0) {
			LOG_DBG("Gateway object create request %s: idx: %d inst: %d", addr_str, idx,
				lcz_lwm2m_gw_obj_get_instance(idx));
		}
	}

	if (idx >= MAX_INSTANCES) {
		LOG_ERR("Invalid index (%d)", idx);
		return -EPERM;
	}

	return idx;
}

static uint16_t *get_flags(uint16_t protocol_id, AdHandle_t *handle)
{
	static uint16_t value = 0;

	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdEvent_t *)handle->pPayload)->flags);
		return &value;

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdCoded_t *)handle->pPayload)->ad.flags);
		return &value;

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		break;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMUnencrAd_t *)handle->pPayload)->flags);
		return &value;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMEncrAd_t *)handle->pPayload)->flags);
		return &value;
	}

	return NULL;
}

static uint16_t *get_network_id(uint16_t protocol_id, AdHandle_t *handle)
{
	static uint16_t value = 0;

	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdEvent_t *)handle->pPayload)->networkId);
		return &value;

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdCoded_t *)handle->pPayload)->ad.networkId);
		return &value;

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		break;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMUnencrAd_t *)handle->pPayload)->networkId);
		return &value;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMEncrAd_t *)handle->pPayload)->networkId);
		return &value;
	}

	return NULL;
}

static uint16_t *get_product_id(uint16_t protocol_id, AdHandle_t *handle)
{
	static uint16_t value = 0;

	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		break;

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdCoded_t *)handle->pPayload)->rsp.productId);
		return &value;

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		value = (((LczSensorRsp_t *)handle->pPayload)->productId);
		return &value;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMUnencrAd_t *)handle->pPayload)->productId);
		return &value;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMEncrAd_t *)handle->pPayload)->productId);
		return &value;
	}

	return NULL;
}

static uint16_t *get_event_id(uint16_t protocol_id, AdHandle_t *handle)
{
	static uint16_t value = 0;

	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdEvent_t *)handle->pPayload)->id);
		return &value;

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdCoded_t *)handle->pPayload)->ad.id);
		return &value;

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		break;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		break;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMEncrAd_t *)handle->pPayload)->id);
		return &value;
	}

	return NULL;
}

static uint8_t *get_event_record_type(uint16_t protocol_id, AdHandle_t *handle)
{
	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		return &(((LczSensorAdEvent_t *)handle->pPayload)->recordType);

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		return &(((LczSensorAdCoded_t *)handle->pPayload)->ad.recordType);

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		break;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		break;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		return &(((LczSensorDMEncrAd_t *)handle->pPayload)->recordType);
		break;
	}

	return NULL;
}

static SensorEventData_t *get_event_data(uint16_t protocol_id, AdHandle_t *handle)
{
	static SensorEventData_t value;

	switch (protocol_id) {
	case BTXXX_1M_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdEvent_t *)handle->pPayload)->data);
		return &value;

	case BTXXX_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorAdCoded_t *)handle->pPayload)->ad.data);
		return &value;

	case BTXXX_1M_PHY_RSP_PROTOCOL_ID:
		break;

	case BTXXX_DM_1M_PHY_AD_PROTOCOL_ID:
	case BTXXX_DM_CODED_PHY_AD_PROTOCOL_ID:
		break;

	case BTXXX_DM_ENC_CODED_PHY_AD_PROTOCOL_ID:
		value = (((LczSensorDMEncrAd_t *)handle->pPayload)->data);
		return &value;
	}

	return NULL;
}

static bool is_in_network(uint16_t network_id)
{
	uint16_t attr_id = (uint16_t)attr_get_uint32(ATTR_ID_network_id_filter, 0);

	if (attr_id == 0 || network_id == attr_id) {
		return true;
	} else {
		return false;
	}
}

static bool is_duplicate(int idx, uint16_t id, uint8_t record_type)
{
	/* If both devices have just powered-up, don't filter event 0 */
	if ((id == lbs.table[idx].last_event_id) &&
	    (record_type == lbs.table[idx].last_record_type)) {
		return true;
	}

	return false;
}

/* Update the last event information for next time */
static void update_last_event(int idx, uint16_t id, uint8_t record_type)
{
	lbs.table[idx].last_event_id = id;
	lbs.table[idx].last_record_type = record_type;
}

static void name_handler(int idx, struct net_buf_simple *ad)
{
	AdHandle_t handle;
	int r;

	handle = AdFind_Name(ad->data, ad->len);
	if (handle.pPayload == NULL) {
		return;
	}

	/* If the LwM2M connection has been proxied or named already, then don't try to set name. */
	if (lcz_lwm2m_gw_obj_inst_created(idx)) {
		return;
	}

	r = lcz_lwm2m_gw_obj_set_endpoint_name(idx, handle.pPayload, handle.size);
	if (r == 0) {
		INCR_STAT(name_updates);
	}

	LOG_DBG("Set endpoint name in database[%d]: %d", idx, r);
}

static int gw_obj_removed(int idx, void *context)
{
	ARG_UNUSED(context);

	if (valid_index(idx)) {
		lbs.table[idx].product_id = INVALID_PRODUCT_ID;
	}

	return 0;
}
