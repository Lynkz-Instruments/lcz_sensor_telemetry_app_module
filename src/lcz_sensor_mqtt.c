/**
 * @file lcz_sensor_mqtt.c
 * @brief Process BLE advertisements for Laird Connectivity sensors,
 * publish advertisements to MQTT server.
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(lcz_sensor_app_mqtt, CONFIG_LCZ_SENSOR_TELEM_APP_LOG_LEVEL);

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <zephyr.h>
#include <init.h>
#include <sys/util.h>

#include "attr.h"
#include "lcz_lwm2m_gateway_obj.h"
#include "lcz_mqtt.h"
#include "lcz_sensor_event.h"
#include "lcz_sensor_adv_format.h"
#include "lcz_sensor_adv_match.h"
#include "lcz_snprintk.h"

/**************************************************************************************************/
/* Local Constant, Macro and Type Definitions                                                     */
/**************************************************************************************************/
/* prefix + postfix + nul */
#define AD_OVERHEAD_SIZE                                                                           \
	(ATTR_MQTT_BLE_PREFIX_MAX_STR_SIZE + ATTR_MQTT_BLE_POSTFIX_MAX_STR_SIZE + 1)

#define AD_LIST_SIZE CONFIG_LCZ_MQTT_BLE_AD_LIST_SIZE

/* Size of string to hold advertisement as hexadecimal string */
#define AD_STRING_SIZE (64 + 1)

struct ad_list {
	uint8_t data[AD_OVERHEAD_SIZE + AD_LIST_SIZE];
	size_t index;
	size_t postfix_index;
	struct k_sem sem;
};

struct mqtt_ble {
	bool restart;
	struct lcz_mqtt_user agent;
	bool mqtt_connected;
	struct k_work_delayable publish_work;
	struct ad_list ad_list;
};

/**************************************************************************************************/
/* Local Data Definitions                                                                         */
/**************************************************************************************************/
static struct mqtt_ble mb;

/**************************************************************************************************/
/* Local Function Prototypes                                                                      */
/**************************************************************************************************/
static void mqtt_ack_callback(int status);
static void mqtt_connect_callback(int status);
static void mqtt_disconnect_callback(int status);

static void publish_list(struct k_work *work);
static void reschedule_publish(k_timeout_t delay);
static int append_ad_list(uint8_t *ad, uint8_t ad_len);
static int append_str(const char *str, bool quote);
static int add_delimiter(void);
static int add_prefix(void);
static int add_postfix(void);
static void discard_postfix(void);
static void flush_ad_list(void);

/**************************************************************************************************/
/* SYS INIT                                                                                       */
/**************************************************************************************************/
static int lcz_sensor_app_mqtt_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	k_sem_init(&mb.ad_list.sem, 1, 1);

	mb.restart = true;

	mb.agent.ack_callback = mqtt_ack_callback;
	mb.agent.connect_callback = mqtt_connect_callback;
	mb.agent.disconnect_callback = mqtt_disconnect_callback;
	lcz_mqtt_register_user(&mb.agent);

	k_work_init_delayable(&mb.publish_work, publish_list);
	reschedule_publish(K_SECONDS(CONFIG_LCZ_MQTT_BLE_FIRST_PUBLISH));

	return 0;
}
SYS_INIT(lcz_sensor_app_mqtt_init, APPLICATION, CONFIG_LCZ_SENSOR_APP_INIT_PRIORITY);

/**************************************************************************************************/
/* Local Function Definitions                                                                     */
/**************************************************************************************************/
static int append_str(const char *str, bool quote)
{
	int r = 0;
	size_t len = strlen(str);

	if (len == 0) {
		/* don't do anything */
	} else if ((mb.ad_list.index + len + (quote ? 2 : 0)) < sizeof(mb.ad_list.data) - 1) {
		if (quote) {
			mb.ad_list.data[mb.ad_list.index++] = '"';
		}
		memcpy(&mb.ad_list.data[mb.ad_list.index], str, len);
		mb.ad_list.index += len;
		if (quote) {
			mb.ad_list.data[mb.ad_list.index++] = '"';
		}
		/* Publish (and logging) require a terminated string */
		mb.ad_list.data[mb.ad_list.index] = '\0';
	} else {
		LOG_ERR("Unable to append to AD list size: %u", len);
		r = -ENOMEM;
	}

	return r;
}

static void flush_ad_list(void)
{
	mb.ad_list.index = 0;
	mb.ad_list.postfix_index = 0;
}

static int add_delimiter(void)
{
	return append_str(attr_get_quasi_static(ATTR_ID_mqtt_ble_delimiter), false);
}

static int add_prefix(void)
{
	return append_str(attr_get_quasi_static(ATTR_ID_mqtt_ble_prefix), false);
}

static int add_postfix(void)
{
	/* save index for use if publish fails */
	mb.ad_list.postfix_index = mb.ad_list.index;

	return append_str(attr_get_quasi_static(ATTR_ID_mqtt_ble_postfix), false);
}

static void discard_postfix(void)
{
	mb.ad_list.index = mb.ad_list.postfix_index;
}

static int append_ad_list(uint8_t *ad, uint8_t ad_len)
{
	uint8_t hex_chunk[AD_STRING_SIZE];
	size_t len;
	int r = 0;

	do {
		if (mb.ad_list.index == 0) {
			r = add_prefix();
		} else {
			r = add_delimiter();
		}
		if (r < 0) {
			break;
		}

		/* Compare against list size (not .data size) so that there is
		 * always room for prefix and postfix.
		 */
		if ((mb.ad_list.index + (ad_len * 2)) > AD_LIST_SIZE) {
			r = -ENOMEM;
			LOG_ERR("AD list full");
			reschedule_publish(K_NO_WAIT);
			break;
		}

		len = bin2hex(ad, ad_len, hex_chunk, sizeof(hex_chunk));
		if (len == (ad_len * 2)) {
			r = append_str(hex_chunk, attr_get_bool(ATTR_ID_mqtt_ble_quote));
		} else {
			LOG_ERR("Unexpected ad length %d", len);
			r = -EINVAL;
		}
		if (r < 0) {
			break;
		}

		if (mb.ad_list.index >= CONFIG_LCZ_MQTT_BLE_PUBLISH_THRESHOLD) {
			LOG_DBG("Ad list publish threshold met");
			reschedule_publish(K_NO_WAIT);
		}

	} while (0);

	return r;
}

static void publish_list(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mqtt_ble *p = CONTAINER_OF(dwork, struct mqtt_ble, publish_work);
	int r;
	__ASSERT(p != &mb, LOG_ERR("Invalid pointer"));

	if (!attr_get_bool(ATTR_ID_mqtt_ble_enable)) {
		lcz_mqtt_disconnect();
		return;
	}

	reschedule_publish(K_SECONDS(CONFIG_LCZ_MQTT_PUBLISH_RATE));

	if (p->ad_list.index == 0) {
		LOG_DBG("Nothing to send");
		return;
	}

	k_sem_take(&mb.ad_list.sem, K_FOREVER);

	r = add_postfix();
	if (r < 0) {
		/* If the postfix can't be added then flush the list (this shouldn't happen) */
		LOG_ERR("%s can't append postfix to ad buffer", __func__);
		flush_ad_list();
	} else {
		r = lcz_mqtt_send_string(p->ad_list.data,
					 attr_get_quasi_static(ATTR_ID_mqtt_ble_topic), &p->agent);
		if (r < 0) {
			LOG_ERR("%s status: %d", __func__, r);
		} else {
			LOG_DBG("status: %d", r);
		}
	}

	if (attr_get_uint32(ATTR_ID_mqtt_publish_qos, 0) == 0) {
		/* Ack requires qos 1 (or 2) */
		flush_ad_list();
		k_sem_give(&mb.ad_list.sem);
	} else if (r < 0) {
		/* Free on error, otherwise wait for publish ack. */
		discard_postfix();
		k_sem_give(&mb.ad_list.sem);
	}
}

static void reschedule_publish(k_timeout_t delay)
{
	int r;

	r = k_work_reschedule(&mb.publish_work, delay);
	if (r < 0) {
		LOG_ERR("Unable to schedule MQTT AD publish: %d", r);
	}
}

static void mqtt_ack_callback(int status)
{
	if (status < 0 && attr_get_bool(ATTR_ID_mqtt_ble_enable)) {
		LOG_ERR("MQTT Publish (ack) error: %d", status);
	} else {
		LOG_INF("MQTT Ack id: %d", status);
	}

	flush_ad_list();

	k_sem_give(&mb.ad_list.sem);
}

static void mqtt_connect_callback(int status)
{
	mb.mqtt_connected = true;
}

static void mqtt_disconnect_callback(int status)
{
	mb.mqtt_connected = false;
}

/**************************************************************************************************/
/* Global Function Definitions                                                                    */
/**************************************************************************************************/
int lcz_sensor_mqtt_telemetry(int idx, uint8_t *ad, uint8_t ad_len)
{
	int r = 0;

	/* Sensor enable is a control point that can be used to indicate that
	 * the gateway has been commissioned.
	 * Here it is used to prevent processing of ads and to reload certs.
	 */
	if (!attr_get_bool(ATTR_ID_mqtt_ble_enable)) {
		if (!mb.restart) {
			/* Disconnect and connect in system workq thread instead of callback */
			reschedule_publish(K_NO_WAIT);
			lcz_mqtt_unload_credentials();
			mb.restart = true;
		}
		return 0;
	}

	if (mb.restart) {
		if (k_sem_take(&mb.ad_list.sem, K_NO_WAIT) == 0) {
			flush_ad_list();
			k_sem_give(&mb.ad_list.sem);
			reschedule_publish(K_SECONDS(CONFIG_LCZ_MQTT_PUBLISH_RATE));
			mb.restart = false;
		}
	}

	/* If a publish is in progress, then try again later.
	 * Default configuration of Laird sensors is to publish each event
	 * for 10-15 seconds.
	 */
	if (k_sem_take(&mb.ad_list.sem, K_NO_WAIT) == 0) {
		append_ad_list(ad, ad_len);
		k_sem_give(&mb.ad_list.sem);
	} else {
		r = -EAGAIN;
	}

	return r;
}