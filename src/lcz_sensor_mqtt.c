/**
 * @file lcz_sensor_mqtt.c
 * @brief Process BLE advertisements for Laird Connectivity sensors,
 * publish advertisements to MQTT server.
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_sensor_app_mqtt, CONFIG_LCZ_SENSOR_TELEM_APP_LOG_LEVEL);

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <zephyr/zephyr.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>

#include <file_system_utilities.h>
#include <attr.h>
#include <lcz_lwm2m_gateway_obj.h>
#include <lcz_mqtt.h>
#include <lcz_sensor_event.h>
#include <lcz_sensor_adv_format.h>
#include <lcz_sensor_adv_match.h>
#include <lcz_snprintk.h>

#include "lcz_sensor_app.h"

/**************************************************************************************************/
/* Local Constant, Macro and Type Definitions                                                     */
/**************************************************************************************************/
/* prefix + postfix + nul */
#define AD_OVERHEAD_SIZE                                                                           \
	(ATTR_MQTT_BLE_PREFIX_MAX_STR_SIZE + ATTR_MQTT_BLE_POSTFIX_MAX_STR_SIZE + 1)

#define AD_LIST_SIZE CONFIG_LCZ_MQTT_BLE_AD_LIST_SIZE

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

#define LYNKZ_RECORD_SIZE 91
/**************************************************************************************************/
/* Local Data Definitions                                                                         */
/**************************************************************************************************/
static struct mqtt_ble mb;

//The following values can only be used after obtaining the ad_list semaphore
uint16_t mqtt_history_offset = 0;

uint16_t hist_file_index = 0;
uint16_t hist_block_index = 0;

uint16_t live_file_index = 0;
uint16_t live_block_index = 0;



bool reachedEOF = false;

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

static int write_to_history(const uint8_t *data, uint16_t length);
static int prepare_history_ad_list();

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

	//Get the last log entry
	if(fsu_single_entry_exists("/lfs1", "mqtt", 1)>=0){
		LOG_INF("Mqtt file found");
		mqtt_history_offset = fsu_get_last_history_file("/lfs1/mqtt")+1;
		hist_file_index = mqtt_history_offset;
		live_file_index = mqtt_history_offset;
	}

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

static int append_bin(uint8_t *bin, uint16_t len)
{
	int r = 0;

	if (len == 0) {
		/* don't do anything */
	} else if ((mb.ad_list.index + len) < sizeof(mb.ad_list.data) - 1) {

		memcpy(&mb.ad_list.data[mb.ad_list.index], bin, len);
		mb.ad_list.index += len;

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
	//if (attr_get_quasi_static(ATTR_ID_mqtt_ble_postfix) == "time")
	//{
	uint8_t timestamp_chunk[5]; // 4 for timestamp (32 bytes) + end of line character
	uint32_t timestamp = lcz_qrtc_get_epoch();
	uint8_t bytes[4];
	bytes[0] = (timestamp >> 24) & 0xFF;
	bytes[1] = (timestamp >> 16) & 0xFF;
	bytes[2] = (timestamp >> 8) & 0xFF;
	bytes[3] = timestamp & 0xFF;

	bin2hex(bytes, sizeof(bytes), timestamp_chunk, sizeof(timestamp_chunk));
	return append_str(timestamp_chunk, false);
	//}
	//return append_str(attr_get_quasi_static(ATTR_ID_mqtt_ble_postfix), false);
}

static int add_postfix_bin(void)
{
	/* save index for use if publish fails */
	mb.ad_list.postfix_index = mb.ad_list.index;
	//if (attr_get_quasi_static(ATTR_ID_mqtt_ble_postfix) == "time")
	//{
	uint32_t timestamp = lcz_qrtc_get_epoch();
	uint8_t bytes[4];
	bytes[0] = (timestamp >> 24) & 0xFF;
	bytes[1] = (timestamp >> 16) & 0xFF;
	bytes[2] = (timestamp >> 8) & 0xFF;
	bytes[3] = timestamp & 0xFF;

	return append_bin(bytes, 4);
	//}
	//return append_str(attr_get_quasi_static(ATTR_ID_mqtt_ble_postfix), false);
}

static void discard_postfix(void)
{
	mb.ad_list.index = mb.ad_list.postfix_index;
}

static int append_ad_list(uint8_t *ad, uint8_t ad_len)
{
	uint8_t hex_chunk[LYNKZ_RECORD_SIZE];
	size_t len;
	int r = 0;
	LOG_INF("Live File Index: %d, Live Block Index: %d", live_file_index, live_block_index);
	LOG_INF("Hist File Index: %d, Hist Block Index: %d", hist_file_index, hist_block_index);
	do {
		if (mb.ad_list.index == 0) {
			r = add_prefix();
		} else {
			r = add_delimiter();
		}
		if (r < 0) {
			break;
		}
		//Check if we must chunk the incoming data
		if(ad_len > LYNKZ_RECORD_SIZE){
			uint8_t iteration = 0;
			uint8_t writeSize = LYNKZ_RECORD_SIZE-6-1-1; //Ble + PacketType + ChunkId
			//Setting Bluetooth address and packet type
			memcpy(hex_chunk, ad, 7);
			while(7+writeSize*iteration<ad_len)
			{
				hex_chunk[7] = iteration;
				if(ad_len-7-writeSize*iteration >= writeSize){
					memcpy(&hex_chunk[8], &ad[7+ iteration*writeSize], writeSize); //CONTINUE HERE
					r = write_to_history(hex_chunk, LYNKZ_RECORD_SIZE);
				}
				else{
					memset(&hex_chunk[8], 0, sizeof(hex_chunk)-8); //Pad the rest with 0
					memcpy(&hex_chunk[8], &ad[7+ iteration*writeSize], ad_len-7-writeSize*iteration);
					r = write_to_history(hex_chunk, LYNKZ_RECORD_SIZE);
				}
				iteration++;
			}
		}
		else{
			r = write_to_history(ad, ad_len);
		}
		if (r < 0) {
			break;
		}
		if(live_file_index != hist_file_index || live_block_index> hist_block_index+10)
		{
			LOG_DBG("Ad list publish threshold met");
			reschedule_publish(K_NO_WAIT);
		}
		if (mb.ad_list.index >= CONFIG_LCZ_MQTT_BLE_PUBLISH_THRESHOLD) {
			LOG_DBG("Ad list publish threshold met");
			reschedule_publish(K_NO_WAIT);
		}

	} while (0);

	return r;
}

static int prepare_history_ad_list()
{
	uint8_t hex_chunk[LYNKZ_RECORD_SIZE];
	size_t len;
	int r = 0;

	flush_ad_list();
	char path[94] ;
	sprintf(path, "/lfs1/mqtt/history%d", hist_file_index);

	//Trying to place a maximum of 10 entries (1040 bytes) in the ad buffer (to send by mqtt)
	for(int blockIndex = hist_block_index; blockIndex< hist_block_index+10; blockIndex++)
	{
		r = fsu_read_abs_block(path, blockIndex*LYNKZ_RECORD_SIZE, hex_chunk, sizeof(hex_chunk));
		if(r<0){
			LOG_ERR("Mqtt History Read error");
			break;
		}
		else if(r != sizeof(hex_chunk))
		{
			LOG_WRN("EOF or Mqtt History data length mismatch");
			//Either we have reached EOF, or the file is corrupted. We switch to the next file if it exists.
			if(live_file_index != hist_file_index){
				reachedEOF = true;
			}
			break;
		}
		r = append_bin(hex_chunk, LYNKZ_RECORD_SIZE);
	}
}

static int write_to_history(const uint8_t *data, uint16_t len)
{
	uint8_t ret = 0;
	uint8_t tmp_buffer[LYNKZ_RECORD_SIZE];
	memset(tmp_buffer, 0, sizeof(tmp_buffer));

	uint32_t timestamp = lcz_qrtc_get_epoch();
	uint16_t fileIndex = (int)(timestamp/(60*60*24)) + mqtt_history_offset; //Days since started

	//When switching history file, set the current block and file index
	if(fileIndex > live_file_index){
		live_file_index = fileIndex;
		live_block_index = 0;
	}

	//Max absolute path is 16+48-1
	char path[] = "/lfs1/mqtt";
	char filename[48] = "history";
	sprintf(filename + strlen(filename), "%d", fileIndex);
	fsu_mkdir_abs(path, false);

	if(len>LYNKZ_RECORD_SIZE)
	{
		LOG_ERR("History write too big. Max Size: %d, Attempting: %d", LYNKZ_RECORD_SIZE, len);
		return -1;
	}

	memcpy(tmp_buffer, data, len);
	fsu_append(path, filename, tmp_buffer, LYNKZ_RECORD_SIZE);
	
	//New data added, increasing live block index
	live_block_index++;

	//Delete oldest file if more than 7 files exist
	if(fileIndex >= 8)
	{
		strcpy(filename, "history");
		sprintf(filename + strlen(filename), "%d", fileIndex-8);
		if(fsu_single_entry_exists(path, filename, 0)>=0){
			fsu_delete(path, filename);
		}
	}
	return ret;
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
	k_sem_take(&mb.ad_list.sem, K_FOREVER);

	if (live_block_index == hist_block_index && live_file_index == hist_file_index) {
		LOG_DBG("Nothing to send");
		k_sem_give(&mb.ad_list.sem);
		return;
	}
	flush_ad_list();
	r = prepare_history_ad_list();

	if (p->ad_list.index == 0) {
		LOG_DBG("Nothing to send");
		k_sem_give(&mb.ad_list.sem);
		return;
	}

	r = add_postfix_bin();
	if (r < 0) {
		/* If the postfix can't be added then flush the list (this shouldn't happen) */
		LOG_ERR("%s can't append postfix to ad buffer", __func__);
		flush_ad_list();
	} else {
		r = lcz_mqtt_send_binary(p->ad_list.data, p->ad_list.index,
					 attr_get_quasi_static(ATTR_ID_mqtt_ble_topic), &p->agent);
		if (r < 0) {
			LOG_ERR("%s status: %d", __func__, r);
		} else {
			LOG_DBG("status: %d", r);
		}
	}

	if (attr_get_uint32(ATTR_ID_mqtt_publish_qos, 0) == 0) {
		/* Ack requires qos 1 (or 2) */

	} else if (r < 0) {
		/* Free on error, otherwise wait for publish ack. */
		discard_postfix();
	}
	else if (r==0){
		if(reachedEOF)
		{
			hist_block_index = 0;
			hist_file_index++;
		}
		else{
			uint8_t len =  (int)(strlen(p->ad_list.data)/(LYNKZ_RECORD_SIZE-1));
			hist_block_index += len;
		}
	}
	flush_ad_list();
	k_sem_give(&mb.ad_list.sem);
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