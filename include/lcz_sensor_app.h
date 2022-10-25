/**
 * @file lcz_sensor_app.h
 * @brief LCZ sensor telemetry application
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */
#ifndef __LCZ_SENSOR_APP_H__
#define __LCZ_SENSOR_APP_H__

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include "lcz_sensor_adv_format.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************/
/* Global Function Prototypes                                                                     */
/**************************************************************************************************/
/**
 * @brief Send sensor telemetry via LwM2M
 *
 * @param[in] idx Device index (in gateway object database) for sensor
 * @param[in] record_type Event record type received
 * @param[in] data Event data
 */
void lcz_sensor_lwm2m_telemetry(int idx, uint16_t product_id, uint8_t record_type,
				SensorEventData_t data);

/**
 * @brief Send sensor telemetry via MQTT
 *
 * @param[in] idx Device index (in gateway object database) for sensor
 * @param[in] ad Raw advertisement data
 * @param[in] ad_len Raw advertisement length
 *
 * @returns 0 if advertisement was published/cached or -EAGAIN if this function should be
 * called again
 */
int lcz_sensor_mqtt_telemetry(int idx, uint8_t *ad, uint8_t ad_len);

#ifdef __cplusplus
}
#endif

#endif /* __LCZ_SENSOR_APP_H__ */
