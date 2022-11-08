/**
 * @file lcz_sensor_lwm2m.c
 * @brief Process BLE advertisements for Laird Connectivity sensors,
 * add LwM2M object instances, and update resource instances when values change.
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_sensor_app_lwm2m, CONFIG_LCZ_SENSOR_TELEM_APP_LOG_LEVEL);

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <zephyr/zephyr.h>
#include <zephyr/bluetooth/addr.h>

#include <lwm2m_resource_ids.h>


#include <lcz_lwm2m_util.h>
#include <lcz_sensor_adv_format.h>

#if defined(CONFIG_LCZ_LWM2M_TEMPERATURE)
#include <lcz_lwm2m_temperature.h>
#endif

#if defined(CONFIG_LCZ_LWM2M_CURRENT)
#include <lcz_lwm2m_current.h>
#endif

#if defined(CONFIG_LCZ_LWM2M_PRESSURE)
#include <lcz_lwm2m_pressure.h>
#endif

#if defined(CONFIG_LCZ_LWM2M_BATTERY)
#include <lcz_lwm2m_battery.h>
#include <lcz_lwm2m_battery_get_level.h>
#endif

#if defined(CONFIG_LCZ_LWM2M_FILL_LEVEL)
#include <lcz_lwm2m_fill_level.h>
#endif

#include "lcz_sensor_app.h"

/**************************************************************************************************/
/* Global Function Definitions                                                                    */
/**************************************************************************************************/
void lcz_sensor_lwm2m_telemetry(int idx, uint16_t product_id, uint8_t record_type,
				SensorEventData_t data)
{
	uint16_t offset = 0;
	double d = 0.0;
	uint8_t percentage;
	int r = -EPERM;

	switch (record_type) {
	case SENSOR_EVENT_TEMPERATURE:
#if defined(CONFIG_LCZ_LWM2M_TEMPERATURE)
		d = (((double)((int16_t)data.u16)) / 100.0);
		r = lcz_lwm2m_managed_temperature_set(idx, offset, d);
#endif
		break;

	case SENSOR_EVENT_TEMPERATURE_1:
	case SENSOR_EVENT_TEMPERATURE_2:
	case SENSOR_EVENT_TEMPERATURE_3:
	case SENSOR_EVENT_TEMPERATURE_4:
#if defined(CONFIG_LCZ_LWM2M_TEMPERATURE)
		d = (double)data.f;
		offset = (record_type - SENSOR_EVENT_TEMPERATURE_1);
		r = lcz_lwm2m_managed_temperature_set(idx, offset, d);
#endif
		break;

	case SENSOR_EVENT_BATTERY_GOOD:
	case SENSOR_EVENT_BATTERY_BAD:
#if defined(CONFIG_LCZ_LWM2M_BATTERY)
		switch (product_id) {
		case BT510_PRODUCT_ID:
			d = ((double)((uint32_t)data.u16)) / 1000.0;
			percentage = lcz_lwm2m_battery_get_level_bt510(d);
			break;
		case BT6XX_PRODUCT_ID:
			d = ((double)((uint32_t)data.s32)) / 1000.0;
			percentage = lcz_lwm2m_battery_get_level_bt610(d);
			break;
		default:
			d = 0;
			percentage = 0;
			break;
		}
		r = lcz_lwm2m_managed_battery_set(idx, offset, d, percentage);
#endif
		break;

	case SENSOR_EVENT_CURRENT_1:
	case SENSOR_EVENT_CURRENT_2:
	case SENSOR_EVENT_CURRENT_3:
	case SENSOR_EVENT_CURRENT_4:
#if defined(CONFIG_LCZ_LWM2M_CURRENT)
		d = (double)data.f;
		offset = (record_type - SENSOR_EVENT_CURRENT_1);
		r = lcz_lwm2m_managed_current_set(idx, offset, d);
#endif
		break;

	case SENSOR_EVENT_PRESSURE_1:
	case SENSOR_EVENT_PRESSURE_2:
#if defined(CONFIG_LCZ_LWM2M_PRESSURE)
		d = (double)data.f;
		offset = (record_type - SENSOR_EVENT_PRESSURE_1);
		r = lcz_lwm2m_managed_pressure_set(idx, offset, d);
#endif
		break;

	case SENSOR_EVENT_ULTRASONIC_1:
#if defined(CONFIG_LCZ_LWM2M_FILL_LEVEL)
		/* Convert from mm (reported) to cm (filling sensor) */
		d = (double)data.f / 10.0;
		r = lcz_lwm2m_managed_fill_level_set(idx, offset, d);
#endif
		break;

	default:
		/* Just ignore record types that we do not support */
		break;
	}
}
