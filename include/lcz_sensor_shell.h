/**
 * @file lcz_sensor_shell.h
 * @brief LCZ sensor telemetry shell
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */
#ifndef __LCZ_SENSOR_SHELL_H__
#define __LCZ_SENSOR_SHELL_H__

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************/
/* Global Constants, Macros and Type Definitions                                                  */
/**************************************************************************************************/
struct obs_file_format {
	bt_addr_le_t addr;
	uint16_t protocol_id;
	uint16_t network_id;
	int8_t rssi;
	bool coded;
};

/**************************************************************************************************/
/* Global Function Prototypes                                                                     */
/**************************************************************************************************/
void lcz_sensor_shell_record_advertisement(struct obs_file_format *data);

#ifdef __cplusplus
}
#endif

#endif /* __LCZ_SENSOR_SHELL_H__ */
