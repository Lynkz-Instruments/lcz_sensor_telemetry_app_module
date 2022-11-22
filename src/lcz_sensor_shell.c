/**
 * @file lcz_sensor_shell.c
 * @brief Utilities for managing allow lists
 *
 * Copyright (c) 2022 Laird Connectivity
 *
 * SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
 */
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lcz_sensor_shell, CONFIG_LCZ_SENSOR_TELEM_APP);

/**************************************************************************************************/
/* Includes                                                                                       */
/**************************************************************************************************/
#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/addr.h>

#include <file_system_utilities.h>

#if defined(CONFIG_LCZ_LWM2M_GATEWAY_OBJ_STATIC_INST_LIST_FILE)
#include <lcz_lwm2m_gateway_obj.h>
#endif

#include "lcz_sensor_shell.h"

/**************************************************************************************************/
/* Local Constant, Macro and Type Definitions                                                     */
/**************************************************************************************************/
#define DEFAULT_OBSERVATIONS 64

#define ALLOW_LIST_NOT_ENABLED_MSG() shell_error(shell, "Allow list not enabled")

/**************************************************************************************************/
/* Local Data Definitions                                                                         */
/**************************************************************************************************/
static ssize_t obs_max;
static ssize_t obs_cnt;

static const char *file_name = "/lfs1/obs.bin";

static K_SEM_DEFINE(obs_sem, 1, 1);

/**************************************************************************************************/
/* Local Function Definitions                                                                     */
/**************************************************************************************************/
static int print_static_instance_list(const struct shell *shell, size_t argc, char **argv)
{
#if defined(CONFIG_LCZ_LWM2M_GATEWAY_OBJ_STATIC_INST_LIST_FILE)
	const char *file_name = CONFIG_LCZ_LWM2M_GATEWAY_OBJ_STATIC_INST_LIST_FILE;
	uint32_t i = 0;
	uint32_t offset = 0;
	ssize_t status;
	bt_addr_le_t inst_addr = { 0 };
	char addr_str[BT_ADDR_LE_STR_LEN];
	const size_t SIZE = sizeof(bt_addr_le_t);
	int r = 0;

	shell_print(shell, "Static Instance List");
	do {
		status = fsu_read_abs_block(file_name, offset, &inst_addr, SIZE);
		if (status == SIZE) {
			r = bt_addr_le_to_str(&inst_addr, addr_str, sizeof(addr_str));
			if (r > 0 && r < BT_ADDR_LE_STR_LEN) {
				shell_print(shell, "%02u %02u %s", i, LCZ_LWM2M_GW_LEGACY_INSTANCE(i),
					    addr_str);
				r = 0;
			} else {
				r = -EINVAL;
				break;
			}
		}
		i += 1;
		offset += SIZE;
	} while (status == SIZE);

	return r;
#else
	shell_error("Static instance list not supported");
	return -EPERM;
#endif
}

static int obs_start(const struct shell *shell, size_t argc, char **argv)
{
	if ((argc == 2) && (argv[1] != NULL)) {
		obs_max = strtol(argv[1], NULL, 0);
	} else {
		obs_max = DEFAULT_OBSERVATIONS;
	}
	shell_print(shell, "maximum ads to be recorded: %d", obs_max);

	k_sem_take(&obs_sem, K_FOREVER);
	obs_cnt = 0;
	if (fsu_get_file_size_abs(file_name) > 0) {
		fsu_delete_abs(file_name);
	}
	k_sem_give(&obs_sem);

	return 0;
}

static int obs_end(const struct shell *shell, size_t argc, char **argv)
{
	k_sem_take(&obs_sem, K_FOREVER);
	/* Negative value is used to stop recording */
	obs_max = -1;
	k_sem_give(&obs_sem);

	return 0;
}

static int obs_get(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t i = 0;
	uint32_t offset = 0;
	ssize_t status;
	struct obs_file_format obs;
	char addr_str[BT_ADDR_LE_STR_LEN];
	const size_t SIZE = sizeof(obs);
	int r = 0;

	k_sem_take(&obs_sem, K_FOREVER);

	shell_print(shell, "Sensor Advertisement Observation Data");
	shell_print(shell, "index--addr--protocol id--network_id--coded--rssi");
	do {
		status = fsu_read_abs_block(file_name, offset, &obs, SIZE);
		if (status == SIZE) {
			r = bt_addr_le_to_str(&obs.addr, addr_str, sizeof(addr_str));
			if (r > 0 && r < BT_ADDR_LE_STR_LEN) {
				shell_print(shell, "%02u %s %02x %02x %s %d", i, addr_str,
					    obs.protocol_id, obs.network_id, obs.coded ? "y" : "n",
					    obs.rssi);
				r = 0;
			} else {
				r = -EINVAL;
				break;
			}
		}
		i += 1;
		offset += SIZE;
	} while (status == SIZE);

	k_sem_give(&obs_sem);

	return r;
}

static int load_allow_list(const struct shell *shell, size_t argc, char **argv)
{
	return lcz_lwm2m_gw_obj_load_allow_list(shell);
}

static int delete_allow_list(const struct shell *shell, size_t argc, char **argv)
{
#if defined(CONFIG_LCZ_LWM2M_GATEWAY_OBJ_ALLOW_LIST)
	const char *fname = CONFIG_LCZ_LWM2M_GATEWAY_OBJ_ALLOW_LIST_FILE;
	int r;

	r = fsu_get_file_size_abs(fname);
	if (r >= 0) {
		r = fsu_delete_abs(fname);
	}
	return r;
#else
	ALLOW_LIST_NOT_ENABLED_MSG();
	return -EPERM;
#endif
}

/**************************************************************************************************/
/* Global Function Definitions                                                                    */
/**************************************************************************************************/
void lcz_sensor_shell_record_advertisement(struct obs_file_format *data)
{
	int r = 0;

	if (k_sem_take(&obs_sem, K_NO_WAIT) == 0) {
		if (obs_cnt < obs_max) {
			obs_cnt += 1;
			r = fsu_append_abs(file_name, data, sizeof(*data));
			LOG_DBG("Appended to obs %d %d", obs_cnt, r);
		}
		k_sem_give(&obs_sem);
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_sensor_shell,
	SHELL_CMD(static_inst, NULL, "Print the static instance list", print_static_instance_list),
	SHELL_CMD(obs_start, NULL, "Start saving advertisements to a file <optional count>",
		  obs_start),
	SHELL_CMD(obs_end, NULL, "Stop saving advertisements to file", obs_end),
	SHELL_CMD(obs_get, NULL, "Print advertisement information from file", obs_get),
	SHELL_CMD(allow_list_load, NULL, "Load the allow list", load_allow_list),
	SHELL_CMD(allow_list_delete, NULL, "Delete the allow list", delete_allow_list),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lcs, &sub_sensor_shell, "Laird Connectivity Sensor (Gateway) Application", NULL);
