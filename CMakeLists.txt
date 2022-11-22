#
# Copyright (c) 2022 Laird Connectivity LLC
#
# SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause
#
if(CONFIG_LCZ_SENSOR_TELEM_APP)

zephyr_include_directories(include)

zephyr_sources(src/lcz_sensor_scan.c)
zephyr_sources_ifdef(CONFIG_LCZ_SENSOR_TELEM_LWM2M src/lcz_sensor_lwm2m.c)
zephyr_sources_ifdef(CONFIG_LCZ_SENSOR_TELEM_MQTT src/lcz_sensor_mqtt.c)
zephyr_sources_ifdef(CONFIG_LCZ_SENSOR_SHELL src/lcz_sensor_shell.c)

endif()
