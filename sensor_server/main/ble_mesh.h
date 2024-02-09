#pragma once

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_time_scene_model_api.h"
#include "esp_ble_mesh_health_model_api.h"

#define CID_ESP     0x02E5

#define BT_MESH_PROP_ID_PRESENT_INDOOR_AMB_TEMP             0x0056
#define BT_MESH_PROP_ID_PRESENT_AMB_LIGHT_LEVEL             0x004E
#define BT_MESH_PROP_ID_PRESENT_INDOOR_RELATIVE_HUMIDITY    0x00A7
#define BT_MESH_PROP_ID_MOTION_SENSED                       0x0042

#define SENSOR_POSITIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION      ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD       ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL      ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL

typedef void (*provisioning_complete_f)(void);

typedef void (*attention_changed_f)(bool);

esp_err_t ble_mesh_init(provisioning_complete_f on_provisioning_complete, 
                        attention_changed_f on_attention_changed);

void ble_mesh_publish_sensors_data();

void ble_mesh_update_temperature(float new_value);

void ble_mesh_update_humidity(float new_value);

void ble_mesh_update_luminocity(float new_value);