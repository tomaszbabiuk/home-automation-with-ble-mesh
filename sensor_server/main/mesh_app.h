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
#define BT_MESH_PROP_ID_PRESENT_AMB_CO2_CONCENTRATION       0x0077
#define BT_MESH_PROP_ID_PRESENT_AMB_VOC_CONCENTRATION       0x0078

#define SENSOR_POSITIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION      ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD       ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL      ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL

typedef void (*provisioning_complete_f)(void);

typedef void (*attention_changed_f)(bool);

struct sensor_setting {
    uint16_t sensor_prop_id;
    uint16_t sensor_setting_prop_id;
} __attribute__((packed));

struct sensor_descriptor {
    uint16_t sensor_prop_id;
    uint32_t pos_tolerance:12,
             neg_tolerance:12,
             sample_func:8;
    uint8_t  measure_period;
    uint8_t  update_interval;
} __attribute__((packed));

esp_err_t mesh_app_init(provisioning_complete_f on_provisioning_complete, 
                        attention_changed_f on_attention_changed);

void mesh_app_publish_sensors_data();

void mesh_app_update_temperature(float new_value);

void mesh_app_update_humidity(float new_value);

void mesh_app_update_luminocity(float new_value);

void mesh_app_update_tvoc(uint16_t new_value);

void mesh_app_update_eco2(uint16_t new_value);