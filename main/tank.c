#include "tank.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
const static char *TAG = "tank_sensor";
#define HC_SR04_TRIG_GPIO  GPIO_NUM_15
#define HC_SR04_ECHO_GPIO  GPIO_NUM_14

#define CLUSTER_ID ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT
#define ATTRIBUTE_ID ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID

static uint16_t cm_delta = 100;         // in 0.01 cm. Only used for reporting.
static uint16_t sample_period_s = 5;    // Used for reporting and sampling.

static void trigger_ultrasonic_read()
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint32_t tof_ticks;
    for (;;) {
        // generate single pulse on Trig pin to start a new sample
        gpio_set_level(HC_SR04_TRIG_GPIO, 1); // set high
        vTaskDelay(1);
        gpio_set_level(HC_SR04_TRIG_GPIO, 0); // set low
        // wait for echo done signal
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us <= 35000) { // not out of range
                // convert the pulse width into measure distance
                float dist = pulse_width_us / 58.0f;
                uint16_t distance = (uint16_t) (100*dist);
                if (esp_zb_lock_acquire(portMAX_DELAY))
                {
                    ESP_LOGI(TAG, "Distance: %0.2f cm. Write %u to attribute", dist, distance);
                    esp_zb_zcl_set_attribute_val(1, CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        ATTRIBUTE_ID, &distance, false);
                    esp_zb_lock_release();
                }
                else {
                    ESP_LOGI(TAG, "esp_zb_lock_acquire failed");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(sample_period_s*1000));
    }
}

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    // NOTE: do not do any logging in here (it's an interrupt service routine), or it'll crash.
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

static esp_err_t deferred_driver_init(void)
{
    // Init ultrasonic sensor stuff
    ESP_LOGI(TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf, &cap_timer));

    ESP_LOGI(TAG, "Install capture channel");
    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = HC_SR04_ECHO_GPIO,
        .prescale = 1,
        // capture on both edge
        .flags.neg_edge = true,
        .flags.pos_edge = true,
        // pull up internally
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(TAG, "Start periodic trigger task");
    TaskHandle_t taskHandle;
    BaseType_t ret = xTaskCreate(trigger_ultrasonic_read, "trigger_ultrasonic_read", 4096, NULL, 4, &taskHandle);

    ESP_LOGI(TAG, "Register capture callback");
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = hc_sr04_echo_callback,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, taskHandle));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Configure Trig pin");
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // drive low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    return (ret == pdTRUE) ? ESP_OK : ESP_FAIL;
}

static void update_reporting()
{
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI, 
        .ep = 1,
        .cluster_id = CLUSTER_ID,
        .attr_id = ATTRIBUTE_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = sample_period_s,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = sample_period_s,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = cm_delta,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *msg)
{
    switch (callback_id) 
    {
        default:
        {
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            esp_zb_zcl_attr_location_info_t filter = {
                .attr_id = ATTRIBUTE_ID,
                .cluster_id=  CLUSTER_ID,
                .endpoint_id = 1,
                .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
            };
            esp_zb_zcl_reporting_info_t *report;
            report = esp_zb_zcl_find_reporting_info(filter);
            if (report != NULL)
            {
                uint16_t new_cm_delta = report->u.send_info.delta.u16;
                uint16_t new_sample_period = report->u.send_info.min_interval;
                if (new_cm_delta != cm_delta || new_sample_period != sample_period_s)
                {
                    cm_delta = new_cm_delta;
                    sample_period_s = new_sample_period;
                    nvs_handle_t my_handle;
                    bool didWrite = false;
                    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));
                    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "cm_delta", cm_delta));
                    ESP_ERROR_CHECK(nvs_set_u16(my_handle, "sample_period_s", sample_period_s));
                    ESP_ERROR_CHECK(nvs_commit(my_handle));
                    nvs_close(my_handle);
                    ESP_LOGW(TAG, "Set new cm_delta: %u, sample_period_s: %u", cm_delta, sample_period_s);
                }
            }
            else {
                ESP_LOGE(TAG, "error filter");
            }
            break;
        }
    }
    return ESP_OK;
}

static void add_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep_num)
{
    esp_zb_endpoint_config_t endpoint = {
        .endpoint = ep_num, 
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
        .app_device_id = ESP_ZB_HA_CONSUMPTION_AWARENESS_DEVICE_ID, 
        .app_device_version = 0
    };
    esp_zb_basic_cluster_cfg_t basic = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE
    };
    esp_zb_identify_cluster_cfg_t identify = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
    };
    esp_zb_illuminance_meas_cluster_cfg_t cfg = {
        .max_value = 65535,
        .min_value = 0,
        .measured_value = 0
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic);
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster_create(&cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint);
}

static void load_config()
{
    nvs_handle_t my_handle;
    bool didWrite = false;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));
    if (nvs_get_u16(my_handle, "cm_delta", &cm_delta) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "cm_delta", cm_delta));
        didWrite = true;
    }
    if (nvs_get_u16(my_handle, "sample_period_s", &sample_period_s) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "sample_period_s", sample_period_s));
        didWrite = true;
    }
    if (didWrite)
    {
        ESP_ERROR_CHECK(nvs_commit(my_handle));
    }
    nvs_close(my_handle);
    ESP_LOGW(TAG, "Read from memory: cm_delta: %u, sample_period_s: %u", cm_delta, sample_period_s);
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    add_endpoint(ep_list, 1);
    esp_zb_device_register(ep_list);
    load_config();
    update_reporting();
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
