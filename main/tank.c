#include "tank.h"
#include "..\light_driver\include\light_driver.h"
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
#include "led_strip.h"
#include "esp_adc/adc_cali.h"

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

const static char *TAG = "tank_sensor";

#define HC_SR04_TRIG_GPIO  GPIO_NUM_15
#define HC_SR04_ECHO_GPIO  GPIO_NUM_14
#define BAT_ADC 0

const float LOW_BATTERY_VOLTAGE = 3.40; // warn user battery low

// - [Output] Distance: uint 0-400
// - [Output] Litres: uint 0-3300
// - [Output] Percentage:  uint 0-100
// - [Input] Tank min cm (dist to top): uint
// - [Input] Tank max cm (dist to bottom): uint
// - [Input] Tank capacity l: uint 3300
// - [Input] cm delta to report on: uint small
//   [Input] Measure period: uint in seconds, so thousands
static uint16_t tank_min_cm = 20;            // distance from ultrasonic sensor to top of water
static uint16_t tank_max_cm = 200;           // distance from ultrasonic sensor to bottom of tank (when it's empty)
static uint16_t tank_capacity_L = 3000;
static uint8_t cm_delta = 3;
static uint32_t read_period_s = 3;


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
                float_t distance = (float_t) pulse_width_us / 58;

                float_t fractionFull = 1 - (distance-tank_min_cm)/(tank_max_cm-tank_min_cm);
                float_t litres = fractionFull * tank_capacity_L;
                float_t percentFull = fractionFull*100;
                
                esp_zb_lock_acquire(portMAX_DELAY);
                esp_zb_zcl_set_attribute_val(1, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &distance, false);
                esp_zb_zcl_set_attribute_val(2, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &litres, false);
                esp_zb_zcl_set_attribute_val(3, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &percentFull, false);

                // TODO save and track history so I know when there's a change big enough to report.

                esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {
                    .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
                    .attributeID = ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID,
                    .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
                    .clusterID = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE,
                    .zcl_basic_cmd.src_endpoint = 1
                };

                esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);

                esp_zb_lock_release();


            }
        }
        vTaskDelay(pdMS_TO_TICKS(read_period_s*1000));
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
    BaseType_t ret = xTaskCreate(trigger_ultrasonic_read, "trigger_ultrasonic_read", 16324, NULL, 4, &taskHandle);

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

static void read_tank_config()
{
    nvs_handle_t my_handle;
    bool didWrite = false;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));

    if (nvs_get_u16(my_handle, "tank_min_cm", &tank_min_cm) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "tank_min_cm", tank_min_cm));
        didWrite = true;
    }
    if (nvs_get_u16(my_handle, "tank_max_cm", &tank_max_cm) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "tank_max_cm", tank_max_cm));
        didWrite = true;
    }
    if (nvs_get_u16(my_handle, "tank_capacity_L", &tank_capacity_L) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "tank_capacity_L", tank_capacity_L));
        didWrite = true;
    }
    if (nvs_get_u8(my_handle, "cm_delta", &cm_delta) == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_ERROR_CHECK(nvs_set_u8(my_handle, "cm_delta", cm_delta));
        didWrite = true;
    }
    if (nvs_get_u32(my_handle, "read_period_s", &read_period_s))
    {
        ESP_ERROR_CHECK(nvs_set_u32(my_handle, "read_period_s", read_period_s) == ESP_ERR_NVS_NOT_FOUND);
        didWrite = true;
    }

    if (didWrite)
    {
        ESP_ERROR_CHECK(nvs_commit(my_handle));
    }
    nvs_close(my_handle);

    esp_zb_lock_acquire(portMAX_DELAY);

    float_t val = (float_t)tank_min_cm;
    esp_zb_zcl_set_attribute_val(4, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &val, false);

    val = (float_t)tank_max_cm;
    esp_zb_zcl_set_attribute_val(5, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &val, false);

    val = (float_t)tank_capacity_L;
    esp_zb_zcl_set_attribute_val(6, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &val, false);

    val = (float_t)cm_delta;
    esp_zb_zcl_set_attribute_val(7, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &val, false);

    val = (float_t)read_period_s;
    esp_zb_zcl_set_attribute_val(8, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID, &val, false);

    esp_zb_lock_release();

    ESP_LOGI(TAG, "tank_min_cm: %u", tank_min_cm);
    ESP_LOGI(TAG, "tank_max_cm: %u", tank_max_cm);
    ESP_LOGI(TAG, "tank_capacity_L: %u", tank_capacity_L);
    ESP_LOGI(TAG, "cm_delta: %u", cm_delta);
    ESP_LOGI(TAG, "read_period_s: %lu", read_period_s);
}

static void update_reporting()
{
    

    // esp_zb_zcl_attr_location_info_t filter;
    // esp_zb_zcl_reporting_info_t *report;

    // filter.attr_id=ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID;
    // filter.cluster_id=ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE;
    // filter.endpoint_id=1;
    // filter.cluster_role=ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    // //filter.manuf_code=ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;

    // report=esp_zb_zcl_find_reporting_info(filter);
    // if(report!=NULL){
    //     report->u.send_info.def_max_interval=0xFFFE;
    //     report->u.send_info.def_min_interval=0x00;
    //     report->u.send_info.max_interval=0xFFFE;
    //     report->u.send_info.min_interval=0x00;
    //     report->u.send_info.delta.u32=1;
    //     if(esp_zb_zcl_update_reporting_info(report)!=ESP_OK) 
    //         ESP_LOGE(TAG, "error conf update");
    // }else ESP_LOGE(TAG, "error filter");


    // esp_zb_zcl_attr_location_info_t info = {
    //     .endpoint_id = 1,
    //     .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC
    // };

    // ESP_ERROR_CHECK(esp_zb_zcl_start_attr_reporting(info));



    // esp_zb_zcl_reporting_info_t reporting_info = {
    //     //.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
    //     .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI, 
        
    //     .ep = 1,
    //     .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE,
    //     .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    //     .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
    //     .u.send_info.min_interval = 5,
    //     .u.send_info.max_interval = 0,
    //     .u.send_info.def_min_interval = 5,
    //     .u.send_info.def_max_interval = 0,
    //     .u.send_info.delta.u8 = cm_delta,
    //     .attr_id = ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID,
    //     .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    // };
    // esp_zb_zcl_update_reporting_info(&reporting_info);
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *msg)
{
    switch (callback_id) 
    {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        {
            esp_zb_zcl_set_attr_value_message_t *message = (esp_zb_zcl_set_attr_value_message_t*)msg;
            if (!message) {
                ESP_LOGI(TAG, "Empty message");
                break;
            }
            if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Received message: error status(%d)", message->info.status);
                break;
            }

            ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", 
                message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);

            if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_VALUE && message->attribute.id == ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID)
            {
                nvs_handle_t my_handle;
                bool didWrite = false;
                ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));

                switch (message->info.dst_endpoint) {
                    case 4:
                    {
                        uint16_t val = (uint16_t)(*(float *)message->attribute.data.value);
                        tank_min_cm = val;
                        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "tank_min_cm", tank_min_cm));
                        didWrite = true;
                        ESP_LOGI(TAG, "Set tank_min_cm to %u", tank_min_cm);
                        break;
                    }
                    case 5:
                    {
                        uint16_t val = (uint16_t)(*(float *)message->attribute.data.value);
                        tank_max_cm = val;
                        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "tank_max_cm", tank_max_cm));
                        didWrite = true;
                        ESP_LOGI(TAG, "Set tank_max_cm to %u", tank_max_cm);
                        break;
                    }
                    case 6:
                    {
                        uint16_t val = (uint16_t)(*(float *)message->attribute.data.value);
                        tank_capacity_L = val;
                        ESP_ERROR_CHECK(nvs_set_u16(my_handle, "tank_capacity_L", tank_capacity_L));
                        didWrite = true;
                        ESP_LOGI(TAG, "Set tank_capacity_L to %u", tank_capacity_L);
                        break;
                    }
                    case 7:
                    {
                        uint8_t val = (uint8_t)(*(float *)message->attribute.data.value);
                        cm_delta = val;
                        ESP_ERROR_CHECK(nvs_set_u8(my_handle, "cm_delta", cm_delta));
                        didWrite = true;
                        update_reporting();
                        ESP_LOGI(TAG, "Set cm_delta to %u", cm_delta);
                        break;
                    }
                    case 8:
                    {
                        uint32_t val = (uint32_t)(*(float *)message->attribute.data.value);
                        read_period_s = val;
                        ESP_ERROR_CHECK(nvs_set_u32(my_handle, "read_period_s", read_period_s) == ESP_ERR_NVS_NOT_FOUND);
                        didWrite = true;
                        ESP_LOGI(TAG, "Set read_period_s to %lu", read_period_s);
                        break;
                    }
                    default:
                    {
                        ESP_LOGW(TAG, "Cannot write to endpoint: %d", message->info.dst_endpoint);
                        break;
                    }
                }
                if (didWrite)
                {
                    ESP_ERROR_CHECK(nvs_commit(my_handle));
                }
                nvs_close(my_handle);
            }
            break;
        }
        default:
        {
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            break;
        }
    }
    return ESP_OK;
}

static void add_av_endpoint(esp_zb_ep_list_t *ep_list, uint8_t ep_num, bool enableReporting)
{
    esp_zb_basic_cluster_cfg_t basic = {
        .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE
    };
    esp_zb_identify_cluster_cfg_t identify = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE
    };
    esp_zb_endpoint_config_t endpoint = {
        .endpoint = ep_num, 
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
        .app_device_id = ESP_ZB_HA_CONSUMPTION_AWARENESS_DEVICE_ID, 
        .app_device_version = 0
    };
    esp_zb_analog_value_cluster_cfg_t av_cfg = {
        .present_value = 0.0f,
        .out_of_service = false,
        .status_flags = 0
    };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic);
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    
    esp_zb_attribute_list_t *attrs = esp_zb_analog_value_cluster_create(&av_cfg);

    esp_zb_attribute_list_t *it = attrs;
    if (enableReporting) 
    {
        // ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID 0x55, ESP_ZB_ZCL_ATTR_ACCESS_REPORTING 0x4
        esp_zb_zcl_attr_t *attr;
        do
        {
            attr = &it->attribute;

            ESP_LOGI(TAG, "DEBUG attribute 0x%x, access 0x%x", attr->id, attr->access);

            if (attr->id == ESP_ZB_ZCL_ATTR_ANALOG_VALUE_PRESENT_VALUE_ID)
            {
                ESP_LOGI(TAG, "DEBUG change attr");
                // https://www.esp32.com/viewtopic.php?t=41114
                //attr->access = (ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING);
                //it->attribute.access = (ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING);
                //it->attribute.access = ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;

                // esp_zb_zcl_attr_t* newAttr = (esp_zb_zcl_attr_t*) malloc(sizeof(esp_zb_zcl_attr_t));
                // newAttr->id = attr->id;
                // newAttr->data_p = attr->data_p;
                // newAttr->manuf_code = attr->manuf_code;
                // newAttr->type = attr->type;
                // newAttr->access = (ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING);
                // it->attribute = *newAttr;

                // break;
            }
            it = it->next;
        }
        while (it != NULL);
    }
    

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_value_cluster(cluster_list, attrs, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint);
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    add_av_endpoint(ep_list, 1, true);  // Enable reporting on distance. This'll trigger sending litres and percent too.
    for (size_t i = 2; i <= 8; i++)
    {
        add_av_endpoint(ep_list, i, false);
    }
    esp_zb_device_register(ep_list);
    read_tank_config();

    //update_reporting();

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


// https://www.reddit.com/r/esp32/comments/1dybanl/comment/lc7gxma/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
// uint32_t getBatteryVoltage(void)
// {
//     float v = 0.0;
//     int adc_raw = analogRead(BAT_ADC);
//     esp_adc_cal_characteristics_t adc_chars;
//     esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
//     v =  2 * (esp_adc_cal_raw_to_voltage(adc_raw, &adc_chars));

//     return v;
// }
// int calculateBatteryPercentage(double v)
// {
//     // this formula was calculated using samples collected from a lipo battery
//     double y = -  144.9390 * v * v * v
//              + 1655.8629 * v * v
//              - 6158.8520 * v
//              + 7501.3202;

//     // enforce bounds, 0-100
//     y = max(y, 0.0);
//     y = min(y, 100.0);
//     y = round(y);
//     return (int)(y);
// }
