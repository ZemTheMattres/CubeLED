// Bluetooth client

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
//LED includes
#include "led_cube.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

/* Declare Deffinitions */
/* Blue Tooth */
#define GATTC_TAG "BT"
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define BT_MIN_RSSI -50
#define MAX_BT_DEVICES 5

/* LED */
#define RMT_LED_STRIP_COUNT         6                       //Number of LED strips
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000                // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_LENGTH            144
#define CHASE_SPEED_MS              10
static const char *TAG = "LED_SETUP";
static const uint8_t RMT_LED_STRIP_GPIO_NUM[] = {0, 16, 2, 17, 4, 5};    //GIO for strips

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* Declare Global Variables*/
typedef enum{
    PowerOn = 0,
    CompanionCube,
    StorageCube,
    OnButton

}ledState;

ledState g_ledState;
esp_ble_gap_cb_param_t bt_Devices[MAX_BT_DEVICES];

//Checks hardware position for defult mode
static void check_mode(void){
    g_ledState = CompanionCube;
}
// Scan parameters
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

// GATT data structure
struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

// GATT data connected to GATT event handler
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

// GATT event handler
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        esp_ble_gap_set_scan_params(&ble_scan_params);
        printf("1 - Register gatt event\n");
        break;
    default:
        break;
    }
}

// GAP callback function - search BT connections
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t minRSSI = 0;
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    for(int i = 0; i < MAX_BT_DEVICES; i++){
        if(bt_Devices[i].scan_rst.rssi>minRSSI){
            minRSSI = bt_Devices[i].scan_rst.rssi;
        };
    };
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    { 
        uint32_t duration = 0;     // the unit of the duration is second set 0 to not stop
        esp_ble_gap_start_scanning(duration);
        printf("2 - Scan parameters set\n");
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        printf("3 - Start scan\n");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (scan_result->scan_rst.rssi > BT_MIN_RSSI){
                esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
                printf("%i\n", scan_result->scan_rst.rssi);
                g_ledState = OnButton;
            }
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

// GATTC callback function for gl_profile_tab structure initialization
// esp_gattc_cb_event_t - GATT Client callback function events
// esp_gatt_if_t - GATT interface type
// esp_ble_gattc_cb_param_t - GATT client callback parameters union
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

/* FreeRTOS Tasks */
void led_control(void *pvParameters){
    //Setup RMTs
    rmt_channel_handle_t led_chans[RMT_LED_STRIP_COUNT];
    rmt_encoder_handle_t led_encoders[RMT_LED_STRIP_COUNT];
    for(int i = 0; i < RMT_LED_STRIP_COUNT; i++){
        ESP_LOGI(TAG, "Create RMT TX channel");
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .gpio_num = RMT_LED_STRIP_GPIO_NUM[i],
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        };

        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chans[i]));

        ESP_LOGI(TAG, "Install led strip encoder");
        led_strip_encoder_config_t encoder_config = {
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
        };
        ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoders[i]));

        ESP_LOGI(TAG, "Enable RMT TX channel");
        ESP_ERROR_CHECK(rmt_enable(led_chans[i]));
    }

    ESP_LOGI(TAG, "Start LED Strip");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    while(1){
        static uint8_t led_strip_pixels[LED_STRIP_LENGTH * 4];
        printf("Switch 1 - Start\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
        switch(g_ledState){
        case PowerOn:
            printf("Switch 2 - Power On\n");
            check_mode();
            break;
        case CompanionCube:
            printf("Switch 3 - CompanionCube\n");
            set_led_strip_to_solid_color(pink, led_strip_pixels, LED_STRIP_LENGTH);
            for(int i = 0; i < RMT_LED_STRIP_COUNT; i++){
                ESP_ERROR_CHECK(rmt_transmit(led_chans[i], led_encoders[i], led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chans[i], portMAX_DELAY));          
            };
            vTaskDelay(pdMS_TO_TICKS(100000));
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));  
            break;
        case StorageCube:
            printf("Switch 4 - StorageCube\n");
            set_led_strip_to_solid_color(blue, led_strip_pixels, LED_STRIP_LENGTH);
            for(int i = 0; i < RMT_LED_STRIP_COUNT; i++){
                ESP_ERROR_CHECK(rmt_transmit(led_chans[i], led_encoders[i], led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chans[i], portMAX_DELAY));
            };
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            break;
        case OnButton:
            printf("Switch 5 - OnButton\n");
            set_led_strip_to_solid_color(yellow, led_strip_pixels, LED_STRIP_LENGTH);
            for(int i = 0; i < RMT_LED_STRIP_COUNT; i++){
                ESP_ERROR_CHECK(rmt_transmit(led_chans[i], led_encoders[i], led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chans[i], portMAX_DELAY));
            };
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            check_mode();
            break;
        default:
            printf("Switch 6 - Default\n");
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            for(int i = 0; i < RMT_LED_STRIP_COUNT; i++){
                ESP_ERROR_CHECK(rmt_transmit(led_chans[i], led_encoders[i], led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chans[i], portMAX_DELAY));
            };
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
    }
}


void freetos_create_tasks(){
    // Create the task for led control
    xTaskCreatePinnedToCore(led_control,                // Task function
        "led_control",                      // Task name (for debugging)
        4096,                               // Stack size (adjust as needed)
        NULL,                               // Task parameters
        1,                                  // Task priority (1 is typically low priority)
        NULL,
        tskNO_AFFINITY);                              // Task handle (optional)

        // Start the FreeRTOS scheduler
        //vTaskStartScheduler();
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    nvs_flash_init();                                      // Initialize NVS.
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT); // Release the controller memory
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();                          // Initialize BT controller to allocate task
    esp_bluedroid_enable();                        // Enable bluetooth
    esp_ble_gap_register_callback(esp_gap_cb);     // Register the callback function to the GAP module
    esp_ble_gattc_register_callback(esp_gattc_cb); // Register the callback function to the GATTC module
    esp_ble_gattc_app_register(PROFILE_A_APP_ID);  // Register application callbacks with GATTC module

    freetos_create_tasks();
}