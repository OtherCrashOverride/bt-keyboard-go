/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is 
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR  
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#include "../components/odroid/odroid_input.h"
#include "../components/odroid/odroid_system.h"
#include "../components/odroid/odroid_keyboard.h"

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 *
 */

#define HID_DEMO_TAG "HID_DEMO"


static xQueueHandle gpio_evt_queue = NULL;
static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "ODROID-GO"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x30,
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void gpio_task_example(void* arg)
{
    static uint8_t i = 0;
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(HID_DEMO_TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(i == 0) {
            ++i;
            }
        }
    }
}



static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
                ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_REG_FINISH HIDD_DEVICE_NAME=%s", HIDD_DEVICE_NAME);
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        }    
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}



enum
{
    REPORT_READY_NONE = 0,
    REPORT_READY_KEYBOARD = (1 << 0),
    REPORT_READY_GAMEPAD = (1 << 1)
};


SemaphoreHandle_t keyboardMutex;

// #define KEYBOARD_QUEUE_LENGTH (20)
// odroid_keyboard_event_t keyboardQueue[KEYBOARD_QUEUE_LENGTH];
// int keyboardQueueCount = 0;

// Must be same or less than HID report
#define KEYBOARD_PRESSED_MAX (6)
uint8_t KeysPressed[KEYBOARD_PRESSED_MAX];
key_mask_t special_key_mask;

SemaphoreHandle_t reportReadySemaphore;
uint8_t reportReadyType;

odroid_gamepad_state gamepad_state;


static uint8_t convert_key_to_hid(odroid_key_t key)
{
    uint8_t result;

    // Map key
    switch(key)
    {
        case ODROID_KEY_GRAVE_ACCENT: result = HID_KEY_GRV_ACCENT; break;
        case ODROID_KEY_1: result = HID_KEY_1; break;
        case ODROID_KEY_2: result = HID_KEY_2; break;
        case ODROID_KEY_3: result = HID_KEY_3; break;
        case ODROID_KEY_4: result = HID_KEY_4; break;
        case ODROID_KEY_5: result = HID_KEY_5; break;
        case ODROID_KEY_6: result = HID_KEY_6; break;
        case ODROID_KEY_7: result = HID_KEY_7; break;

        case ODROID_KEY_8 : result = HID_KEY_8; break;
        case ODROID_KEY_9: result = HID_KEY_9; break;
        case ODROID_KEY_0: result = HID_KEY_0; break;
        case ODROID_KEY_ESCAPE: result = HID_KEY_ESCAPE; break;
        case ODROID_KEY_Q: result = HID_KEY_Q; break;
        case ODROID_KEY_W: result = HID_KEY_W; break;
        case ODROID_KEY_E: result = HID_KEY_E; break;
        case ODROID_KEY_R: result = HID_KEY_R; break;

        case ODROID_KEY_T : result = HID_KEY_T; break;
        case ODROID_KEY_Y: result = HID_KEY_Y; break;
        case ODROID_KEY_U: result = HID_KEY_U; break;
        case ODROID_KEY_I: result = HID_KEY_I; break;
        case ODROID_KEY_O: result = HID_KEY_O; break;
        case ODROID_KEY_P: result = HID_KEY_P; break;
        //case ODROID_KEY_CONTROL: result = HID_KEY_GRV_ACCENT; break;
        case ODROID_KEY_A: result = HID_KEY_A; break;

        case ODROID_KEY_S: result = HID_KEY_S; break;
        case ODROID_KEY_D: result = HID_KEY_D; break;
        case ODROID_KEY_F: result = HID_KEY_F; break;
        case ODROID_KEY_G: result = HID_KEY_G; break;
        case ODROID_KEY_H: result = HID_KEY_H; break;
        case ODROID_KEY_J: result = HID_KEY_J; break;
        case ODROID_KEY_K: result = HID_KEY_K; break;
        case ODROID_KEY_L: result = HID_KEY_L; break;

        case ODROID_KEY_BACKSPACE : result = HID_KEY_DELETE; break;
        //case ODROID_KEY_ALTERNATE: result = HID_KEY_GRV_ACCENT; break;
        case ODROID_KEY_Z: result = HID_KEY_Z; break;
        case ODROID_KEY_X: result = HID_KEY_X; break;
        case ODROID_KEY_C: result = HID_KEY_C; break;
        case ODROID_KEY_V: result = HID_KEY_V; break;
        case ODROID_KEY_B: result = HID_KEY_B; break;
        case ODROID_KEY_N: result = HID_KEY_N; break;

        case ODROID_KEY_M : result = HID_KEY_M; break;
        case ODROID_KEY_BACKSLASH: result = HID_KEY_BACK_SLASH; break;
        case ODROID_KEY_ENTER: result = HID_KEY_RETURN; break;
        //case ODROID_KEY_SHIFT: result = HID_KEY_GRV_ACCENT; break;
        case ODROID_KEY_SEMICOLON: result = HID_KEY_SEMI_COLON; break;
        case ODROID_KEY_APOSTROPHE: result = HID_KEY_SGL_QUOTE; break;
        case ODROID_KEY_MINUS: result = HID_KEY_MINUS; break;
        case ODROID_KEY_EQUALS: result = HID_KEY_EQUAL; break;
        
        case ODROID_KEY_SPACE : result = HID_KEY_SPACEBAR; break;
        case ODROID_KEY_COMMA: result = HID_KEY_COMMA; break;
        case ODROID_KEY_PERIOD: result = HID_KEY_DOT; break;
        case ODROID_KEY_SLASH: result = HID_KEY_FWD_SLASH; break;
        case ODROID_KEY_LEFTBRACKET: result = HID_KEY_LEFT_BRKT; break;
        case ODROID_KEY_RIGHTBRACKET: result = HID_KEY_RIGHT_BRKT; break;

        default:
            result = 0;
            break;
    }
    return (uint8_t)result;
}

void keyboard_callback(odroid_keystate_t state, odroid_key_t key)
{
	odroid_keyboard_event_t event;
	event.state = state;
	event.key = key;
	
    uint8_t hidKey = convert_key_to_hid(key);

	xSemaphoreTake(keyboardMutex, portMAX_DELAY);
    
    switch(key)
    {
        case ODROID_KEY_CONTROL:
            if (state == ODROID_KEY_PRESSED)
            {
                special_key_mask |= LEFT_CONTROL_KEY_MASK;
            }
            else
            {
                special_key_mask &= (~LEFT_CONTROL_KEY_MASK);
            }
            break;

        case ODROID_KEY_ALTERNATE:
            if (state == ODROID_KEY_PRESSED)
            {
                special_key_mask |= LEFT_ALT_KEY_MASK;
            }
            else
            {
                special_key_mask &= (~LEFT_ALT_KEY_MASK);
            }
            break;

        case ODROID_KEY_SHIFT:
            if (state == ODROID_KEY_PRESSED)
            {
                special_key_mask |= LEFT_SHIFT_KEY_MASK;
            }
            else
            {
                special_key_mask &= (~LEFT_SHIFT_KEY_MASK);
            }
            break;

        default:
            break;
    }

    if (hidKey > 0)
    {
        for (int i = 0; i < KEYBOARD_PRESSED_MAX; ++i)
        {
            uint8_t entry = KeysPressed[i];

            if (state == ODROID_KEY_PRESSED)
            {            
                if (entry == 0)
                {
                    // Store the key press
                    KeysPressed[i] = hidKey;
                    printf("Key added: hidKey=%d\n", hidKey);
                    break;
                }
                else if (entry == hidKey)
                {
                    printf("WARNING: Key already pressed: hidKey=%d\n", hidKey);
                }
            }
            else
            {
                if (entry == hidKey)
                {
                    // Release the key
                    KeysPressed[i] = 0;
                    printf("Key removed: hidKey=%d\n", hidKey);
                    break;
                }
            }
        }
    }

    reportReadyType |= REPORT_READY_KEYBOARD;

    xSemaphoreGive(keyboardMutex);
    

    // Wake sending thread
    xSemaphoreGive(reportReadySemaphore);
}

void gamepad_callback(odroid_gamepad_state* out_state) 
{
    xSemaphoreTake(keyboardMutex, portMAX_DELAY);

    gamepad_state = *out_state;
    reportReadyType |= REPORT_READY_GAMEPAD;

    xSemaphoreGive(keyboardMutex);
    

    // Wake sending thread
    xSemaphoreGive(reportReadySemaphore);
}


void hid_demo_task(void *pvParameters)
{
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1)
    {
        //vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (sec_conn)
        {
            // ESP_LOGI(HID_DEMO_TAG, "Send the volume");
            // send_volum_up = true;
            // //uint8_t key_vaule = {HID_KEY_A};
            // //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            // esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
            // vTaskDelay(3000 / portTICK_PERIOD_MS);
            // if (send_volum_up) {
            //     send_volum_up = false;
            //     esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
            //     esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
            //     vTaskDelay(3000 / portTICK_PERIOD_MS);
            //     esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
            // }

            // void esp_hidd_send_keyboard_value(uint16_t conn_id, key_mask_t special_key_mask, uint8_t *keyboard_cmd, uint8_t num_key)
            if(xSemaphoreTake(reportReadySemaphore, portMAX_DELAY) != pdTRUE )
            {
                abort();
            }


            xSemaphoreTake(keyboardMutex, portMAX_DELAY);

            if (reportReadyType & REPORT_READY_KEYBOARD)
            {
                esp_hidd_send_keyboard_value(hid_conn_id, special_key_mask, KeysPressed, KEYBOARD_PRESSED_MAX);
            }

            if (reportReadyType & REPORT_READY_GAMEPAD)
            {
                uint8_t up_down = 0;
                uint8_t left_right = 0;
                uint8_t buttons = 0;

                if (gamepad_state.values[ODROID_INPUT_UP])
                    up_down = -1;
                else if (gamepad_state.values[ODROID_INPUT_DOWN])
                    up_down = 1;
                else
                    up_down = 0;

                if (gamepad_state.values[ODROID_INPUT_LEFT])
                    left_right = -1;
                else if (gamepad_state.values[ODROID_INPUT_RIGHT])
                    left_right = 1;
                else
                    left_right = 0;

                
                if (gamepad_state.values[ODROID_INPUT_A]) buttons |= 1;
                if (gamepad_state.values[ODROID_INPUT_B]) buttons |= 2;
                if (gamepad_state.values[ODROID_INPUT_SELECT]) buttons |= 4;
                if (gamepad_state.values[ODROID_INPUT_START]) buttons |= 8;

                esp_hidd_send_gamepad_value(hid_conn_id, up_down, left_right, buttons);
            }

            reportReadyType = REPORT_READY_NONE;

            xSemaphoreGive(keyboardMutex);        
        }
        else
        {
            // // Ignore key presses
            // xSemaphoreTake(keyboardMutex, portMAX_DELAY);
    
            // keyboardQueueCount = 0;)
                        
            // xSemaphoreGive(keyboardMutex);
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


void app_main()
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


	keyboardMutex = xSemaphoreCreateMutex();
	if(keyboardMutex == NULL) abort();

    reportReadySemaphore = xSemaphoreCreateBinary();
    if(reportReadySemaphore == NULL) abort();


    odroid_system_init();

    odroid_input_event_callback_set(&gamepad_callback);
    odroid_input_gamepad_init();

    odroid_input_battery_level_init();

	odroid_keyboard_event_callback_set(&keyboard_callback);
	if (!odroid_keyboard_init())
    {
        ESP_LOGI(HID_DEMO_TAG, "%s keyboard not detected\n", __func__);
    }


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s esp_bt_controller_init failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s esp_bt_controller_enable failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s esp_bluedroid_init failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s esp_bluedroid_enable failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s esp_hidd_profile_init failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribut to you,
    and the response key means which key you can distribut to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribut to you, 
    and the init key means which key you can distribut to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    //init the gpio pin
    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);

}

