/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
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


#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */
// LAB FOUR ONE STUFF
//

static const char *TAG = "icm42670p_example";

// I2C communication parameters
#define I2C_MASTER_SCL_IO 8       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 10      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// ICM42670-P sensor specific defines
#define ICM42670P_SENSOR_ADDR                                                  \
  0x68 /*!< Default I2C address of ICM42670-P (AD0 pin low) */
#define ICM42670P_SENSOR_ADDR_ALT                                              \
  0x69 /*!< Alternate I2C address of ICM42670-P (AD0 pin high) */
#define ICM42670P_WHO_AM_I_REG_ADDR 0x75 /*!< WHO_AM_I register address */
#define ICM42670P_WHO_AM_I_VAL 0x67      /*!< Expected WHO_AM_I value */
#define ICM42670P_WHO_AM_I_VAL_ALT                                             \
  0x24 /*!< Alternative WHO_AM_I value (some revisions) */

// Power management registers
#define ICM42670P_PWR_MGMT0_REG_ADDR 0x1F /*!< Power Management 0 register */
#define ICM42670P_ACCEL_CONFIG0_REG                                            \
  0x21 /*!< Accelerometer configuration register */
#define ICM42670P_GYRO_CONFIG0_REG                                             \
  0x20 /*!< Gyroscope configuration register                                   \
        */

// Data registers
#define ICM42670P_TEMP_DATA1 0x09 /*!< Temperature data high byte */
#define ICM42670P_TEMP_DATA0 0x0A /*!< Temperature data low byte */
#define ICM42670P_ACCEL_DATA_X1                                                \
  0x0B /*!< Accelerometer X-axis data high byte                                \
        */
#define ICM42670P_ACCEL_DATA_X0                                                \
  0x0C /*!< Accelerometer X-axis data low byte                                 \
        */
#define ICM42670P_ACCEL_DATA_Y1                                                \
  0x0D /*!< Accelerometer Y-axis data high byte                                \
        */
#define ICM42670P_ACCEL_DATA_Y0                                                \
  0x0E /*!< Accelerometer Y-axis data low byte                                 \
        */
#define ICM42670P_ACCEL_DATA_Z1                                                \
  0x0F /*!< Accelerometer Z-axis data high byte                                \
        */
#define ICM42670P_ACCEL_DATA_Z0                                                \
  0x10 /*!< Accelerometer Z-axis data low byte                                 \
        */

// Power management values
#define ICM42670P_PWR_MGMT0_ACCEL_MODE_LN                                      \
  (3 << 0) /*!< Accelerometer Low Noise mode */
#define ICM42670P_PWR_MGMT0_GYRO_MODE_OFF (0 << 2) /*!< Gyroscope off */
#define ICM42670P_PWR_MGMT0_TEMP_DISABLE_OFF                                   \
  (0 << 5) /*!< Temperature sensor enabled */

// Accelerometer configuration values (±16g full scale, 1kHz ODR)
#define ICM42670P_ACCEL_CONFIG0_FS_SEL_16G (3 << 5) /*!< ±16g full scale */
#define ICM42670P_ACCEL_CONFIG0_ODR_1KHZ (6 << 0)   /*!< 1kHz ODR */

// Accelerometer sensitivity at ±16g full scale (in g/LSB)
#define ICM42670P_ACCEL_SENSITIVITY_16G (1.0f / 2048.0f) // 16g / 2^15 LSB

/**
 * @brief Read a sequence of bytes from ICM42670-P sensor registers
 */
static esp_err_t icm42670p_register_read(i2c_master_dev_handle_t dev_handle,
                                         uint8_t reg_addr, uint8_t *data,
                                         size_t len) {
  return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len,
                                     I2C_MASTER_TIMEOUT_MS /
                                         portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to an ICM42670-P sensor register
 */
static esp_err_t
icm42670p_register_write_byte(i2c_master_dev_handle_t dev_handle,
                              uint8_t reg_addr, uint8_t data) {
  uint8_t write_buf[2] = {reg_addr, data};
  return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf),
                             I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Read raw accelerometer data from ICM42670-P
 */
static esp_err_t icm42670p_get_accel_data(i2c_master_dev_handle_t dev_handle,
                                          int16_t *accel_x, int16_t *accel_y,
                                          int16_t *accel_z) {
  uint8_t data[6];

  // Read all accelerometer data registers (X, Y, Z - high and low bytes)
  esp_err_t ret =
      icm42670p_register_read(dev_handle, ICM42670P_ACCEL_DATA_X1, data, 6);
  if (ret != ESP_OK) {
    return ret;
  }

  // Combine high and low bytes into 16-bit values (big-endian)
  *accel_x = ((int16_t)data[0] << 8) | data[1];
  *accel_y = ((int16_t)data[2] << 8) | data[3];
  *accel_z = ((int16_t)data[4] << 8) | data[5];

  return ESP_OK;
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle,
                            i2c_master_dev_handle_t *dev_handle) {
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_MASTER_NUM,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = ICM42670P_SENSOR_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/**
 * @brief Configure the ICM42670-P sensor
 */
static esp_err_t icm42670p_init(i2c_master_dev_handle_t dev_handle) {
  uint8_t data;
  esp_err_t ret;
  int retry_count = 0;
  const int max_retries = 3;

  // Try reading WHO_AM_I register with retries
  while (retry_count < max_retries) {
    ret = icm42670p_register_read(dev_handle, ICM42670P_WHO_AM_I_REG_ADDR,
                                  &data, 1);
    if (ret == ESP_OK) {
      break;
    }
    ESP_LOGW(TAG, "Failed to read WHO_AM_I register, retrying (%d/%d)",
             retry_count + 1, max_retries);
    vTaskDelay(pdMS_TO_TICKS(10)); // Short delay before retry
    retry_count++;
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read WHO_AM_I register after %d attempts",
             max_retries);
    return ret;
  }

  ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", data);
  if (data != ICM42670P_WHO_AM_I_VAL && data != ICM42670P_WHO_AM_I_VAL_ALT) {
    ESP_LOGE(TAG, "Wrong WHO_AM_I value, expected 0x%02X or 0x%02X",
             ICM42670P_WHO_AM_I_VAL, ICM42670P_WHO_AM_I_VAL_ALT);
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Reset retry counter
  retry_count = 0;

  // Configure accelerometer with retries
  while (retry_count < max_retries) {
    ret = icm42670p_register_write_byte(dev_handle, ICM42670P_ACCEL_CONFIG0_REG,
                                        ICM42670P_ACCEL_CONFIG0_FS_SEL_16G |
                                            ICM42670P_ACCEL_CONFIG0_ODR_1KHZ);
    if (ret == ESP_OK) {
      break;
    }
    ESP_LOGW(TAG, "Failed to configure accelerometer, retrying (%d/%d)",
             retry_count + 1, max_retries);
    vTaskDelay(pdMS_TO_TICKS(10));
    retry_count++;
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure accelerometer after %d attempts",
             max_retries);
    return ret;
  }

  // Reset retry counter
  retry_count = 0;

  // Power on accelerometer with retries
  while (retry_count < max_retries) {
    ret = icm42670p_register_write_byte(
        dev_handle, ICM42670P_PWR_MGMT0_REG_ADDR,
        ICM42670P_PWR_MGMT0_ACCEL_MODE_LN | ICM42670P_PWR_MGMT0_GYRO_MODE_OFF |
            ICM42670P_PWR_MGMT0_TEMP_DISABLE_OFF);
    if (ret == ESP_OK) {
      break;
    }
    ESP_LOGW(TAG, "Failed to power on accelerometer, retrying (%d/%d)",
             retry_count + 1, max_retries);
    vTaskDelay(pdMS_TO_TICKS(10));
    retry_count++;
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to power on accelerometer after %d attempts",
             max_retries);
    return ret;
  }

  // Longer delay to allow sensor to power up and stabilize
  vTaskDelay(pdMS_TO_TICKS(50));

  // Read back config to verify
  ret = icm42670p_register_read(dev_handle, ICM42670P_ACCEL_CONFIG0_REG, &data,
                                1);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "ACCEL_CONFIG0 = 0x%02X", data);
  }

  ret = icm42670p_register_read(dev_handle, ICM42670P_PWR_MGMT0_REG_ADDR, &data,
                                1);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "PWR_MGMT0 = 0x%02X", data);
  }

  return ESP_OK;
}

/**
 * @brief Scan I2C bus for available devices
 */
static void i2c_scan(i2c_master_bus_handle_t bus_handle) {
  uint8_t address;
  ESP_LOGI(TAG, "Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    i2c_master_dev_handle_t temp_dev_handle;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    if (i2c_master_bus_add_device(bus_handle, &dev_config, &temp_dev_handle) ==
        ESP_OK) {
      uint8_t dummy;
      esp_err_t ret = i2c_master_transmit_receive(
          temp_dev_handle, &dummy, 0, &dummy, 0, 50 / portTICK_PERIOD_MS);
      i2c_master_bus_rm_device(temp_dev_handle);

      if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device found at address 0x%02X", address);
      }
    }
  }
  ESP_LOGI(TAG, "Scan completed");
}
// END OF LAB 4 1 STUFF ========================
#define HID_DEMO_TAG "HID_DEMO"


static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
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


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

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
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            break;
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

void hid_demo_task(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (sec_conn) {
            ESP_LOGI(HID_DEMO_TAG, "Send the mouse");
            send_volum_up = true;
            //uint8_t key_vaule = {HID_KEY_A};
            //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            esp_hidd_send_mouse_value(hid_conn_id, 0, 10, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (send_volum_up) {
                send_volum_up = false;
                esp_hidd_send_mouse_value(hid_conn_id, 0, 10, 0);
                esp_hidd_send_mouse_value(hid_conn_id, 0, -10, 0);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                esp_hidd_send_mouse_value(hid_conn_id, 0, -10, 0);
            }
        }
    }
}


void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
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
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    /*xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);*/

  //START I2C STUFF ===============================
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t dev_handle;
  esp_err_t ret1;
  bool sensor_found = false;
  uint8_t sensor_addr = ICM42670P_SENSOR_ADDR;

  // Initialize I2C
  i2c_master_init(&bus_handle, &dev_handle);
  ESP_LOGI(TAG, "I2C initialized successfully");

  // First scan the I2C bus to see what devices are available
  i2c_scan(bus_handle);

  // Try to initialize with default address
  ESP_LOGI(TAG, "Trying ICM42670-P at address 0x%02X", sensor_addr);
  ret1 = icm42670p_init(dev_handle);

  // If defaulteaddress failed, try alternative address
  if (ret1 != ESP_OK) {
    ESP_LOGI(TAG, "Trying alternative address 0x%02X",
             ICM42670P_SENSOR_ADDR_ALT);

    // Remove previous device
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));

    // Add device with alternative address
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM42670P_SENSOR_ADDR_ALT,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));

    // Try to initialize with alternative address
    ret1 = icm42670p_init(dev_handle);
    if (ret1 == ESP_OK) {
      sensor_found = true;
      sensor_addr = ICM42670P_SENSOR_ADDR_ALT;
    }
  } else {
    sensor_found = true;
  }

  if (!sensor_found) {
    ESP_LOGE(TAG, "Failed to initialize ICM42670-P sensor. Check connections "
                  "and pullup resistors.");
  }

  ESP_LOGI(TAG, "ICM42670-P sensor found at address 0x%02X", sensor_addr);

  // Read and print accelerometer data continuously
  int consecutive_errors = 0;
  const int max_consecutive_errors = 5;
  int retry_delay_ms = 10;

  while (1) {
    int16_t accel_x, accel_y, accel_z;
    float accel_x_g, accel_y_g, accel_z_g;

    ret1 = icm42670p_get_accel_data(dev_handle, &accel_x, &accel_y, &accel_z);
    if (ret1 == ESP_OK) {
      // Convert raw accelerometer values to g units
      accel_x_g = accel_x * ICM42670P_ACCEL_SENSITIVITY_16G;
      accel_y_g = accel_y * ICM42670P_ACCEL_SENSITIVITY_16G;
      accel_z_g = accel_z * ICM42670P_ACCEL_SENSITIVITY_16G;

      char *direction;
      // find direction
      if (accel_x_g > 4.00) {
        if (accel_y_g < -2.0){
          direction = "forwards left";
          esp_hidd_send_mouse_value(hid_conn_id, 0, -10, -10);
        }
        else if (accel_y_g > 2.0){
          direction = "backwards left";
          esp_hidd_send_mouse_value(hid_conn_id, 0, -10, 10);
        }
        else {
          direction = "left";
          esp_hidd_send_mouse_value(hid_conn_id, 0, -10, 0);
        }
      } else if (accel_x_g < -4.0) {
        if (accel_y_g < -2.0){
          direction = "forwards right";
          esp_hidd_send_mouse_value(hid_conn_id, 0, 10, -10);
        }
        else if (accel_y_g > 2.0){
          direction = "backwards right";
          esp_hidd_send_mouse_value(hid_conn_id, 0, 10, 10);
        }
        else {
          direction = "right";
          esp_hidd_send_mouse_value(hid_conn_id, 0, 10, 0);
        }
      } else if (accel_y_g < -4.0) {
        direction = "forwards";
        esp_hidd_send_mouse_value(hid_conn_id, 0, 0, -10);
      } else if (accel_y_g > 4.0) {
        direction = "backwards";
        esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 10);
      } else {
        direction = "None";
      }

      ESP_LOGI(TAG,
               "Accelerometer: X=%.2f g, Y=%.2f g, Z=%.2f g (direction: %s)",
               accel_x_g, accel_y_g, accel_z_g, direction);

      consecutive_errors = 0;
      retry_delay_ms = 10; // Reset retry delay
    } else {
      ESP_LOGE(TAG, "Failed to read accelerometer data");
      consecutive_errors++;

      if (consecutive_errors >= max_consecutive_errors) {
        ESP_LOGW(TAG,
                 "Too many consecutive errors, trying to reinitialize sensor");
        ret1 = icm42670p_init(dev_handle);
        if (ret1 == ESP_OK) {
          ESP_LOGI(TAG, "Sensor reinitialized successfully");
          consecutive_errors = 0;
          retry_delay_ms = 10;
        } else {
          ESP_LOGE(TAG, "Failed to reinitialize sensor");
          // Increase delay between retries to prevent flooding with errors
          retry_delay_ms = (retry_delay_ms < 1000) ? retry_delay_ms * 2 : 1000;
        }
      }

      // Short delay before retry
      vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Read data every 500ms
  }
}
