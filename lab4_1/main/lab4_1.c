/*
 * Example code for reading accelerometer data from ICM42670-P sensor on
 * ESP32-C3 Based on Espressif I2C example
 */
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

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

void app_main(void) {
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t dev_handle;
  esp_err_t ret;
  bool sensor_found = false;
  uint8_t sensor_addr = ICM42670P_SENSOR_ADDR;

  // Initialize I2C
  i2c_master_init(&bus_handle, &dev_handle);
  ESP_LOGI(TAG, "I2C initialized successfully");

  // First scan the I2C bus to see what devices are available
  i2c_scan(bus_handle);

  // Try to initialize with default address
  ESP_LOGI(TAG, "Trying ICM42670-P at address 0x%02X", sensor_addr);
  ret = icm42670p_init(dev_handle);

  // If default address failed, try alternative address
  if (ret != ESP_OK) {
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
    ret = icm42670p_init(dev_handle);
    if (ret == ESP_OK) {
      sensor_found = true;
      sensor_addr = ICM42670P_SENSOR_ADDR_ALT;
    }
  } else {
    sensor_found = true;
  }

  if (!sensor_found) {
    ESP_LOGE(TAG, "Failed to initialize ICM42670-P sensor. Check connections "
                  "and pullup resistors.");
    goto cleanup;
  }

  ESP_LOGI(TAG, "ICM42670-P sensor found at address 0x%02X", sensor_addr);

  // Read and print accelerometer data continuously
  int consecutive_errors = 0;
  const int max_consecutive_errors = 5;
  int retry_delay_ms = 10;

  while (1) {
    int16_t accel_x, accel_y, accel_z;
    float accel_x_g, accel_y_g, accel_z_g;

    ret = icm42670p_get_accel_data(dev_handle, &accel_x, &accel_y, &accel_z);
    if (ret == ESP_OK) {
      // Convert raw accelerometer values to g units
      accel_x_g = accel_x * ICM42670P_ACCEL_SENSITIVITY_16G;
      accel_y_g = accel_y * ICM42670P_ACCEL_SENSITIVITY_16G;
      accel_z_g = accel_z * ICM42670P_ACCEL_SENSITIVITY_16G;

      char *direction;
      // find direction
      if (accel_x_g > 5.00) {
        if (accel_y_g < -3.0)
          direction = "forwards left";
        else if (accel_y_g > 3.0)
          direction = "backwards left";
        else
          direction = "left";
      } else if (accel_x_g < -5.0) {
        if (accel_y_g < -3.0)
          direction = "forwards right";
        else if (accel_y_g > 3.0)
          direction = "backwards right";
        else
          direction = "right";
      } else if (accel_y_g < -5.0) {
        direction = "forwards";
      } else if (accel_y_g > 5.0) {
        direction = "backwards";
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
        ret = icm42670p_init(dev_handle);
        if (ret == ESP_OK) {
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

    vTaskDelay(pdMS_TO_TICKS(500)); // Read data every 500ms
  }

cleanup:
  // Clean up I2C resources
  ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
  ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
  ESP_LOGI(TAG, "I2C de-initialized successfully");
}
