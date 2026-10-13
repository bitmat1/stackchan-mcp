// Minimal drivers for the CoreS3's on-board BMI270 IMU and LTR-553
// proximity sensor (its light channel is left off), on the board's internal i2c_master bus.
//
// Espressif's bmi270_sensor component is built on the legacy i2c_bus driver,
// which ESP-IDF refuses to mix with i2c_master on the same port, so only its
// config blob is reused here. Include from one translation unit only.
#pragma once

#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cstdint>

#include "i2c_device.h"

namespace body_sensors {

namespace blob {
#include "../../../managed_components/espressif__bmi270_sensor/firmware/bmi270_image.h"
}

class Bmi270 : public I2cDevice {
public:
    static constexpr uint8_t kAddr = 0x69;  // SDO high on the CoreS3

    explicit Bmi270(i2c_master_bus_handle_t bus) : I2cDevice(bus, kAddr) {}

    // Uploads the feature config and starts accel (±4 g) and gyro
    // (±1000 dps) at 100 Hz. Returns false if the chip is absent.
    bool Begin() {
        uint8_t id = 0;
        if (!Read(0x00, &id, 1) || id != 0x24) {
            ESP_LOGW(kTag, "BMI270 not found (chip id 0x%02X)", id);
            return false;
        }
        Write(0x7C, 0x00);  // PWR_CONF: advanced power save off
        vTaskDelay(pdMS_TO_TICKS(2));
        Write(0x59, 0x00);  // INIT_CTRL: start config load
        constexpr size_t kChunk = 64;
        const size_t total = sizeof(blob::bmi270_config_file);
        for (size_t off = 0; off < total; off += kChunk) {
            Write(0x5B, static_cast<uint8_t>((off / 2) & 0x0F));  // INIT_ADDR_0
            Write(0x5C, static_cast<uint8_t>((off / 2) >> 4));    // INIT_ADDR_1
            uint8_t buf[kChunk + 1];
            const size_t n = std::min(kChunk, total - off);
            buf[0] = 0x5E;  // INIT_DATA
            std::copy(blob::bmi270_config_file + off, blob::bmi270_config_file + off + n, buf + 1);
            if (i2c_master_transmit(i2c_device_, buf, n + 1, 100) != ESP_OK) {
                ESP_LOGW(kTag, "BMI270 config upload failed at %u", static_cast<unsigned>(off));
                return false;
            }
        }
        Write(0x59, 0x01);  // INIT_CTRL: done
        vTaskDelay(pdMS_TO_TICKS(30));
        uint8_t status = 0;
        Read(0x21, &status, 1);  // INTERNAL_STATUS
        if ((status & 0x0F) != 0x01) {
            ESP_LOGW(kTag, "BMI270 init status 0x%02X", status);
            return false;
        }
        Write(0x7D, 0x0E);  // PWR_CTRL: accel, gyro, temperature on
        Write(0x40, 0xA8);  // ACC_CONF: 100 Hz, normal filter, performance mode
        Write(0x41, 0x01);  // ACC_RANGE: ±4 g
        Write(0x42, 0xA8);  // GYR_CONF: 100 Hz, normal filter, performance mode
        Write(0x43, 0x01);  // GYR_RANGE: ±1000 dps
        Write(0x7C, 0x02);  // PWR_CONF: keep advanced power save off
        ESP_LOGI(kTag, "BMI270 ready");
        return true;
    }

    // accel in g, gyro in deg/s.
    bool ReadMotion(float accel[3], float gyro[3]) {
        uint8_t raw[12];
        if (!Read(0x0C, raw, sizeof(raw))) return false;  // ACC_X_LSB .. GYR_Z_MSB
        for (int i = 0; i < 3; i++) {
            accel[i] = static_cast<int16_t>(raw[2 * i] | raw[2 * i + 1] << 8) / 8192.0f;
            gyro[i] = static_cast<int16_t>(raw[6 + 2 * i] | raw[7 + 2 * i] << 8) / 32.768f;
        }
        return true;
    }

private:
    static constexpr const char* kTag = "Bmi270";

    bool Read(uint8_t reg, uint8_t* out, size_t len) {
        return i2c_master_transmit_receive(i2c_device_, &reg, 1, out, len, 100) == ESP_OK;
    }
    bool Write(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        return i2c_master_transmit(i2c_device_, buf, 2, 100) == ESP_OK;
    }
};

class Ltr553 : public I2cDevice {
public:
    static constexpr uint8_t kAddr = 0x23;

    explicit Ltr553(i2c_master_bus_handle_t bus) : I2cDevice(bus, kAddr) {}

    bool Begin() {
        uint8_t id = 0;
        if (!Read(0x86, &id, 1) || (id & 0xF0) != 0x90) {  // PART_ID
            ESP_LOGW(kTag, "LTR-553 not found (part id 0x%02X)", id);
            return false;
        }
        // The sensor sits behind the CoreS3's dark front glass, so one LED
        // pulse per reading (the default) barely registers a hand.
        Write(0x82, 0x7F);  // PS_LED: 60 kHz, 100 % duty, 100 mA
        Write(0x83, 0x0F);  // PS_N_PULSES: 15 pulses per reading
        Write(0x84, 0x02);  // PS_MEAS_RATE: 100 ms
        Write(0x81, 0x03);  // PS_CONTR: active
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI(kTag, "LTR-553 ready");
        return true;
    }

    // 0..2047, larger = closer; -1 on a bus error.
    int ReadProximity() {
        uint8_t raw[2];
        if (!Read(0x8D, raw, 2)) return -1;  // PS_DATA_0, PS_DATA_1
        return raw[0] | (raw[1] & 0x07) << 8;
    }

private:
    static constexpr const char* kTag = "Ltr553";

    bool Read(uint8_t reg, uint8_t* out, size_t len) {
        return i2c_master_transmit_receive(i2c_device_, &reg, 1, out, len, 100) == ESP_OK;
    }
    bool Write(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        return i2c_master_transmit(i2c_device_, buf, 2, 100) == ESP_OK;
    }
};

}  // namespace body_sensors
