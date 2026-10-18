#include "wifi_board.h"
#include "cores3_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "config.h"
#include "power_save_timer.h"
#include "i2c_device.h"
#include "axp2101.h"
#include "mcp_server.h"
#include "settings.h"
#include "assets/lang_config.h"
#include "led_strip.h"
// Issue #79: servo driver is selectable at build time via Kconfig.
//   - CONFIG_STACKCHAN_SERVO_SCSCL  (default): GPL-3.0 SCServo_lib
//   - CONFIG_STACKCHAN_SERVO_FEETECH: MIT clean-room driver vendored at
//     firmware/components/feetech_scs/.
// Both drivers share the same begin / WritePos / ReadPos call signatures
// used by this board, but their WritePos success value differs (see
// ServoWritePosOk() below). The rest of stackchan.cc treats both drivers
// uniformly through the ScsBus type alias plus that helper.
#if CONFIG_STACKCHAN_SERVO_DYNAMIXEL
// Stack-chan RT ver.: DYNAMIXEL XL330 driver with the same call shapes as
// FeetechScs, translating SCS0009 raw units per axis.
#include "dynamixel_bus.h"
using ScsBus = DynamixelBus;
// DynamixelBus::WritePos returns 0 on status ACK and -1 on bus error.
static inline bool ServoWritePosOk(int r) { return r >= 0; }
#elif CONFIG_STACKCHAN_SERVO_FEETECH
#include "feetech_scs.h"
using ScsBus = FeetechScs;
// FeetechScs::WritePos returns 0 on ACK and -1 on bus error.
static inline bool ServoWritePosOk(int r) { return r >= 0; }
#else
#include "SCSCL.h"
using ScsBus = SCSCL;
// SCSCL::WritePos returns 1 on ACK, 0 on ACK timeout, -1 on bus error.
// Treat ACK timeout as failure to keep the original behaviour intact.
static inline bool ServoWritePosOk(int r) { return r > 0; }
#endif
#include "avatar_images.h"
#include "avatar_set.h"
#include "avatar_set_fetcher.h"
#include "display/lvgl_display/lvgl_theme.h"
#include <font_awesome.h>
#include <esp_netif_sntp.h>
#include <ctime>
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
// Procedurally drawn face with ~27 expressions (components/procedural_face).
// Replaces the built-in six-image avatar whenever no custom AvatarSet is
// loaded; load_avatar_set still takes precedence.
#include "procedural_face.h"
#endif
#include "lvgl_display/gif/gifdec.h"
#if CONFIG_STACKCHAN_BODY_SENSORS
#include "body_events.h"
#include "body_sensors.h"
#endif

#include <smooth_ui_toolkit.hpp>
#include <esp_log.h>
#include <esp_wifi.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_commands.h>
#include <esp_lcd_ili9341.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/idf_additions.h>
#include "esp_video.h"
#include <cJSON.h>
#include <lvgl.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#define TAG "StackChanBoard"

#if CONFIG_STACKCHAN_PROCEDURAL_FACE
#define STACKCHAN_FACE_NAMES                                                       \
    "idle, happy, thinking, sad, surprised, embarrassed, laughing, excited, "      \
    "love, wink, smug, cool, crying, angry, annoyed, shocked, confused, "          \
    "doubtful, sleepy, bored, nervous, scared, curious, dizzy, relieved, "         \
    "listening, kiss"
#else
#define STACKCHAN_FACE_NAMES "idle, happy, thinking, sad, surprised, embarrassed"
#endif

#ifndef STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS
#define STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS 300
#endif

#define STACKCHAN_SCREEN_OFF_MAX_TIMEOUT_SECONDS 86400
#if STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS < 0 || \
    STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS > STACKCHAN_SCREEN_OFF_MAX_TIMEOUT_SECONDS
#error "STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS must be in 0..86400"
#endif
#define STACKCHAN_LCD_SLEEP_TRANSITION_MS 120
#define STACKCHAN_SCREEN_OFF_NVS_NAMESPACE "display"
#define STACKCHAN_SCREEN_OFF_NVS_KEY "off_timeout"

// Charge-control boot default (AXP2101 reg 0x18 bit1, see class Pmic below).
// Keep the AXP2101 reset default and the pre-feature firmware behaviour unless
// a build explicitly overrides it.
#ifndef STACKCHAN_DEFAULT_CHARGE_ENABLED
#define STACKCHAN_DEFAULT_CHARGE_ENABLED 1
#endif

// Automatic charge hysteresis. These values may be overridden with compiler
// -D options. The existing one-second status-bar battery poll drives the
// decision at a 60-second cadence; no dedicated timer or task is created.
#ifndef STACKCHAN_CHARGE_AUTO
#define STACKCHAN_CHARGE_AUTO 0
#endif

#ifndef STACKCHAN_CHARGE_ON_BELOW
#define STACKCHAN_CHARGE_ON_BELOW 30
#endif

#ifndef STACKCHAN_CHARGE_OFF_ABOVE
#define STACKCHAN_CHARGE_OFF_ABOVE 70
#endif

#if STACKCHAN_CHARGE_ON_BELOW < 0 || STACKCHAN_CHARGE_ON_BELOW > 100
#error "STACKCHAN_CHARGE_ON_BELOW must be between 0 and 100"
#endif

#if STACKCHAN_CHARGE_OFF_ABOVE < 0 || STACKCHAN_CHARGE_OFF_ABOVE > 100
#error "STACKCHAN_CHARGE_OFF_ABOVE must be between 0 and 100"
#endif

#if STACKCHAN_CHARGE_ON_BELOW >= STACKCHAN_CHARGE_OFF_ABOVE
#error "STACKCHAN_CHARGE_ON_BELOW must be lower than STACKCHAN_CHARGE_OFF_ABOVE"
#endif

class Pmic : public Axp2101 {
public:
    // Power Init
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        uint8_t data = ReadReg(0x90);
        data |= 0b10110100;
        WriteReg(0x90, data);
        WriteReg(0x99, (0b11110 - 5));
        WriteReg(0x97, (0b11110 - 2));
        WriteReg(0x69, 0b00110101);
        WriteReg(0x30, 0b111111);
        WriteReg(0x90, 0xBF);
        WriteReg(0x94, 33 - 5);
        WriteReg(0x95, 33 - 5);

        // Apply the charge-control boot default last, after every other
        // Power Init write above. This preserves the established PMIC
        // initialization order and changes charging only after all other
        // controls are configured.
        SetChargeEnabled(STACKCHAN_DEFAULT_CHARGE_ENABLED != 0);
    }

    // AXP2101 reg 0x18 bit1 = Cell Battery charge enable (datasheet
    // §6.13.2.14: 0=disable, 1=enable, System Reset default=1).
    // Read-Modify-Write only bit1; bit3 (Gauge Module enable), bit2
    // (Button Battery charge enable), and bit0 (Watchdog Module enable)
    // must survive untouched.
    void SetChargeEnabled(bool enable) {
        uint8_t value = ReadReg(0x18);
        if (enable) {
            value |= 0b00000010;
        } else {
            value &= static_cast<uint8_t>(~0b00000010);
        }
        WriteReg(0x18, value);
    }

    bool IsChargeEnabled() {
        return (ReadReg(0x18) & 0b00000010) != 0;
    }

    uint8_t GetChargeControlRegister() {
        return ReadReg(0x18);
    }

    // Read the same AXP2101 fuel-gauge register used by
    // Axp2101::GetBatteryLevel(), but return a failure instead of aborting the
    // firmware on an I2C error. Values outside the percentage range are also
    // treated as unavailable so automatic control can fail safe to charging.
    bool TryGetBatteryLevel(int* level) {
        uint8_t reg = 0xA4;
        uint8_t value = 0;
        esp_err_t err =
            i2c_master_transmit_receive(i2c_device_, &reg, 1, &value, 1, 100);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Battery level read failed: %s",
                     esp_err_to_name(err));
            return false;
        }
        if (value > 100) {
            ESP_LOGW(TAG, "Battery level is invalid: %u",
                     static_cast<unsigned>(value));
            return false;
        }
        *level = value;
        return true;
    }

    void SetBrightness(uint8_t brightness) {
        brightness = ((brightness + 641) >> 5);
        WriteReg(0x99, brightness);
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(Pmic *pmic) : pmic_(pmic) {}

    void SetBrightnessImmediate(uint8_t brightness) {
        if (brightness > 100) {
            brightness = 100;
        }
        if (transition_timer_ != nullptr) {
            esp_timer_stop(transition_timer_);
        }
        target_brightness_ = brightness;
        SetBrightnessImpl(brightness);
    }

    void SetBrightnessImpl(uint8_t brightness) override {
        pmic_->SetBrightness(target_brightness_);
        brightness_ = target_brightness_;
    }

private:
    Pmic *pmic_;
};

class Aw9523 : public I2cDevice {
public:
    // Exanpd IO Init
    Aw9523(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x02, 0b00000111);  // P0
        WriteReg(0x03, 0b10001111);  // P1
        WriteReg(0x04, 0b00011000);  // CONFIG_P0
        WriteReg(0x05, 0b00001100);  // CONFIG_P1
        WriteReg(0x11, 0b00010000);  // GCR P0 port is Push-Pull mode.
        WriteReg(0x12, 0b11111111);  // LEDMODE_P0
        WriteReg(0x13, 0b11111111);  // LEDMODE_P1
    }

    void ResetAw88298() {
        ESP_LOGI(TAG, "Reset AW88298");
        WriteReg(0x02, 0b00000011);
        vTaskDelay(pdMS_TO_TICKS(10));
        WriteReg(0x02, 0b00000111);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void ResetIli9342() {
        ESP_LOGI(TAG, "Reset IlI9342");
        WriteReg(0x03, 0b10000001);
        vTaskDelay(pdMS_TO_TICKS(20));
        WriteReg(0x03, 0b10000011);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    
    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

// Minimal PY32 IO Expander driver (servo power switch on pin 0 / VM EN).
// Ported from M5Stack-BSP PY32IOExpander.cpp.
//
// IMPORTANT: this class deliberately does NOT inherit from I2cDevice.
// The base I2cDevice registers every device at scl_speed_hz = 400 kHz, but
// the M5 reference implementation (`PY32IOExpander_Class`) defaults to
// 100 kHz, and 400 kHz appears to leave PY32 in a half-finished slave
// state — `i2c_master_probe` returns ACK but the very next
// `i2c_master_transmit_receive` for REG_VERSION times out (0x103) every
// time. We register our own i2c_master device handle at 100 kHz to match
// the M5 default. Other peripherals on the bus (Si12T at 0x68, AXP2101,
// AW9523, FT6336) keep using the 400 kHz path through I2cDevice.
//
// Reliability notes:
//  - Each I2C op transparently retries up to I2C_INNER_RETRIES on transient
//    errors, with a short vTaskDelay between attempts.
//  - All bit-level write helpers propagate success as bool so the caller
//    can decide whether the GPIO actually got configured.
//  - Begin() can optionally return the version byte so the caller can log
//    which attempt finally talked to the chip.
class Py32IoExpander {
public:
    static constexpr uint8_t  DEFAULT_ADDR = 0x6F;
    static constexpr uint32_t I2C_FREQ_HZ  = 100000;  // 100 kHz (M5 default)
    static constexpr uint8_t  REG_GPIO_O_L_PUBLIC = 0x05;  // exposed for verify

    Py32IoExpander(i2c_master_bus_handle_t i2c_bus, uint8_t addr = DEFAULT_ADDR) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = addr,
            .scl_speed_hz    = I2C_FREQ_HZ,
            .scl_wait_us     = 0,
            .flags           = { .disable_ack_check = 0 },
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &cfg, &i2c_device_));
    }

    // Probe the chip. On success, returns true and (if non-null) writes the
    // version byte to out_version. Internal reads use SafeReadReg, which
    // already retries on transient I2C errors — so the chip is genuinely
    // unreachable / not yet ready when this returns false.
    bool Begin(uint8_t* out_version = nullptr) {
        uint8_t version = 0;
        if (!SafeReadReg(REG_VERSION, &version)) {
            return false;
        }
        if (version == 0x00 || version == 0xFF) {
            return false;
        }
        if (out_version != nullptr) {
            *out_version = version;
        }
        return true;
    }

    // direction: false=input, true=output. Accepts pin 0..15 (PY32 has 14
    // GPIOs; the WS2812 data line is on pin 13, in the high byte).
    bool SetDirection(uint8_t pin, bool output) {
        return WriteBitWideSafe(REG_GPIO_M_L, REG_GPIO_M_H, pin, output);
    }

    // mode: false=pull down, true=pull up. Accepts pin 0..15.
    bool SetPullMode(uint8_t pin, bool up) {
        if (up) {
            bool a = WriteBitWideSafe(REG_GPIO_PD_L, REG_GPIO_PD_H, pin, false);
            bool b = WriteBitWideSafe(REG_GPIO_PU_L, REG_GPIO_PU_H, pin, true);
            return a && b;
        } else {
            bool a = WriteBitWideSafe(REG_GPIO_PU_L, REG_GPIO_PU_H, pin, false);
            bool b = WriteBitWideSafe(REG_GPIO_PD_L, REG_GPIO_PD_H, pin, true);
            return a && b;
        }
    }

    bool DigitalWrite(uint8_t pin, bool level) {
        return WriteBitSafe(REG_GPIO_O_L, pin, level);
    }

    // Read back the current output low-byte register (pins 0..7) for
    // verification after DigitalWrite. Returns false if the read failed.
    bool ReadOutputLow(uint8_t* out) {
        return SafeReadReg(REG_GPIO_O_L, out);
    }

    // Drive mode for any pin (0..15). false=push-pull, true=open-drain.
    // The WS2812 data line on pin 13 must be push-pull.
    bool SetDriveMode(uint8_t pin, bool open_drain) {
        return WriteBitWideSafe(REG_GPIO_DRV_L, REG_GPIO_DRV_H, pin, open_drain);
    }

    // ---- LED (WS2812 driven by the PY32 itself, data line on pin 13) ----
    // REG_LED_CFG packs both the LED count (bits 0-5, max 32) and the latch
    // trigger (bit 6). Writing the count clears bit 6, which is fine because
    // it's a self-clearing strobe. RefreshLeds() does read-modify-write so
    // the count is preserved when we latch.
    bool SetLedCount(uint8_t count) {
        if (count > 32) count = 32;
        return SafeWriteReg(REG_LED_CFG, count & 0x3F);
    }

    // Set one LED to RGB888. RGB888 → RGB565 packing matches the M5 BSP:
    // ((r&0xF8)<<8) | ((g&0xFC)<<3) | (b>>3), little-endian on the wire.
    // Does NOT latch — call RefreshLeds() once after a batch of updates.
    bool SetLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (index >= 32) return false;
        uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        uint8_t buf[3] = { (uint8_t)(REG_LED_RAM_START + index * 2),
                           (uint8_t)(v & 0xFF),
                           (uint8_t)((v >> 8) & 0xFF) };
        return SafeWriteRaw(buf, sizeof(buf));
    }

    // Burst-write up to N LED RGB565 pairs starting at index 0. data is
    // packed { lo0, hi0, lo1, hi1, ... } and len is the byte count
    // (=2*num_leds, max 64). Single I2C transaction — much faster than
    // calling SetLedColor in a loop. Does NOT latch.
    bool SetLedData(const uint8_t* data, size_t len) {
        if (data == nullptr || len == 0) return false;
        if (len > 64) len = 64;
        uint8_t buf[1 + 64];
        buf[0] = REG_LED_RAM_START;
        for (size_t i = 0; i < len; i++) buf[1 + i] = data[i];
        return SafeWriteRaw(buf, 1 + len);
    }

    // Latch the LED RAM out to the WS2812 strip. Read-modify-write so the
    // current count (bits 0-5) is preserved alongside the latch bit (bit 6).
    bool RefreshLeds() {
        uint8_t cfg = 0;
        if (!SafeReadReg(REG_LED_CFG, &cfg)) return false;
        return SafeWriteReg(REG_LED_CFG, (uint8_t)(cfg | (1u << 6)));
    }

private:
    // Owned device handle (NOT inherited from I2cDevice — see class comment).
    // Registered at 100 kHz in the constructor so this transport runs slower
    // than the rest of the bus.
    i2c_master_dev_handle_t i2c_device_ = nullptr;

    static constexpr uint8_t REG_VERSION  = 0x02;
    static constexpr uint8_t REG_GPIO_M_L = 0x03;  // Direction (mode) low byte
    static constexpr uint8_t REG_GPIO_M_H = 0x04;  // Direction (mode) high byte
    static constexpr uint8_t REG_GPIO_O_L = 0x05;  // Output low byte
    static constexpr uint8_t REG_GPIO_O_H = 0x06;  // Output high byte
    static constexpr uint8_t REG_GPIO_PU_L = 0x09; // Pull-up low byte
    static constexpr uint8_t REG_GPIO_PU_H = 0x0A; // Pull-up high byte
    static constexpr uint8_t REG_GPIO_PD_L = 0x0B; // Pull-down low byte
    static constexpr uint8_t REG_GPIO_PD_H = 0x0C; // Pull-down high byte
    static constexpr uint8_t REG_GPIO_DRV_L = 0x13; // Drive mode low byte
    static constexpr uint8_t REG_GPIO_DRV_H = 0x14; // Drive mode high byte
    static constexpr uint8_t REG_LED_CFG       = 0x24;  // count[5:0] + latch[6]
    static constexpr uint8_t REG_LED_RAM_START = 0x30;  // 2 bytes per LED, RGB565 LE

    // I2C op transient-retry parameters. Total budget per failed op is
    // (I2C_INNER_RETRIES - 1) * I2C_RETRY_DELAY_MS, i.e. ~30 ms here.
    static constexpr int I2C_INNER_RETRIES  = 3;
    static constexpr int I2C_RETRY_DELAY_MS = 15;

    // Safe I2C read — retries up to I2C_INNER_RETRIES on transient errors,
    // logs WARN only on the final failure to keep the log readable.
    bool SafeReadReg(uint8_t reg, uint8_t* out) {
        esp_err_t err = ESP_FAIL;
        for (int i = 0; i < I2C_INNER_RETRIES; i++) {
            err = i2c_master_transmit_receive(i2c_device_, &reg, 1, out, 1, 100);
            if (err == ESP_OK) {
                return true;
            }
            if (i + 1 < I2C_INNER_RETRIES) {
                vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
            }
        }
        ESP_LOGW("Py32IoExpander", "I2C read reg 0x%02X failed after %d tries: 0x%X",
                 reg, I2C_INNER_RETRIES, err);
        return false;
    }

    // Safe I2C write — same retry semantics as SafeReadReg.
    bool SafeWriteReg(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        esp_err_t err = ESP_FAIL;
        for (int i = 0; i < I2C_INNER_RETRIES; i++) {
            err = i2c_master_transmit(i2c_device_, buffer, 2, 100);
            if (err == ESP_OK) {
                return true;
            }
            if (i + 1 < I2C_INNER_RETRIES) {
                vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
            }
        }
        ESP_LOGW("Py32IoExpander", "I2C write reg 0x%02X failed after %d tries: 0x%X",
                 reg, I2C_INNER_RETRIES, err);
        return false;
    }

    // Read-Modify-Write a single bit using safe I2C. Pin 0..7 only.
    // Returns false if either the read or the write step ultimately failed.
    bool WriteBitSafe(uint8_t reg, uint8_t pin, bool value) {
        if (pin >= 8) {
            return false;
        }
        uint8_t v = 0;
        if (!SafeReadReg(reg, &v)) {
            return false;
        }
        if (value) {
            v |= (uint8_t)(1u << pin);
        } else {
            v &= (uint8_t)~(1u << pin);
        }
        return SafeWriteReg(reg, v);
    }

    // 16-bit RMW: pin 0..7 -> reg_l, pin 8..15 -> reg_h. Used for any pin
    // beyond pin 7 (LED data line is on pin 13, so all the LED setup goes
    // through this path).
    bool WriteBitWideSafe(uint8_t reg_l, uint8_t reg_h, uint8_t pin, bool value) {
        if (pin >= 16) return false;
        uint8_t reg = (pin < 8) ? reg_l : reg_h;
        uint8_t bit = (uint8_t)(pin & 0x07);
        uint8_t v = 0;
        if (!SafeReadReg(reg, &v)) return false;
        if (value) v |= (uint8_t)(1u << bit);
        else       v &= (uint8_t)~(1u << bit);
        return SafeWriteReg(reg, v);
    }

    // Burst write: ship a pre-built {reg, ...payload} buffer in a single
    // i2c_master_transmit. Used by the LED RAM writes which would otherwise
    // require dozens of individual register writes. Same retry semantics
    // as SafeWriteReg.
    bool SafeWriteRaw(const uint8_t* buf, size_t len) {
        esp_err_t err = ESP_FAIL;
        for (int i = 0; i < I2C_INNER_RETRIES; i++) {
            err = i2c_master_transmit(i2c_device_, buf, len, 100);
            if (err == ESP_OK) {
                return true;
            }
            if (i + 1 < I2C_INNER_RETRIES) {
                vTaskDelay(pdMS_TO_TICKS(I2C_RETRY_DELAY_MS));
            }
        }
        ESP_LOGW("Py32IoExpander", "I2C raw write (len=%u) failed after %d tries: 0x%X",
                 (unsigned)len, I2C_INNER_RETRIES, err);
        return false;
    }
};

// Minimal Si12T driver (12-channel capacitive touch sensor, TSM12-compatible).
// Used for the StackChan head-stroke / head-tap detection. Only the read path
// (Output1 register, channels 1-4) is needed for Phase 7. We expose just the
// first three channels through ReadTouchState() because the StackChan head
// has 3 conductive zones wired to TS1..TS3.
//
// Datasheet excerpt:
//   - I2C 7-bit address: 0xD0 >> 1 == 0x68 when ID_SEL pin is tied to GND.
//     This matches the address probed at boot ("0x68").
//   - Reset value of CTRL (0x09) is 0b00000111 (SLEEP=1). We must clear SLEEP
//     to enter normal sensing mode: CTRL = 0b00000011.
//   - Output1 (0x10) packs four channels into one byte (2 bits per channel):
//       bit[1:0] = OUT1, bit[3:2] = OUT2, bit[5:4] = OUT3, bit[7:6] = OUT4
//       00 = no output, 01 = low, 10 = medium, 11 = high.
//   - There is no dedicated chip-id register, so Begin() validates the device
//     via successful I2C ACK on the CTRL read + non-0xFF Output1 read.
class Si12T : public I2cDevice {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x68;  // ID_SEL = GND

    struct TouchState {
        bool zone[3];          // CH1, CH2, CH3 — true if any output level set
        uint8_t output1_raw;   // raw Output1 register byte (0x10)
        bool ok;               // false if the I2C read failed
    };

    Si12T(i2c_master_bus_handle_t i2c_bus, uint8_t addr = DEFAULT_ADDR)
        : I2cDevice(i2c_bus, addr) {}

    // Probe the chip and bring it out of sleep. Returns true on success.
    bool Begin() {
        uint8_t ctrl = 0;
        if (!SafeReadReg(REG_CTRL, &ctrl)) {
            return false;
        }
        // CTRL bit1 = SLEEP. Clear it; bit1:0 must hold 1 per datasheet
        // ("CTRL Bit1, Bit0 = 1 1" reset value), so write 0b00000011.
        if (!SafeWriteReg(REG_CTRL, 0x03)) {
            return false;
        }
        // Verify the device actually responds on the output register.
        // 0xFF would indicate an open bus / no device.
        uint8_t out1 = 0;
        if (!SafeReadReg(REG_OUTPUT1, &out1)) {
            return false;
        }
        if (out1 == 0xFF) {
            ESP_LOGW("Si12T", "Output1 read 0xFF (likely no device)");
            return false;
        }
        ESP_LOGI("Si12T", "init OK: ctrl=0x%02X out1=0x%02X (sleep cleared)", ctrl, out1);
        return true;
    }

    // Sample channels CH1..CH3 from Output1 (0x10). Single-shot read; the
    // caller is expected to debounce / interpret duration externally.
    TouchState ReadTouchState() {
        TouchState s = {};
        s.ok = false;
        if (!SafeReadReg(REG_OUTPUT1, &s.output1_raw)) {
            return s;
        }
        s.ok = true;
        // Each channel uses 2 bits; nonzero = touched at some level.
        s.zone[0] = ((s.output1_raw >> 0) & 0x3) != 0;  // CH1
        s.zone[1] = ((s.output1_raw >> 2) & 0x3) != 0;  // CH2
        s.zone[2] = ((s.output1_raw >> 4) & 0x3) != 0;  // CH3
        return s;
    }

private:
    static constexpr uint8_t REG_CTRL    = 0x09;  // CTRL, SLEEP bit etc.
    static constexpr uint8_t REG_OUTPUT1 = 0x10;  // CH1..CH4 packed (2bpp)

    bool SafeReadReg(uint8_t reg, uint8_t* out) {
        esp_err_t err = i2c_master_transmit_receive(i2c_device_, &reg, 1, out, 1, 100);
        if (err != ESP_OK) {
            ESP_LOGW("Si12T", "I2C read reg 0x%02X failed: 0x%X", reg, err);
            return false;
        }
        return true;
    }

    bool SafeWriteReg(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        esp_err_t err = i2c_master_transmit(i2c_device_, buffer, 2, 100);
        if (err != ESP_OK) {
            ESP_LOGW("Si12T", "I2C write reg 0x%02X failed: 0x%X", reg, err);
            return false;
        }
        return true;
    }
};

class StackChanBoard : public WifiBoard {
private:
    // Internal I2C bus (shared by AXP2101 / AW9523 / FT6336 / PY32 / Si12T /
    // audio codec / IMU). Direct on-board ICs only; not exposed through
    // self.i2c.* MCP tools.
    i2c_master_bus_handle_t i2c_bus_;
    // External I2C bus dedicated to Grove Port A. Exposed through self.i2c.*
    // MCP tools so the gateway can drive attached M5Stack Unit modules.
    i2c_master_bus_handle_t port_a_i2c_bus_;
    // Port B WS2812 generic strip state (driven from MCP tools self.port_b.ws2812.*).
    // Independent from the on-board PY32-driven 12-LED base strip (self.led.*),
    // which uses I2C -> PY32 internal WS2812 engine. The two paths share no
    // hardware peripheral and no software state; existing self.led.* behaviour
    // is byte-for-byte unchanged.
    bool ws2812_ok_ = false;
    uint16_t ws2812_led_count_ = 0;
    led_strip_handle_t ws2812_handle_ = nullptr;
    static constexpr gpio_num_t PORT_B_WS2812_DATA_PIN = GPIO_NUM_9;  // CoreS3 HY2.0-4P (Port B) digital OUTPUT
    static constexpr uint16_t PORT_B_WS2812_MAX_LEDS = 256;
    // Port C WS2812 generic strip state (driven from MCP tools self.port_c.ws2812.*).
    bool port_c_ws2812_ok_ = false;
    uint16_t port_c_ws2812_led_count_ = 0;
    led_strip_handle_t port_c_ws2812_handle_ = nullptr;
    static constexpr gpio_num_t PORT_C_WS2812_DATA_PIN = GPIO_NUM_17;  // CoreS3 HY2.0-4P (Port C) signal 1
    static constexpr uint16_t PORT_C_WS2812_MAX_LEDS = 256;
    Pmic* pmic_;
    enum class ChargeAutoDecision : uint8_t {
        kNotRun = 0,
        kEnabledLowBattery,
        kDisabledHighBattery,
        kKeptBetweenThresholds,
        kEnabledBatteryReadFailed,
        kAutoDisabled,
    };
    static constexpr uint32_t CHARGE_AUTO_INTERVAL_MS = 60 * 1000;
    std::atomic<ChargeAutoDecision> last_charge_auto_decision_{
        (STACKCHAN_CHARGE_AUTO != 0)
            ? ChargeAutoDecision::kNotRun
            : ChargeAutoDecision::kAutoDisabled};
    std::atomic<int> last_charge_auto_battery_level_{-1};
    std::atomic<uint32_t> last_charge_auto_check_ms_{0};
    Aw9523* aw9523_;
    Ft6336* ft6336_;
    LcdDisplay* display_;
    EspVideo* camera_;
    esp_timer_handle_t touchpad_timer_;
    PowerSaveTimer* power_save_timer_;
    esp_lcd_panel_io_handle_t lcd_panel_io_ = nullptr;
    esp_lcd_panel_handle_t lcd_panel_ = nullptr;
    esp_timer_handle_t screen_off_timer_ = nullptr;
    SemaphoreHandle_t screen_power_mutex_ = nullptr;
    std::atomic<int> screen_off_timeout_seconds_{
        STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS};
    std::atomic<bool> screen_is_off_{false};
    std::atomic<bool> dimmed_{false};  // power-save sleep (backlight at 10)
    int64_t screen_off_deadline_us_ = 0;  // guarded by screen_power_mutex_
    int64_t panel_sleep_started_us_ = 0;  // guarded by screen_power_mutex_
    uint8_t screen_saved_brightness_ = 75;  // guarded by screen_power_mutex_
    bool panel_sleeping_ = false;  // guarded by screen_power_mutex_
    bool panel_display_disabled_ = false;  // guarded by screen_power_mutex_
    ScsBus scs_bus_;
    std::unique_ptr<Py32IoExpander> io_expander_;

    // Avatar overlay state. avatar_img_ is created lazily on the active LVGL
    // screen because the screen tree (container_, emoji_label_, ...) is built
    // by Application::Start() -> Display::SetupUI(), which runs after this
    // board's constructor completes. avatar_init_timer_ retries every 500 ms
    // until the screen is ready, then stops itself.
    lv_obj_t* avatar_img_ = nullptr;
    esp_timer_handle_t avatar_init_timer_ = nullptr;
    std::string current_avatar_face_ = "idle";

    // Board-local listening cue shown above the full-screen avatar layer.
    lv_obj_t* listening_indicator_ = nullptr;
    lv_obj_t* listening_indicator_dot_ = nullptr;
    std::atomic<bool> listening_indicator_visible_{false};
    static constexpr int LISTENING_INDICATOR_DOT_SIZE_PX = 14;

    // Dynamic avatar set loaded via the load_avatar_set MCP tool. Stays
    // unloaded by default — the index-based image lookups then fall back
    // to the static const tables in avatar_images.h (placeholder or local
    // override). See docs/intent/stackchan_avatar_pipeline.md in the
    // SAIVerse repository.
    AvatarSet avatar_set_;

    // ---- Avatar rendering state (Phase 4.5-a) -----------------------------
    //
    // The avatar is represented as three independent axes — face, eyes,
    // mouth — each carrying a 0-indexed slot identical to AvatarSet's
    // GetFace / GetEyes / GetMouth layout. The on-screen image is then
    // derived from this state in a mode-aware way (RenderAvatarLocked):
    //
    //   - Layered mode (or AvatarSet not loaded): there is no compositor
    //     on the firmware side — avatar_img_ shows exactly one image at a
    //     time. active_layer_ selects which axis drives the current frame
    //     (face during rest / mouth during set_mouth / eyes during blink),
    //     matching the upstream Phase 2 behaviour where blink temporarily
    //     replaces the face image and is then restored.
    //   - Matrix mode: avatar_set_.GetMatrix(face, eyes, mouth) returns the
    //     pre-composed image for the current (face, eyes, mouth) triple.
    //     active_layer_ is ignored; every state change updates the
    //     composed frame.
    //
    // Indices remain valid across mode switches so a future load_avatar_set
    // call into a different mode does not lose the persona's current
    // expression. current_avatar_face_ is kept as the string form because
    // existing internal callers (touch reactions, SetAvatarOff resume,
    // mouth-sequence restore) still address the face by name.
    enum class ActiveLayer : uint8_t {
        FACE = 0,
        EYES = 1,
        MOUTH = 2,
    };
    int current_face_index_ = 0;   // 0..5  (idle / happy / thinking / sad / surprised / embarrassed)
                                   // 0..26 with CONFIG_STACKCHAN_PROCEDURAL_FACE
    int current_eyes_index_ = 0;   // 0..2  (open / half / closed) — 0 is the resting state
    int current_mouth_index_ = 0;  // 0..4  (closed / half / open / e / u) — 0 is the resting state
    ActiveLayer active_layer_ = ActiveLayer::FACE;
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
    // Two full-screen RGB565 frames in PSRAM, alternated on every render so
    // LVGL always sees a new image source and redraws.
    // Drawn at full panel resolution and shown unscaled. A half-size image
    // shown at 2x made LVGL software-transform every frame, which kept
    // core 1 saturated and corrupted the panel stream once faces animated.
    static constexpr int kProcFaceW = 320;
    static constexpr int kProcFaceH = 240;
    uint16_t* proc_face_buf_[2] = {nullptr, nullptr};
    lv_image_dsc_t proc_face_dsc_[2]{};
    int proc_face_front_ = 0;
    // Animation frames are drawn by a low-priority task into this scratch
    // buffer without holding the display lock; only the swap is locked.
    uint16_t* proc_face_scratch_ = nullptr;
    std::atomic<uint32_t> proc_face_generation_{0};
    TaskHandle_t proc_face_anim_task_ = nullptr;
    // Animation period (0 = only redraw on changes), adjustable at runtime
    // via self.display.set_animation for diagnosing display glitches.
    std::atomic<int> proc_face_anim_ms_{200};
    // Frame timing stats since the last self.display.get_face_stats call.
    std::atomic<uint32_t> face_stat_frames_{0};
    std::atomic<uint32_t> face_stat_render_us_sum_{0};
    std::atomic<uint32_t> face_stat_render_us_max_{0};
    std::atomic<uint32_t> face_stat_lock_us_max_{0};
    std::atomic<uint32_t> face_stat_lock_us_sum_{0};
    // Eye direction set via self.display.set_gaze, in design pixels.
    float gaze_x_ = 0.0f;
    float gaze_y_ = 0.0f;
#endif

    // Screen overlays drawn above the face on lv_layer_top():
    // self.display.show_image (a 320x240 RGB565 picture fetched from the
    // gateway) and self.display.show_text (a caption). Each hides after its
    // timeout, and a short tap on the screen dismisses them instead of
    // starting to listen.
    static constexpr int kOverlayImageW = 320;
    static constexpr int kOverlayImageH = 240;
    static constexpr size_t kOverlayImageBytes =
        static_cast<size_t>(kOverlayImageW) * kOverlayImageH * 2;
    uint8_t* overlay_image_buf_ = nullptr;
    lv_image_dsc_t overlay_image_dsc_{};
    lv_obj_t* overlay_image_ = nullptr;
    lv_obj_t* overlay_text_ = nullptr;
    esp_timer_handle_t overlay_image_timer_ = nullptr;
    esp_timer_handle_t overlay_text_timer_ = nullptr;
    std::atomic<bool> overlay_image_loading_{false};
    std::string overlay_image_last_error_;

    // Drawing canvas (self.display.canvas): a 320x240 RGB565 canvas on
    // lv_layer_top() above the face. Shapes arrive in batches, are drawn
    // with LVGL's software renderer and stay until canvas_hide,
    // clear_overlay or a tap. Hiding keeps the drawing for the next batch.
    static constexpr int kCanvasMaxOps = 256;
    uint8_t* canvas_buf_ = nullptr;
    lv_obj_t* canvas_ = nullptr;

    // ---- GIF player (self.display.show_gif) ----
    // One persistent task downloads the GIF into PSRAM and plays it through a
    // PSRAM RGB565 frame. Each frame only the rectangle that changed is
    // converted and redrawn, and it is rendered at once with lv_refr_now
    // under the display lock, so every frame reaches the panel and LVGL never
    // reads the frame mid-update (no second buffer needed). While it plays
    // the face is hidden and its task paused, so nothing redraws in between.
    static constexpr size_t kGifMaxBytes = 3 * 1024 * 1024;
    struct GifJob {
        std::string url, token;
        int seconds = 0;  // 0 = no time limit
        int loops = 0;    // 0 = repeat until seconds / tap / clear
        bool fast = false;  // ignore the GIF's frame delays
    };
    TaskHandle_t gif_task_ = nullptr;
    SemaphoreHandle_t gif_mutex_ = nullptr;  // guards gif_pending_
    std::unique_ptr<GifJob> gif_pending_;
    std::atomic<bool> gif_stop_{false};
    std::atomic<bool> gif_playing_{false};
    std::atomic<int> gif_fps_x10_{0};
    std::string gif_last_error_;
    std::string gif_last_stats_;  // per-stage timing of the last GIF
    uint16_t* gif_buf_ = nullptr;
    lv_image_dsc_t gif_dsc_ = {};
    lv_obj_t* gif_img_ = nullptr;
    bool gif_hid_avatar_ = false;  // restore only what the GIF hid

    // Wi-Fi (top left) and battery (top right) indicators drawn above the
    // face, which otherwise covers xiaozhi's own status bar.
    lv_obj_t* status_wifi_label_ = nullptr;
    lv_obj_t* status_battery_label_ = nullptr;
    lv_obj_t* status_battery_pct_label_ = nullptr;
    esp_timer_handle_t status_icons_timer_ = nullptr;
    esp_timer_handle_t lcd_refresh_timer_ = nullptr;
    // Off by default while diagnosing: the re-send itself talks to the panel.
    std::atomic<bool> lcd_refresh_enabled_{false};
    // Whether the gateway reports a Claude client (Claude Code / Desktop)
    // connected; pushed by the gateway via self.display.set_link_status.
    std::atomic<bool> claude_connected_{false};

    // Clock screensaver: after CONFIG_STACKCHAN_CLOCK_SCREENSAVER_SECONDS
    // without activity (speech, touch, face/tool changes) the face is
    // covered by a large clock set from SNTP. Any activity removes it.
    uint16_t* clock_buf_ = nullptr;
    lv_image_dsc_t clock_dsc_{};
    lv_obj_t* clock_img_ = nullptr;
    lv_obj_t* clock_date_ = nullptr;
    esp_timer_handle_t clock_tick_timer_ = nullptr;
    std::atomic<int64_t> last_activity_us_{0};
    bool clock_visible_ = false;
    int64_t clock_shown_us_ = 0;
    int64_t clock_hidden_us_ = 0;
    int clock_drawn_minute_ = -1;
    bool sntp_started_ = false;

    // Bottom text strip shared by the ticker (self.display.set_ticker) and
    // speech subtitles (self.display.set_subtitle, sent by the gateway's
    // say()). A subtitle temporarily replaces the ticker.
    lv_obj_t* strip_bar_ = nullptr;
    lv_obj_t* strip_label_ = nullptr;
    std::string ticker_text_;
    int ticker_speed_px_s_ = 60;
    esp_timer_handle_t ticker_timer_ = nullptr;
    std::string subtitle_text_;
    int subtitle_duration_ms_ = 0;
    int64_t subtitle_scroll_end_us_ = 0;  // when the one-pass scroll finishes
    esp_timer_handle_t subtitle_clear_timer_ = nullptr;
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
    procedural_face::Style face_style_ = procedural_face::Style::kStackchan;
#endif

    // Pending state captured while an avatar_set_fetch is in flight.
    // Calls to set_avatar / set_mouth_shape / set_blink during a fetch
    // are buffered here and replayed once the new AvatarSet is loaded,
    // so the UI does not flicker between the old and partially-loaded
    // new sets. avatar_fetch_in_progress_ is the entry guard: a
    // concurrent avatar_set_fetch is rejected with
    // avatar_set_loaded error="fetch_in_progress" rather than racing
    // the worker task that is already running.
    std::atomic<bool> avatar_fetch_in_progress_{false};
    SemaphoreHandle_t avatar_pending_lock_ = nullptr;
    struct PendingAvatarState {
        bool has_off = false;
        bool has_face = false;     std::string face_name;
        bool has_mouth = false;    std::string mouth_shape;
        bool has_blink = false;    bool blink_enabled = false;
    };
    PendingAvatarState avatar_pending_;

    // Phase 2: blinking + lip-sync overlay state.
    // Blink works as a four-step state machine driven by blink_step_timer_:
    //   FACE -> EYES_HALF -> EYES_CLOSED -> EYES_HALF -> FACE (restore last face)
    // Each step is BLINK_STEP_MS apart. While a blink is in progress, further
    // schedule events are dropped (we run to completion before re-arming).
    // blink_schedule_timer_ fires every 3-6s (re-armed each cycle) and triggers
    // a new blink only if blink_enabled_ and no other blink is in flight.
    enum class BlinkState : uint8_t {
        IDLE = 0,
        EYES_HALF_DOWN,
        EYES_CLOSED,
        EYES_HALF_UP,
    };
    static constexpr int BLINK_STEP_MS = 100;
    static constexpr int BLINK_MIN_GAP_MS = 3000;
    static constexpr int BLINK_MAX_GAP_MS = 6000;
    esp_timer_handle_t blink_schedule_timer_ = nullptr;
    esp_timer_handle_t blink_step_timer_ = nullptr;
    BlinkState blink_state_ = BlinkState::IDLE;
    bool blink_enabled_ = false;
    // Captures blink_enabled_ at the moment SetAvatarOff() runs, so that a
    // later set_avatar(<other face>) can restore the previous blink state.
    // Only meaningful while current_avatar_face_ == "off".
    bool blink_enabled_before_off_ = false;

    // Phase 4 audio (Issue #76): state-driven TTS lip-sync animation.
    // Driven by the gateway's tts.start / tts.stop notifications (see
    // Application::OnIncomingJson) via Board::OnTtsStart / OnTtsStop;
    // cycles the mouth through closed -> half -> open -> half on a fixed
    // TTS_LIPSYNC_STEP_MS cadence until stopped. Autonomous blink is paused
    // while active (same Phase 2 trade-off as the mouth-sequence task: a
    // blink ending would otherwise restore the full-face image and overwrite
    // the mouth overlay). The user's most recent blink intent is read from
    // blink_desired_ at stop so a set_blink call issued during playback is
    // honoured.
    enum class TtsLipSyncShape : uint8_t {
        CLOSED = 0,
        HALF_RISING,   // closed -> open transition
        OPEN,
        HALF_FALLING,  // open -> closed transition
    };
    static constexpr int TTS_LIPSYNC_STEP_MS = 150;
    esp_timer_handle_t tts_lipsync_timer_ = nullptr;
    std::atomic<bool> tts_lipsync_active_{false};
    TtsLipSyncShape tts_lipsync_shape_ = TtsLipSyncShape::CLOSED;

    // Phase 7: Si12T head-touch sensing.
    // Polling every TOUCH_POLL_MS samples Output1 (CH1..CH3 -> 3 head zones).
    // Edge detection on the OR of the three zones produces TAP / STROKE
    // gestures based on hold duration:
    //   duration <  TAP_MAX_MS (400 ms)  -> TAP    -> face=surprised
    //   duration >= STROKE_MIN_MS (600 ms) -> STROKE -> face=embarrassed + servo wobble
    //   400 <= duration < 600 ms         -> treated as TAP (greyzone)
    // Reactions auto-revert to "idle" after REACTION_HOLD_MS (3 s). A
    // post-reaction COOLDOWN_MS lock-out prevents one head-pat from firing
    // a chain of events.
    enum class TouchEvent : uint8_t {
        IDLE = 0,
        TAP,
        STROKE,
    };
    static constexpr int TOUCH_POLL_MS    = 100;  // 100 Hz polling
    static constexpr int TAP_MAX_MS       = 400;
    static constexpr int STROKE_MIN_MS    = 400;  // was 600; lowered because
                                                  // finger-glide between zones
                                                  // and Si12T auto-recalibration
                                                  // inject brief "all-false"
                                                  // gaps that cut a real stroke
                                                  // short of 600 ms.
    static constexpr int REACTION_HOLD_MS = 3000;
    static constexpr int COOLDOWN_MS      = 800;  // post-reaction noise gate
    // With 2-sample debounce this gives ~200 ms confirm latency, fast enough
    // to catch a quick "pon" (~200 ms press) while still rejecting single-
    // sample jitter. Was 200 ms polling -> 400 ms confirm, which silently
    // dropped most short taps.
    static constexpr int SERVO_WOBBLE_STEP_MS = 350;  // was 200; SCS0009 needs
                                                       // ~125 ms to physically
                                                       // travel ±20°, plus the
                                                       // ACK round-trip + IFG.
                                                       // Tighter steps caused
                                                       // bus hangs.
    static constexpr int SERVO_WOBBLE_AMPLITUDE_DEG = 20;

    std::atomic<bool> touch_sensor_enabled_{true};
    std::unique_ptr<Si12T> si12t_;
    bool si12t_ok_ = false;
    esp_timer_handle_t touch_poll_timer_ = nullptr;
    esp_timer_handle_t touch_revert_timer_ = nullptr;
    // Spoken listen cue (CONFIG_STACKCHAN_LISTEN_VOICE_CUE): the mic opens
    // once the cue has finished playing.
    static constexpr int LISTEN_CUE_MAX_MS = 2500;
    esp_timer_handle_t listen_cue_timer_ = nullptr;
    std::atomic<bool> listen_cue_pending_{false};
    bool listen_cue_drained_ = false;
    uint64_t listen_cue_started_us_ = 0;

#if CONFIG_STACKCHAN_BODY_SENSORS
    // On-board IMU and proximity sensor (body_sensors.h). Polled from the
    // body_poll esp_timer; the latest readings are kept for the get_* tools.
    // 50 Hz so the knock detector sees the short jolt of a tap (the IMU
    // itself runs at 100 Hz).
    static constexpr int BODY_POLL_MS = 20;
    static constexpr int PROXIMITY_EVERY = 10;  // proximity every 200 ms
    // No knocks while speaking or just after: the speaker shakes the body.
    static constexpr uint32_t KNOCK_AFTER_SPEECH_MS = 1000;
    uint32_t last_speaking_ms_ = 0;
    std::unique_ptr<body_sensors::Bmi270> imu_;
    std::unique_ptr<body_sensors::Ltr553> prox_;
    body_events::MotionDetector motion_detector_;
    body_events::ProximityDetector proximity_detector_;
    esp_timer_handle_t body_poll_timer_ = nullptr;
    int body_poll_count_ = 0;
    portMUX_TYPE body_mux_ = portMUX_INITIALIZER_UNLOCKED;
    float body_accel_[3] = {};
    float body_gyro_[3] = {};
    int body_proximity_ = -1;
#endif

    // Touch detection state (single-thread access from the touch_poll_timer_
    // callback, which runs on the ESP_TIMER_TASK).
    bool touch_pressed_prev_ = false;          // last sample (debounced)
    bool touch_pressed_pending_ = false;       // candidate awaiting confirm
    int  touch_pending_count_ = 0;             // consecutive samples matching
    uint64_t touch_press_start_us_ = 0;        // when pressed_prev_ went true
    uint64_t cooldown_until_us_ = 0;           // ignore press until this ts

    // Last reported event for MCP get_touch_state.
    TouchEvent last_event_ = TouchEvent::IDLE;
    uint64_t   last_event_us_ = 0;
    bool       last_zone_snapshot_[3] = {false, false, false};
    uint8_t    last_output1_raw_ = 0;
    // Press-start snapshot. last_* fields above are overwritten every poll
    // tick, so by the time HandleTap / HandleStroke fires on the falling edge
    // they reflect the release state (zones=000 raw=0x00). press_start_*
    // captures the rising-edge state so the log can show what the sensor
    // actually saw when the touch began. Useful for distinguishing genuine
    // touches (CH1〜CH3 set) from false positives (e.g. CH4 noise, raw=0x00
    // with press judged via debounce, etc.).
    bool       press_start_zones_[3] = {false, false, false};
    uint8_t    press_start_output1_raw_ = 0;

    // Servo wobble sub-state. Keeps the previously-set angles untouched
    // before/after the wobble so that an external set_head_angles call is
    // not silently overwritten beyond the wobble window.
    std::atomic<int> servo_wobble_step_{0};       // 0..3 sequence index
    std::atomic<bool> servo_wobble_active_{false};

    // Shared motion state. This stays on the board singleton because boot-init
    // ReadPos restore / re-sync phases seed the same state before and after the
    // concrete MotionDriver is selected. Drivers only borrow these references.
    struct AxisMotion {
        int target_deg = 0;
        int start_deg = 0;
        int current_deg = 0;
        uint32_t move_start_ms = 0;
        // For the delegated driver: time the WritePos for this request
        // was successfully ACK'd by the servo (i.e. when the physical
        // motion actually began on the SCS0009's internal clock). May
        // be later than move_start_ms when the dispatch is delayed by
        // Tick wake or retry rounds. ApplyReadMoveResult's stuck-high
        // timeout is measured from dispatch_start_ms, not from staging,
        // so that degraded-bus latency does not cause premature force-
        // clear while the servo is genuinely still mid-motion. 0 means
        // "not yet dispatched"; ApplyReadMoveResult skips the
        // stuck-high check until dispatch_start_ms is populated by
        // FinishDispatch's write_ok branch. HostInterpolation path
        // does not consult this field.
        uint32_t dispatch_start_ms = 0;
        uint32_t move_duration_ms = 0;
        bool moving = false;
        // Monotonic counter incremented by ServoDelegatedMotionDriver::
        // StartMove on each dispatch stage. Used by Tick() to detect
        // when a newer StartMove has raced in between the snapshot at
        // the top of Tick() and the post-WritePos / post-ReadMove
        // commit (motion_mutex_ is dropped while the bus operation
        // runs). esp_timer_get_time()/1000 has 1 ms resolution and is
        // not unique enough on its own — two StartMove calls within
        // the same millisecond would collide on move_start_ms. The
        // HostInterpolation path does not consult this field; the
        // monotonic counter is owned by the delegated driver.
        uint64_t request_token = 0;
        // Set by the delegated driver when ApplyReadMoveResult's
        // 5-consecutive-failure force-clear fires: WritePos ACK
        // confirmed the servo received the command, but ReadMove
        // polling never observed completion. current_deg holds the
        // last optimistic commit (the requested target), but the
        // physical head may be mid-trajectory or at the wrong angle.
        // The next StartMove on this axis treats position_unknown as
        // a "force re-dispatch" signal (no-op skip is suppressed),
        // so a same-target retry surfaces the failure rather than
        // hiding it behind a stale-but-equal current_deg. The
        // HostInterpolation path does not consult this field.
        bool position_unknown = false;
    };
    class MotionDriver;
    // TODO: motion_mutex_/scs_bus_mutex_/servo_task_handle_ have no destroy path; board is singleton via DECLARE_BOARD.
    AxisMotion yaw_motion_;
    AxisMotion pitch_motion_;
    SemaphoreHandle_t motion_mutex_ = nullptr;     // protects AxisMotion fields
    SemaphoreHandle_t scs_bus_mutex_ = nullptr;    // serializes UART access (WritePos/ReadPos)
    std::unique_ptr<MotionDriver> motion_driver_;
    TaskHandle_t servo_task_handle_ = nullptr;
    uint32_t last_motion_end_ms_ = 0;              // ServoTask-private
    bool last_motion_end_valid_ = false;           // ServoTask-private
    std::atomic<bool> idle_timer_reset_pending_{false};
    enum class TorqueState : uint8_t {
        kEngaged = 0,
        kPartial = 1,
        kReleased = 2,
        kReleasing = 3,
        // Published when InternalSetServoTorque cannot confirm bus success
        // for both axes. Forward-progress invariant: the next motion / manual
        // call issues a real bus frame instead of short-circuiting. Mirrors
        // the WritePos retries-exhausted -> position_unknown convention for
        // the torque domain.
        kUncertain = 4,
    };
    std::atomic<TorqueState> torque_state_{TorqueState::kEngaged};
    std::atomic<uint32_t> torque_release_epoch_{0};
    // Per-axis commanded torque state is protected by scs_bus_mutex_;
    // torque_state_ publishes the derived cross-task summary.
    bool yaw_torque_enabled_ = true;               // protected by scs_bus_mutex_
    bool pitch_torque_enabled_ = true;             // protected by scs_bus_mutex_
    std::atomic<bool> boot_init_done_{false};
#if CONFIG_STACKCHAN_AUTO_TORQUE_RELEASE_ENABLED
    std::atomic<bool> auto_release_enabled_{true};
#else
    std::atomic<bool> auto_release_enabled_{false};
#endif
    static constexpr uint32_t MOTION_TICK_MS = 20;
    static constexpr uint32_t MOTION_DEFAULT_DURATION_MS = 600;
    // Speed-based motion API (Issue #129).
    // MIN_STEP_SAFE_SPEED_DPS prevents the raw-integer speed_dps escape hatch
    // from advancing less than one SCS0009 step per ServoTask tick. The
    // physical step is 300 deg / 1024 = 0.293 deg; at MOTION_TICK_MS=20 ms
    // this is 14.65 deg/s, rounded up to 15 deg/s for headroom. This shares
    // the same physical origin as BOOT_INIT_TARGET_DEG_PER_SEC (#121/#141),
    // but stays separate because the boot path carries its own duration-floor
    // semantics. See the Issue #129 -> #134 stepped-motion observation lineage.
    static constexpr int MIN_STEP_SAFE_SPEED_DPS = 15;
    // MIN_SMOOTH_SPEED_DPS is the on-device measured smoothness floor
    // (5 step/tick "transition out", measured 2026-05-15). Speeds below
    // this look textured on SCS0009 at MOTION_TICK_MS=20 ms; the firmware
    // permits sub-floor speeds (logged with ESP_LOGW) so callers like the
    // gateway "low" preset (30 dps) can deliver deliberately slow motion.
    static constexpr int MIN_SMOOTH_SPEED_DPS = 72;
    // MAX_SPEED_DPS is the SCS0009 datasheet reliability test working speed
    // (60 deg / 0.25 s = 240 deg/s, validated for >50k cycles at 1/2 rated load).
    static constexpr int MAX_SPEED_DPS = 240;
    // DEFAULT_SPEED_DPS is used when the caller passes speed_dps <= 0.
    // Matches the gateway "mid" preset.
    static constexpr int DEFAULT_SPEED_DPS = 120;
    static constexpr uint32_t MOTION_PER_WRITE_TIME_MS = 30;
    static constexpr uint32_t MOTION_POLL_INTERVAL_MS = 50;
    static constexpr uint32_t AUTO_TORQUE_RELEASE_MIN_MS = 500;
    static constexpr uint32_t AUTO_TORQUE_RELEASE_MAX_MS = 600000;
    static constexpr int kMaxReengageRetries = 3;
    static constexpr int kMaxManualReengageRetries = 3;
#ifdef CONFIG_STACKCHAN_AUTO_TORQUE_RELEASE_MS
    static constexpr uint32_t AUTO_TORQUE_RELEASE_DEFAULT_MS =
        CONFIG_STACKCHAN_AUTO_TORQUE_RELEASE_MS;
#else
    static constexpr uint32_t AUTO_TORQUE_RELEASE_DEFAULT_MS = 5000;
#endif
    std::atomic<uint32_t> auto_release_timeout_ms_{
        AUTO_TORQUE_RELEASE_DEFAULT_MS};

    // Issue #80 / #98: pitch is guarded by two complementary tiers.
    //
    // Tier 1 — Hard clamp [SAFE_PITCH_MIN, SAFE_PITCH_MAX]:
    //   The absolute mechanical safety net. Its only job is to prevent
    //   physical damage to the servo / gear / chassis. Values are silently
    //   clamped to this range at every servo-write boundary and an
    //   ESP_LOGW is emitted when clamping occurs.
    //   - Lower bound 0°: the mechanical end-stop on the M5Stack CoreS3 +
    //     SCS0009 hardware sits very close to pitch=-1° (validated on a
    //     real unit, PR #81). Driving below 0° presses the servo gear into
    //     the physical stopper and produces an audible click.
    //   - Upper bound (SAFE_PITCH_MAX): chosen 1° inside the validated
    //     mechanical upper limit. The M5Stack-documented servo features
    //     advertise "90-degree movement on the vertical axis", so the
    //     mechanical upper end-stop is expected near 90°; the precise
    //     value is established by real-device sweep (Issue #98 validation).
    //
    // Tier 2 — Recommended operating range [RECOMMENDED_PITCH_MIN,
    //          RECOMMENDED_PITCH_MAX]:
    //   The M5Stack-documented sweet spot for long-term servo reliability
    //   (https://docs.m5stack.com/en/StackChan, "Motion Angle Notice":
    //   "The movement angle of the StackChan Y-axis servo (vertical
    //   direction) is recommended to be controlled within 5 ~ 85°.
    //   Operating at extreme angles may cause servo stall and permanent
    //   damage.").
    //   Values inside the hard clamp but outside this range are accepted
    //   (they are not hardware-damaging on a single call), and an
    //   ESP_LOGI is emitted so callers / agents can notice the deviation
    //   without blocking the motion.
    //
    // Defense-in-depth: the hard clamp is enforced at every servo-write
    // boundary —
    //   1. PitchDegToPos() clamps its input (covers motion-task
    //      interpolation and any future caller that bypasses the MCP
    //      layer).
    //   2. The start-up restore from ReadPos clamps the recovered angle
    //      so a device booting with the head physically pushed past the
    //      safe range does not carry that out-of-range starting angle
    //      into motion interpolation.
    //   3. The set_head_angles MCP handler additionally clamps the
    //      request target so the original out-of-range value is logged.
#ifdef CONFIG_STACKCHAN_PITCH_MIN
    // Board-specific mechanical range (Stack-chan RT ver.: 50..90).
    static constexpr int SAFE_PITCH_MIN = CONFIG_STACKCHAN_PITCH_MIN;
    static constexpr int SAFE_PITCH_MAX = CONFIG_STACKCHAN_PITCH_MAX;
#else
    static constexpr int SAFE_PITCH_MIN = 0;
    static constexpr int SAFE_PITCH_MAX = 88;  // Issue #98: validated on real hardware
                                                // (M5Stack CoreS3 + SCS0009 ×2). On-device
                                                // sweep observed clean motion at pitch=85
                                                // and pitch=88 reached without end-stop, but
                                                // pitch=89 exhibited an audible sub-stall
                                                // ("ji-ji-" gear strain sound). Mirrors PR #81
                                                // lower-bound rationale: stay 1° inside the
                                                // observed servo-strain boundary.
#endif
#ifdef CONFIG_STACKCHAN_PITCH_MIN
    static constexpr int RECOMMENDED_PITCH_MIN = SAFE_PITCH_MIN;
    static constexpr int RECOMMENDED_PITCH_MAX = SAFE_PITCH_MAX;
#else
    static constexpr int RECOMMENDED_PITCH_MIN = 5;   // M5Stack official docs
    static constexpr int RECOMMENDED_PITCH_MAX = 85;  // M5Stack official docs
#endif

    // Issue #115: boot-time initialization target. Fall-safe neutral pose
    // well clear of both mechanical end-stops, in the centre of the
    // M5Stack-recommended 5..85° pitch range. Design follows the
    // goHome() pattern in m5stack/StackChan
    // (apps/app_setup/workers/servo.cpp:144) and the 1-second
    // positioning timing established in mongonta0716/stackchan-arduino
    // attachServos().
    //
    // Speed policy history (#121 Problem 2 -> #141 follow-ups):
    // - #121 Problem 2 originally raised BOOT_INIT_MOVE_MS from 1000 ms
    //   (the historical default that produced a startling "ブルンっ" boot
    //   motion) to 4000 ms (~11°/s on a 45° climb).
    // - Real-device verification then showed even 11°/s reads as
    //   perceptibly fast for the first boot-time servo motion, so the
    //   Phase 0 climb is pinned to an angular-speed cap of
    //   BOOT_INIT_TARGET_DEG_PER_SEC=15 deg/s. The WriteHeadAngles call
    //   sizes its duration from the actual yaw / pitch deltas so this
    //   cap holds on every axis. On the PMIC OFF/ON path Phase 0 stays
    //   a no-op of effect (the #138 safe-fallback seed makes start_deg
    //   == target_deg), so the BOOT_INIT_MOVE_MS budget simply elapses
    //   without WritePos movement.
    // The 100 ms post-settle vTaskDelay in InitializeServo() is
    // unchanged. The separate "unintended downward drop on power-on"
    // (#121 Problem 1) is addressed by the snap-suppress hold in
    // InitializeServo() Phase 1a (PR #137) and the ReadPos retry +
    // safe-fallback seed in this file (#138).
    //
    // Issue #138: promoted from local block scope inside
    // InitializeServo() to class-level static constexpr so the
    // safe-fallback branch in Phase 2 can seed
    // pitch_motion_.current_deg with BOOT_INIT_PITCH_DEG when the
    // pre-init ReadPos retries all fail. Without that seed, the
    // boot-init `WriteHeadAngles(0, 45, 4000)` interpolation would
    // start from the struct-default `current_deg=0` (== pos=620 at
    // deg=0, the lower mechanical end-stop) and walk WritePos calls
    // upward through end-stop-adjacent positions before reaching the
    // target, risking servo bus degradation if the SCS0009 wakes up
    // mid-sequence.
    static constexpr int BOOT_INIT_YAW_DEG = 0;
#ifdef CONFIG_STACKCHAN_REST_PITCH_DEG
    static constexpr int BOOT_INIT_PITCH_DEG = CONFIG_STACKCHAN_REST_PITCH_DEG;
#else
    static constexpr int BOOT_INIT_PITCH_DEG = 45;
#endif
    // BOOT_INIT_MOVE_MS=3000: minimum duration of the Phase 0 climb.
    // Used as a floor so the boot-init `WriteHeadAngles(0, 45, X)`
    // always elapses at least this long — required on the PMIC OFF/ON
    // path where the #138 safe-fallback seed makes Phase 0 a no-op of
    // effect, and the BOOT_INIT_MOVE_MS budget instead serves to span
    // the SCS0009 wake-up latency window so the post-init ReadPos
    // (Phase 0') lands well past it. The actual Phase 0 duration is
    // computed at call time from the current_deg → BOOT_INIT_*
    // deltas at BOOT_INIT_TARGET_DEG_PER_SEC=15 deg/s, then floored at this
    // constant; e.g. on the ESP32-only reset path with a yaw-90° prior
    // set-point, Phase 0 needs 6000 ms to honour the speed cap while a
    // yaw-0 prior is rounded up to this 3000 ms floor. This is the
    // no-stutter Smooth lower bound established under #121 Problem 2
    // (Issue #121 / PR #125 history: 1000 -> 4000 was a partial step
    // toward this; on-device feedback under #141 verification confirmed
    // 15 deg/s is the speed at which the ServoTask MOTION_TICK_MS=20 ms
    // interpolation stops being perceptible as individual position
    // jumps without sliding into a startling regime). Boot-time budget
    // is intentionally not optimised: operator safety and avoiding
    // mechanical stress take precedence over shaving seconds off the
    // initialization duration.
    //
    // On the PMIC OFF/ON path Phase 0 stays a no-op of effect
    // because the #138 safe-fallback seeds current_deg to
    // BOOT_INIT_PITCH_DEG, making start_deg == target_deg; the
    // BOOT_INIT_MOVE_MS budget then simply elapses without WritePos
    // movement.
    static constexpr uint32_t BOOT_INIT_MOVE_MS = 3000;
    // Single-source Phase 0 speed cap. 15 deg/s is the no-stutter Smooth
    // lower bound (#121 Problem 2 + #141 verification).
    static constexpr int BOOT_INIT_TARGET_DEG_PER_SEC = 15;

    static int YawDegToPos(int deg) {
        int pos = 460 + deg * 16 / 5;
        if (pos < 0) pos = 0;
        if (pos > 1000) pos = 1000;
        return pos;
    }

    static int PitchDegToPos(int deg) {
        // Issue #80: defense-in-depth — clamp at the servo-write boundary so
        // motion-task interpolation and any other future caller cannot
        // bypass the input-layer clamp.
        if (deg < SAFE_PITCH_MIN) deg = SAFE_PITCH_MIN;
        if (deg > SAFE_PITCH_MAX) deg = SAFE_PITCH_MAX;
        int pos = 620 + deg * 16 / 5;
        if (pos < 0) pos = 0;
        if (pos > 1000) pos = 1000;
        return pos;
    }

    static uint16_t clamp_u16(uint32_t v) {
        if (v > std::numeric_limits<uint16_t>::max()) {
            return std::numeric_limits<uint16_t>::max();
        }
        return static_cast<uint16_t>(v);
    }

    // Map the StartMove duration contract to spring options that approximate
    // the requested timing. This is the stackchan-mcp side of
    // m5stack/StackChan's map_speed_to_spring_options(speed): shorter
    // duration -> higher stiffness/damping, longer duration -> lower
    // stiffness/damping, with critical damping for no overshoot.
    static smooth_ui_toolkit::SpringOptions_t MapDurationToSpringOptions(
        uint32_t duration_ms) {
        if (duration_ms == 0) {
            duration_ms = 1;
        }

        float speed_f = 500.0f *
            (static_cast<float>(MOTION_DEFAULT_DURATION_MS) /
             static_cast<float>(duration_ms));
        if (speed_f < 1.0f) speed_f = 1.0f;
        if (speed_f > 1000.0f) speed_f = 1000.0f;
        int speed = static_cast<int>(speed_f);

        constexpr float kMin = 10.0f;
        constexpr float kMax = 650.0f;
        constexpr float kMass = 1.0f;
        float normalized_speed = static_cast<float>(speed) / 1000.0f;
        float stiffness =
            kMin + (normalized_speed * normalized_speed) * (kMax - kMin);
        float damping = 2.0f * std::sqrt(kMass * stiffness);

        smooth_ui_toolkit::SpringOptions_t options;
        options.stiffness = stiffness;
        options.damping = damping;
        options.mass = kMass;
        options.velocity = 0.0f;
        options.restDelta = speed > 800 ? 0.5f : 0.1f;
        options.restSpeed = speed > 800 ? 0.5f : 0.1f;
        options.duration = 0.0f;
        options.bounce = 0.0f;
        options.visualDuration = 0.0f;
        return options;
    }

    enum class ReleaseReason : uint8_t {
        kManual = 0,
        kAutoIdle,
        kReengagement,
    };

    static const char* ReleaseReasonName(ReleaseReason reason) {
        switch (reason) {
            case ReleaseReason::kManual:
                return "manual";
            case ReleaseReason::kAutoIdle:
                return "auto_idle";
            case ReleaseReason::kReengagement:
                return "reengagement";
        }
        return "unknown";
    }

    // Caller must hold scs_bus_mutex_.
    void PublishTorqueState() {
        TorqueState state;
        if (yaw_torque_enabled_ && pitch_torque_enabled_) {
            state = TorqueState::kEngaged;
        } else if (!yaw_torque_enabled_ && !pitch_torque_enabled_) {
            state = TorqueState::kReleased;
        } else {
            state = TorqueState::kPartial;
        }
        TorqueState old_state =
            torque_state_.load(std::memory_order_acquire);
        if (state == TorqueState::kEngaged &&
            old_state != TorqueState::kEngaged) {
            // Reset the ServoTask-owned idle window even when OFF->ON
            // happens between ServoTask ticks.
            idle_timer_reset_pending_.store(true,
                                            std::memory_order_release);
        }
        torque_state_.store(state, std::memory_order_release);
    }

    // Marks a fully-OFF transition while the bus write is still pending.
    // Returns this call's release epoch so auto-idle rollback can detect
    // another release publisher that interleaved after it.
    uint32_t MarkReleasing() {
        uint32_t epoch =
            torque_release_epoch_.fetch_add(1, std::memory_order_acq_rel) +
            1;
        torque_state_.store(TorqueState::kReleasing,
                            std::memory_order_release);
        return epoch;
    }

    // Block until torque_state_ leaves kReleasing or the elapsed-time
    // budget expires. Caller must NOT hold motion_mutex_ or scs_bus_mutex_.
    // Uses esp_timer_get_time() so the budget is honored at real time
    // regardless of CONFIG_FREERTOS_HZ.
    //
    // Returns true if the state is no longer kReleasing (proceed safely),
    // false if the wait budget was exhausted while still kReleasing
    // (caller decides how to handle: either skip with ESP_LOGW or defer to
    // its own bounded retry).
    bool WaitForKReleasingToClear() {
        constexpr uint32_t kMaxKReleasingWaitMs = 200;
        constexpr uint32_t kKReleasingPollIntervalMs = 5;
        const TickType_t kDelayTicks =
            std::max<TickType_t>(1,
                                 pdMS_TO_TICKS(kKReleasingPollIntervalMs));
        const uint32_t start_us =
            static_cast<uint32_t>(esp_timer_get_time());
        auto state = torque_state_.load(std::memory_order_acquire);
        while (state == TorqueState::kReleasing) {
            const uint32_t elapsed_ms =
                (static_cast<uint32_t>(esp_timer_get_time()) - start_us) /
                1000;
            if (elapsed_ms >= kMaxKReleasingWaitMs) {
                return false;
            }
            vTaskDelay(kDelayTicks);
            state = torque_state_.load(std::memory_order_acquire);
        }
        return true;
    }

    struct ServoTorqueResult {
        // -1 means "no bus frame was issued for this axis". In every
        // short-circuit path (idempotent_short_circuit or wait_exhausted)
        // the function returns before any EnableTorque() call, so both
        // bus-return fields keep this -1 default (Issue #171).
        int yaw_bus_return = -1;
        int pitch_bus_return = -1;
        bool yaw_ok = false;
        bool pitch_ok = false;
        // Issue #171: the old single `short_circuited` flag was overloaded
        // (set both for idempotent no-ops AND for wait-budget exhaustion),
        // so callers could not distinguish degraded-bus wait-exhaustion from
        // a legitimate no-op success. These two flags are orthogonal and
        // mutually exclusive: at most one is ever true.
        //   * idempotent_short_circuit: returned without a bus frame because
        //     the per-axis state already matched the request (success no-op).
        //   * wait_exhausted: returned without a bus frame because
        //     WaitForKReleasingToClear() hit its budget while still
        //     kReleasing (failure: the requested transition did not happen).
        bool idempotent_short_circuit = false;
        bool wait_exhausted = false;
    };

    class MotionDriver {
    public:
        virtual ~MotionDriver() = default;

        // Non-blocking. Both axes are dispatched within a single call.
        // WriteHeadAngles holds motion_mutex_ while calling this method; Tick()
        // and getters take the mutex internally for their own state access.
        virtual void StartMove(float yaw_deg, float pitch_deg,
                               uint32_t duration_ms,
                               bool prefer_linear = false) = 0;

        // Last-known committed angle for each axis.
        virtual float GetYawDeg() const = 0;
        virtual float GetPitchDeg() const = 0;

        // True iff at least one axis is currently in motion.
        virtual bool IsMoving() const = 0;

        // Called from ServoTask body at a driver-dependent cadence.
        virtual void Tick() = 0;

        // Optional hooks for drivers that need setup or shutdown.
        virtual bool Initialize() { return true; }
        virtual void Shutdown() {}

        // Invalidate the freshness token for one axis. Used by board-
        // level code that mutates AxisMotion fields directly outside
        // StartMove (currently InitializeServo's Phase 0' post-init
        // ReadPos re-sync, and the set_servo_torque MCP tool's
        // disable path). Caller must hold motion_mutex_.
        //
        // HostInterpolationMotionDriver: bumps the per-axis
        // request_token, defeating any post-bus freshness check from a
        // Tick() snapshot taken before the external mutation.
        //
        // ServoDelegatedMotionDriver: bumps the per-axis request_token
        // AND clears the corresponding AxisServo's per-axis private
        // cancellation state (pending_dispatch_, dispatch_failures_,
        // readmove_failures_), atomically with the caller's motion_mutex_
        // hold.
        //
        // Drivers without a token-based freshness guard treat this as
        // a no-op (default implementation). Argument is SERVO_YAW_ID
        // or SERVO_PITCH_ID; unknown values are ignored.
        virtual void InvalidateAxisToken(int /*axis_id*/) {}
    };

    class HostInterpolationMotionDriver final : public MotionDriver {
    public:
        HostInterpolationMotionDriver(ScsBus& scs_bus,
                                      SemaphoreHandle_t& scs_bus_mutex,
                                      SemaphoreHandle_t& motion_mutex,
                                      AxisMotion& yaw_motion,
                                      AxisMotion& pitch_motion)
            : scs_bus_(scs_bus),
              scs_bus_mutex_(scs_bus_mutex),
              motion_mutex_(motion_mutex),
              yaw_motion_(yaw_motion),
              pitch_motion_(pitch_motion),
              next_request_token_(0) {
            yaw_anim_.teleport(static_cast<float>(yaw_motion_.current_deg));
            pitch_anim_.teleport(static_cast<float>(pitch_motion_.current_deg));
        }

        void StartMove(float yaw_deg, float pitch_deg,
                       uint32_t duration_ms,
                       bool prefer_linear) override {
            uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
            int yaw = static_cast<int>(yaw_deg);
            int pitch = static_cast<int>(pitch_deg);

            yaw_motion_.request_token = ++next_request_token_;
            yaw_motion_.target_deg = yaw;
            yaw_motion_.start_deg = yaw_motion_.current_deg;
            yaw_motion_.move_start_ms = now_ms;
            yaw_motion_.move_duration_ms = duration_ms;
            yaw_motion_.moving = (yaw_motion_.target_deg != yaw_motion_.current_deg);
            yaw_linear_mode_ = prefer_linear;

            pitch_motion_.request_token = ++next_request_token_;
            pitch_motion_.target_deg = pitch;
            pitch_motion_.start_deg = pitch_motion_.current_deg;
            pitch_motion_.move_start_ms = now_ms;
            pitch_motion_.move_duration_ms = duration_ms;
            pitch_motion_.moving = (pitch_motion_.target_deg != pitch_motion_.current_deg);
            pitch_linear_mode_ = prefer_linear;
            if (prefer_linear) {
                yaw_anim_.teleport(static_cast<float>(yaw_motion_.current_deg));
                yaw_snap_on_rest_ = false;
                pitch_anim_.teleport(static_cast<float>(pitch_motion_.current_deg));
                pitch_snap_on_rest_ = false;
                return;
            }

            smooth_ui_toolkit::SpringOptions_t spring_options =
                MapDurationToSpringOptions(duration_ms);
            StartAxisSpring(yaw_anim_, yaw_snap_on_rest_,
                            yaw_motion_.current_deg, yaw,
                            yaw_motion_.moving, spring_options);
            StartAxisSpring(pitch_anim_, pitch_snap_on_rest_,
                            pitch_motion_.current_deg, pitch,
                            pitch_motion_.moving, spring_options);
        }

        float GetYawDeg() const override {
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            int yaw = yaw_motion_.current_deg;
            xSemaphoreGive(motion_mutex_);
            return static_cast<float>(yaw);
        }

        float GetPitchDeg() const override {
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            int pitch = pitch_motion_.current_deg;
            xSemaphoreGive(motion_mutex_);
            return static_cast<float>(pitch);
        }

        bool IsMoving() const override {
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            bool moving = yaw_motion_.moving || pitch_motion_.moving;
            xSemaphoreGive(motion_mutex_);
            return moving;
        }

        void Tick() override {
            constexpr TickType_t kInterFrameGap = pdMS_TO_TICKS(10);

            vTaskDelay(pdMS_TO_TICKS(MOTION_TICK_MS));

            AxisMotion yaw_local;
            AxisMotion pitch_local;
            int new_yaw_current;
            bool new_yaw_moving;
            int new_pitch_current;
            bool new_pitch_moving;
            bool yaw_linear_mode;
            bool pitch_linear_mode;
            uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
            float dt_s;
            // Spring mode follows real elapsed time so bus ACK latency or
            // mutex contention does not stretch animation time indefinitely.
            // Clamp deep preemption to avoid a single large lurch.
            if (last_tick_us_ == 0) {
                dt_s = static_cast<float>(MOTION_TICK_MS) / 1000.0f;
            } else {
                dt_s = static_cast<float>(now_us - last_tick_us_) / 1000000.0f;
                if (dt_s > 0.1f) {
                    dt_s = 0.1f;
                }
            }
            last_tick_us_ = now_us;
            uint32_t now_ms = static_cast<uint32_t>(now_us / 1000);

            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            yaw_local = yaw_motion_;
            pitch_local = pitch_motion_;
            yaw_linear_mode = yaw_linear_mode_;
            pitch_linear_mode = pitch_linear_mode_;
            if (!yaw_local.moving && !pitch_local.moving) {
                xSemaphoreGive(motion_mutex_);
                return;
            }
            new_yaw_current = yaw_local.current_deg;
            new_yaw_moving = yaw_local.moving;
            if (yaw_linear_mode) {
                AdvanceAxisLinear(yaw_local, now_ms,
                                  new_yaw_current, new_yaw_moving);
            } else {
                AdvanceAxisSpring(yaw_local, yaw_anim_, yaw_snap_on_rest_,
                                  dt_s, new_yaw_current, new_yaw_moving);
            }
            new_pitch_current = pitch_local.current_deg;
            new_pitch_moving = pitch_local.moving;
            if (pitch_linear_mode) {
                AdvanceAxisLinear(pitch_local, now_ms,
                                  new_pitch_current, new_pitch_moving);
            } else {
                AdvanceAxisSpring(pitch_local, pitch_anim_, pitch_snap_on_rest_,
                                  dt_s, new_pitch_current, new_pitch_moving);
            }
            xSemaphoreGive(motion_mutex_);

            // Known carve-out (#161): motion_mutex_ is released here
            // and re-acquired after the WritePos block. If StartMove
            // or InvalidateAxisToken (Phase 0' / torque disable) fires
            // inside this release window, the WritePos calls below
            // still send a stale interpolation step on the bus. The
            // post-bus request_token guard below then correctly skips
            // the current_deg / moving commit, but the physical
            // intermediate position has already been issued. The
            // pre-PR move_start_ms guard had the same surface; this PR
            // does not regress that behavior. Closing the pre-bus gate
            // is tracked separately under #161.
            xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
            if (yaw_local.moving) {
                int yaw_pos = YawDegToPos(new_yaw_current);
                int r = scs_bus_.WritePos(SERVO_YAW_ID, yaw_pos, MOTION_PER_WRITE_TIME_MS, 0);
                if (!ServoWritePosOk(r)) {
                    ESP_LOGW(TAG, "Motion yaw WritePos failed: r=%d (deg=%d, pos=%d)",
                             r, new_yaw_current, yaw_pos);
                }
            }
            vTaskDelay(kInterFrameGap);
            if (pitch_local.moving) {
                int pitch_pos = PitchDegToPos(new_pitch_current);
                int r = scs_bus_.WritePos(SERVO_PITCH_ID, pitch_pos, MOTION_PER_WRITE_TIME_MS, 0);
                if (!ServoWritePosOk(r)) {
                    ESP_LOGW(TAG, "Motion pitch WritePos failed: r=%d (deg=%d, pos=%d)",
                             r, new_pitch_current, pitch_pos);
                }
            }
            xSemaphoreGive(scs_bus_mutex_);

            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            if (yaw_motion_.request_token == yaw_local.request_token) {
                yaw_motion_.current_deg = new_yaw_current;
            }
            if (!new_yaw_moving && yaw_motion_.target_deg == yaw_local.target_deg
                && yaw_motion_.request_token == yaw_local.request_token) {
                yaw_motion_.moving = false;
            }
            if (pitch_motion_.request_token == pitch_local.request_token) {
                pitch_motion_.current_deg = new_pitch_current;
            }
            if (!new_pitch_moving && pitch_motion_.target_deg == pitch_local.target_deg
                && pitch_motion_.request_token == pitch_local.request_token) {
                pitch_motion_.moving = false;
            }
            xSemaphoreGive(motion_mutex_);
        }

        // Bump the request token for the specified axis. Used by
        // InitializeServo's Phase 0' re-sync so a Tick() snapshot
        // taken before the re-sync no longer passes the post-bus
        // freshness guard (which would otherwise overwrite the just-
        // re-synced current_deg / moving state). Caller must hold
        // motion_mutex_; this method does not take it.
        void InvalidateAxisToken(int axis_id) override {
            if (axis_id == SERVO_YAW_ID) {
                yaw_motion_.request_token = ++next_request_token_;
            } else if (axis_id == SERVO_PITCH_ID) {
                pitch_motion_.request_token = ++next_request_token_;
            }
        }

    private:
        static void StartAxisSpring(
            smooth_ui_toolkit::AnimateValue& axis_anim,
            bool& snap_on_rest,
            int current_deg,
            int target_deg,
            bool moving,
            const smooth_ui_toolkit::SpringOptions_t& spring_options) {
            if (!moving) {
                axis_anim.teleport(static_cast<float>(current_deg));
                snap_on_rest = false;
                return;
            }

            axis_anim.springOptions() = spring_options;
            axis_anim.teleport(static_cast<float>(current_deg));
            axis_anim = static_cast<float>(target_deg);
            snap_on_rest = true;
        }

        static void AdvanceAxisSpring(
            const AxisMotion& axis_local,
            smooth_ui_toolkit::AnimateValue& axis_anim,
            bool& snap_on_rest,
            float dt_s,
            int& new_current_deg,
            bool& new_moving) {
            if (!axis_local.moving) {
                return;
            }

            axis_anim.updateWithDelta(dt_s);
            new_current_deg = static_cast<int>(axis_anim.directValue());
            if (axis_anim.done()) {
                new_moving = false;
                if (snap_on_rest) {
                    new_current_deg = static_cast<int>(axis_anim.end);
                    snap_on_rest = false;
                }
            }
        }

        static void AdvanceAxisLinear(
            const AxisMotion& axis_local,
            uint32_t now_ms,
            int& new_current_deg,
            bool& new_moving) {
            if (!axis_local.moving) {
                return;
            }

            // Linear mode intentionally stays wall-clock based, matching the
            // pre-spring interpolation path used for boot-init slow climbs;
            // spring mode uses the real elapsed Tick delta instead.
            uint32_t elapsed = now_ms - axis_local.move_start_ms;
            if (axis_local.move_duration_ms == 0 ||
                elapsed >= axis_local.move_duration_ms) {
                new_current_deg = axis_local.target_deg;
                new_moving = false;
            } else {
                int delta = axis_local.target_deg - axis_local.start_deg;
                new_current_deg = axis_local.start_deg +
                    static_cast<int>(
                        static_cast<int64_t>(delta) * elapsed /
                        axis_local.move_duration_ms);
            }
        }

        ScsBus& scs_bus_;
        SemaphoreHandle_t& scs_bus_mutex_;
        SemaphoreHandle_t& motion_mutex_;
        AxisMotion& yaw_motion_;
        AxisMotion& pitch_motion_;
        // Monotonically increasing request id. Each StartMove increments
        // this and writes the new value into yaw_motion_.request_token
        // and pitch_motion_.request_token. Tick() snapshots both fields
        // with the rest of AxisMotion and uses request_token equality
        // (rather than move_start_ms, which only has ms resolution) to
        // detect whether a snapshot is still the live request.
        // InvalidateAxisToken() also bumps this counter for board-level
        // direct AxisMotion resets that do not go through StartMove
        // (currently InitializeServo's Phase 0' post-init ReadPos
        // re-sync and the set_servo_torque disable path); without that
        // bump, a Tick() snapshot taken before such a reset would pass
        // the post-bus freshness guard and overwrite the just-reset
        // state. motion_mutex_ guards this counter. The pre-bus
        // stale-WritePos race, where an external reset lands after the
        // snapshot but before the bus frame, is tracked separately
        // under #161.
        uint64_t next_request_token_ = 0;
        smooth_ui_toolkit::AnimateValue yaw_anim_;
        smooth_ui_toolkit::AnimateValue pitch_anim_;
        bool yaw_snap_on_rest_ = false;
        bool pitch_snap_on_rest_ = false;
        bool yaw_linear_mode_ = false;
        bool pitch_linear_mode_ = false;
        uint64_t last_tick_us_ = 0;
    };

    class ServoDelegatedMotionDriver final : public MotionDriver {
    public:
        ServoDelegatedMotionDriver(ScsBus& scs_bus,
                                   SemaphoreHandle_t& scs_bus_mutex,
                                   SemaphoreHandle_t& motion_mutex,
                                   AxisMotion& yaw_motion,
                                   AxisMotion& pitch_motion)
            : motion_mutex_(motion_mutex),
              yaw_motion_(yaw_motion),
              pitch_motion_(pitch_motion),
              next_request_token_(0),
              yaw_axis_(SERVO_YAW_ID, YawDegToPos, "yaw", yaw_motion,
                        scs_bus, scs_bus_mutex, motion_mutex,
                        next_request_token_,
                        /* post_dispatch_quiet_gap_ms = */ 10),
              pitch_axis_(SERVO_PITCH_ID, PitchDegToPos, "pitch",
                          pitch_motion, scs_bus, scs_bus_mutex,
                          motion_mutex, next_request_token_,
                          /* post_dispatch_quiet_gap_ms = */ 0) {}

        // StartMove only mutates AxisMotion state (under the caller's
        // motion_mutex_ per the MotionDriver::StartMove contract) and marks
        // each non-noop axis as having a pending dispatch. The actual WritePos
        // is performed by Tick() on the servo_motion task, so callers running
        // on timer tasks (e.g. TouchPollCb -> StartServoWobble) never block
        // on UART I/O. This mirrors HostInterpolationMotionDriver's
        // "StartMove writes state; Tick drives the bus" split and keeps
        // timer-task latency bounded.
        void StartMove(float yaw_deg, float pitch_deg,
                       uint32_t duration_ms,
                       bool prefer_linear) override {
            // The delegated path is already duration-bounded by the SCS0009
            // internal interpolation time argument; there is no host-side
            // profile to switch.
            (void)prefer_linear;
            uint16_t clamped = clamp_u16(duration_ms);
            if (duration_ms > clamped) {
                static bool duration_overflow_warned = false;
                if (!duration_overflow_warned) {
                    ESP_LOGW(TAG,
                             "Servo-delegated motion duration overflow: axis=yaw/pitch requested_ms=%u clamped_ms=%u",
                             (unsigned)duration_ms, (unsigned)clamped);
                    duration_overflow_warned = true;
                } else {
                    ESP_LOGD(TAG,
                             "Servo-delegated motion duration overflow: axis=yaw/pitch requested_ms=%u clamped_ms=%u",
                             (unsigned)duration_ms, (unsigned)clamped);
                }
            }

            // Per-axis no-op detection: when the axis is idle AND the request
            // matches the last-known current_deg AND the position is not
            // marked unknown, skip staging a dispatch. Issuing WritePos in
            // that case would start a delegated motion toward host-side
            // current_deg, which may diverge from the physical position
            // (e.g. after a boot ReadPos-failure path where current_deg was
            // seeded to BOOT_INIT_* via the safe fallback but the head sits
            // elsewhere). HostInterpolation path keeps its always-WritePos
            // behaviour for backward compatibility.
            //
            // position_unknown is the recovery signal from a prior ReadMove
            // force-clear: current_deg holds the requested target but
            // physical completion was never confirmed, so we MUST re-dispatch
            // (even when target == current_deg) to surface a persistent
            // failure rather than silently treat the axis as at-target.
            //
            // motion_mutex_ is held by the WriteHeadAngles caller per the
            // MotionDriver::StartMove contract (declared at the base class).
            // Taking it again here would deadlock the non-recursive FreeRTOS
            // semaphore — Stage() reads its AxisMotion directly.
            yaw_axis_.Stage(static_cast<int>(yaw_deg), clamped);
            pitch_axis_.Stage(static_cast<int>(pitch_deg), clamped);
        }

        float GetYawDeg() const override {
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            int yaw = yaw_motion_.current_deg;
            xSemaphoreGive(motion_mutex_);
            return static_cast<float>(yaw);
        }

        float GetPitchDeg() const override {
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            int pitch = pitch_motion_.current_deg;
            xSemaphoreGive(motion_mutex_);
            return static_cast<float>(pitch);
        }

        bool IsMoving() const override {
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            bool moving = yaw_motion_.moving || pitch_motion_.moving;
            xSemaphoreGive(motion_mutex_);
            return moving;
        }

        void Tick() override {
            vTaskDelay(pdMS_TO_TICKS(MOTION_POLL_INTERVAL_MS));

            // Per-axis update. Post-bus-frame quiet period is held INSIDE
            // each axis's Dispatch() (WritePos) and PollReadMove()
            // (ReadMove) atomically with the bus frame itself (no release
            // / reacquire window where concurrent MCP callers could inject
            // bus frames). yaw is configured with 10 ms via
            // post_dispatch_quiet_gap_ms_ (the member name pre-dates the
            // post-ReadMove path but the value applies symmetrically to
            // both frame types); pitch is configured with 0 ms matching
            // the PR #146 empirical model.
            //
            // Inter-axis quiet period coverage:
            // - yaw Dispatch tick (WritePos): in-Dispatch 10 ms hold
            //   provides inter-axis spacing before pitch_axis_.Update().
            // - yaw PollReadMove tick (ReadMove): in-PollReadMove 10 ms
            //   hold provides the same inter-axis spacing — prevents the
            //   ReadMove -> WritePos 0 ms inter-frame sequence on the
            //   shared SCS bus that Phase 2's per-axis grain would
            //   otherwise expose (PR #146 had no such ordering because
            //   dispatch and poll Tick phases were mutually exclusive).
            // - yaw no-op tick: no yaw bus frame, no quiet period needed;
            //   pitch_axis_.Update() runs immediately.
            //
            // Two-axis simultaneous dispatch (the necessary side of the
            // PR #146 E2 / E4-cumulative hang trigger) remains
            // structurally eliminated: yaw and pitch each take
            // scs_bus_mutex_ in their own separate short-hold critical
            // section inside Update(), and pitch_axis_.Update() runs
            // only after yaw_axis_.Update() returns (i.e. after yaw's
            // scs_bus_mutex_ hold has been released).
            //
            // No inter-axis vTaskDelay at this wrapper level: every yaw
            // bus-frame-emitting branch already provides the 10 ms
            // wall-clock spacing inside its in-Method hold. The 10 ms
            // inter-frame budget also remains in the unchanged
            // HostInterpolationMotionDriver::Tick path.
            yaw_axis_.Update();
            pitch_axis_.Update();
        }

        // Caller holds motion_mutex_. Bumps next_request_token_ for the
        // specified axis (mirrors HostInterpolationMotionDriver's
        // implementation) AND clears the per-AxisServo private
        // cancellation state via OnExternalReset(), so that a Stage()
        // call that preceded the external mutation does not leak a stale
        // WritePos onto the bus from the next Update() tick.
        //
        // Used by InitializeServo's Phase 0' post-init ReadPos re-sync
        // and by the set_servo_torque MCP tool's disable path.
        //
        // Closing this cancellation boundary required a paired AxisMotion
        // (visible) + AxisServo (driver-private) atomic reset: bumping
        // the visible request_token alone (the default no-op fallback
        // this driver used to inherit) was insufficient because
        // pending_dispatch_ / dispatch_failures_ / readmove_failures_
        // survived a Phase 0' direct AxisMotion mutation and the next
        // Update() tick would re-issue a WritePos for the stale staged
        // target. Issue #160 tracks the design discussion and the
        // adversarial review that converged on this Option A design.
        //
        // Scope note: this closes the post-Update / next-tick race only.
        // A snapshot taken by Update() BEFORE the external reset still
        // carries a local `dispatch = true` boolean (Update()'s local
        // variable, not the member field) that survives this member-
        // state clear. The in-flight Dispatch() then passes the
        // request_token freshness gate at the pre-WritePos check (which
        // now sees a bumped token) and skips the WritePos itself, so
        // the bus is not touched with a stale frame — but the call
        // still acquires scs_bus_mutex_ once before returning. The
        // pre-bus stale-command race (Issue #161) is the remaining
        // cancellation-boundary layer and is intentionally NOT closed
        // by this PR.
        void InvalidateAxisToken(int axis_id) override {
            if (axis_id == SERVO_YAW_ID) {
                yaw_motion_.request_token = ++next_request_token_;
                yaw_axis_.OnExternalReset();
            } else if (axis_id == SERVO_PITCH_ID) {
                pitch_motion_.request_token = ++next_request_token_;
                pitch_axis_.OnExternalReset();
            }
        }

    private:
        static constexpr int kReadMoveFailureLimit = 5;
        static constexpr int kDispatchFailureLimit = 5;
        // Settle margin past move_duration_ms before treating a
        // stuck-high ReadMove (servo returns 1 forever after the
        // requested completion) as a failure. Without this bound,
        // a degraded servo / register path would keep moving=true
        // indefinitely; wobble would never advance and same-target
        // recovery would never run.
        static constexpr uint32_t kReadMoveStuckMarginMs = 1000;
        // Margin below move_duration_ms in which an early ReadMove==0
        // is allowed as a genuine completion (the SCS0009's internal
        // interpolation can finish slightly early). A ReadMove==0
        // arriving further before the commanded completion is
        // implausible and likely a stuck-low / false-zero status read
        // from a degraded register path; treat it as suspicious and
        // mark position_unknown so the next StartMove forces a fresh
        // dispatch instead of trusting the stale optimistic commit.
        static constexpr uint32_t kReadMoveEarlyMarginMs = 200;

        class AxisServo {
            // Lock-order audit for Update():
            // - Snapshot: motion_mutex_ only.
            // - Dispatch freshness gate: scs_bus_mutex_ -> motion_mutex_;
            //   motion_mutex_ is released before WritePos.
            // - Dispatch commit: motion_mutex_ only after scs_bus_mutex_ is
            //   released.
            // - ReadMove poll: scs_bus_mutex_ only, then motion_mutex_ only
            //   for commit. No path takes motion_mutex_ before scs_bus_mutex_.

        public:
            AxisServo(uint8_t servo_id, int (*deg_to_pos)(int),
                      const char* axis_name, AxisMotion& motion,
                      ScsBus& scs_bus, SemaphoreHandle_t& scs_bus_mutex,
                      SemaphoreHandle_t& motion_mutex,
                      uint64_t& next_request_token,
                      uint32_t post_dispatch_quiet_gap_ms)
                : servo_id_(servo_id),
                  deg_to_pos_(deg_to_pos),
                  axis_name_(axis_name),
                  motion_(motion),
                  scs_bus_(scs_bus),
                  scs_bus_mutex_(scs_bus_mutex),
                  motion_mutex_(motion_mutex),
                  next_request_token_(next_request_token),
                  post_dispatch_quiet_gap_ms_(post_dispatch_quiet_gap_ms) {}

            void Stage(int target_deg, uint16_t duration_ms) {
                // motion_mutex_ is held by the WriteHeadAngles caller per
                // the MotionDriver::StartMove contract. Taking it again here
                // would deadlock the non-recursive FreeRTOS semaphore.
                bool noop =
                    !motion_.moving && !motion_.position_unknown &&
                    target_deg == motion_.current_deg;
                if (noop) {
                    return;
                }

                uint32_t now_ms =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000);
                motion_.start_deg = motion_.current_deg;
                motion_.target_deg = target_deg;
                motion_.move_start_ms = now_ms;
                // dispatch_start_ms stays 0 until FinishDispatch confirms
                // a WritePos ACK. ApplyReadMoveResult's stuck-high timeout
                // skips the check while dispatch_start_ms is 0, so retry
                // latency does not eat into the servo-internal duration
                // budget.
                motion_.dispatch_start_ms = 0;
                motion_.move_duration_ms = duration_ms;
                motion_.moving = true;
                motion_.request_token = ++next_request_token_;
                // NOTE: position_unknown is NOT cleared here. It is cleared
                // by FinishDispatch only after a successful WritePos ACK.
                // Clearing it on stage would let dispatch retry exhaustion
                // leave position_unknown=false despite no confirmed physical
                // motion; then the next same-target StartMove would no-op
                // skip on current_deg==target_deg and hide the bus failure.
                pending_dispatch_ = true;
                // Fresh request: reset the per-axis dispatch retry budget so
                // previous failures do not shorten this request's runway.
                dispatch_failures_ = 0;
            }

            // Returns true if this tick emitted any bus frame on the SCS
            // bus (WritePos via Dispatch() or ReadMove via PollReadMove()).
            // The wrapper Tick() does not use the bool to apply any
            // additional hold — both Dispatch() and PollReadMove() each
            // hold scs_bus_mutex_ atomically across their bus frame AND
            // the post-frame post_dispatch_quiet_gap_ms_ (yaw: 10 ms,
            // pitch: 0 ms), so the inter-axis quiet period is enforced
            // inside each axis's method without any release/reacquire
            // window. The return value is informational (kept for
            // diagnostic clarity and potential future use).
            bool Update() {
                AxisMotion snapshot;
                bool dispatch = false;
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                snapshot = motion_;
                dispatch = pending_dispatch_;
                // Do NOT clear pending_dispatch here. FinishDispatch consumes
                // it only on success or retry exhaustion; transient WritePos
                // failures keep it true so the next tick retries the same
                // target instead of silently dropping the request.
                xSemaphoreGive(motion_mutex_);

                // With per-axis Update(), dispatch-vs-poll is chosen per
                // servo. One axis can spend this tick dispatching while the
                // other axis polls ReadMove after the wrapper's inter-axis
                // wall-clock gap.
                if (dispatch) {
                    return Dispatch(snapshot);
                }
                if (snapshot.moving) {
                    PollReadMove(snapshot);
                }
                return false;
            }

            // Caller must hold motion_mutex_. Clears the per-axis private
            // cancellation state so that a subsequent Update() tick observes
            // a clean slate after the board-level code directly mutates
            // AxisMotion outside the Stage() path (currently
            // InitializeServo's Phase 0' post-init ReadPos re-sync and the
            // set_servo_torque MCP tool's disable path).
            //
            // Does NOT take any semaphore (motion_mutex_ is already held by
            // the caller; double-take of the non-recursive FreeRTOS
            // semaphore would deadlock). Does NOT touch the SCS bus. Does
            // NOT touch motion_ (AxisMotion); the caller has already mutated
            // it before invoking this method through
            // ServoDelegatedMotionDriver::InvalidateAxisToken.
            //
            // INVARIANT: every new AxisServo private cancellation-state
            // field added in the future MUST be added to this reset.
            // Otherwise external resets (Phase 0' / torque disable / any
            // future cancellation caller) will leave stale state that the
            // next Update() may act on, regressing the Issue #160 fix.
            void OnExternalReset() {
                pending_dispatch_ = false;
                dispatch_failures_ = 0;
                readmove_failures_ = 0;
            }

        private:
            // Returns true if WritePos was actually issued on the bus
            // (i.e. the snapshot was still the live request at the
            // pre-WritePos freshness gate). Returns false when a newer
            // StartMove superseded the snapshot between Update's
            // motion_mutex_ release and Dispatch's freshness gate — in
            // that case the bus was not touched, and the wrapper Tick()
            // does not need to hold scs_bus_mutex_ across the
            // inter-frame gap.
            bool Dispatch(const AxisMotion& snapshot) {
                int result = 0;
                bool live = false;
                int pos = deg_to_pos_(snapshot.target_deg);
                uint16_t duration =
                    static_cast<uint16_t>(snapshot.move_duration_ms);

                xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                live = motion_.request_token == snapshot.request_token;
                xSemaphoreGive(motion_mutex_);
                if (live) {
                    result = scs_bus_.WritePos(servo_id_, pos, duration, 0);
                    // Hold scs_bus_mutex_ across the post-WritePos quiet
                    // period atomically with the WritePos itself. Without
                    // this, releasing the mutex here would expose a
                    // release/reacquire window where concurrent MCP
                    // callers (get_head_angles ReadPos, uart_diag raw
                    // frames) could acquire the bus and inject traffic
                    // before any wrapper-level quiet-period guard
                    // starts. The original PR #146 bundled critical
                    // section incidentally protected this window;
                    // Phase 2's per-axis short-hold grain restores it
                    // per axis instead. Skipped when the quiet gap is
                    // 0 ms (pitch axis) or the WritePos was superseded
                    // (!live) — see post_dispatch_quiet_gap_ms_ member
                    // comment for per-axis policy rationale.
                    if (post_dispatch_quiet_gap_ms_ > 0) {
                        vTaskDelay(pdMS_TO_TICKS(post_dispatch_quiet_gap_ms_));
                    }
                }
                xSemaphoreGive(scs_bus_mutex_);

                bool write_ok = !live || ServoWritePosOk(result);
                if (live && !write_ok) {
                    ESP_LOGW(TAG,
                             "Motion %s WritePos failed: r=%d (deg=%d, pos=%d)",
                             axis_name_, result, snapshot.target_deg, pos);
                }

                // dispatch_now_ms captures the time WritePos completed
                // (ACK or timeout). FinishDispatch uses this for
                // dispatch_start_ms on success, so ApplyReadMoveResult
                // measures from physical acceptance rather than staging.
                uint32_t dispatch_now_ms =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000);

                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                // Commit / consume pending only if the snapshot is still
                // live and the WritePos actually ran. Superseded dispatches
                // keep the new request's pending flag intact and reset only
                // the stale retry counter.
                if (live && motion_.request_token == snapshot.request_token) {
                    FinishDispatch(snapshot.target_deg, write_ok,
                                   dispatch_now_ms);
                } else {
                    dispatch_failures_ = 0;
                }
                xSemaphoreGive(motion_mutex_);

                return live;
            }

            void PollReadMove(const AxisMotion& snapshot) {
                int read_move = -1;
                xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
                read_move = scs_bus_.ReadMove(servo_id_);
                // Hold scs_bus_mutex_ across the post-ReadMove quiet
                // period atomically with the ReadMove itself, mirroring
                // the post-WritePos pattern in Dispatch(). Without this,
                // releasing the mutex here would expose a window where
                // the wrapper Tick()'s subsequent pitch_axis_.Update()
                // could enter Dispatch() and issue a pitch WritePos
                // immediately, creating a yaw-ReadMove -> pitch-WritePos
                // sequence with effectively 0 ms inter-frame spacing on
                // the shared SCS bus. PR #146's bundled critical section
                // separated dispatch ticks from poll ticks (Tick step 1
                // vs step 2 mutually exclusive), so this ReadMove ->
                // WritePos ordering never arose; Phase 2's per-axis
                // grain makes it possible, so the guard is restored
                // per axis here. Skipped when post_dispatch_quiet_gap_ms_
                // is 0 (pitch axis); the member is shared between
                // post-WritePos and post-ReadMove paths because the
                // bus-quiet rationale is identical for both frame types.
                if (post_dispatch_quiet_gap_ms_ > 0) {
                    vTaskDelay(pdMS_TO_TICKS(post_dispatch_quiet_gap_ms_));
                }
                xSemaphoreGive(scs_bus_mutex_);

                uint32_t now_ms =
                    static_cast<uint32_t>(esp_timer_get_time() / 1000);

                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                // request_token guards against a newer StartMove racing in
                // between the Update snapshot and this commit; ms-resolution
                // move_start_ms can collide for back-to-back requests.
                if (motion_.request_token == snapshot.request_token) {
                    ApplyReadMoveResult(read_move, now_ms);
                }
                xSemaphoreGive(motion_mutex_);
            }

            // Finalises a dispatched WritePos. Caller holds motion_mutex_.
            // - write_ok==true: commit current_deg = target, consume the
            //   pending_dispatch flag, reset dispatch + ReadMove failure
            //   counters. ReadMove poll then tracks the in-flight delegated
            //   motion to completion.
            // - write_ok==false: keep pending_dispatch=true so the next tick
            //   retries the same target (transient ACK timeout / UART error
            //   should not silently drop the request). Bound the retry by
            //   kDispatchFailureLimit; when exhausted, log once, consume
            //   pending_dispatch, and clear moving so the axis returns to
            //   idle rather than spinning the retry loop forever.
            // readmove_failures_ is reset on success only; it tracks ReadMove
            // polling and is independent of WritePos ack semantics.
            void FinishDispatch(int target_deg, bool write_ok,
                                uint32_t dispatch_now_ms) {
                if (write_ok) {
                    motion_.current_deg = target_deg;
                    // dispatch_start_ms records when the servo actually
                    // received the GOAL_POSITION / GOAL_TIME write. This
                    // (not the staging timestamp in move_start_ms) is what
                    // ApplyReadMoveResult uses for the stuck-high timeout,
                    // so degraded-bus dispatch latency doesn't eat into
                    // the servo-internal duration budget.
                    motion_.dispatch_start_ms = dispatch_now_ms;
                    // Confirmed WritePos ACK supersedes any prior
                    // position_unknown mark. The new WritePos+ReadMove
                    // cycle is what proves (or fails to prove) the
                    // physical position.
                    motion_.position_unknown = false;
                    pending_dispatch_ = false;
                    dispatch_failures_ = 0;
                    readmove_failures_ = 0;
                    return;
                }
                dispatch_failures_++;
                if (dispatch_failures_ >= kDispatchFailureLimit) {
                    ESP_LOGW(TAG,
                             "Motion %s WritePos retries exhausted: current_deg=%d target_deg=%d; %d consecutive dispatch failures, abandoning request and marking position unknown",
                             axis_name_, motion_.current_deg,
                             motion_.target_deg, kDispatchFailureLimit);
                    pending_dispatch_ = false;
                    motion_.moving = false;
                    // A WritePos ACK timeout is NOT proof that the servo
                    // ignored the command — the command may have reached
                    // the servo while only the ACK/readback path failed.
                    // In that case the physical head has already moved to
                    // target_deg, but current_deg still holds the old
                    // value. Without position_unknown=true here, the next
                    // same-old-position StartMove would no-op-skip on the
                    // stale current_deg and silently drop the recovery
                    // request — exactly the degraded-bus condition this
                    // path is meant to handle. Mark unknown so a same-
                    // target retry forces a fresh dispatch and either
                    // confirms (FinishDispatch write_ok clears the flag)
                    // or surfaces another failure.
                    motion_.position_unknown = true;
                    dispatch_failures_ = 0;
                }
                // else: pending_dispatch stays true; next Update will retry the
                // same target (same start_deg / move_start_ms / move_duration_ms).
            }

            void ApplyReadMoveResult(int read_move, uint32_t now_ms) {
                if (read_move >= 0) {
                    readmove_failures_ = 0;
                    if (read_move == 0) {
                        // Sanity check against a stuck-low / false-zero
                        // status register: FinishDispatch optimistically
                        // committed current_deg to target_deg on WritePos
                        // ACK, so a transient ReadMove==0 returned before
                        // the servo could physically reach target would
                        // make the host treat the axis as "at target" and
                        // let the next same-target StartMove no-op-skip.
                        // If the elapsed time since confirmed dispatch is
                        // implausibly short relative to the commanded
                        // move_duration_ms (allowing kReadMoveEarlyMarginMs
                        // for genuine early arrival), treat the zero as
                        // suspicious and mark the position unknown.
                        if (motion_.dispatch_start_ms != 0 &&
                            motion_.move_duration_ms > kReadMoveEarlyMarginMs) {
                            uint32_t elapsed = now_ms - motion_.dispatch_start_ms;
                            uint32_t plausible_min =
                                motion_.move_duration_ms - kReadMoveEarlyMarginMs;
                            if (elapsed < plausible_min) {
                                ESP_LOGW(TAG,
                                         "Motion %s ReadMove=0 implausibly early: current_deg=%d target_deg=%d, elapsed=%ums but commanded duration=%ums (early margin=%ums); marking position unknown",
                                         axis_name_, motion_.current_deg,
                                         motion_.target_deg, (unsigned)elapsed,
                                         (unsigned)motion_.move_duration_ms,
                                         (unsigned)kReadMoveEarlyMarginMs);
                                motion_.moving = false;
                                motion_.position_unknown = true;
                                return;
                            }
                        }
                        motion_.moving = false;
                        return;
                    }
                    // read_move > 0: servo reports still moving. Bound the
                    // wait by move_duration_ms + kReadMoveStuckMarginMs to
                    // guard against a stuck-high ReadMove (the servo or
                    // register path degrades such that the motion-status
                    // bit never clears even after the requested completion
                    // time has elapsed).
                    //
                    // Elapsed is measured from dispatch_start_ms (when the
                    // servo actually received the command via a successful
                    // WritePos ACK), not from move_start_ms (staging time),
                    // so degraded-bus dispatch latency does not cause
                    // premature force-clear while the servo is genuinely
                    // still mid-motion. If dispatch_start_ms is still 0 the
                    // WritePos has not yet ACK'd; skip the timeout check
                    // until the dispatch is confirmed. Unsigned subtraction
                    // stays wrap-safe across the uint32 ms counter.
                    if (motion_.dispatch_start_ms == 0) {
                        return;
                    }
                    uint32_t elapsed = now_ms - motion_.dispatch_start_ms;
                    if (elapsed > motion_.move_duration_ms + kReadMoveStuckMarginMs) {
                        ESP_LOGW(TAG,
                                 "Motion %s ReadMove stuck-high: current_deg=%d target_deg=%d, %ums past commanded completion; marking position unknown and force-clearing moving",
                                 axis_name_, motion_.current_deg,
                                 motion_.target_deg,
                                 (unsigned)(elapsed - motion_.move_duration_ms));
                        motion_.moving = false;
                        motion_.position_unknown = true;
                    }
                    return;
                }

                readmove_failures_++;
                if (readmove_failures_ >= kReadMoveFailureLimit) {
                    ESP_LOGW(TAG,
                             "Motion %s ReadMove failed: current_deg=%d target_deg=%d; %d consecutive ReadMove failures, marking position unknown and force-clearing moving (next StartMove will re-dispatch even if target matches current_deg)",
                             axis_name_, motion_.current_deg,
                             motion_.target_deg, kReadMoveFailureLimit);
                    motion_.moving = false;
                    // Without this flag, a subsequent same-target StartMove
                    // would no-op-skip on current_deg==target_deg and the
                    // bus failure would stay hidden behind the optimistic
                    // commit. Marking the position unknown forces the next
                    // StartMove to re-dispatch and surface (or recover from)
                    // the underlying ReadMove fault.
                    motion_.position_unknown = true;
                    readmove_failures_ = 0;
                }
            }

            uint8_t servo_id_;
            int (*deg_to_pos_)(int);
            const char* axis_name_;
            AxisMotion& motion_;
            ScsBus& scs_bus_;
            SemaphoreHandle_t& scs_bus_mutex_;
            SemaphoreHandle_t& motion_mutex_;
            uint64_t& next_request_token_;
            // Post-bus-frame quiet period held INSIDE scs_bus_mutex_ on
            // a successful bus operation. Applies to BOTH frame types:
            // - Dispatch() WritePos: scs_bus_mutex_ is not released between
            //   the WritePos and this vTaskDelay.
            // - PollReadMove() ReadMove: scs_bus_mutex_ is not released
            //   between the ReadMove and this vTaskDelay.
            //
            // yaw is configured with 10 ms to preserve the SCS bus quiet
            // period that the original PR #146 bundled critical section
            // incidentally protected — for WritePos -> next-frame ordering
            // (PR #146 empirical model) AND for the new ReadMove ->
            // pitch-WritePos ordering introduced by Phase 2's per-axis
            // grain (PR #146 had no such ordering because dispatch and
            // poll Tick phases were mutually exclusive).
            //
            // pitch is configured with 0 ms because the PR #146 empirical
            // model (E1 / E4-fresh / E5 / E6 all clean) shows post-pitch
            // quiet was not required for bus stability. Set to 0 to
            // disable the per-axis post-frame hold entirely. Holding
            // scs_bus_mutex_ across a vTaskDelay is intentional here —
            // it blocks concurrent MCP bus callers (get_head_angles
            // ReadPos, uart_diag raw frames) for the quiet-period
            // duration, which is the explicit invariant being restored.
            //
            // Name retained as "post_dispatch_quiet_gap_ms_" for
            // historical continuity; semantically it is "post-bus-frame
            // quiet gap" and applies symmetrically to both WritePos and
            // ReadMove paths.
            uint32_t post_dispatch_quiet_gap_ms_;
            int readmove_failures_ = 0;
            bool pending_dispatch_ = false;
            int dispatch_failures_ = 0;
        };

        SemaphoreHandle_t& motion_mutex_;
        AxisMotion& yaw_motion_;
        AxisMotion& pitch_motion_;
        // Monotonically increasing request id. Each StartMove that stages
        // a dispatch picks ++next_request_token_ and writes it into the
        // corresponding AxisMotion::request_token. Tick() then uses
        // request_token equality (rather than move_start_ms, which only
        // has ms resolution) to detect whether a snapshot is still the
        // live request. motion_mutex_ guards this counter.
        uint64_t next_request_token_ = 0;
        AxisServo yaw_axis_;
        AxisServo pitch_axis_;
    };

    void InitializePowerSaveTimer() {
        // Dim after 1 min idle, power off after 10 min.
        power_save_timer_ = new PowerSaveTimer(-1, 60, 600);
        power_save_timer_->OnEnterSleepMode([this]() {
            dimmed_.store(true);
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(10);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            dimmed_.store(false);
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void StopScreenOffTimerLocked() {
        if (screen_off_timer_ == nullptr) {
            return;
        }
        esp_err_t err = esp_timer_stop(screen_off_timer_);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Failed to stop screen-off timer: %s",
                     esp_err_to_name(err));
        }
    }

    void StartScreenOffTimerLocked(int64_t delay_us) {
        if (screen_off_timer_ == nullptr || delay_us <= 0) {
            return;
        }
        StopScreenOffTimerLocked();
        esp_err_t err = esp_timer_start_once(screen_off_timer_,
                                             static_cast<uint64_t>(delay_us));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start screen-off timer: %s",
                     esp_err_to_name(err));
        }
    }

    void ArmScreenOffTimerLocked() {
        StopScreenOffTimerLocked();
        int timeout_seconds =
            screen_off_timeout_seconds_.load(std::memory_order_acquire);
        if (timeout_seconds == 0) {
            screen_off_deadline_us_ = 0;
            return;
        }
        int64_t delay_us = static_cast<int64_t>(timeout_seconds) * 1000000LL;
        screen_off_deadline_us_ = esp_timer_get_time() + delay_us;
        StartScreenOffTimerLocked(delay_us);
    }

    void EnterPanelSleepLocked() {
        if (lcd_panel_ == nullptr || lcd_panel_io_ == nullptr) {
            ESP_LOGW(TAG, "LCD handles are not ready; using backlight-only screen off");
            return;
        }

        esp_err_t display_err = esp_lcd_panel_disp_on_off(lcd_panel_, false);
        if (display_err == ESP_OK) {
            panel_display_disabled_ = true;
        } else {
            ESP_LOGW(TAG, "LCD display-off command failed (%s); backlight remains off",
                     esp_err_to_name(display_err));
        }

        // esp_lcd_ili9341 v1.2.0 implements disp_on_off but does not install
        // the esp_lcd_panel_t::disp_sleep callback. Use the same panel IO to
        // issue the ILI9342C-compatible SLPIN command. Frame memory is kept.
        esp_err_t sleep_err = esp_lcd_panel_io_tx_param(
            lcd_panel_io_, LCD_CMD_SLPIN, nullptr, 0);
        if (sleep_err == ESP_OK) {
            panel_sleeping_ = true;
            panel_sleep_started_us_ = esp_timer_get_time();
        } else {
            ESP_LOGW(TAG, "LCD sleep-in command failed (%s); backlight remains off",
                     esp_err_to_name(sleep_err));
        }
    }

    bool ExitPanelSleepLocked() {
        if (lcd_panel_ == nullptr || lcd_panel_io_ == nullptr) {
            ESP_LOGW(TAG, "LCD handles are not ready; restoring backlight only");
            return true;
        }

        if (panel_sleeping_) {
            // ILI9342C requires 120 ms between SLPIN and SLPOUT. A quick
            // activity immediately after timeout therefore waits only the
            // unelapsed remainder.
            int64_t elapsed_ms =
                (esp_timer_get_time() - panel_sleep_started_us_) / 1000LL;
            if (elapsed_ms < STACKCHAN_LCD_SLEEP_TRANSITION_MS) {
                vTaskDelay(pdMS_TO_TICKS(
                    STACKCHAN_LCD_SLEEP_TRANSITION_MS - elapsed_ms));
            }

            esp_err_t wake_err = esp_lcd_panel_io_tx_param(
                lcd_panel_io_, LCD_CMD_SLPOUT, nullptr, 0);
            if (wake_err != ESP_OK) {
                ESP_LOGW(TAG, "LCD sleep-out command failed (%s); keeping backlight off",
                         esp_err_to_name(wake_err));
                return false;
            }
            panel_sleeping_ = false;
            panel_sleep_started_us_ = 0;

            // ILI9342C needs 120 ms after SLPOUT before normal display
            // operation is guaranteed.
            vTaskDelay(pdMS_TO_TICKS(STACKCHAN_LCD_SLEEP_TRANSITION_MS));
        }

        if (panel_display_disabled_) {
            esp_err_t display_err = esp_lcd_panel_disp_on_off(lcd_panel_, true);
            if (display_err != ESP_OK) {
                ESP_LOGW(TAG, "LCD display-on command failed (%s); keeping backlight off",
                         esp_err_to_name(display_err));
                return false;
            }
            panel_display_disabled_ = false;
        }
        return true;
    }

    void TurnScreenOffLocked() {
        if (screen_is_off_.load(std::memory_order_acquire)) {
            return;
        }

        auto backlight = static_cast<CustomBacklight*>(GetBacklight());
        if (backlight != nullptr) {
            screen_saved_brightness_ = backlight->brightness();
            // CustomBacklight's normal SetBrightness path is driven by an
            // esp_timer. Force the PMIC write here so brightness reaches zero
            // before any panel power command is issued.
            backlight->SetBrightnessImmediate(0);
        }

        if (display_ != nullptr) {
            DisplayLockGuard lock(display_);
            EnterPanelSleepLocked();
        } else {
            EnterPanelSleepLocked();
        }
        screen_is_off_.store(true, std::memory_order_release);
        ESP_LOGI(TAG, "Screen turned off after idle timeout");
    }

    bool WakeScreenLocked() {
        if (!screen_is_off_.load(std::memory_order_acquire)) {
            return true;
        }

        bool panel_ready = false;
        if (display_ != nullptr) {
            DisplayLockGuard lock(display_);
            panel_ready = ExitPanelSleepLocked();
        } else {
            panel_ready = ExitPanelSleepLocked();
        }
        if (!panel_ready) {
            return false;
        }

        auto backlight = static_cast<CustomBacklight*>(GetBacklight());
        if (backlight != nullptr) {
            backlight->SetBrightnessImmediate(screen_saved_brightness_);
        }

        // Sleep In/Out on this panel retains frame memory, but the LVGL
        // compositor's dirty-area tracking does not know the panel briefly
        // stopped scanning it out. Without a forced full-area invalidate,
        // the next natural partial redraw can flush a stale/half-composited
        // frame onto the just-woken panel, seen as avatar double-display and
        // horizontal tearing until an unrelated activity (e.g. an emotion
        // change) happens to trigger a full redraw.
        if (display_ != nullptr) {
            DisplayLockGuard lock(display_);
            lv_obj_invalidate(lv_screen_active());
            ESP_LOGI(TAG, "Forcing full-screen redraw after wake");
        }

        screen_is_off_.store(false, std::memory_order_release);
        ESP_LOGI(TAG, "Screen woke from idle timeout");
        return true;
    }

    void ScreenOffTimerTick() {
        if (screen_power_mutex_ == nullptr ||
            xSemaphoreTake(screen_power_mutex_, portMAX_DELAY) != pdTRUE) {
            return;
        }

        int timeout_seconds =
            screen_off_timeout_seconds_.load(std::memory_order_acquire);
        int64_t now_us = esp_timer_get_time();
        if (timeout_seconds == 0) {
            screen_off_deadline_us_ = 0;
        } else if (screen_off_deadline_us_ > now_us) {
            // An activity raced with an already-queued one-shot callback.
            // Keep the newer deadline instead of turning the screen off.
            StartScreenOffTimerLocked(screen_off_deadline_us_ - now_us);
        } else {
            TurnScreenOffLocked();
        }

        xSemaphoreGive(screen_power_mutex_);
    }

    static void ScreenOffTimerCb(void* arg) {
        static_cast<StackChanBoard*>(arg)->ScreenOffTimerTick();
    }

    void InitializeScreenOffTimer() {
        Settings settings(STACKCHAN_SCREEN_OFF_NVS_NAMESPACE, false);
        int timeout_seconds = settings.GetInt(
            STACKCHAN_SCREEN_OFF_NVS_KEY,
            STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS);
        if (timeout_seconds < 0 ||
            timeout_seconds > STACKCHAN_SCREEN_OFF_MAX_TIMEOUT_SECONDS) {
            ESP_LOGW(TAG,
                     "Ignoring invalid persisted screen-off timeout %d; using default %d",
                     timeout_seconds,
                     STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS);
            timeout_seconds = STACKCHAN_SCREEN_OFF_DEFAULT_TIMEOUT_SECONDS;
        }
        screen_off_timeout_seconds_.store(timeout_seconds,
                                          std::memory_order_release);

        screen_power_mutex_ = xSemaphoreCreateMutex();
        if (screen_power_mutex_ == nullptr) {
            ESP_LOGW(TAG, "Failed to create screen power mutex; screen-off disabled");
            return;
        }

        const esp_timer_create_args_t timer_args = {
            .callback = &StackChanBoard::ScreenOffTimerCb,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "screen_off",
            .skip_unhandled_events = true,
        };
        esp_err_t err = esp_timer_create(&timer_args, &screen_off_timer_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to create screen-off timer: %s",
                     esp_err_to_name(err));
            return;
        }

        if (xSemaphoreTake(screen_power_mutex_, portMAX_DELAY) == pdTRUE) {
            ArmScreenOffTimerLocked();
            xSemaphoreGive(screen_power_mutex_);
        }
        ESP_LOGI(TAG, "Screen-off timeout: %d seconds%s", timeout_seconds,
                 timeout_seconds == 0 ? " (disabled)" : "");
    }

    void HandleScreenActivity() {
        NoteActivity();
        if (screen_power_mutex_ == nullptr ||
            xSemaphoreTake(screen_power_mutex_, portMAX_DELAY) != pdTRUE) {
            return;
        }
        WakeScreenLocked();
        ArmScreenOffTimerLocked();
        xSemaphoreGive(screen_power_mutex_);
    }

    bool SetScreenOffTimeoutSeconds(int timeout_seconds) {
        if (timeout_seconds < 0 ||
            timeout_seconds > STACKCHAN_SCREEN_OFF_MAX_TIMEOUT_SECONDS) {
            return false;
        }

        {
            Settings settings(STACKCHAN_SCREEN_OFF_NVS_NAMESPACE, true);
            settings.SetInt(STACKCHAN_SCREEN_OFF_NVS_KEY, timeout_seconds);
        }

        if (screen_power_mutex_ == nullptr ||
            xSemaphoreTake(screen_power_mutex_, portMAX_DELAY) != pdTRUE) {
            screen_off_timeout_seconds_.store(timeout_seconds,
                                              std::memory_order_release);
            return true;
        }

        screen_off_timeout_seconds_.store(timeout_seconds,
                                          std::memory_order_release);
        if (timeout_seconds == 0) {
            WakeScreenLocked();
        }
        ArmScreenOffTimerLocked();
        xSemaphoreGive(screen_power_mutex_);
        return true;
    }

    cJSON* GetScreenOffTimeoutJson(bool applied = true) const {
        int timeout_seconds =
            screen_off_timeout_seconds_.load(std::memory_order_acquire);
        cJSON* root = cJSON_CreateObject();
        cJSON_AddBoolToObject(root, "ok", applied);
        cJSON_AddBoolToObject(root, "applied", applied);
        cJSON_AddNumberToObject(root, "timeout_seconds", timeout_seconds);
        cJSON_AddBoolToObject(root, "persistent", true);
        cJSON_AddStringToObject(root, "persistence", "nvs");
        cJSON_AddBoolToObject(
            root, "enabled",
            timeout_seconds != 0 && screen_off_timer_ != nullptr);
        cJSON_AddBoolToObject(
            root, "screen_off",
            screen_is_off_.load(std::memory_order_acquire));
        return root;
    }

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializePortAI2c() {
        // Grove Port A bus. Uses I2C controller 0 (the internal bus above
        // uses controller 1) so the two run independently. Attached Unit
        // modules typically include their own 10 kΩ pull-ups in the Grove
        // hub, but enable internal pull-ups as a fall-back for bare wiring.
        i2c_master_bus_config_t port_a_cfg = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = PORT_A_I2C_SDA_PIN,
            .scl_io_num = PORT_A_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&port_a_cfg, &port_a_i2c_bus_));
    }

    esp_err_t InitPortBWs2812(uint16_t led_count) {
        if (ws2812_ok_ && ws2812_led_count_ == led_count) {
            return ESP_OK;  // idempotent: same led_count is a no-op
        }
        if (ws2812_handle_ != nullptr) {
            led_strip_del(ws2812_handle_);
            ws2812_handle_ = nullptr;
            ws2812_ok_ = false;
            ws2812_led_count_ = 0;
        }

        led_strip_config_t strip_config = {
            .strip_gpio_num = PORT_B_WS2812_DATA_PIN,
            .max_leds = led_count,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags = { .invert_out = false },
        };
        led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,  // 10 MHz, standard WS2812 bit timing
            .mem_block_symbols = 0,              // 0 = driver default block size
            .flags = { .with_dma = false },
        };
        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &ws2812_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "port_b.ws2812.init led_strip_new_rmt_device failed: %s",
                     esp_err_to_name(err));
            return err;
        }

        err = led_strip_clear(ws2812_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "port_b.ws2812.init led_strip_clear failed: %s",
                     esp_err_to_name(err));
            led_strip_del(ws2812_handle_);
            ws2812_handle_ = nullptr;
            return err;
        }
        ws2812_led_count_ = led_count;
        ws2812_ok_ = true;
        ESP_LOGI(TAG, "port_b.ws2812 initialized: %u LEDs on GPIO %d",
                 (unsigned)led_count, (int)PORT_B_WS2812_DATA_PIN);
        return ESP_OK;
    }

    esp_err_t InitPortCWs2812(uint16_t led_count) {
        if (port_c_ws2812_ok_ && port_c_ws2812_led_count_ == led_count) {
            return ESP_OK;  // idempotent: same led_count is a no-op
        }
        if (port_c_ws2812_handle_ != nullptr) {
            led_strip_del(port_c_ws2812_handle_);
            port_c_ws2812_handle_ = nullptr;
            port_c_ws2812_ok_ = false;
            port_c_ws2812_led_count_ = 0;
        }

        led_strip_config_t strip_config = {
            .strip_gpio_num = PORT_C_WS2812_DATA_PIN,
            .max_leds = led_count,
            .led_model = LED_MODEL_WS2812,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .flags = { .invert_out = false },
        };
        // The led_strip driver auto-allocates an RMT TX channel. ESP32-S3 has
        // multiple TX channels, and StackChan uses RMT only for the Port B/C
        // strips, so both WS2812 handles can coexist.
        led_strip_rmt_config_t rmt_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000,  // 10 MHz, standard WS2812 bit timing
            .mem_block_symbols = 0,              // 0 = driver default block size
            .flags = { .with_dma = false },
        };
        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &port_c_ws2812_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "port_c.ws2812.init Port C GPIO %d led_strip_new_rmt_device failed: %s",
                     (int)PORT_C_WS2812_DATA_PIN, esp_err_to_name(err));
            return err;
        }

        err = led_strip_clear(port_c_ws2812_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "port_c.ws2812.init Port C GPIO %d led_strip_clear failed: %s",
                     (int)PORT_C_WS2812_DATA_PIN, esp_err_to_name(err));
            led_strip_del(port_c_ws2812_handle_);
            port_c_ws2812_handle_ = nullptr;
            return err;
        }
        port_c_ws2812_led_count_ = led_count;
        port_c_ws2812_ok_ = true;
        ESP_LOGI(TAG, "port_c.ws2812 initialized: %u LEDs on Port C GPIO %d",
                 (unsigned)led_count, (int)PORT_C_WS2812_DATA_PIN);
        return ESP_OK;
    }

    void I2cDetect() {
        uint8_t address;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        for (int i = 0; i < 128; i += 16) {
            printf("%02x: ", i);
            for (int j = 0; j < 16; j++) {
                fflush(stdout);
                address = i + j;
                esp_err_t ret = i2c_master_probe(i2c_bus_, address, pdMS_TO_TICKS(200));
                if (ret == ESP_OK) {
                    printf("%02x ", address);
                } else if (ret == ESP_ERR_TIMEOUT) {
                    printf("UU ");
                } else {
                    printf("-- ");
                }
            }
            printf("\r\n");
        }
    }

    static const char* ChargeAutoDecisionName(ChargeAutoDecision decision) {
        switch (decision) {
            case ChargeAutoDecision::kEnabledLowBattery:
                return "enabled_low_battery";
            case ChargeAutoDecision::kDisabledHighBattery:
                return "disabled_high_battery";
            case ChargeAutoDecision::kKeptBetweenThresholds:
                return "kept_between_thresholds";
            case ChargeAutoDecision::kEnabledBatteryReadFailed:
                return "enabled_battery_read_failed";
            case ChargeAutoDecision::kAutoDisabled:
                return "auto_disabled";
            case ChargeAutoDecision::kNotRun:
            default:
                return "not_run";
        }
    }

    bool ReadBatteryLevel(int* level) {
        return pmic_ != nullptr && pmic_->TryGetBatteryLevel(level);
    }

    // Apply the automatic policy to a battery sample. Charging is the resting
    // state at startup; protection engages only at the upper threshold.
    void ApplyAutomaticChargePolicy(int level, bool level_available) {
        last_charge_auto_battery_level_.store(
            level_available ? level : -1, std::memory_order_relaxed);

        if (!level_available) {
            pmic_->SetChargeEnabled(true);
            last_charge_auto_decision_.store(
                ChargeAutoDecision::kEnabledBatteryReadFailed,
                std::memory_order_release);
            ESP_LOGW(TAG,
                     "Battery level unavailable; charging enabled as fail-safe");
            return;
        }

        ChargeAutoDecision decision;
        if (level <= STACKCHAN_CHARGE_ON_BELOW) {
            pmic_->SetChargeEnabled(true);
            decision = ChargeAutoDecision::kEnabledLowBattery;
        } else if (level >= STACKCHAN_CHARGE_OFF_ABOVE) {
            pmic_->SetChargeEnabled(false);
            decision = ChargeAutoDecision::kDisabledHighBattery;
        } else {
            // Hysteresis band: intentionally make no PMIC write.
            decision = ChargeAutoDecision::kKeptBetweenThresholds;
        }

        last_charge_auto_decision_.store(decision,
                                         std::memory_order_release);
        ESP_LOGI(TAG, "Automatic charge check: level=%d decision=%s",
                 level, ChargeAutoDecisionName(decision));
    }

    // Reuse the existing one-second status-bar battery poll, but only make an
    // automatic decision approximately once per 60 seconds. Unsigned
    // subtraction keeps the interval correct when the 32-bit millisecond
    // counter wraps. The startup call bypasses the interval exactly once.
    void MaybeApplyAutomaticChargePolicy(int level, bool level_available,
                                         bool startup = false) {
        if (STACKCHAN_CHARGE_AUTO == 0) {
            return;
        }

        uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (startup) {
            last_charge_auto_check_ms_.store(now_ms,
                                             std::memory_order_relaxed);
        } else {
            uint32_t last_ms =
                last_charge_auto_check_ms_.load(std::memory_order_relaxed);
            do {
                if (static_cast<uint32_t>(now_ms - last_ms) <
                    CHARGE_AUTO_INTERVAL_MS) {
                    return;
                }
            } while (!last_charge_auto_check_ms_.compare_exchange_weak(
                last_ms, now_ms, std::memory_order_relaxed));
        }

        ApplyAutomaticChargePolicy(level, level_available);
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
        if (STACKCHAN_CHARGE_AUTO != 0) {
            int level = -1;
            bool level_available = ReadBatteryLevel(&level);
            MaybeApplyAutomaticChargePolicy(level, level_available, true);
        } else {
            ESP_LOGI(TAG,
                     "Automatic charge control disabled at build time");
        }
    }

    void InitializeAw9523() {
        ESP_LOGI(TAG, "Init AW9523");
        aw9523_ = new Aw9523(i2c_bus_, 0x58);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    bool EnsureListeningIndicatorObjectLocked() {
        if (listening_indicator_ != nullptr) {
            if (lv_obj_is_valid(listening_indicator_)) {
                if (listening_indicator_dot_ != nullptr &&
                    lv_obj_is_valid(listening_indicator_dot_)) {
                    return true;
                }
                StopListeningIndicatorPulseLocked();
                lv_obj_del(listening_indicator_);
            } else {
                StopListeningIndicatorPulseLocked();
            }
        }
        listening_indicator_ = nullptr;
        listening_indicator_dot_ = nullptr;

        lv_obj_t* screen = lv_screen_active();
        if (screen == nullptr) {
            return false;
        }

        listening_indicator_ = lv_obj_create(screen);
        if (listening_indicator_ == nullptr) {
            return false;
        }

        lv_obj_set_size(listening_indicator_, 36, 30);
        lv_obj_align(listening_indicator_, LV_ALIGN_TOP_RIGHT, -8, 8);
        lv_obj_clear_flag(listening_indicator_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_radius(listening_indicator_, 10, 0);
        lv_obj_set_style_bg_color(listening_indicator_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(listening_indicator_, LV_OPA_70, 0);
        lv_obj_set_style_border_width(listening_indicator_, 0, 0);
        lv_obj_set_style_pad_all(listening_indicator_, 0, 0);

        listening_indicator_dot_ = lv_obj_create(listening_indicator_);
        if (listening_indicator_dot_ == nullptr) {
            lv_obj_del(listening_indicator_);
            listening_indicator_ = nullptr;
            listening_indicator_dot_ = nullptr;
            return false;
        }
        lv_obj_set_size(listening_indicator_dot_,
                        LISTENING_INDICATOR_DOT_SIZE_PX,
                        LISTENING_INDICATOR_DOT_SIZE_PX);
        lv_obj_clear_flag(listening_indicator_dot_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_radius(listening_indicator_dot_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(listening_indicator_dot_,
                                  lv_color_hex(0xE0352B), 0);
        lv_obj_set_style_bg_opa(listening_indicator_dot_, LV_OPA_COVER, 0);
        lv_obj_set_style_opa(listening_indicator_dot_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(listening_indicator_dot_, 0, 0);
        lv_obj_set_style_pad_all(listening_indicator_dot_, 0, 0);
        lv_obj_center(listening_indicator_dot_);

        lv_obj_add_flag(listening_indicator_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(listening_indicator_);
        ESP_LOGI(TAG, "Listening indicator created on active screen");
        return true;
    }

    static void SetListeningDotOpacity(void* obj, int32_t opa) {
        auto* dot = static_cast<lv_obj_t*>(obj);
        if (dot == nullptr || !lv_obj_is_valid(dot)) {
            return;
        }
        lv_obj_set_style_opa(dot, static_cast<lv_opa_t>(opa), 0);
    }

    void StopListeningIndicatorPulseLocked() {
        if (listening_indicator_dot_ == nullptr) {
            return;
        }
        lv_anim_delete(listening_indicator_dot_, nullptr);
        if (lv_obj_is_valid(listening_indicator_dot_)) {
            lv_obj_set_style_opa(listening_indicator_dot_, LV_OPA_COVER, 0);
        }
    }

    void StartListeningIndicatorPulseLocked() {
        if (listening_indicator_dot_ == nullptr ||
            !lv_obj_is_valid(listening_indicator_dot_)) {
            return;
        }

        StopListeningIndicatorPulseLocked();

        lv_anim_t anim;
        lv_anim_init(&anim);
        lv_anim_set_var(&anim, listening_indicator_dot_);
        lv_anim_set_values(&anim, LV_OPA_COVER, LV_OPA_40);
        lv_anim_set_exec_cb(&anim, SetListeningDotOpacity);
        lv_anim_set_duration(&anim, 700);
        lv_anim_set_reverse_duration(&anim, 700);
        lv_anim_set_path_cb(&anim, lv_anim_path_ease_in_out);
        lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&anim);
    }

    void BringListeningIndicatorToFrontLocked() {
        if (listening_indicator_ == nullptr) {
            return;
        }
        if (!lv_obj_is_valid(listening_indicator_)) {
            StopListeningIndicatorPulseLocked();
            listening_indicator_ = nullptr;
            listening_indicator_dot_ = nullptr;
            listening_indicator_visible_.store(false, std::memory_order_release);
            return;
        }
        lv_obj_move_foreground(listening_indicator_);
    }

    void SetListeningIndicatorVisibleLocked(bool visible) {
        if (visible) {
            if (!EnsureListeningIndicatorObjectLocked()) {
                listening_indicator_visible_.store(false, std::memory_order_release);
                return;
            }
            lv_obj_clear_flag(listening_indicator_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(listening_indicator_);
            StartListeningIndicatorPulseLocked();
            listening_indicator_visible_.store(true, std::memory_order_release);
            return;
        }

        if (listening_indicator_ != nullptr) {
            StopListeningIndicatorPulseLocked();
            if (lv_obj_is_valid(listening_indicator_)) {
                lv_obj_add_flag(listening_indicator_, LV_OBJ_FLAG_HIDDEN);
            } else {
                listening_indicator_ = nullptr;
                listening_indicator_dot_ = nullptr;
            }
        }
        listening_indicator_visible_.store(false, std::memory_order_release);
    }

    void UpdateListeningIndicatorForState(bool is_listening) {
        if (display_ == nullptr) {
            return;
        }
        if (listening_indicator_visible_.load(std::memory_order_acquire) ==
            is_listening) {
            return;
        }
        DisplayLockGuard lock(display_);
        SetListeningIndicatorVisibleLocked(is_listening);
    }

    void PollTouchpad() {
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        static int64_t last_release_ms = 0;       // デバウンス用 (= 直前 release 時刻)
        static int64_t listening_started_ms = 0;  // タイムアウト用 (= listening 突入時刻)
        static bool was_listening = false;        // listening 突入のエッジ検出
        const int64_t TOUCH_THRESHOLD_MS = 500;   // 触摸时长阈值，超过500ms视为长按
        const int64_t DEBOUNCE_MS = 300;          // 直前 release から N ms 以内の press は無視
        const int64_t LISTEN_TIMEOUT_MS = 30000;  // listening 状態に N ms 以上滞在で auto stop

        auto& app = Application::GetInstance();
        int64_t now_ms = esp_timer_get_time() / 1000;

        // --- listening 状態の上界 (タイムアウト) 管理 ---
        // 状態遷移のエッジ検出で突入時刻を記録、 滞在時間が LISTEN_TIMEOUT_MS を
        // 超えたら StopListening を自動発火する。 タッチ忘れ放置で listen が
        // 無限持続するのを防ぐ。 StopListening 後は listening_started_ms を 0 に
        // 戻して再発火を抑止 (次に listening 突入したら再セット)。
        bool is_listening = (app.GetDeviceState() == kDeviceStateListening);
        UpdateListeningIndicatorForState(is_listening);
        if (is_listening && !was_listening) {
            listening_started_ms = now_ms;
            ESP_LOGI(TAG, "Listening entered at %d ms (timeout in %d ms)",
                     (int)now_ms, (int)LISTEN_TIMEOUT_MS);
        }
        was_listening = is_listening;
        if (is_listening && listening_started_ms != 0 &&
            (now_ms - listening_started_ms) > LISTEN_TIMEOUT_MS) {
            ESP_LOGI(TAG, "Listening timeout reached (%d ms) -> StopListening",
                     (int)(now_ms - listening_started_ms));
            SetAllRgbLeds(0, 0, 0);
            app.StopListening();
            listening_started_ms = 0;
        }

        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        if (touch_point.num > 0) {
            HandleScreenActivity();
        }

        // 检测触摸开始
        if (touch_point.num > 0 && !was_touched) {
            // デバウンス: 直前 release から DEBOUNCE_MS 以内の press は無視。
            // FT6336 のチャタリングや「タッチした直後にもう一度触れてしまう」
            // 連打事故を防止。
            if (last_release_ms != 0 && (now_ms - last_release_ms) < DEBOUNCE_MS) {
                // was_touched は更新しない。 次の poll でも press 判定を再評価
                // するが、 デバウンス期間を超えれば通常 press として処理される。
                return;
            }
            was_touched = true;
            touch_start_time = now_ms;
            // タッチ瞬時の PlaySound 直接呼び出しは行わない。 直後に
            // StartListening → EnableVoiceProcessing(true) → ResetDecoder で
            // playback queue がクリアされて音が消えるため。 代わりに
            // Application::StartListening 側で play_popup_on_listening_ flag を
            // 立てて、 HandleStateChangedEvent の Listening 分岐後半 (ResetDecoder
            // の後) で OGG_POPUP を鳴らす経路に乗せる (= xiaozhi 標準の WakeWord
            // 経路と同じ仕組み)。
        }
        // 检测触摸释放
        else if (touch_point.num == 0 && was_touched) {
            was_touched = false;
            int64_t touch_duration = now_ms - touch_start_time;
            last_release_ms = now_ms;

            // 只有短触才触发
            if (touch_duration < TOUCH_THRESHOLD_MS) {
                // A tap on a picture or caption dismisses it rather than
                // starting to listen.
                if (DismissOverlaysOnTap()) {
                    return;
                }
                if (app.GetDeviceState() == kDeviceStateStarting) {
                    EnterWifiConfigMode();
                    return;
                }
                // kDeviceStateAudioTesting は WiFi config 完了直後の audio test
                // モードに居る状態。 ここから WifiConfiguring に戻る経路は
                // ToggleChatState() しか持っていない (= HandleStartListeningEvent
                // は AudioTesting を扱わない)。 StartListening にだけ分岐すると
                // タッチで設定モードに復帰できなくなるので、 AudioTesting だけ
                // は従来通り ToggleChatState() に流して状態機械任せにする。
                if (app.GetDeviceState() == kDeviceStateAudioTesting) {
                    app.ToggleChatState();
                    return;
                }
                // listening 中の2回目タッチは Application::HandleToggleChatEvent
                // の既定経路 (CloseAudioChannel = WS 切断 → gateway の recording
                // slot が aborted_mid_capture として buffer 破棄) ではなく
                // StopListening (= SendStopListening) に分岐させる。これで
                // device-driven audio capture push 経路 (gateway 側
                // audio_input_hook) が listen.stop を受けて buffer を Ogg 化 +
                // 外部 hook へ POST できる。Vessel UX として「タッチで listen
                // 開始 → 発話 → タッチで送信」を成立させるための fork 専用分岐。
                // listening 中の2回目タッチは StopListening、 それ以外は
                // StartListening (詳細は ToggleTouchListening)。
                ToggleTouchListening();
            }
        }
    }

    // Start listening, or stop and send what was heard if already
    // listening. Shared by a screen tap and a head tap.
    void ToggleTouchListening() {
        auto& app = Application::GetInstance();
        const auto state = app.GetDeviceState();
        if (listen_cue_pending_.load()) {
            return;  // already on the way to listening
        }
        if (state == kDeviceStateListening) {
            // 録音終了のフィードバック (= 全 LED 消灯)。 デバッグ目的、
            // MCP self.led.set_* 経由で上書き可能。
            SetAllRgbLeds(0, 0, 0);
            app.StopListening();
        } else {
            // listening 開始は ToggleChatState ではなく StartListening
            // を使う。 ToggleChatState 経由は SetListeningMode に
            // GetDefaultListeningMode() (= AutoStop) を渡すため、
            // ペルソナ発話終了 (tts.stop) の Schedule 内で device が
            // 自動的に Listening 状態に再復帰してしまい (= xiaozhi の
            // 連続会話モデル、 application.cc:565)、 「タッチ駆動」 が
            // 破綻する (= 次のタッチが listen.stop 経路に入って即送信)。
            // StartListening 経由は HandleStartListeningEvent で
            // SetListeningMode(ManualStop) を強制するので、 tts.stop 後
            // は Idle に留まり、 次のタッチで明示的に listen 開始する
            // Vessel UX が成立する。 Idle 以外 (Speaking 等) でも
            // HandleStartListeningEvent が AbortSpeaking → ManualStop で
            // 適切に処理する。
            // 録音開始想定のフィードバック (= 全 LED 緑点灯、 控えめ
            // な輝度)。 実際の listen 起動は StartListening 経由で
            // 非同期処理。 タッチが取れたかどうかの体感を優先。
            SetAllRgbLeds(0, 32, 0);
#if CONFIG_STACKCHAN_LISTEN_VOICE_CUE
            if (state == kDeviceStateIdle) {
                StartListeningAfterCue();
                return;
            }
#endif
            app.StartListening();
        }
    }

#if CONFIG_STACKCHAN_LISTEN_VOICE_CUE
    // Say listen_cue.ogg ("What do you want?"), then start listening without the
    // popup beep. The mic opens only after the cue has played out, as
    // there is no echo cancellation and it would otherwise be transcribed.
    void StartListeningAfterCue() {
        if (listen_cue_pending_.exchange(true)) return;
        if (listen_cue_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = [](void* arg) { static_cast<StackChanBoard*>(arg)->ListenCueTick(); },
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "listen_cue",
                .skip_unhandled_events = true,
            };
            if (esp_timer_create(&args, &listen_cue_timer_) != ESP_OK) {
                listen_cue_pending_.store(false);
                Application::GetInstance().StartListening();
                return;
            }
        }
        listen_cue_drained_ = false;
        listen_cue_started_us_ = esp_timer_get_time();
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_LISTEN_CUE);
        esp_timer_start_once(listen_cue_timer_, 300 * 1000);
    }

    void ListenCueTick() {
        auto& app = Application::GetInstance();
        const uint64_t elapsed_ms = (esp_timer_get_time() - listen_cue_started_us_) / 1000;
        if (!listen_cue_drained_ && elapsed_ms < LISTEN_CUE_MAX_MS) {
            if (!app.GetAudioService().IsPlaybackIdle()) {
                esp_timer_start_once(listen_cue_timer_, 50 * 1000);
                return;
            }
            // Queues are empty; let the last frame leave the speaker.
            listen_cue_drained_ = true;
            esp_timer_start_once(listen_cue_timer_, 150 * 1000);
            return;
        }
        listen_cue_pending_.store(false);
        if (app.GetDeviceState() == kDeviceStateIdle) {
            app.SkipNextListeningCue();
            app.StartListening();
        } else {
            SetAllRgbLeds(0, 0, 0);
        }
    }
#endif

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);
        
        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                StackChanBoard* board = (StackChanBoard*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        // The microSD slot shares this bus, and its data-out line is the
        // LCD's D/C pin (GPIO35). With the card's CS (GPIO4) left floating,
        // a card can answer pixel data that happens to look like SD commands
        // and flip D/C, so the panel takes pixels as commands (inverted,
        // asleep, black until reboot). Colourful frames trigger it, black and
        // white ones never do. Hold the card deselected, as M5Unified does.
        gpio_config_t sd_cs = {};
        sd_cs.pin_bit_mask = 1ULL << GPIO_NUM_4;
        sd_cs.mode = GPIO_MODE_OUTPUT;
        sd_cs.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&sd_cs);
        gpio_set_level(GPIO_NUM_4, 1);

        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_37;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_36;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeIli9342Display() {
        ESP_LOGI(TAG, "Init IlI9342");

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_3;
        io_config.dc_gpio_num = GPIO_NUM_35;
        io_config.spi_mode = 2;
#ifdef CONFIG_STACKCHAN_LCD_PCLK_MHZ
        io_config.pclk_hz = CONFIG_STACKCHAN_LCD_PCLK_MHZ * 1000 * 1000;
#else
        io_config.pclk_hz = 40 * 1000 * 1000;
#endif
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));
#if CONFIG_STACKCHAN_LCD_WEAK_DRIVE
        // The LCD D/C line (GPIO35) runs beside MOSI (37) and SCLK (36).
        // With busy, bright frames the fast edges couple into D/C and the
        // panel latches pixel bytes as commands (dark / flashing screen).
        // The weakest drive strength softens the edges; it is ample for
        // the short on-board traces at the reduced clock.
        for (gpio_num_t pin : {GPIO_NUM_37, GPIO_NUM_36, GPIO_NUM_35, GPIO_NUM_3}) {
            gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_0);
        }
#endif

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        aw9523_->ResetIli9342();

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        lcd_panel_io_ = panel_io;
        lcd_panel_ = panel;
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

     void InitializeCamera() {
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io = {
                [0] = CAMERA_PIN_D0,
                [1] = CAMERA_PIN_D1,
                [2] = CAMERA_PIN_D2,
                [3] = CAMERA_PIN_D3,
                [4] = CAMERA_PIN_D4,
                [5] = CAMERA_PIN_D5,
                [6] = CAMERA_PIN_D6,
                [7] = CAMERA_PIN_D7,
            },
            .vsync_io = CAMERA_PIN_VSYNC,
            .de_io = CAMERA_PIN_HREF,
            .pclk_io = CAMERA_PIN_PCLK,
            .xclk_io = CAMERA_PIN_XCLK,
        };

        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_bus_,
            .freq = 100000,
        };

        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = CAMERA_PIN_RESET,
            .pwdn_pin = CAMERA_PIN_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = XCLK_FREQ_HZ,
        };

        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);
        camera_->SetHMirror(false);
    }

    bool servo_ok_ = false;
    bool rgb_ok_ = false;
    static constexpr uint8_t RGB_LED_COUNT = 12;  // StackChan base has 12 WS2812C
    static constexpr uint8_t RGB_DATA_PIN  = 13;  // PY32 expander pin (not ESP32 GPIO)

    void InitializeIOExpander() {
        ESP_LOGI(TAG, "Init PY32 IO expander (I2C addr 0x%02X)", Py32IoExpander::DEFAULT_ADDR);
        io_expander_ = std::unique_ptr<Py32IoExpander>(new Py32IoExpander(i2c_bus_));

        // PY32 boots slowly and is unreliable in the first few hundred ms
        // after power-on. Retry the probe up to 5 times with 500ms gaps —
        // total budget ~2.5 s, which dominates boot latency by maybe 1.5 s
        // in the worst case but is still well under the time spent on
        // I2C scan + LCD panel init that happen earlier.
        constexpr int kBeginRetries  = 5;
        constexpr int kBeginDelayMs  = 500;
        bool   ok = false;
        uint8_t version = 0;
        int     winning_attempt = 0;
        for (int i = 0; i < kBeginRetries; i++) {
            vTaskDelay(pdMS_TO_TICKS(kBeginDelayMs));
            if (io_expander_->Begin(&version)) {
                ok = true;
                winning_attempt = i + 1;
                break;
            }
            ESP_LOGW(TAG, "PY32 not responding, retry %d/%d", i + 1, kBeginRetries);
        }

        if (!ok) {
            ESP_LOGE(TAG, "PY32 IO expander FAILED after %d attempts; servo will be POWERLESS",
                     kBeginRetries);
            io_expander_.reset();
            return;
        }
        ESP_LOGI(TAG, "PY32 IO expander READY (version=0x%02X, attempt=%d/%d)",
                 version, winning_attempt, kBeginRetries);

        // Pin 0 = VM EN (servo power switch). Output, pull-up, drive HIGH.
        // We track each step so a partial success is reported precisely
        // (e.g. direction set but pull-up failed) — much easier to debug
        // than the previous "all-void, hope it stuck" version.
        bool ok_dir   = io_expander_->SetDirection(0, true);
        bool ok_pull  = io_expander_->SetPullMode(0, true);
        bool ok_write = io_expander_->DigitalWrite(0, true);
        vTaskDelay(pdMS_TO_TICKS(200));

        if (!ok_dir || !ok_pull || !ok_write) {
            const char* failed = "?";
            if (!ok_dir)        failed = "SetDirection";
            else if (!ok_pull)  failed = "SetPullMode";
            else if (!ok_write) failed = "DigitalWrite";
            ESP_LOGE(TAG, "Servo power ENABLE FAILED at step=%s", failed);
            return;
        }

        // Verify by reading back the output low-byte register. Bit 0 must
        // be high. If not, the chip ACK'd but the level didn't latch — log
        // it loudly so we know the next move_head will be silent.
        uint8_t out_low = 0;
        if (io_expander_->ReadOutputLow(&out_low)) {
            if (out_low & 0x01) {
                ESP_LOGI(TAG, "Servo power ENABLED via PY32 pin 0 "
                              "(VM EN HIGH confirmed, REG_GPIO_O_L=0x%02X)", out_low);
            } else {
                ESP_LOGE(TAG, "Servo power write succeeded but readback shows "
                              "pin 0 LOW (REG_GPIO_O_L=0x%02X) — VM EN may be off!",
                              out_low);
            }
        } else {
            // Read failed but writes succeeded; assume the writes took.
            ESP_LOGW(TAG, "Servo power writes OK, but readback verify failed "
                          "(can't confirm VM EN level)");
        }

        // ---- RGB strip init (12x WS2812C on the StackChan base) ----
        // The data line is on PY32 pin 13 (not an ESP32 GPIO); the PY32
        // bit-bangs the WS2812 protocol itself. We just write RGB565 into
        // its LED RAM and toggle the latch bit. Sequence is the same as the
        // M5 BSP: configure pin 13 as push-pull output with pull-up,
        // SetLedCount(12), small settle delay, then clear all LEDs.
        bool ok_d   = io_expander_->SetDirection(RGB_DATA_PIN, true);
        bool ok_p   = io_expander_->SetPullMode(RGB_DATA_PIN, true);
        bool ok_dr  = io_expander_->SetDriveMode(RGB_DATA_PIN, false);
        bool ok_cnt = io_expander_->SetLedCount(RGB_LED_COUNT);
        if (!ok_d || !ok_p || !ok_dr || !ok_cnt) {
            const char* failed = "?";
            if      (!ok_d)   failed = "SetDirection(13)";
            else if (!ok_p)   failed = "SetPullMode(13)";
            else if (!ok_dr) failed = "SetDriveMode(13)";
            else if (!ok_cnt) failed = "SetLedCount";
            ESP_LOGE(TAG, "RGB strip init FAILED at step=%s; LEDs disabled", failed);
            return;
        }
        // M5 reference firmware waits 200 ms after SetLedCount before the
        // first refresh — the PY32 internal LED engine needs the settle.
        vTaskDelay(pdMS_TO_TICKS(200));

        // Clear strip: zero RAM in one burst, then latch.
        uint8_t clear_buf[RGB_LED_COUNT * 2] = {0};
        bool ok_clear = io_expander_->SetLedData(clear_buf, sizeof(clear_buf));
        bool ok_ref   = io_expander_->RefreshLeds();
        if (!ok_clear || !ok_ref) {
            ESP_LOGE(TAG, "RGB strip clear FAILED (data=%d refresh=%d); LEDs disabled",
                     ok_clear, ok_ref);
            return;
        }
        rgb_ok_ = true;
        ESP_LOGI(TAG, "RGB strip READY (%d WS2812C via PY32 pin %d, all cleared)",
                 RGB_LED_COUNT, RGB_DATA_PIN);
    }

    // Helpers for the LED MCP tools below. Centralised so the parsing/
    // clamping logic isn't duplicated in three handlers.
    static uint8_t ClampByte(int v) {
        if (v < 0) return 0;
        if (v > 255) return 255;
        return (uint8_t)v;
    }

    static bool JsonByte(cJSON* item, uint8_t* out) {
        if (!cJSON_IsNumber(item)) return false;
        if (item->valuedouble != static_cast<double>(item->valueint)) return false;
        if (item->valueint < 0 || item->valueint > 255) return false;
        *out = static_cast<uint8_t>(item->valueint);
        return true;
    }

    // Pack one RGB888 sample into the {lo, hi} RGB565 pair the PY32
    // expects in its LED RAM.
    static void PackRgb565(uint8_t r, uint8_t g, uint8_t b, uint8_t out[2]) {
        uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        out[0] = (uint8_t)(v & 0xFF);
        out[1] = (uint8_t)((v >> 8) & 0xFF);
    }

    // 全 RGB LED を同じ色にする helper。 self.led.set_all MCP tool と同じ I2C 経路
    // (PY32 経由 WS2812)。 PollTouchpad のタッチフィードバック等、 MCP 以外の
    // 経路から LED を駆動するときに使う。 PY32 init 失敗時 (rgb_ok_ == false)
    // は no-op で安全に抜ける。
    void SetAllRgbLeds(uint8_t r, uint8_t g, uint8_t b) {
        if (!rgb_ok_ || io_expander_ == nullptr) {
            return;
        }
        uint8_t buf[RGB_LED_COUNT * 2];
        uint8_t pair[2];
        PackRgb565(r, g, b, pair);
        for (int i = 0; i < RGB_LED_COUNT; i++) {
            buf[i * 2 + 0] = pair[0];
            buf[i * 2 + 1] = pair[1];
        }
        if (io_expander_->SetLedData(buf, sizeof(buf))) {
            io_expander_->RefreshLeds();
        }
    }

#if CONFIG_STACKCHAN_SERVO_DYNAMIXEL
    // Stack-chan RT ver. geometry. The board code speaks the official kit's
    // angle convention: yaw -90..90, pitch 0 (down end-stop) .. 88 (up)
    // with 45 as the rest pose. The RT head sits level at XL330 centre
    // (2048), tilts up toward lower raw values and has a narrower range
    // (rt-net's firmware clamps tilt to -30..+10 deg), so pitch is mapped
    // piecewise: 45 -> level, 88 -> full tilt-up, 0 -> full tilt-down.
    // Kit pitch that maps to a level RT head. Fixed at 45 (the kit's
    // standard rest pose) so changing the rest pose does not move "level".
    static constexpr float kKitPitchRest = 45;
    static constexpr float kKitPitchMax = SAFE_PITCH_MAX;

    static float RtYawToDxl(float deg) {
#if CONFIG_STACKCHAN_DXL_YAW_INVERT
        return -deg;
#else
        return deg;
#endif
    }

    static float RtPitchToDxl(float deg) {
        const float up = CONFIG_STACKCHAN_DXL_PITCH_UP_DEG;
        const float down = CONFIG_STACKCHAN_DXL_PITCH_DOWN_DEG;
        float tilt = deg >= kKitPitchRest
            ? -(deg - kKitPitchRest) * up / (kKitPitchMax - kKitPitchRest)
            : (kKitPitchRest - deg) * down / kKitPitchRest;
#if CONFIG_STACKCHAN_DXL_PITCH_INVERT
        tilt = -tilt;
#endif
        return tilt;
    }

    static float DxlPitchToRt(float tilt) {
        const float up = CONFIG_STACKCHAN_DXL_PITCH_UP_DEG;
        const float down = CONFIG_STACKCHAN_DXL_PITCH_DOWN_DEG;
#if CONFIG_STACKCHAN_DXL_PITCH_INVERT
        tilt = -tilt;
#endif
        if (tilt <= 0) {
            return kKitPitchRest - tilt * (kKitPitchMax - kKitPitchRest) / up;
        }
        return down > 0 ? kKitPitchRest - tilt * kKitPitchRest / down : kKitPitchRest;
    }

    void ConfigureDynamixelAxes() {
        DynamixelBus::Axis yaw;
        yaw.id = SERVO_YAW_ID;
        yaw.scs_zero = 460;  // YawDegToPos(0)
        yaw.scs_deg_to_dxl_deg = RtYawToDxl;
        yaw.dxl_deg_to_scs_deg = RtYawToDxl;  // negation is its own inverse
        yaw.goal_current_ma = CONFIG_STACKCHAN_DXL_YAW_GOAL_CURRENT_MA;
        scs_bus_.ConfigureAxis(yaw);

        DynamixelBus::Axis pitch;
        pitch.id = SERVO_PITCH_ID;
        pitch.scs_zero = 620;  // PitchDegToPos(0)
        pitch.scs_deg_to_dxl_deg = RtPitchToDxl;
        pitch.dxl_deg_to_scs_deg = DxlPitchToRt;
        pitch.goal_current_ma = CONFIG_STACKCHAN_DXL_PITCH_GOAL_CURRENT_MA;
        scs_bus_.ConfigureAxis(pitch);
    }
#endif

    void InitializeServo() {
        ESP_LOGI(TAG, "Init SCS0009 servo bus (UART%d, baud=%d, tx=%d, rx=%d)",
                 SERVO_UART_NUM, SERVO_BAUDRATE, SERVO_TX_PIN, SERVO_RX_PIN);
#if CONFIG_STACKCHAN_SERVO_DYNAMIXEL
        ConfigureDynamixelAxes();
        // DynamixelBus::begin() returns void and aborts on UART errors,
        // like FeetechScs::begin() below.
        scs_bus_.begin(SERVO_UART_NUM, SERVO_BAUDRATE, SERVO_TX_PIN, SERVO_RX_PIN);
        servo_ok_ = true;
#elif CONFIG_STACKCHAN_SERVO_FEETECH
        // FeetechScs::begin() returns void and uses ESP_ERROR_CHECK internally,
        // so a UART configuration error aborts the boot rather than reporting
        // false. If begin() returns to us, init succeeded.
        scs_bus_.begin(SERVO_UART_NUM, SERVO_BAUDRATE, SERVO_TX_PIN, SERVO_RX_PIN);
        servo_ok_ = true;
#else
        // SCServo_lib SCSCL::begin() returns bool — false if UART setup failed.
        servo_ok_ = scs_bus_.begin(SERVO_UART_NUM, SERVO_BAUDRATE, SERVO_TX_PIN, SERVO_RX_PIN);
#endif
        // ACK reading is enabled (SCS::Level defaults to 1). genWrite() will
        // wait for the SCS0009's 6-byte ACK packet before returning, which
        // implicitly enforces an inter-frame gap and prevents a follow-up
        // WritePos from colliding with a still-processing servo. This aligns
        // with the M5 StackChan official BSP behaviour (which never touches
        // Level). Was: scs_bus_.Level = 0 — turned out to silently drop
        // every WritePos after the first one ("starts moving once, then
        // never again" symptom).
        ESP_LOGI(TAG, "Servo bus init: %s (Level=1, ACK enabled)", servo_ok_ ? "OK" : "FAILED");

        if (servo_ok_) {
            motion_mutex_ = xSemaphoreCreateMutex();
            scs_bus_mutex_ = xSemaphoreCreateMutex();
            if (motion_mutex_ == nullptr || scs_bus_mutex_ == nullptr) {
                ESP_LOGE(TAG, "Failed to create servo mutexes: motion=%p scs_bus=%p; disabling servo",
                         motion_mutex_, scs_bus_mutex_);
                if (motion_mutex_ != nullptr) {
                    vSemaphoreDelete(motion_mutex_);
                    motion_mutex_ = nullptr;
                }
                if (scs_bus_mutex_ != nullptr) {
                    vSemaphoreDelete(scs_bus_mutex_);
                    scs_bus_mutex_ = nullptr;
                }
                servo_ok_ = false;
                return;
            }

#if CONFIG_STACKCHAN_SERVO_DELEGATED_MOTION
            motion_driver_ = std::make_unique<ServoDelegatedMotionDriver>(
                scs_bus_, scs_bus_mutex_, motion_mutex_,
                yaw_motion_, pitch_motion_);
#else
            motion_driver_ = std::make_unique<HostInterpolationMotionDriver>(
                scs_bus_, scs_bus_mutex_, motion_mutex_,
                yaw_motion_, pitch_motion_);
#endif
            if (!motion_driver_->Initialize()) {
                ESP_LOGE(TAG, "Failed to initialize motion driver; disabling servo");
                motion_driver_.reset();
                vSemaphoreDelete(motion_mutex_);
                motion_mutex_ = nullptr;
                vSemaphoreDelete(scs_bus_mutex_);
                scs_bus_mutex_ = nullptr;
                servo_ok_ = false;
                return;
            }

            // Issue #121 (Problem 1, "downward drop on power-on") + #123
            // (boot-init diagnostics).
            //
            // Background: the SCS0009 retains its commanded set-point
            // across power cycles (Hypothesis 1 in #121, confirmed by
            // the firmware-v1.4.1 clean-install reproduction -- after a
            // full NVS reset on the ESP32 side, the boot pre-init
            // `ReadPos` still matched the pre-power-off pose exactly,
            // demonstrating the set-point lives in the servo itself,
            // not in firmware-side NVS). When VM_EN asserts at boot,
            // the servo restores torque and snaps toward that retained
            // target before any firmware-side speed limiting can apply
            // -- audible as a mechanical end-stop impact when the
            // previous session ended near pitch=0, and visible as a
            // downward drop in general.
            //
            // The mitigation is a `WritePos(id, current_pos, time=0,
            // speed=0)` per servo, which the SCS0009 treats as a new
            // target equal to its current position. This interrupts any
            // in-progress snap motion and leaves the servo stationary
            // at the raw position the immediately-preceding `ReadPos`
            // observed, until the subsequent interpolating boot-init
            // climb begins.
            //
            // Efficacy depends on the `ReadPos` + `WritePos` pair
            // completing while the servo is still mid-snap. To minimize
            // that window, pitch (the only axis that exhibits the
            // downward drop) is read AND held BEFORE the yaw axis is
            // touched on the SCS0009 bus -- any yaw `ReadPos` /
            // `WritePos` / ACK wait interposed between the pitch
            // `ReadPos` and pitch `WritePos` would widen the window
            // and risk the snap completing into an end-stop before the
            // pitch hold reaches the servo. The unified "Boot pre-init
            // ReadPos" diagnostic line (#123) is emitted after both
            // holds because it is purely informational and not on the
            // timing-critical path. If a `ReadPos` value lands close
            // to a previous session's commanded target (e.g. raw pitch
            // near 620 for pitch=0 deg), the snap was likely still in
            // progress and the hold is expected to truncate it; if a
            // `ReadPos` value is already at an end-stop or fails, the
            // hold for that boot is a no-op or skipped and a deeper
            // fix (e.g. firmware-controlled VM_EN sequencing through
            // the PY32 IO-expander) would be required, tracked
            // separately.

            // Phase 1a: pitch first -- read and immediately hold.
            //
            // Issue #138: retry ReadPos to absorb the SCS0009 ~200 ms
            // startup latency observed after VM_EN HIGH on the PMIC
            // long-press OFF/ON path. Without retry, the first ReadPos
            // (measured at around tick 140 on this hardware) typically
            // lands inside the wake-up window and returns -1, causing
            // the snap-suppress hold below to skip on exactly the path
            // #121 Problem 1 targets. Budget: 5 attempts × 50 ms = 250 ms
            // total per axis, well above the observed ~200 ms latency.
            // This is distinct from the SCS0009 bus hang (#100), which a
            // fixed retry budget cannot clear; in that case all attempts
            // fail and the safe-fallback branch in Phase 2 below seeds
            // pitch_motion_.current_deg with BOOT_INIT_PITCH_DEG to avoid
            // an end-stop walk during the subsequent boot-init
            // `WriteHeadAngles` interpolation.
            // Loop form mirrors the `get_head_angles` MCP-tool retry below
            // (set attempts to `i + 1` inside the loop so the final value
            // equals the number of attempts actually made, even when all
            // retries fail). The previous `for (attempts = 1; attempts <=
            // MAX; ++attempts)` form left attempts at MAX+1 on failure
            // and made the diagnostic log overstate the attempt count.
            constexpr int BOOT_READPOS_MAX_ATTEMPTS = 5;
            constexpr uint32_t BOOT_READPOS_RETRY_MS = 50;
            int pitch_pos_actual = -1;
            int pitch_attempts = 0;
            for (int i = 0; i < BOOT_READPOS_MAX_ATTEMPTS; ++i) {
                pitch_attempts = i + 1;
                pitch_pos_actual = scs_bus_.ReadPos(SERVO_PITCH_ID);
                if (pitch_pos_actual >= 0) break;
                if (i + 1 < BOOT_READPOS_MAX_ATTEMPTS) {
                    vTaskDelay(pdMS_TO_TICKS(BOOT_READPOS_RETRY_MS));
                }
            }
            if (pitch_pos_actual >= 0) {
                // Bound the snap-suppress hold to the SAFE_PITCH_MIN..
                // SAFE_PITCH_MAX range applied at every other pitch
                // servo-write boundary in this file (see PitchDegToPos
                // and the Phase 2 restored_pitch clamp below). If the
                // boot `ReadPos` lands outside that range -- for example
                // because the servo bus came back online holding a
                // previous session's out-of-range set-point, or the
                // head was hand-pushed beyond an end-stop -- writing
                // the raw position back would bypass that safety
                // boundary and pin the servo against the stall current
                // it is held there from. In that case skip the hold
                // and let the subsequent interpolating boot-init climb
                // to (yaw=0, pitch=45) drive the head back into the
                // safe range through the existing speed-limited path.
                constexpr int PITCH_SAFE_RAW_MIN =
                    620 + SAFE_PITCH_MIN * 16 / 5;  // raw 620 at deg=0
                constexpr int PITCH_SAFE_RAW_MAX =
                    620 + SAFE_PITCH_MAX * 16 / 5;  // raw 901 at deg=88
                if (pitch_pos_actual >= PITCH_SAFE_RAW_MIN &&
                    pitch_pos_actual <= PITCH_SAFE_RAW_MAX) {
                    int pitch_hold_r = scs_bus_.WritePos(
                        SERVO_PITCH_ID, pitch_pos_actual, 0, 0);
                    ESP_LOGI(TAG,
                             "Boot snap-suppress pitch hold(pos=%d): r=%d",
                             pitch_pos_actual, pitch_hold_r);
                } else {
                    ESP_LOGW(TAG,
                             "Boot snap-suppress pitch skipped: ReadPos=%d outside safe raw range [%d, %d]; relying on boot-init climb",
                             pitch_pos_actual,
                             PITCH_SAFE_RAW_MIN, PITCH_SAFE_RAW_MAX);
                }
            }

            // Phase 1b: yaw second -- no analogous snap-into-end-stop
            // failure mode, so timing is not critical. Retry budget
            // matches pitch (Issue #138) for symmetry; in practice yaw
            // typically succeeds on the first attempt because the pitch
            // retries above have already consumed the SCS0009 startup-
            // latency window on the shared bus.
            int yaw_pos_actual = -1;
            int yaw_attempts = 0;
            for (int i = 0; i < BOOT_READPOS_MAX_ATTEMPTS; ++i) {
                yaw_attempts = i + 1;
                yaw_pos_actual = scs_bus_.ReadPos(SERVO_YAW_ID);
                if (yaw_pos_actual >= 0) break;
                if (i + 1 < BOOT_READPOS_MAX_ATTEMPTS) {
                    vTaskDelay(pdMS_TO_TICKS(BOOT_READPOS_RETRY_MS));
                }
            }
            if (yaw_pos_actual >= 0) {
                int yaw_hold_r = scs_bus_.WritePos(
                    SERVO_YAW_ID, yaw_pos_actual, 0, 0);
                ESP_LOGI(TAG,
                         "Boot snap-suppress yaw hold(pos=%d): r=%d",
                         yaw_pos_actual, yaw_hold_r);
            }

            // Phase 1c (diagnostic, #123): unified pre-init ReadPos log
            // with tick timestamp. Off the timing-critical path
            // intentionally; ServoTask has not been created yet, so no
            // `scs_bus_mutex_` contention is possible at this point.
            ESP_LOGI(TAG,
                     "Boot pre-init ReadPos: yaw_raw=%d (attempts=%d) "
                     "pitch_raw=%d (attempts=%d) tick=%u",
                     yaw_pos_actual, yaw_attempts,
                     pitch_pos_actual, pitch_attempts,
                     (unsigned)xTaskGetTickCount());

            // Phase 2: software-side current_deg restore. Order does not
            // affect the SCS0009 bus -- these only update firmware-side
            // motion state for the upcoming interpolating boot-init
            // climb.
            if (yaw_pos_actual >= 0) {
                yaw_motion_.current_deg = (yaw_pos_actual - 460) * 5 / 16;
                ESP_LOGI(TAG, "Restored yaw_motion_.current_deg=%d from ReadPos=%d",
                         yaw_motion_.current_deg, yaw_pos_actual);
            } else {
                // Issue #138: seed yaw current_deg to BOOT_INIT_YAW_DEG
                // rather than leaving the struct default. The default
                // happens to be 0 (== BOOT_INIT_YAW_DEG today), so this
                // is a no-op assignment in current numerical terms; the
                // explicit form keeps intent visible and propagates any
                // future change to BOOT_INIT_YAW_DEG.
                yaw_motion_.current_deg = BOOT_INIT_YAW_DEG;
                ESP_LOGW(TAG,
                         "Failed to ReadPos(yaw) after %d attempts; "
                         "seeded current_deg=%d (BOOT_INIT_YAW_DEG)",
                         BOOT_READPOS_MAX_ATTEMPTS, BOOT_INIT_YAW_DEG);
            }
            if (pitch_pos_actual >= 0) {
                int restored_pitch = (pitch_pos_actual - 620) * 5 / 16;
                // Issue #80: if the device booted with the head physically
                // pushed below the safe range (e.g. previous unsafe firmware
                // or manual handling), don't carry that negative starting
                // angle into motion interpolation — clamp before storing so
                // subsequent interpolation runs only over safe positions.
                if (restored_pitch < SAFE_PITCH_MIN) restored_pitch = SAFE_PITCH_MIN;
                if (restored_pitch > SAFE_PITCH_MAX) restored_pitch = SAFE_PITCH_MAX;
                pitch_motion_.current_deg = restored_pitch;
                ESP_LOGI(TAG, "Restored pitch_motion_.current_deg=%d from ReadPos=%d (clamped to safe range %d..%d)",
                         pitch_motion_.current_deg, pitch_pos_actual, SAFE_PITCH_MIN, SAFE_PITCH_MAX);
            } else {
                // Issue #138: seed pitch current_deg to BOOT_INIT_PITCH_DEG.
                // Without this, the boot-init `WriteHeadAngles(0, 45, 4000)`
                // interpolation below would start from the struct-default
                // `current_deg=0` (== pos=620 at deg=0, the lower mechanical
                // end-stop) and walk WritePos calls upward (pos=620, 623,
                // 626, ...) through end-stop-adjacent positions before
                // reaching the target -- risking servo bus degradation if
                // the SCS0009 wakes up mid-sequence. Seeding to
                // BOOT_INIT_PITCH_DEG makes the subsequent interpolation a
                // near-no-op (start_deg == target_deg == 45) which keeps
                // the servo away from end-stop territory throughout the
                // wake-up window. This is the firmware-side counterpart to
                // the Phase 1a ReadPos retry: retry absorbs the typical
                // wake-up case so the snap-suppress hold can fire; this
                // seed handles the residual case where wake-up exceeds the
                // retry budget or the bus is genuinely hung (#100).
                pitch_motion_.current_deg = BOOT_INIT_PITCH_DEG;
                ESP_LOGW(TAG,
                         "Failed to ReadPos(pitch) after %d attempts; "
                         "seeded current_deg=%d (BOOT_INIT_PITCH_DEG) "
                         "to avoid end-stop walk during boot-init climb",
                         BOOT_READPOS_MAX_ATTEMPTS, BOOT_INIT_PITCH_DEG);
            }

            BaseType_t ok = xTaskCreate(&StackChanBoard::ServoTaskTrampoline,
                                        "servo_motion", 4096, this, 5,
                                        &servo_task_handle_);
            if (ok != pdPASS) {
                ESP_LOGE(TAG, "Failed to create servo_motion task; disabling servo");
                if (motion_mutex_ != nullptr) {
                    vSemaphoreDelete(motion_mutex_);
                    motion_mutex_ = nullptr;
                }
                if (scs_bus_mutex_ != nullptr) {
                    vSemaphoreDelete(scs_bus_mutex_);
                    scs_bus_mutex_ = nullptr;
                }
                motion_driver_.reset();
                servo_task_handle_ = nullptr;
                servo_ok_ = false;
                return;
            }

            // Issue #115: boot-time initialization to a fall-safe neutral
            // pose. Without this, the head retains whatever angle it was
            // left at on power-down — including end-stop positions (e.g.
            // pitch=0) that trigger the SCS0009 bus-hang documented in
            // #100 on the very first user-driven motion. By the time any
            // MCP command can arrive, we want the head already moved to
            // the center of the M5Stack-recommended 5..85° pitch range,
            // well clear of both mechanical end-stops.
            //
            // Design follows the goHome() pattern in m5stack/StackChan
            // (apps/app_setup/workers/servo.cpp:144) where the setup
            // app calls motion.goHome(speed) at boot, and the 1-second
            // positioning timing established in mongonta0716/stackchan-
            // arduino attachServos(). 1000ms move via the existing
            // interpolating WriteHeadAngles path keeps frame-rate stall
            // currents low; the extra 100ms vTaskDelay covers servo
            // settling before any subsequent motion can arrive.
            //
            // Implements #99 Option C and the boot-init aspect of #100
            // direction E. Existing pitch guards (#80 / #98 / #109)
            // continue to apply unchanged.
            // BOOT_INIT_YAW_DEG / BOOT_INIT_PITCH_DEG / BOOT_INIT_MOVE_MS
            // are class-level static constexpr; see the comment block at
            // their declaration above the YawDegToPos helper for the full
            // rationale (Issue #115 target pose, Issue #121 Problem 2
            // slower climb, Issue #138 promotion to class scope for the
            // safe-fallback seed in Phase 2 above).
            // Compute Phase 0 duration from the current_deg → target
            // deltas at BOOT_INIT_TARGET_DEG_PER_SEC, floored at
            // BOOT_INIT_MOVE_MS to keep the SCS0009 wake-up latency
            // window covered on the PMIC OFF/ON path (where Phase 0
            // is a no-op of effect but the BOOT_INIT_MOVE_MS budget
            // still needs to elapse before Phase 0' ReadPos). Without
            // this calculation, a yaw-90° (or any large pre-power-off
            // angle) prior set-point would run the boot-init yaw
            // motion at 30 deg/s+, exceeding the 15 deg/s cap.
            int phase0_yaw_delta;
            int phase0_pitch_delta;
            phase0_yaw_delta =
                BOOT_INIT_YAW_DEG - static_cast<int>(motion_driver_->GetYawDeg());
            phase0_pitch_delta =
                BOOT_INIT_PITCH_DEG - static_cast<int>(motion_driver_->GetPitchDeg());
            if (phase0_yaw_delta < 0) phase0_yaw_delta = -phase0_yaw_delta;
            if (phase0_pitch_delta < 0) phase0_pitch_delta = -phase0_pitch_delta;
            int phase0_max_delta = phase0_yaw_delta > phase0_pitch_delta
                ? phase0_yaw_delta : phase0_pitch_delta;
            uint32_t phase0_duration_ms =
                (uint32_t)phase0_max_delta * 1000U /
                BOOT_INIT_TARGET_DEG_PER_SEC;
            if (phase0_duration_ms < BOOT_INIT_MOVE_MS) {
                phase0_duration_ms = BOOT_INIT_MOVE_MS;
            }
            TickType_t boot_init_start_tick = xTaskGetTickCount();
            WriteHeadAngles(BOOT_INIT_YAW_DEG, BOOT_INIT_PITCH_DEG,
                            phase0_duration_ms,
                            /* prefer_linear = */ true);
            // Two-phase boot-init wait:
            //
            // (1) Mandatory minimum: vTaskDelay until
            //     phase0_duration_ms + 100 ms has elapsed. This must
            //     run UNCONDITIONALLY — independent of IsMoving() —
            //     because phase0_duration_ms is floored to
            //     BOOT_INIT_MOVE_MS specifically to cover the
            //     SCS0009 wake-up window on the PMIC OFF/ON path,
            //     even when WriteHeadAngles is a no-op (Phase 2
            //     safe-fallback seeded current_deg to exactly the
            //     boot target → motion_driver_->IsMoving() == false
            //     immediately → Phase 0' ReadPos would otherwise run
            //     inside the wake-up latency window and exhaust).
            //
            // (2) Optional extension: while motion_driver_->IsMoving()
            //     reports a motion still in flight, keep waiting up
            //     to a safety deadline. This covers the ServoDelegated
            //     path where the actual servo motion can start late
            //     due to dispatch retry latency (max 5 ×
            //     MOTION_POLL_INTERVAL_MS = 250 ms) — phase (1)'s
            //     budget may finish before the servo has completed
            //     its internal interpolation in that worst case.
            TickType_t boot_init_min_deadline =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(phase0_duration_ms + 100);
            // Safety extension covers the worst-case ServoDelegated
            // dispatch latency. Each retry round dispatches yaw and
            // pitch sequentially under scs_bus_mutex_, so one fully-
            // timing-out round on both axes costs approximately:
            //   MOTION_POLL_INTERVAL_MS (50 ms tick wake)
            // + yaw WritePos ACK timeout (~100 ms; SCSCL's
            //   SCSerial::IOTimeOut = 100 ms, FeetechScs comparable)
            // + kInterFrameGap (10 ms)
            // + pitch WritePos ACK timeout (~100 ms)
            // ≈ 260 ms per retry round.
            //
            // 4 failing rounds + 1 successful round bound the
            // worst-case dispatch latency at roughly 4 × 260 ≈ 1040 ms.
            // Add a settle margin so the wait outlasts a genuine
            // delayed dispatch instead of breaking while IsMoving()
            // is legitimately true. 2000 ms covers the full retry
            // budget plus margin.
            //
            // HostInterpolation path is unaffected (dispatch latency
            // is 0 there; the wait exits well before this deadline
            // regardless).
            TickType_t boot_init_safety_deadline =
                boot_init_min_deadline + pdMS_TO_TICKS(2000);
            while ((int32_t)(xTaskGetTickCount() -
                             boot_init_min_deadline) < 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            while (motion_driver_->IsMoving()) {
                if ((int32_t)(xTaskGetTickCount() -
                              boot_init_safety_deadline) >= 0) {
                    ESP_LOGW(TAG,
                             "Boot init Phase 0 wait safety deadline elapsed while motion_driver_ still reports moving; proceeding to Phase 0' ReadPos anyway");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // Issue #123: capture post-init ReadPos so the boot-init effect
            // is observable in the serial log. ServoTask is now running, so
            // hold scs_bus_mutex_ across the ReadPos pair.
            //
            // Retry budget mirrors Phase 1a / 1b and the get_head_angles
            // MCP-tool retry. A transient `ReadPos == -1` on a healthy
            // servo would otherwise cause Phase 0' re-sync to skip,
            // leaving current_deg slightly stale for the session.
            int post_yaw_pos = -1;
            int post_pitch_pos = -1;
            int post_yaw_attempts = 0;
            int post_pitch_attempts = 0;
            xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
            for (int i = 0; i < BOOT_READPOS_MAX_ATTEMPTS; ++i) {
                post_yaw_attempts = i + 1;
                post_yaw_pos = scs_bus_.ReadPos(SERVO_YAW_ID);
                if (post_yaw_pos >= 0) break;
                if (i + 1 < BOOT_READPOS_MAX_ATTEMPTS) {
                    vTaskDelay(pdMS_TO_TICKS(BOOT_READPOS_RETRY_MS));
                }
            }
            for (int i = 0; i < BOOT_READPOS_MAX_ATTEMPTS; ++i) {
                post_pitch_attempts = i + 1;
                post_pitch_pos = scs_bus_.ReadPos(SERVO_PITCH_ID);
                if (post_pitch_pos >= 0) break;
                if (i + 1 < BOOT_READPOS_MAX_ATTEMPTS) {
                    vTaskDelay(pdMS_TO_TICKS(BOOT_READPOS_RETRY_MS));
                }
            }
            xSemaphoreGive(scs_bus_mutex_);
            TickType_t boot_init_end_tick = xTaskGetTickCount();
            ESP_LOGI(TAG,
                     "Boot-time servo init complete: target yaw=%d pitch=%d "
                     "(move=%ums), post-ReadPos: yaw_raw=%d (attempts=%d) "
                     "pitch_raw=%d (attempts=%d), elapsed_ms=%u",
                     BOOT_INIT_YAW_DEG, BOOT_INIT_PITCH_DEG,
                     (unsigned)phase0_duration_ms,
                     post_yaw_pos, post_yaw_attempts,
                     post_pitch_pos, post_pitch_attempts,
                     (unsigned)((boot_init_end_tick - boot_init_start_tick) *
                                portTICK_PERIOD_MS));

            // Phase 0': mandatory current_deg re-sync from post-init
            // ReadPos before boot-time servo initialization completes.
            //
            // Background: on the PMIC long-press OFF / ON path, Phase 1
            // ReadPos retries can exhaust the budget while the SCS0009
            // is still in its wake-up latency window; Phase 2 then
            // seeds current_deg with BOOT_INIT_*_DEG so the Phase 0
            // interpolation is a no-op of effect. By the time the
            // BOOT_INIT_MOVE_MS-long vTaskDelay above has elapsed,
            // the SCS0009 has been powered for several seconds and a
            // ReadPos here is almost certain to succeed (the
            // observed boot log shows yaw_raw / pitch_raw populated
            // at tick ≥ ~6 s).
            //
            // Re-syncing current_deg with the actual physical position
            // now keeps the next move_head interpolation anchored to
            // where the servo really is, rather than to the Phase 2
            // safe-fallback seed.
            //
            // If a post-init ReadPos still fails, leave current_deg as
            // restored-or-seeded by Phase 2. That is safer than issuing
            // additional boot-time WritePos commands on a degraded bus.
            if (post_yaw_pos >= 0) {
                int actual_yaw_deg = (post_yaw_pos - 460) * 5 / 16;
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                if (yaw_motion_.current_deg != actual_yaw_deg) {
                    int prev_yaw_deg = yaw_motion_.current_deg;
                    yaw_motion_.current_deg = actual_yaw_deg;
                    yaw_motion_.start_deg = actual_yaw_deg;
                    yaw_motion_.target_deg = actual_yaw_deg;
                    yaw_motion_.moving = false;
                    yaw_motion_.move_start_ms =
                        (uint32_t)(boot_init_end_tick * portTICK_PERIOD_MS);
                    // Bump the driver's request_token so any Tick()
                    // snapshot taken before this re-sync no longer
                    // passes the post-bus freshness guard and does
                    // not overwrite the just-re-synced state. The
                    // delegated driver also clears its per-axis
                    // private cancellation state under the same
                    // motion_mutex_ hold.
                    motion_driver_->InvalidateAxisToken(SERVO_YAW_ID);
                    xSemaphoreGive(motion_mutex_);
                    ESP_LOGI(TAG,
                             "Phase 0' yaw re-sync: current_deg %d -> %d "
                             "(actual ReadPos=%d)",
                             prev_yaw_deg, actual_yaw_deg, post_yaw_pos);
                } else {
                    xSemaphoreGive(motion_mutex_);
                }
            } else {
                // Phase 0' ReadPos failed: current_deg holds the
                // Phase 2 restored-or-seeded value, which has NOT
                // been physically verified. Mark the axis position
                // unknown so the ServoDelegated path's no-op gate
                // does not silently skip a same-target recovery
                // command. The HostInterpolation path ignores this
                // flag; on that path the next WriteHeadAngles still
                // dispatches a fresh interpolation as before.
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                yaw_motion_.position_unknown = true;
                // Token/state invalidation covers the position_unknown
                // mutation using the same external-reset boundary as
                // the success branch above.
                motion_driver_->InvalidateAxisToken(SERVO_YAW_ID);
                xSemaphoreGive(motion_mutex_);
                ESP_LOGW(TAG,
                         "Phase 0' yaw re-sync skipped: ReadPos failed; "
                         "leaving current_deg at restored-or-seeded value "
                         "and marking position unknown for delegated no-op gate");
            }
            if (post_pitch_pos >= 0) {
                int actual_pitch_deg = (post_pitch_pos - 620) * 5 / 16;
                if (actual_pitch_deg < SAFE_PITCH_MIN) actual_pitch_deg = SAFE_PITCH_MIN;
                if (actual_pitch_deg > SAFE_PITCH_MAX) actual_pitch_deg = SAFE_PITCH_MAX;
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                if (pitch_motion_.current_deg != actual_pitch_deg) {
                    int prev_pitch_deg = pitch_motion_.current_deg;
                    pitch_motion_.current_deg = actual_pitch_deg;
                    pitch_motion_.start_deg = actual_pitch_deg;
                    pitch_motion_.target_deg = actual_pitch_deg;
                    pitch_motion_.moving = false;
                    pitch_motion_.move_start_ms =
                        (uint32_t)(boot_init_end_tick * portTICK_PERIOD_MS);
                    // Invalidate the driver's token/state boundary
                    // (see the yaw branch above for the rationale).
                    motion_driver_->InvalidateAxisToken(SERVO_PITCH_ID);
                    xSemaphoreGive(motion_mutex_);
                    ESP_LOGI(TAG,
                             "Phase 0' pitch re-sync: current_deg %d -> %d "
                             "(actual ReadPos=%d, clamped to safe range %d..%d)",
                             prev_pitch_deg, actual_pitch_deg, post_pitch_pos,
                             SAFE_PITCH_MIN, SAFE_PITCH_MAX);
                } else {
                    xSemaphoreGive(motion_mutex_);
                }
            } else {
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                pitch_motion_.position_unknown = true;
                // Invalidate the driver's token/state boundary for the
                // position_unknown mutation (see the yaw fail branch
                // above).
                motion_driver_->InvalidateAxisToken(SERVO_PITCH_ID);
                xSemaphoreGive(motion_mutex_);
                ESP_LOGW(TAG,
                         "Phase 0' pitch re-sync skipped: ReadPos failed; "
                         "leaving current_deg at restored-or-seeded value "
                         "and marking position unknown for delegated no-op gate");
            }
            boot_init_done_.store(true, std::memory_order_release);
        }
    }

    ServoTorqueResult InternalSetServoTorque(bool yaw_enabled,
                                             bool pitch_enabled,
                                             ReleaseReason reason,
                                             uint32_t expected_release_epoch =
                                                 0) {
        ServoTorqueResult result;
        const bool disables_axis = !yaw_enabled || !pitch_enabled;

        auto update_bus_ok = [&]() {
#if CONFIG_STACKCHAN_SERVO_FEETECH || CONFIG_STACKCHAN_SERVO_DYNAMIXEL
            result.yaw_ok = (result.yaw_bus_return == 0);
            result.pitch_ok = (result.pitch_bus_return == 0);
#else
            result.yaw_ok = (result.yaw_bus_return > 0);
            result.pitch_ok = (result.pitch_bus_return > 0);
#endif
        };

        // Issue #171: classify each exit path with a single 3-valued tag so
        // that idempotent_short_circuit and wait_exhausted can never both be
        // set. A one-bool flag could not express the two orthogonal outcomes;
        // a single enum makes "both true" structurally unrepresentable.
        //   * kBusAction:     a real EnableTorque() bus write was attempted
        //                     (or the servo subsystem was unavailable); the
        //                     outcome is carried by yaw_ok/pitch_ok. Neither
        //                     short-circuit flag is set.
        //   * kIdempotent:    returned without a bus frame, state already
        //                     matched the request (success no-op).
        //   * kWaitExhausted: returned without a bus frame, the kReleasing
        //                     wait budget was exhausted (failure).
        enum class ExitKind { kBusAction, kIdempotent, kWaitExhausted };

        auto log_result = [&](ExitKind kind) {
            result.idempotent_short_circuit = (kind == ExitKind::kIdempotent);
            result.wait_exhausted = (kind == ExitKind::kWaitExhausted);
            // Defensive: the enum makes this impossible, but assert anyway so
            // any future direct field mutation is caught in debug builds.
            assert(!(result.idempotent_short_circuit && result.wait_exhausted));
            ESP_LOGI(TAG,
                     "set_servo_torque (reason=%s): servo_ok=%d "
                     "yaw_enabled=%d (r=%d) pitch_enabled=%d (r=%d) "
                     "idempotent_short_circuit=%d wait_exhausted=%d",
                     ReleaseReasonName(reason),
                     servo_ok_ ? 1 : 0,
                     yaw_enabled ? 1 : 0, result.yaw_bus_return,
                     pitch_enabled ? 1 : 0, result.pitch_bus_return,
                     result.idempotent_short_circuit ? 1 : 0,
                     result.wait_exhausted ? 1 : 0);
        };

        auto finish = [&](ExitKind kind) -> ServoTorqueResult {
            log_result(kind);
            return result;
        };

        auto publish_after_bus_attempt = [&](TorqueState pre_bus_state) {
            const bool any_axis_bus_failed =
                !result.yaw_ok || !result.pitch_ok;
            if (!any_axis_bus_failed) {
                // Success-path cached-state ownership is pre-existing and
                // tracked separately under Issue #172.
                PublishTorqueState();
                return;
            }
            TorqueState expected = pre_bus_state;
            if (torque_state_.compare_exchange_strong(
                    expected,
                    TorqueState::kUncertain,
                    std::memory_order_release,
                    std::memory_order_acquire)) {
                ESP_LOGW(TAG,
                         "set_servo_torque (reason=%s): publishing kUncertain; "
                         "bus confirmation failed for yaw_failed=%d (r=%d) "
                         "pitch_failed=%d (r=%d)",
                         ReleaseReasonName(reason),
                         result.yaw_ok ? 0 : 1, result.yaw_bus_return,
                         result.pitch_ok ? 0 : 1, result.pitch_bus_return);
            } else {
                ESP_LOGW(TAG,
                         "set_servo_torque (reason=%s): kUncertain publish "
                         "skipped; torque_state_ advanced from %d to %d "
                         "(likely MarkReleasing()); leaving concurrent state "
                         "intact. bus_return yaw=%d pitch=%d",
                         ReleaseReasonName(reason),
                         static_cast<int>(pre_bus_state),
                         static_cast<int>(expected),
                         result.yaw_bus_return, result.pitch_bus_return);
            }
        };

        if (!servo_ok_ || scs_bus_mutex_ == nullptr) {
            // Servo subsystem unavailable: not a short-circuit and not a
            // wait timeout. yaw_ok/pitch_ok stay false, so ok is false.
            return finish(ExitKind::kBusAction);
        }

        // Fully-symmetric re-engage remains bus-ordered even when it becomes
        // a no-op. If an auto-idle OFF has already published kReleasing, the
        // manual path waits for that pending transition before entering the
        // bus section.
        if (yaw_enabled && pitch_enabled) {
            // Pre-mutex wait: this reduces obvious contention before
            // attempting scs_bus_mutex_. The post-mutex re-check below closes
            // the pre-check-to-mutex TOCTOU window.
            if (reason == ReleaseReason::kManual) {
                auto state_pre =
                    torque_state_.load(std::memory_order_acquire);
                if (state_pre == TorqueState::kReleasing) {
                    if (!WaitForKReleasingToClear()) {
                        ESP_LOGW(TAG,
                                 "set_servo_torque (reason=%s): kReleasing "
                                 "not clearing within wait budget; skipping "
                                 "bus frames, caller may retry.",
                                 ReleaseReasonName(reason));
                        // Pre-mutex wait budget exhausted: no bus frame went
                        // out, the requested ON did not happen (Issue #171).
                        log_result(ExitKind::kWaitExhausted);
                        return result;
                    }
                    // After the wait, state may be kEngaged (OFF rolled
                    // back), kReleased (OFF succeeded), or kUncertain (OFF
                    // bus confirmation failed). Fall through to the existing
                    // (true, true) logic, which short-circuits on kEngaged and
                    // proceeds normally on kReleased/kPartial/kUncertain.
                }
            }

            // Bounded retry for the remaining TOCTOU window: torque_state_
            // can flip to kReleasing between the pre-mutex check and
            // xSemaphoreTake(). Once the bus mutex is held, re-check; if an
            // auto-release OFF was published in that gap, release the mutex,
            // wait, and retry. kReengagement is exempt because
            // EnsureTorqueEngagedBeforeMove() already waited; kAutoIdle does
            // not enter this (true, true) branch.
            for (int attempt = 0; attempt < kMaxManualReengageRetries;
                 ++attempt) {
                xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);

                if (reason == ReleaseReason::kManual &&
                    torque_state_.load(std::memory_order_acquire) ==
                        TorqueState::kReleasing) {
                    xSemaphoreGive(scs_bus_mutex_);
                    if (!WaitForKReleasingToClear()) {
                        ESP_LOGW(TAG,
                                 "set_servo_torque (reason=%s): kReleasing "
                                 "persists after attempt %d; skipping bus "
                                 "frames, caller may retry.",
                                 ReleaseReasonName(reason),
                                 attempt);
                        // Post-mutex re-check wait budget exhausted: no bus
                        // frame, requested ON did not happen (Issue #171).
                        log_result(ExitKind::kWaitExhausted);
                        return result;
                    }
                    continue;
                }

                if (torque_state_.load(std::memory_order_acquire) ==
                    TorqueState::kEngaged) {
                    // Already engaged (reached here only after any kReleasing
                    // wait already cleared): legitimate no-op success, so ok
                    // stays true (Issue #171).
                    log_result(ExitKind::kIdempotent);
                    xSemaphoreGive(scs_bus_mutex_);
                    return result;
                }
                const TorqueState pre_bus_state =
                    torque_state_.load(std::memory_order_acquire);
                result.yaw_bus_return =
                    scs_bus_.EnableTorque(SERVO_YAW_ID, 1);
                result.pitch_bus_return =
                    scs_bus_.EnableTorque(SERVO_PITCH_ID, 1);
                update_bus_ok();
                if (result.yaw_ok) {
                    yaw_torque_enabled_ = true;
                }
                if (result.pitch_ok) {
                    pitch_torque_enabled_ = true;
                }
                publish_after_bus_attempt(pre_bus_state);
                // Real bus write attempted; ok is governed by yaw_ok/pitch_ok.
                log_result(ExitKind::kBusAction);
                xSemaphoreGive(scs_bus_mutex_);
                return result;
            }

            ESP_LOGW(TAG,
                     "set_servo_torque (reason=%s): kReleasing observed in "
                     "all %d post-mutex retries; skipping bus frames.",
                     ReleaseReasonName(reason),
                     kMaxManualReengageRetries);
            // All retries exhausted while still kReleasing: no bus frame, the
            // requested ON did not happen (Issue #171).
            log_result(ExitKind::kWaitExhausted);
            return result;
        } else {
            if (!yaw_enabled && !pitch_enabled) {
                xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
                bool already_released =
                    torque_state_.load(std::memory_order_acquire) ==
                    TorqueState::kReleased;
                if (already_released) {
                    // Already released for an OFF request: legitimate no-op
                    // success, so ok stays true (Issue #171).
                    log_result(ExitKind::kIdempotent);
                    xSemaphoreGive(scs_bus_mutex_);
                    return result;
                }
                xSemaphoreGive(scs_bus_mutex_);
            }

            // Preserve the existing cancellation-first disable path exactly:
            // reset MotionDriver state for axes being disabled before the
            // EnableTorque(OFF) bus frames can race with ServoTask writes.
            if (motion_driver_ != nullptr && motion_mutex_ != nullptr &&
                disables_axis) {
                xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                if (reason == ReleaseReason::kAutoIdle &&
                    (yaw_motion_.moving || pitch_motion_.moving ||
                     servo_wobble_active_.load(std::memory_order_acquire))) {
                    // Auto-idle deferring because motion is still in progress:
                    // a benign no-op (no wait budget consumed), not a timeout.
                    xSemaphoreGive(motion_mutex_);
                    return finish(ExitKind::kIdempotent);
                }
                if (!yaw_enabled) {
                    yaw_motion_.moving = false;
                    yaw_motion_.position_unknown = true;
                    motion_driver_->InvalidateAxisToken(SERVO_YAW_ID);
                }
                if (!pitch_enabled) {
                    pitch_motion_.moving = false;
                    pitch_motion_.position_unknown = true;
                    motion_driver_->InvalidateAxisToken(SERVO_PITCH_ID);
                }
                servo_wobble_active_.store(false, std::memory_order_release);
                servo_wobble_step_.store(0, std::memory_order_release);
                // Mark the fully-OFF transition before releasing
                // motion_mutex_ so concurrent motion entries do not observe
                // the old engaged state while the OFF bus write is pending.
                if (!yaw_enabled && !pitch_enabled) {
                    (void)MarkReleasing();
                }
                xSemaphoreGive(motion_mutex_);
            }

            xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
            if (reason == ReleaseReason::kAutoIdle &&
                expected_release_epoch != 0) {
                uint32_t current_epoch =
                    torque_release_epoch_.load(std::memory_order_acquire);
                auto current_state =
                    torque_state_.load(std::memory_order_acquire);
                if (current_epoch != expected_release_epoch ||
                    current_state != TorqueState::kReleasing) {
                    ESP_LOGW(TAG,
                             "auto-release OFF aborted at bus check: epoch=%u "
                             "(expected %u), state=%d (expected kReleasing); "
                             "skipping EnableTorque(0,0) frames.",
                             (unsigned)current_epoch,
                             (unsigned)expected_release_epoch,
                             (int)current_state);
                    xSemaphoreGive(scs_bus_mutex_);
                    // Stale auto-release OFF superseded by a newer epoch/state:
                    // the OFF is already obsolete, a benign no-op (Issue #171).
                    log_result(ExitKind::kIdempotent);
                    return result;
                }
            }
            const TorqueState pre_bus_state =
                torque_state_.load(std::memory_order_acquire);
            result.yaw_bus_return = scs_bus_.EnableTorque(
                SERVO_YAW_ID, yaw_enabled ? 1 : 0);
            result.pitch_bus_return = scs_bus_.EnableTorque(
                SERVO_PITCH_ID, pitch_enabled ? 1 : 0);
            update_bus_ok();
            if (result.yaw_ok) {
                yaw_torque_enabled_ = yaw_enabled;
            }
            if (result.pitch_ok) {
                pitch_torque_enabled_ = pitch_enabled;
            }
            publish_after_bus_attempt(pre_bus_state);
            // Real bus write attempted; ok is governed by yaw_ok/pitch_ok.
            log_result(ExitKind::kBusAction);
            xSemaphoreGive(scs_bus_mutex_);
            return result;
        }
    }

    void EnsureTorqueEngagedBeforeMove() {
        auto state = torque_state_.load(std::memory_order_acquire);
        if (state == TorqueState::kEngaged) {
            return;
        }
        if (!servo_ok_) {
            return;
        }

        if (state == TorqueState::kReleasing) {
            if (!WaitForKReleasingToClear()) {
                ESP_LOGW(TAG,
                         "EnsureTorqueEngagedBeforeMove: kReleasing not "
                         "clearing within wait budget, deferring to caller "
                         "retry.");
                return;
            }
            state = torque_state_.load(std::memory_order_acquire);
            if (state == TorqueState::kEngaged) {
                return;  // OFF rolled back to kEngaged
            }
        }

        // state is kPartial, kReleased, or kUncertain -- safe to
        // re-engage now.
        InternalSetServoTorque(true, true, ReleaseReason::kReengagement);
    }

    bool TakeMotionMutexAfterTorqueEngaged() {
        for (int attempt = 0; attempt < kMaxReengageRetries; ++attempt) {
            EnsureTorqueEngagedBeforeMove();
            xSemaphoreTake(motion_mutex_, portMAX_DELAY);
            if (torque_state_.load(std::memory_order_acquire) ==
                TorqueState::kEngaged) {
                return true;
            }
            xSemaphoreGive(motion_mutex_);
            vTaskDelay(pdMS_TO_TICKS(MOTION_TICK_MS));
        }
        ESP_LOGW(TAG,
                 "EnsureTorqueEngagedBeforeMove: re-engagement failed after "
                 "%d attempts; skipping motion entry to avoid silent "
                 "torque-off WritePos.",
                 kMaxReengageRetries);
        return false;
    }

    void MaybeAutoReleaseTorque() {
        if (!auto_release_enabled_.load(std::memory_order_acquire)) {
            return;
        }
        if (!servo_ok_) {
            return;
        }
        if (!boot_init_done_.load(std::memory_order_acquire)) {
            return;
        }
        // PublishTorqueState() raises this when torque re-engages between
        // ServoTask ticks, so a stale idle timer cannot immediately re-OFF.
        if (idle_timer_reset_pending_.exchange(
                false, std::memory_order_acq_rel)) {
            last_motion_end_valid_ = false;
        }
        // Keep the idle window scoped to the currently engaged interval.
        // Released/partial/releasing states must not age a stale timer
        // into the next re-engage. kUncertain is treated as engaged for
        // auto-release purposes: if the kAutoIdle OFF bus frame was lost
        // on the UART path, the auto-release retry must continue so the
        // device does not strand with torque physically ON; if the OFF
        // was actually delivered, the next retry short-circuits via the
        // idempotent path (Issue #170 follow-up).
        auto current_state =
            torque_state_.load(std::memory_order_acquire);
        if (current_state != TorqueState::kEngaged &&
            current_state != TorqueState::kUncertain) {
            last_motion_end_valid_ = false;
            return;
        }
        if (servo_wobble_active_.load(std::memory_order_acquire)) {
            last_motion_end_valid_ = false;
            return;
        }

        bool moving = motion_driver_->IsMoving();
        uint32_t now_ms =
            static_cast<uint32_t>(esp_timer_get_time() / 1000);

        if (moving) {
            last_motion_end_valid_ = false;
            return;
        }

        if (!last_motion_end_valid_) {
            last_motion_end_ms_ = now_ms;
            last_motion_end_valid_ = true;
            return;
        }

        uint32_t idle_ms = now_ms - last_motion_end_ms_;
        uint32_t timeout_ms =
            auto_release_timeout_ms_.load(std::memory_order_acquire);
        if (idle_ms >= timeout_ms) {
            // Publish the pending OFF before InternalSetServoTorque() can
            // block on motion_mutex_, so a concurrent manual ON is routed
            // through the kReleasing wait/retry path instead of treating the
            // already-expired engaged state as a successful no-op.
            uint32_t my_pre_epoch = MarkReleasing();
            ServoTorqueResult r = InternalSetServoTorque(
                false, false, ReleaseReason::kAutoIdle,
                /*expected_release_epoch=*/my_pre_epoch + 1);
            auto state_after =
                torque_state_.load(std::memory_order_acquire);
            if (state_after == TorqueState::kReleased) {
                last_motion_end_valid_ = false;
            } else if (r.idempotent_short_circuit || r.wait_exhausted) {
                // Either short-circuit flag means the OFF returned without a
                // completed bus frame (Issue #171 split the old
                // short_circuited flag; for this kAutoIdle path only the
                // idempotent flag can fire, but the OR keeps the "no bus
                // action" intent explicit and future-proof).
                uint32_t current_epoch =
                    torque_release_epoch_.load(std::memory_order_acquire);
                if (current_epoch == my_pre_epoch) {
                    // Auto-idle re-observed motion or wobble under
                    // motion_mutex_ and returned before any bus frame went
                    // out, so the per-axis torque state is still fully
                    // engaged.
                    torque_state_.store(TorqueState::kEngaged,
                                        std::memory_order_release);
                    ESP_LOGW(TAG,
                             "auto-release OFF aborted by motion/wobble "
                             "re-check; rolled back torque_state_ to "
                             "kEngaged (epoch=%u), retry after "
                             "one idle window.",
                             (unsigned)my_pre_epoch);
                } else if (current_epoch == my_pre_epoch + 1) {
                    ESP_LOGW(TAG,
                             "auto-release OFF aborted at bus check; leaving "
                             "torque_state_ for concurrent publisher "
                             "(my_pre_epoch=%u current_epoch=%u).",
                             (unsigned)my_pre_epoch,
                             (unsigned)current_epoch);
                } else {
                    ESP_LOGW(TAG,
                             "auto-release OFF: release epoch advanced past "
                             "ours (my_pre_epoch=%u current_epoch=%u); "
                             "leaving torque_state_ for concurrent publisher.",
                             (unsigned)my_pre_epoch,
                             (unsigned)current_epoch);
                }
                last_motion_end_ms_ = now_ms;
            } else {
                last_motion_end_ms_ = now_ms;
                ESP_LOGW(TAG,
                         "auto-release OFF bus write failed: yaw_ok=%d "
                         "(r=%d) pitch_ok=%d (r=%d). Retrying after one "
                         "idle window.",
                         r.yaw_ok ? 1 : 0, r.yaw_bus_return,
                         r.pitch_ok ? 1 : 0, r.pitch_bus_return);
            }
        }
    }

    // ---- Phase 7: head-touch (Si12T) sensing + reaction ----------------

    // Convenience wrapper around the existing servo write path. Mirrors the
    // math used in the self.robot.set_head_angles MCP tool so that touch
    // reactions and explicit MCP calls produce identical motion.
    //
    // Issue #1: previously this issued WritePos(id, pos, 100, 0) directly,
    // which hung the SCS0009 bus on large-angle reversals (the second
    // servo's frame collided with the first servo still being driven).
    // Now it sets the target and lets the servo_motion task interpolate.
    //
    // The wobble-cancel + StartMove sequence runs under motion_mutex_ so a
    // concurrent ServoWobbleStepAdvance() on servo_motion task cannot pass
    // its active-load gate after this call has cleared
    // servo_wobble_active_ but before it dispatches the user-driven
    // target — which would let a stale wobble step overwrite the new
    // command.
    void WriteHeadAngles(int yaw_deg, int pitch_deg,
                         uint32_t duration_ms = MOTION_DEFAULT_DURATION_MS,
                         bool prefer_linear = false) {
        if (!servo_ok_ || motion_driver_ == nullptr) {
            ESP_LOGW(TAG, "WriteHeadAngles skipped: servo not initialized");
            return;
        }
        if (!TakeMotionMutexAfterTorqueEngaged()) {
            return;
        }
        if (servo_wobble_active_.load()) {
            servo_wobble_active_.store(false);
            servo_wobble_step_.store(0);
        }
        motion_driver_->StartMove(yaw_deg, pitch_deg, duration_ms,
                                  prefer_linear);
        xSemaphoreGive(motion_mutex_);
    }

    void WriteHeadAngles(int yaw_deg, int pitch_deg, int speed_dps) {
        if (!servo_ok_ || motion_driver_ == nullptr) {
            ESP_LOGW(TAG, "WriteHeadAngles(speed_dps) skipped: servo not initialized");
            return;
        }
        int safe_speed = speed_dps;
        if (safe_speed <= 0) {
            safe_speed = DEFAULT_SPEED_DPS;
        } else if (safe_speed < MIN_STEP_SAFE_SPEED_DPS) {
            ESP_LOGW(TAG, "WriteHeadAngles: speed_dps=%d below MIN_STEP_SAFE_SPEED_DPS=%d, clamping",
                     speed_dps, MIN_STEP_SAFE_SPEED_DPS);
            safe_speed = MIN_STEP_SAFE_SPEED_DPS;
        } else if (safe_speed < MIN_SMOOTH_SPEED_DPS) {
            // Below the on-device measured smoothness floor -- motion will look
            // textured on SCS0009 at MOTION_TICK_MS=20 ms. This is intentionally
            // permitted (the gateway "low" preset is 30 dps, deliberately below
            // the floor for slow, expressive motion per Issue #129 design).
            // Log once so callers can see they are below the smooth zone.
            ESP_LOGW(TAG, "WriteHeadAngles: speed_dps=%d below MIN_SMOOTH_SPEED_DPS=%d (textured motion is expected)",
                     speed_dps, MIN_SMOOTH_SPEED_DPS);
        } else if (safe_speed > MAX_SPEED_DPS) {
            ESP_LOGW(TAG, "WriteHeadAngles: speed_dps=%d above MAX_SPEED_DPS=%d, clamping",
                     speed_dps, MAX_SPEED_DPS);
            safe_speed = MAX_SPEED_DPS;
        }

        int yaw_delta = std::abs(yaw_deg - static_cast<int>(motion_driver_->GetYawDeg()));
        int pitch_delta = std::abs(pitch_deg - static_cast<int>(motion_driver_->GetPitchDeg()));
        int max_delta = std::max(yaw_delta, pitch_delta);
        uint32_t duration_ms = std::max<uint32_t>(
            MOTION_TICK_MS,
            static_cast<uint32_t>(max_delta) * 1000U / static_cast<uint32_t>(safe_speed));
        WriteHeadAngles(yaw_deg, pitch_deg, duration_ms);
    }

    // Servo wobble: yaw -A -> +A -> -A -> 0. Each step is dispatched only
    // after the active MotionDriver reports idle, so the delegated path never
    // overwrites an in-flight SCS0009 internal motion.
    //
    // The initial active check stays outside motion_mutex_ so idle ServoTask
    // ticks do not run the torque re-engagement path. The dispatch body runs
    // under motion_mutex_; without that hold, a concurrent non-wobble
    // WriteHeadAngles() could clear servo_wobble_active_ AFTER this function
    // has passed the active-load + IsMoving gate but BEFORE the switch reaches
    // dispatch, which would let a wobble step overwrite the user's freshly-
    // staged target. Holding the mutex makes "wobble-active check → idle check
    // → step dispatch" atomic w.r.t. any external StartMove() request.
    void ServoWobbleStepAdvance() {
        if (!servo_ok_ || motion_driver_ == nullptr) {
            return;
        }
        if (!servo_wobble_active_.load(std::memory_order_acquire)) {
            return;
        }
        if (!TakeMotionMutexAfterTorqueEngaged()) {
            return;
        }
        if (!servo_wobble_active_.load()) {
            xSemaphoreGive(motion_mutex_);
            return;
        }
        // Direct AxisMotion field access under motion_mutex_; calling
        // motion_driver_->IsMoving() here would re-take the non-recursive
        // semaphore and deadlock.
        bool moving = yaw_motion_.moving || pitch_motion_.moving;
        if (moving) {
            xSemaphoreGive(motion_mutex_);
            return;
        }
        const int A = SERVO_WOBBLE_AMPLITUDE_DEG;
        int step = servo_wobble_step_.load();
        int target_yaw = 0;
        // Preserve the current pitch through the wobble sequence; only the
        // yaw axis is animated. A hardcoded `target_pitch = 0` on every
        // step would command the SCS0009 pitch axis toward the lower
        // end-stop (raw pos ~620 ≈ pitch 0°) on a device whose standard
        // rest pose is BOOT_INIT_PITCH_DEG=45°, accelerating #165
        // cumulative WritePos protection-mode onset within a single
        // STROKE gesture. Direct field read is safe here because
        // motion_mutex_ is already held (see contract above). #175.
        int target_pitch = pitch_motion_.current_deg;
        switch (step) {
            case 0: target_yaw = -A; break;
            case 1: target_yaw = +A; break;
            case 2: target_yaw = -A; break;
            case 3: target_yaw =  0; break;
            default:
                servo_wobble_active_.store(false);
                xSemaphoreGive(motion_mutex_);
                return;
        }
        // StartMove contract: caller holds motion_mutex_. Direct call
        // (not via WriteHeadAngles) avoids the re-entrant mutex take.
        motion_driver_->StartMove(target_yaw, target_pitch, SERVO_WOBBLE_STEP_MS);
        servo_wobble_step_.store(step + 1);
        if (step + 1 > 3) {
            servo_wobble_active_.store(false);
        }
        xSemaphoreGive(motion_mutex_);
    }

    void StartServoWobble() {
        if (!servo_ok_ || motion_driver_ == nullptr) {
            ESP_LOGW(TAG, "Servo wobble skipped: servo not initialized");
            return;
        }
        // Stage the wobble; the actual ServoWobbleStepAdvance() is
        // performed on servo_motion task next tick (see ServoTaskMain).
        // Calling ServoWobbleStepAdvance() directly from here — which is
        // commonly reached via the ESP_TIMER_TASK touch-poll callback —
        // would race with the servo_motion task's own per-tick
        // ServoWobbleStepAdvance() at idle: the atomic step load/store
        // does not exclude the read-switch-store compound operation, so
        // two callers can both observe step==0 and end up dispatching
        // step 0 and step 1 concurrently. Centralising advance on the
        // servo_motion task makes the step sequence deterministic, at
        // the cost of a single MOTION_POLL_INTERVAL_MS / MOTION_TICK_MS
        // delay on the first wobble step (well under human perception).
        //
        // motion_mutex_ also serialises this restart against a
        // concurrent ServoWobbleStepAdvance() running on servo_motion:
        // without the hold, a final-step advancement finishing at the
        // same moment as this restart could overwrite our step=0 /
        // active=true with step+1 or active=false, silently dropping
        // the new stroke's wobble. The mutex is staging-only (no UART
        // I/O is performed under it), so taking it from ESP_TIMER_TASK
        // is bounded to sub-millisecond hold time.
        xSemaphoreTake(motion_mutex_, portMAX_DELAY);
        servo_wobble_step_.store(0);
        servo_wobble_active_.store(true);
        xSemaphoreGive(motion_mutex_);
    }

    static void ServoTaskTrampoline(void* arg) {
        static_cast<StackChanBoard*>(arg)->ServoTaskMain();
    }

    void ServoTaskMain() {
        while (true) {
            if (!servo_ok_ || motion_driver_ == nullptr) {
                vTaskDelay(pdMS_TO_TICKS(MOTION_TICK_MS));
                continue;
            }
            motion_driver_->Tick();
            MaybeAutoReleaseTorque();
            ServoWobbleStepAdvance();
            taskYIELD();
        }
    }

    // Schedule a single-shot revert to "idle" face REACTION_HOLD_MS later.
    // Re-arming overwrites any pending revert. Skipped while the avatar
    // has been hidden via set_avatar("off"), so a stale revert timer
    // does not re-cover the LCD after the user explicitly hid it.
    static void TouchRevertCb(void* arg) {
        StackChanBoard* self = static_cast<StackChanBoard*>(arg);
        self->SetAvatarExpressionIfActive("idle");
    }

    void ScheduleIdleRevert() {
        if (touch_revert_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = &StackChanBoard::TouchRevertCb,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "touch_revert",
                .skip_unhandled_events = true,
            };
            ESP_ERROR_CHECK(esp_timer_create(&args, &touch_revert_timer_));
        }
        esp_timer_stop(touch_revert_timer_);  // ok if not running
        esp_timer_start_once(touch_revert_timer_,
                             (uint64_t)REACTION_HOLD_MS * 1000);
    }

    // Decode a 2-bit channel level from the Si12T Output1 byte.
    // 00 = no output, 01 = low, 10 = medium, 11 = high.
    static inline char Si12tChLevelChar(uint8_t raw, int ch) {
        uint8_t v = (raw >> (ch * 2)) & 0x3;
        return "0LMH"[v];
    }

    // Emit the touch event log line. press_zones/press_raw are the
    // rising-edge snapshot (= the touch the user actually made);
    // release_raw is whatever the sensor reports at the falling edge
    // (normally 0x00 — anything else hints at debounce / hysteresis quirks).
    // ch=%c%c%c%c spells CH1〜CH4 levels using 0/L/M/H. CH4 is unused on
    // stack-chan (the head has 3 zones), so anything non-0 on CH4 is a
    // wiring noise / EMI signature worth investigating.
    void LogTouchEvent(const char* event_name, uint64_t duration_ms) {
        ESP_LOGI(TAG,
                 "touch event: %s start_zones=%d%d%d start_raw=0x%02X ch=%c%c%c%c "
                 "release_raw=0x%02X duration=%u ms",
                 event_name,
                 press_start_zones_[0], press_start_zones_[1], press_start_zones_[2],
                 press_start_output1_raw_,
                 Si12tChLevelChar(press_start_output1_raw_, 0),
                 Si12tChLevelChar(press_start_output1_raw_, 1),
                 Si12tChLevelChar(press_start_output1_raw_, 2),
                 Si12tChLevelChar(press_start_output1_raw_, 3),
                 last_output1_raw_,
                 (unsigned)duration_ms);
    }

    void HandleTap(uint64_t duration_ms) {
        if (!touch_sensor_enabled_.load(std::memory_order_acquire)) {
            return;
        }
        LogTouchEvent("TAP", duration_ms);
        last_event_ = TouchEvent::TAP;
        last_event_us_ = esp_timer_get_time();
        // A tap on the head starts (or ends) listening, like a screen tap,
        // whenever the robot is ready to talk. A stroke stays a pat.
        auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateIdle || state == kDeviceStateListening ||
            state == kDeviceStateSpeaking) {
            ToggleTouchListening();
        } else {
            // Use the IfActive variant so a tap during set_avatar("off") does
            // not pop the avatar back over the WiFi config / settings screens.
            SetAvatarExpressionIfActive("surprised");
            ScheduleIdleRevert();
        }
        Application::GetInstance().SendStackChanEvent("touch", "tap", duration_ms);
    }

    void HandleStroke(uint64_t duration_ms) {
        if (!touch_sensor_enabled_.load(std::memory_order_acquire)) {
            return;
        }
        LogTouchEvent("STROKE", duration_ms);
        last_event_ = TouchEvent::STROKE;
        last_event_us_ = esp_timer_get_time();
        SetAvatarExpressionIfActive("embarrassed");
        StartServoWobble();
        ScheduleIdleRevert();
        Application::GetInstance().SendStackChanEvent("touch", "stroke", duration_ms);
    }

    // 200 ms periodic poll. Reads the sensor, applies a 2-sample debounce on
    // the OR of the three head zones, and emits TAP/STROKE on falling edges.
    static void TouchPollCb(void* arg) {
        StackChanBoard* self = static_cast<StackChanBoard*>(arg);
        self->TouchPollTick();
    }

    void TouchPollTick() {
        if (!si12t_ok_ || si12t_ == nullptr) {
            return;
        }
        Si12T::TouchState s = si12t_->ReadTouchState();
        if (!s.ok) {
            return;
        }
        // Snapshot for MCP visibility.
        last_output1_raw_ = s.output1_raw;
        last_zone_snapshot_[0] = s.zone[0];
        last_zone_snapshot_[1] = s.zone[1];
        last_zone_snapshot_[2] = s.zone[2];

        bool any_pressed = s.zone[0] || s.zone[1] || s.zone[2];
        if (any_pressed) {
            HandleScreenActivity();
        }

        // Asymmetric debounce:
        //   press   confirm = 2 samples ( 200 ms) — fast tap detection
        //   release confirm = 4 samples ( 400 ms) — bridges Si12T recalibration
        //                                            and finger-glide gaps that
        //                                            otherwise cut a stroke
        //                                            short and mis-classify it
        //                                            as a tap.
        // Keeping a press "sticky" through brief no-press blips is essential
        // for the stroke gesture to reach STROKE_MIN_MS.
        if (any_pressed == touch_pressed_pending_) {
            touch_pending_count_++;
        } else {
            touch_pending_count_ = 1;
            touch_pressed_pending_ = any_pressed;
        }
        const int needed = touch_pressed_pending_ ? 2 : 4;
        if (touch_pending_count_ < needed) {
            return;  // not yet debounced
        }

        bool now = touch_pressed_pending_;
        if (now == touch_pressed_prev_) {
            return;  // no edge
        }

        uint64_t now_us = esp_timer_get_time();

        if (now) {
            // Rising edge. Capture the sensor state for the falling-edge
            // log either way — without this, a press that begins during
            // the post-reaction cooldown and is held until the cooldown
            // expires would log the previous touch's start_zones /
            // start_raw on its falling edge, exactly the
            // repeated-touch / noise-overlap scenario this logging is
            // meant to clarify.
            press_start_zones_[0] = s.zone[0];
            press_start_zones_[1] = s.zone[1];
            press_start_zones_[2] = s.zone[2];
            press_start_output1_raw_ = s.output1_raw;
            if (now_us < cooldown_until_us_) {
                // Suppress press event while in post-reaction cooldown.
                touch_pressed_prev_ = now;
                touch_press_start_us_ = now_us;
                return;
            }
            touch_pressed_prev_ = true;
            touch_press_start_us_ = now_us;
        } else {
            // Falling edge: classify by hold duration.
            touch_pressed_prev_ = false;
            uint64_t duration_ms = (now_us - touch_press_start_us_) / 1000ULL;
            if (now_us < cooldown_until_us_) {
                // We were in cooldown when pressed — drop the release event too.
                return;
            }
            if (duration_ms >= STROKE_MIN_MS) {
                HandleStroke(duration_ms);
            } else {
                // Treat the 400-600 ms grey zone as TAP.
                HandleTap(duration_ms);
            }
            cooldown_until_us_ = now_us + (uint64_t)COOLDOWN_MS * 1000ULL;
        }
    }

    void InitializeTouchSettings() {
        Settings settings("touch", false);
        bool enabled = settings.GetBool("enabled", true);
        touch_sensor_enabled_.store(enabled, std::memory_order_release);
        ESP_LOGI(TAG, "Touch sensor setting loaded: enabled=%d", enabled ? 1 : 0);
    }

    void InitializeSi12tTouch() {
        ESP_LOGI(TAG, "Init Si12T head-touch sensor (I2C addr 0x%02X)", Si12T::DEFAULT_ADDR);
        si12t_ = std::unique_ptr<Si12T>(new Si12T(i2c_bus_));
        si12t_ok_ = si12t_->Begin();
        if (!si12t_ok_) {
            ESP_LOGW(TAG, "Si12T not detected; head-touch disabled (other features unaffected)");
            si12t_.reset();
            return;
        }

        esp_timer_create_args_t poll_args = {
            .callback = &StackChanBoard::TouchPollCb,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touch_poll",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&poll_args, &touch_poll_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touch_poll_timer_,
                                                 (uint64_t)TOUCH_POLL_MS * 1000));
        ESP_LOGI(TAG, "Si12T touch poll started (%d ms interval)", TOUCH_POLL_MS);
    }

#if CONFIG_STACKCHAN_BODY_SENSORS
    void InitializeBodySensors() {
        imu_ = std::make_unique<body_sensors::Bmi270>(i2c_bus_);
        if (!imu_->Begin()) imu_.reset();
        {
            Settings settings("body", false);
            const int mg = settings.GetInt("knock_mg", 0);
            if (mg > 0) motion_detector_.set_knock_threshold(mg / 1000.0f);
        }
        prox_ = std::make_unique<body_sensors::Ltr553>(i2c_bus_);
        if (!prox_->Begin()) prox_.reset();
        if (!imu_ && !prox_) return;
        esp_timer_create_args_t args = {
            .callback = [](void* arg) { static_cast<StackChanBoard*>(arg)->BodyPollTick(); },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "body_poll",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &body_poll_timer_) == ESP_OK) {
            esp_timer_start_periodic(body_poll_timer_, BODY_POLL_MS * 1000);
        }
    }

    // A local reaction (like a head touch) plus a stackchan-event so the
    // gateway can tell Claude. An event that finds the robot in its
    // screensaver (clock, dimmed or screen off) wakes it looking surprised.
    // Runs in the esp_timer task, like ClockTick, so clock_visible_ is safe.
    void BodyReact(const char* event_type, const char* subtype, uint32_t duration_ms,
                   const char* face, bool wakes = true) {
        ESP_LOGI(TAG, "body event: %s/%s (%u ms)", event_type, subtype,
                 static_cast<unsigned>(duration_ms));
        if (wakes) {
            const bool asleep = clock_visible_ || dimmed_.load() || screen_is_off_.load();
            OnUserActivity();
            NoteActivity();
            if (asleep) {
                HideClock();
                face = "surprised";
            }
        }
        if (face != nullptr && SetAvatarExpressionIfActive(face)) ScheduleIdleRevert();
        Application::GetInstance().SendStackChanEvent(event_type, subtype, duration_ms);
    }

    // A double knock on the robot acts like a screen tap: start listening
    // when idle, stop and send when listening. Ignored while (or just
    // after) speaking, when the speaker can shake the body.
    void HandleKnock(uint32_t now_ms) {
#if CONFIG_STACKCHAN_KNOCK_TO_LISTEN
        const auto state = Application::GetInstance().GetDeviceState();
        const bool quiet = now_ms - last_speaking_ms_ > KNOCK_AFTER_SPEECH_MS;
        if ((state == kDeviceStateIdle && quiet) || state == kDeviceStateListening) {
            BodyReact("imu", "knock", 0, nullptr);
            ToggleTouchListening();
            return;
        }
        ESP_LOGI(TAG, "knock ignored (state %d)", static_cast<int>(state));
#else
        BodyReact("imu", "knock", 0, nullptr);
#endif
    }

    void BodyPollTick() {
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (imu_) {
            float a[3], g[3];
            if (imu_->ReadMotion(a, g)) {
                taskENTER_CRITICAL(&body_mux_);
                std::copy(a, a + 3, body_accel_);
                std::copy(g, g + 3, body_gyro_);
                taskEXIT_CRITICAL(&body_mux_);
                const bool head_moving = motion_driver_ != nullptr && motion_driver_->IsMoving();
                uint32_t held_ms = 0;
                using body_events::MotionEvent;
                const MotionEvent e = motion_detector_.Update(a, g, head_moving, now_ms, &held_ms);
                switch (e) {
                    case MotionEvent::kPickedUp: BodyReact("imu", Name(e), 0, "surprised"); break;
                    case MotionEvent::kPutDown: BodyReact("imu", Name(e), held_ms, "relieved"); break;
                    case MotionEvent::kTippedOver: BodyReact("imu", Name(e), held_ms, "dizzy"); break;
                    case MotionEvent::kShake: BodyReact("imu", Name(e), 0, "dizzy"); break;
                    case MotionEvent::kKnock: HandleKnock(now_ms); break;
                    default: break;
                }
            }
        }
        if (Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking) {
            last_speaking_ms_ = now_ms;
        }
        if (prox_ && ++body_poll_count_ >= PROXIMITY_EVERY) {
            body_poll_count_ = 0;
            const int raw = prox_->ReadProximity();
            taskENTER_CRITICAL(&body_mux_);
            body_proximity_ = raw;
            taskEXIT_CRITICAL(&body_mux_);
            uint32_t near_ms = 0;
            using body_events::ProximityEvent;
            const ProximityEvent e = proximity_detector_.Update(raw, now_ms, &near_ms);
            if (e == ProximityEvent::kNear) BodyReact("proximity", Name(e), 0, "curious");
            if (e == ProximityEvent::kFar) BodyReact("proximity", Name(e), near_ms, nullptr, false);
        }
    }
#endif

    // Map a face name to AvatarSet's 0-indexed slot, or -1 if unknown.
    static int FaceNameToIndex(const char* face) {
        if (face == nullptr) return -1;
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        // Same indices as below for the first six names, plus the extended
        // expressions and a few aliases.
        return procedural_face::ExpressionFromName(face);
#else
        if (strcmp(face, "idle") == 0)        return 0;
        if (strcmp(face, "happy") == 0)       return 1;
        if (strcmp(face, "thinking") == 0)    return 2;
        if (strcmp(face, "sad") == 0)         return 3;
        if (strcmp(face, "surprised") == 0)   return 4;
        if (strcmp(face, "embarrassed") == 0) return 5;
        return -1;
#endif
    }

    // Map a mouth shape name to AvatarSet's 0-indexed slot, or -1 if unknown.
    // Indices match avatar_set_.GetMouth() order (closed/half/open/e/u).
    static int MouthShapeToIndex(const char* shape) {
        if (shape == nullptr) return -1;
        if (strcmp(shape, "closed") == 0) return 0;
        if (strcmp(shape, "half") == 0)   return 1;
        if (strcmp(shape, "open") == 0)   return 2;
        if (strcmp(shape, "e") == 0)      return 3;
        if (strcmp(shape, "u") == 0)      return 4;
        return -1;
    }

    // ---- Layered-mode image lookups (face / eyes / mouth) -----------------
    //
    // Resolution order for each axis:
    //   1. If a dynamic AvatarSet has been loaded in layered mode, use
    //      its GetFace / GetEyes / GetMouth lookup. nullptr from AvatarSet
    //      is treated as "not in this set"; we fall through to (2).
    //   2. Static const tables in avatar_images.h (placeholder by default;
    //      avatar_images.local.cc swaps in real art for static-art users).
    //
    // Matrix-mode rendering uses avatar_set_.GetMatrix() directly inside
    // RenderAvatarLocked() and bypasses these helpers entirely.

    const lv_image_dsc_t* FaceImageForIndex(int face_index) const {
        if (avatar_set_.is_loaded() &&
            avatar_set_.mode() == AvatarSet::Mode::kLayered) {
            const lv_image_dsc_t* dsc = avatar_set_.GetFace(face_index);
            if (dsc != nullptr) return dsc;
        }
        switch (face_index) {
            case 0: return &avatar_idle;
            case 1: return &avatar_happy;
            case 2: return &avatar_thinking;
            case 3: return &avatar_sad;
            case 4: return &avatar_surprised;
            case 5: return &avatar_embarrassed;
            default: return nullptr;
        }
    }

    const lv_image_dsc_t* EyesImageForIndex(int eyes_index) const {
        if (avatar_set_.is_loaded() &&
            avatar_set_.mode() == AvatarSet::Mode::kLayered) {
            const lv_image_dsc_t* dsc = avatar_set_.GetEyes(eyes_index);
            if (dsc != nullptr) return dsc;
        }
        switch (eyes_index) {
            case 0: return &avatar_eyes_open;
            case 1: return &avatar_eyes_half;
            case 2: return &avatar_eyes_closed;
            default: return nullptr;
        }
    }

    const lv_image_dsc_t* MouthImageForIndex(int mouth_index) const {
        if (avatar_set_.is_loaded() &&
            avatar_set_.mode() == AvatarSet::Mode::kLayered) {
            const lv_image_dsc_t* dsc = avatar_set_.GetMouth(mouth_index);
            if (dsc != nullptr) return dsc;
        }
        switch (mouth_index) {
            case 0: return &avatar_mouth_closed;
            case 1: return &avatar_mouth_half;
            case 2: return &avatar_mouth_open;
            case 3: return &avatar_mouth_e;
            case 4: return &avatar_mouth_u;
            default: return nullptr;
        }
    }

    // Central mode-aware renderer. Caller must hold the display lock.
    //
    // Layered mode: picks a single image from the axis selected by
    // active_layer_ (no firmware-side compositing).
    // Matrix mode: looks up the pre-composed (face, eyes, mouth) image from
    // the AvatarSet's matrix table.
    //
    // Returns false if the requested image is unavailable (e.g. AvatarSet
    // loaded in matrix mode but the index triple is out of range, or the
    // avatar lv_obj cannot be created yet because the screen tree isn't up).
    bool RenderAvatarLocked() {
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        if (!avatar_set_.is_loaded()) {
            return RenderProceduralFaceLocked();
        }
#endif
        const lv_image_dsc_t* dsc = nullptr;
        if (avatar_set_.is_loaded() &&
            avatar_set_.mode() == AvatarSet::Mode::kMatrix) {
            dsc = avatar_set_.GetMatrix(current_face_index_,
                                        current_eyes_index_,
                                        current_mouth_index_);
        } else {
            switch (active_layer_) {
                case ActiveLayer::FACE:
                    dsc = FaceImageForIndex(current_face_index_);
                    break;
                case ActiveLayer::EYES:
                    dsc = EyesImageForIndex(current_eyes_index_);
                    break;
                case ActiveLayer::MOUTH:
                    dsc = MouthImageForIndex(current_mouth_index_);
                    break;
            }
        }
        if (dsc == nullptr) return false;
        if (!EnsureAvatarObject()) return false;
        // 160x120 avatar images are shown at 2x.
        lv_image_set_scale(avatar_img_, 512);
        lv_image_set_src(avatar_img_, dsc);
        lv_obj_move_foreground(avatar_img_);
        BringListeningIndicatorToFrontLocked();
        return true;
    }

#if CONFIG_STACKCHAN_PROCEDURAL_FACE
    bool EnsureProceduralFaceBuffers() {
        if (proc_face_scratch_ != nullptr) return true;
        const size_t bytes = static_cast<size_t>(kProcFaceW) * kProcFaceH * 2;
        for (int i = 0; i < 2; i++) {
            proc_face_buf_[i] = static_cast<uint16_t*>(
                heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (proc_face_buf_[i] == nullptr) {
                ESP_LOGE(TAG, "Procedural face: PSRAM alloc of %u bytes failed", (unsigned)bytes);
                return false;
            }
            lv_image_dsc_t& d = proc_face_dsc_[i];
            d.header.magic = LV_IMAGE_HEADER_MAGIC;
            d.header.cf = LV_COLOR_FORMAT_RGB565;
            d.header.w = kProcFaceW;
            d.header.h = kProcFaceH;
            d.header.stride = kProcFaceW * 2;
            d.data_size = bytes;
            d.data = reinterpret_cast<const uint8_t*>(proc_face_buf_[i]);
        }
        proc_face_scratch_ = static_cast<uint16_t*>(
            heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        return proc_face_scratch_ != nullptr;
    }

    procedural_face::State CurrentFaceState() const {
        procedural_face::State state;
        state.expression = static_cast<procedural_face::Expression>(current_face_index_);
        state.eyes = active_layer_ == ActiveLayer::EYES ? current_eyes_index_ : 0;
        state.mouth = active_layer_ == ActiveLayer::MOUTH ? current_mouth_index_ : -1;
        state.tick_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        state.speaking = tts_lipsync_active_.load(std::memory_order_acquire);
        state.style = face_style_;
        state.gaze_x = gaze_x_;
        state.gaze_y = gaze_y_;
        return state;
    }

    // Shows the back buffer. Caller holds the display lock.
    void ShowProceduralBackLocked(int back) {
        lv_image_set_scale(avatar_img_, 256);
        lv_image_set_src(avatar_img_, &proc_face_dsc_[back]);
        proc_face_front_ = back;
        lv_obj_move_foreground(avatar_img_);
        BringListeningIndicatorToFrontLocked();
    }

    // Requests a redraw of the current expression / blink / mouth state.
    // Drawing never happens on the caller's stack: blink and touch timers
    // run on the small esp_timer task and lip-sync on its own task, and the
    // Bender style needs more stack than they have (overflowing it
    // corrupted memory and killed the display). The face task below draws
    // with its own large stack and is woken immediately.
    // Caller must hold the display lock.
    bool RenderProceduralFaceLocked() {
        if (!EnsureProceduralFaceBuffers()) return false;
        if (!EnsureAvatarObject()) return false;
        proc_face_generation_.fetch_add(1);
        if (proc_face_anim_task_ != nullptr) xTaskNotifyGive(proc_face_anim_task_);
        return true;
    }

    // The only place faces are drawn. Woken by RenderProceduralFaceLocked
    // for changes, and every 200 ms to animate expressions that move. The
    // frame is drawn into a scratch buffer without the display lock, so
    // LVGL, audio and networking never wait on it; only the copy-and-swap
    // is locked, and a frame is redrawn if the state changed meanwhile.
    static void ProceduralFaceAnimTask(void* arg) {
        auto* board = static_cast<StackChanBoard*>(arg);
        const size_t bytes = static_cast<size_t>(kProcFaceW) * kProcFaceH * 2;
        uint32_t drawn_generation = 0;
        int last_render_ms = 0;
        while (true) {
            // Never use more than about half a core: wait at least as long
            // as the last frame took to draw before animating again.
            const int period_ms = board->proc_face_anim_ms_.load();
            const int wait_ms = std::max(period_ms, last_render_ms);
            const bool woken = ulTaskNotifyTake(
                pdTRUE, period_ms > 0 ? pdMS_TO_TICKS(wait_ms) : portMAX_DELAY) > 0;
            if (board->display_ == nullptr || board->avatar_img_ == nullptr ||
                board->proc_face_scratch_ == nullptr || board->avatar_set_.is_loaded() ||
                board->gif_playing_.load()) {
                continue;
            }
            for (int attempt = 0; attempt < 3; attempt++) {
                const uint32_t generation = board->proc_face_generation_.load();
                const auto state = board->CurrentFaceState();
                const bool changed = generation != drawn_generation;
                if (!changed && (!woken && (board->clock_visible_ || period_ms == 0 ||
                                            !procedural_face::IsAnimated(state.expression)))) {
                    break;  // nothing new and nothing moving
                }
                const int64_t t0 = esp_timer_get_time();
                procedural_face::Render(state, procedural_face::Theme{}, board->proc_face_scratch_,
                                        kProcFaceW, kProcFaceH);
                const int64_t t1 = esp_timer_get_time();
                DisplayLockGuard lock(board->display_);
                const int64_t t2 = esp_timer_get_time();
                const uint32_t render_us = static_cast<uint32_t>(t1 - t0);
                last_render_ms = static_cast<int>(render_us / 1000);
                const uint32_t lock_us = static_cast<uint32_t>(t2 - t1);
                board->face_stat_frames_.fetch_add(1);
                board->face_stat_render_us_sum_.fetch_add(render_us);
                board->face_stat_lock_us_sum_.fetch_add(lock_us);
                if (render_us > board->face_stat_render_us_max_.load()) board->face_stat_render_us_max_.store(render_us);
                if (lock_us > board->face_stat_lock_us_max_.load()) board->face_stat_lock_us_max_.store(lock_us);
                if (generation != board->proc_face_generation_.load()) {
                    continue;  // state changed while drawing; draw again
                }
                if (board->current_avatar_face_ != "off") {
                    const int back = board->proc_face_front_ ^ 1;
                    memcpy(board->proc_face_buf_[back], board->proc_face_scratch_, bytes);
                    board->ShowProceduralBackLocked(back);
                }
                drawn_generation = generation;
                break;
            }
        }
    }

    void StartProceduralFaceAnimation() {
        // Same priority (1) and core (1) as LVGL's task, so the two
        // time-slice instead of the face starving the flush (at priority 2
        // a ~250 ms frame preempted LVGL mid-frame and garbled the panel).
        // Core 0 is no good: the always-on AFE / wake-word pipeline there
        // starves a priority-1 task completely. Internal-RAM stack: with a
        // PSRAM stack drawing took ~245 ms per frame instead of ~5 ms.
        xTaskCreatePinnedToCore(ProceduralFaceAnimTask, "face_draw", 8 * 1024, this, 1,
                                &proc_face_anim_task_, 1);
        if (proc_face_anim_task_ == nullptr) {
            ESP_LOGE(TAG, "Face draw task could not be created");
        }
    }
#endif

    // ---- Screen overlays (show_image / show_text) ------------------------

    // Creates or restarts a one-shot timer that runs `fn` on this board.
    void ArmOverlayTimer(esp_timer_handle_t& timer, const char* name,
                         void (*fn)(void*), int seconds) {
        if (timer == nullptr) {
            esp_timer_create_args_t args = {
                .callback = fn,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = name,
                .skip_unhandled_events = true,
            };
            if (esp_timer_create(&args, &timer) != ESP_OK) return;
        }
        esp_timer_stop(timer);
        if (seconds > 0) {
            esp_timer_start_once(timer, static_cast<uint64_t>(seconds) * 1000 * 1000);
        }
    }

    void HideOverlayImage() {
        if (overlay_image_timer_ != nullptr) esp_timer_stop(overlay_image_timer_);
        if (display_ == nullptr) return;
        DisplayLockGuard lock(display_);
        if (overlay_image_ != nullptr) lv_obj_add_flag(overlay_image_, LV_OBJ_FLAG_HIDDEN);
    }

    void HideOverlayText() {
        if (overlay_text_timer_ != nullptr) esp_timer_stop(overlay_text_timer_);
        if (display_ == nullptr) return;
        DisplayLockGuard lock(display_);
        if (overlay_text_ != nullptr) lv_obj_add_flag(overlay_text_, LV_OBJ_FLAG_HIDDEN);
    }

    bool IsOverlayVisible() {
        if (display_ == nullptr) return false;
        DisplayLockGuard lock(display_);
        bool image = overlay_image_ != nullptr && !lv_obj_has_flag(overlay_image_, LV_OBJ_FLAG_HIDDEN);
        bool text = overlay_text_ != nullptr && !lv_obj_has_flag(overlay_text_, LV_OBJ_FLAG_HIDDEN);
        bool canvas = canvas_ != nullptr && !lv_obj_has_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
        return image || text || canvas || gif_playing_.load();
    }

    // Short-tap hook: hides any overlay and reports whether one was shown,
    // so the tap is consumed instead of toggling listening.
    bool DismissOverlaysOnTap() {
        // The press itself already counts as activity and may have removed
        // the clock a moment ago; swallow that tap too.
        if (clock_visible_ || esp_timer_get_time() - clock_hidden_us_ < 1500 * 1000) {
            HideClock();
            return true;
        }
        if (!IsOverlayVisible()) return false;
        HideOverlayImage();
        HideOverlayText();
        HideCanvas();
        StopGif();
        ESP_LOGI(TAG, "Overlay dismissed by tap");
        return true;
    }

    struct OverlayImageJob {
        StackChanBoard* board;
        std::string url;
        std::string token;
        int seconds;
    };

    // Runs on its own task: downloads the RGB565 frame from the gateway's
    // one-time staging URL and shows it. Tool calls run on the main loop,
    // so the HTTP transfer must not happen there.
    static void OverlayImageTask(void* arg) {
        std::unique_ptr<OverlayImageJob> job(static_cast<OverlayImageJob*>(arg));
        StackChanBoard* self = job->board;
        std::string error = self->FetchOverlayImage(job->url, job->token);
        if (error.empty()) {
            self->ShowOverlayImage(job->seconds);
        } else {
            ESP_LOGW(TAG, "show_image failed: %s", error.c_str());
        }
        self->overlay_image_last_error_ = error;
        self->overlay_image_loading_.store(false);
        vTaskDelete(nullptr);
    }

    std::string FetchOverlayImage(const std::string& url, const std::string& token) {
        if (overlay_image_buf_ == nullptr) {
            overlay_image_buf_ = static_cast<uint8_t*>(
                heap_caps_malloc(kOverlayImageBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (overlay_image_buf_ == nullptr) return "psram_oom";
            overlay_image_dsc_.header.magic = LV_IMAGE_HEADER_MAGIC;
            overlay_image_dsc_.header.cf = LV_COLOR_FORMAT_RGB565;
            overlay_image_dsc_.header.w = kOverlayImageW;
            overlay_image_dsc_.header.h = kOverlayImageH;
            overlay_image_dsc_.header.stride = kOverlayImageW * 2;
            overlay_image_dsc_.data_size = kOverlayImageBytes;
            overlay_image_dsc_.data = overlay_image_buf_;
        }
        // Hide the previous picture before overwriting its pixels.
        HideOverlayImage();

        auto network = Board::GetInstance().GetNetwork();
        if (network == nullptr) return "no_network";
        auto http = network->CreateHttp(0);
        if (http == nullptr) return "http_open_failed";
        if (!token.empty()) {
            http->SetHeader("Authorization", (std::string("Bearer ") + token).c_str());
        }
        if (!http->Open("GET", url)) return "http_open_failed";
        if (http->GetStatusCode() != 200) {
            http->Close();
            return "http_status_" + std::to_string(http->GetStatusCode());
        }
        if (http->GetBodyLength() != kOverlayImageBytes) {
            http->Close();
            return "content_length_mismatch";
        }
        size_t total = 0;
        while (total < kOverlayImageBytes) {
            int n = http->Read(reinterpret_cast<char*>(overlay_image_buf_ + total),
                               static_cast<int>(std::min<size_t>(4096, kOverlayImageBytes - total)));
            if (n <= 0) {
                http->Close();
                return "read_failed";
            }
            total += static_cast<size_t>(n);
        }
        http->Close();
        return "";
    }

    void ShowOverlayImage(int seconds) {
        if (display_ == nullptr) return;
        {
            DisplayLockGuard lock(display_);
            if (overlay_image_ == nullptr) {
                overlay_image_ = lv_image_create(lv_layer_top());
                if (overlay_image_ == nullptr) return;
                lv_obj_center(overlay_image_);
                lv_obj_clear_flag(overlay_image_, LV_OBJ_FLAG_SCROLLABLE);
            }
            lv_image_set_src(overlay_image_, &overlay_image_dsc_);
            lv_obj_invalidate(overlay_image_);
            lv_obj_clear_flag(overlay_image_, LV_OBJ_FLAG_HIDDEN);
            // Keep any caption readable on top of the picture.
            if (overlay_text_ != nullptr) lv_obj_move_foreground(overlay_text_);
        }
        HandleScreenActivity();
        ArmOverlayTimer(overlay_image_timer_, "overlay_img",
                        [](void* arg) { static_cast<StackChanBoard*>(arg)->HideOverlayImage(); },
                        seconds);
        ESP_LOGI(TAG, "show_image: shown for %d s (0 = until tap)", seconds);
    }

    // ---- Drawing canvas (self.display.canvas / canvas_hide) --------------

    // "#rrggbb" (or "rrggbb"); a missing color keeps *out.
    static bool ParseCanvasColor(const cJSON* item, lv_color_t* out) {
        if (item == nullptr) return true;
        if (!cJSON_IsString(item) || item->valuestring == nullptr) return false;
        const char* s = item->valuestring;
        if (*s == '#') s++;
        if (strlen(s) != 6) return false;
        for (const char* p = s; *p; p++) {
            if (!isxdigit(static_cast<unsigned char>(*p))) return false;
        }
        *out = lv_color_hex(static_cast<uint32_t>(strtoul(s, nullptr, 16)));
        return true;
    }

    static int CanvasInt(const cJSON* op, const char* key, int fallback) {
        const cJSON* v = cJSON_GetObjectItem(op, key);
        return cJSON_IsNumber(v) ? static_cast<int>(v->valuedouble) : fallback;
    }

    static bool CanvasBool(const cJSON* op, const char* key) {
        return cJSON_IsTrue(cJSON_GetObjectItem(op, key));
    }

    bool EnsureCanvasLocked() {
        const uint32_t stride = lv_draw_buf_width_to_stride(kOverlayImageW, LV_COLOR_FORMAT_RGB565);
        if (canvas_buf_ == nullptr) {
            canvas_buf_ = static_cast<uint8_t*>(heap_caps_aligned_alloc(
                LV_DRAW_BUF_ALIGN, stride * kOverlayImageH, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (canvas_buf_ == nullptr) return false;
        }
        if (canvas_ == nullptr) {
            canvas_ = lv_canvas_create(lv_layer_top());
            if (canvas_ == nullptr) return false;
            lv_canvas_set_buffer(canvas_, canvas_buf_, kOverlayImageW, kOverlayImageH,
                                 LV_COLOR_FORMAT_RGB565);
            lv_obj_center(canvas_);
            lv_obj_clear_flag(canvas_, LV_OBJ_FLAG_SCROLLABLE);
            lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);
            lv_obj_add_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
        }
        return true;
    }

    void HideCanvas() {
        if (display_ == nullptr) return;
        DisplayLockGuard lock(display_);
        if (canvas_ != nullptr) lv_obj_add_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
    }

    // Checks one op before anything is drawn, so a bad op leaves the canvas
    // as it was. Returns "" when the op is valid.
    static std::string ValidateCanvasOp(const cJSON* op) {
        if (!cJSON_IsObject(op)) return "not an object";
        const cJSON* name = cJSON_GetObjectItem(op, "op");
        if (!cJSON_IsString(name)) return "missing op";
        const std::string kind = name->valuestring;
        if (kind != "clear" && kind != "pixel" && kind != "line" && kind != "rect" &&
            kind != "circle" && kind != "text") {
            return "unknown op " + kind;
        }
        lv_color_t color = lv_color_white();
        if (!ParseCanvasColor(cJSON_GetObjectItem(op, "color"), &color)) {
            return "color must be \"#rrggbb\"";
        }
        if (kind == "text" && !cJSON_IsString(cJSON_GetObjectItem(op, "text"))) {
            return "text op needs a text string";
        }
        return "";
    }

    void DrawCanvasOp(lv_layer_t* layer, const cJSON* op, const lv_font_t* font) {
        const std::string kind = cJSON_GetObjectItem(op, "op")->valuestring;
        lv_color_t color = kind == "clear" ? lv_color_black() : lv_color_white();
        ParseCanvasColor(cJSON_GetObjectItem(op, "color"), &color);
        const int width = std::max(1, std::min(CanvasInt(op, "width", 1), 40));

        if (kind == "line") {
            lv_draw_line_dsc_t d;
            lv_draw_line_dsc_init(&d);
            d.color = color;
            d.width = width;
            d.round_start = d.round_end = 1;
            d.p1.x = CanvasInt(op, "x1", 0);
            d.p1.y = CanvasInt(op, "y1", 0);
            d.p2.x = CanvasInt(op, "x2", 0);
            d.p2.y = CanvasInt(op, "y2", 0);
            lv_draw_line(layer, &d);
            return;
        }
        if (kind == "text") {
            lv_draw_label_dsc_t d;
            lv_draw_label_dsc_init(&d);
            d.color = color;
            d.font = font;
            d.text = cJSON_GetObjectItem(op, "text")->valuestring;
            d.text_local = 1;
            const int x = CanvasInt(op, "x", 0);
            const int y = CanvasInt(op, "y", 0);
            lv_area_t area = {x, y, kOverlayImageW - 1, kOverlayImageH - 1};
            lv_draw_label(layer, &d, &area);
            return;
        }

        // clear, pixel, rect and circle are all rectangles.
        lv_area_t area;
        bool fill = true;
        int radius = 0;
        if (kind == "clear") {
            area = {0, 0, kOverlayImageW - 1, kOverlayImageH - 1};
        } else if (kind == "pixel") {
            const int x = CanvasInt(op, "x", 0);
            const int y = CanvasInt(op, "y", 0);
            const int size = std::max(1, std::min(CanvasInt(op, "size", 1), 40));
            area = {x, y, x + size - 1, y + size - 1};
        } else if (kind == "rect") {
            const int x = CanvasInt(op, "x", 0);
            const int y = CanvasInt(op, "y", 0);
            const int w = std::max(1, CanvasInt(op, "w", 1));
            const int h = std::max(1, CanvasInt(op, "h", 1));
            area = {x, y, x + w - 1, y + h - 1};
            fill = CanvasBool(op, "fill");
            radius = std::max(0, CanvasInt(op, "radius", 0));
        } else {  // circle
            const int x = CanvasInt(op, "x", 0);
            const int y = CanvasInt(op, "y", 0);
            const int r = std::max(1, CanvasInt(op, "r", 1));
            area = {x - r, y - r, x + r, y + r};
            fill = CanvasBool(op, "fill");
            radius = LV_RADIUS_CIRCLE;
        }
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.radius = radius;
        if (fill) {
            d.bg_color = color;
            d.bg_opa = LV_OPA_COVER;
            d.border_width = 0;
        } else {
            d.bg_opa = LV_OPA_TRANSP;
            d.border_color = color;
            d.border_opa = LV_OPA_COVER;
            d.border_width = width;
        }
        lv_draw_rect(layer, &d, &area);
    }

    // Draws a JSON array of shape ops and shows the canvas. Every op is
    // checked first, so a bad batch draws nothing. Returns "" on success.
    std::string DrawCanvas(const std::string& json, int* drawn) {
        *drawn = 0;
        if (display_ == nullptr) return "no_display";
        cJSON* ops = cJSON_Parse(json.c_str());
        if (ops == nullptr || !cJSON_IsArray(ops)) {
            if (ops != nullptr) cJSON_Delete(ops);
            return "ops must be a JSON array of shape objects";
        }
        const int n = cJSON_GetArraySize(ops);
        if (n > kCanvasMaxOps) {
            cJSON_Delete(ops);
            return "too many ops (max " + std::to_string(kCanvasMaxOps) + ")";
        }
        for (int i = 0; i < n; i++) {
            std::string error = ValidateCanvasOp(cJSON_GetArrayItem(ops, i));
            if (!error.empty()) {
                cJSON_Delete(ops);
                return "op " + std::to_string(i) + ": " + error;
            }
        }
        {
            DisplayLockGuard lock(display_);
            if (!EnsureCanvasLocked()) {
                cJSON_Delete(ops);
                return "psram_oom";
            }
            const lv_font_t* font = LV_FONT_DEFAULT;
            auto* theme = static_cast<LvglTheme*>(display_->GetTheme());
            if (theme != nullptr && theme->text_font() != nullptr) font = theme->text_font()->font();
            lv_layer_t layer;
            lv_canvas_init_layer(canvas_, &layer);
            for (int i = 0; i < n; i++) DrawCanvasOp(&layer, cJSON_GetArrayItem(ops, i), font);
            lv_canvas_finish_layer(canvas_, &layer);
            lv_obj_clear_flag(canvas_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(canvas_);
            lv_obj_invalidate(canvas_);
        }
        cJSON_Delete(ops);
        *drawn = n;
        HandleScreenActivity();
        return "";
    }

    // Queues a GIF; a playing one stops first. Returns false if the player
    // task could not be created.
    bool QueueGif(std::unique_ptr<GifJob> job) {
        if (gif_mutex_ == nullptr) gif_mutex_ = xSemaphoreCreateMutex();
        if (gif_mutex_ == nullptr) return false;
        xSemaphoreTake(gif_mutex_, portMAX_DELAY);
        gif_pending_ = std::move(job);
        xSemaphoreGive(gif_mutex_);
        gif_stop_.store(true);  // ends the current GIF; the task picks up the new one
        if (gif_task_ == nullptr &&
            xTaskCreatePinnedToCore(GifTask, "gif_play", 6144, this, 2, &gif_task_, 1) != pdPASS) {
            gif_task_ = nullptr;
            return false;
        }
        xTaskNotifyGive(gif_task_);
        return true;
    }

    void StopGif() { gif_stop_.store(true); }

    static void GifTask(void* arg) {
        auto* self = static_cast<StackChanBoard*>(arg);
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            while (true) {
                xSemaphoreTake(self->gif_mutex_, portMAX_DELAY);
                std::unique_ptr<GifJob> job = std::move(self->gif_pending_);
                xSemaphoreGive(self->gif_mutex_);
                if (!job) break;
                self->gif_stop_.store(false);
                std::string error = self->PlayGif(*job);
                if (!error.empty()) ESP_LOGW(TAG, "show_gif failed: %s", error.c_str());
                self->gif_last_error_ = error;
            }
        }
    }

    bool EnsureGifBuffersLocked() {
        if (gif_buf_ == nullptr) {
            gif_buf_ = static_cast<uint16_t*>(
                heap_caps_malloc(kOverlayImageBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (gif_buf_ == nullptr) return false;
            auto& d = gif_dsc_;
            d.header.magic = LV_IMAGE_HEADER_MAGIC;
            d.header.cf = LV_COLOR_FORMAT_RGB565;
            d.header.w = kOverlayImageW;
            d.header.h = kOverlayImageH;
            d.header.stride = kOverlayImageW * 2;
            d.data_size = kOverlayImageBytes;
            d.data = reinterpret_cast<const uint8_t*>(gif_buf_);
        }
        if (gif_img_ == nullptr) {
            gif_img_ = lv_image_create(lv_layer_top());
            if (gif_img_ == nullptr) return false;
            lv_obj_center(gif_img_);
            lv_obj_clear_flag(gif_img_, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_add_flag(gif_img_, LV_OBJ_FLAG_HIDDEN);
        }
        return true;
    }

    std::string DownloadGif(const GifJob& job, std::unique_ptr<uint8_t, void (*)(void*)>& out) {
        auto network = Board::GetInstance().GetNetwork();
        if (network == nullptr) return "no_network";
        auto http = network->CreateHttp(0);
        if (http == nullptr) return "http_open_failed";
        if (!job.token.empty()) {
            http->SetHeader("Authorization", (std::string("Bearer ") + job.token).c_str());
        }
        if (!http->Open("GET", job.url)) return "http_open_failed";
        if (http->GetStatusCode() != 200) {
            http->Close();
            return "http_status_" + std::to_string(http->GetStatusCode());
        }
        const size_t len = http->GetBodyLength();
        if (len < 16 || len > kGifMaxBytes) {
            http->Close();
            return "bad_size_" + std::to_string(len);
        }
        out.reset(static_cast<uint8_t*>(heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
        if (!out) {
            http->Close();
            return "psram_oom";
        }
        size_t total = 0;
        while (total < len) {
            int n = http->Read(reinterpret_cast<char*>(out.get() + total),
                               static_cast<int>(std::min<size_t>(8192, len - total)));
            if (n <= 0) {
                http->Close();
                return "read_failed";
            }
            total += static_cast<size_t>(n);
        }
        http->Close();
        return "";
    }

    struct GifRect {
        int x0 = 0, y0 = 0, x1 = -1, y1 = -1;  // inclusive, gif coordinates
        bool empty() const { return x1 < x0 || y1 < y0; }
        void add(int x, int y, int w, int h) {
            if (w <= 0 || h <= 0) return;
            if (empty()) {
                *this = {x, y, x + w - 1, y + h - 1};
                return;
            }
            x0 = std::min(x0, x);
            y0 = std::min(y0, y);
            x1 = std::max(x1, x + w - 1);
            y1 = std::max(y1, y + h - 1);
        }
    };

    // gifdec keeps the composited frame as B,G,R,A bytes; copy rectangle r
    // of it, centred, into the RGB565 frame (transparent pixels become black).
    // One 32-bit PSRAM read per pixel and -O2: this runs for every frame.
    __attribute__((optimize("O2")))
    static void GifCanvasToRgb565(const gd_GIF* gif, GifRect r, uint16_t* dst) {
        const int w = std::min<int>(gif->width, kOverlayImageW);
        const int h = std::min<int>(gif->height, kOverlayImageH);
        const int ox = (kOverlayImageW - w) / 2, oy = (kOverlayImageH - h) / 2;
        r.x1 = std::min(r.x1, w - 1);
        r.y1 = std::min(r.y1, h - 1);
        for (int y = r.y0; y <= r.y1; y++) {
            // canvas rows start 4-byte aligned (gifdec allocates it after the
            // struct with lv_malloc), so read whole B,G,R,A pixels at once.
            const uint32_t* src = reinterpret_cast<const uint32_t*>(gif->canvas) +
                                  static_cast<size_t>(y) * gif->width + r.x0;
            uint16_t* row = dst + (oy + y) * kOverlayImageW + ox;
            for (int x = r.x0; x <= r.x1; x++) {
                const uint32_t p = *src++;  // little-endian: B | G << 8 | R << 16 | A << 24
                row[x] = (p >> 24) == 0 ? 0
                                        : static_cast<uint16_t>(((p >> 8) & 0xF800) |
                                                                ((p >> 5) & 0x07E0) | ((p >> 3) & 0x001F));
            }
        }
    }

    void SetGifVisibleLocked(bool visible) {
        if (visible) {
            lv_obj_clear_flag(gif_img_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_background(gif_img_);  // captions and subtitles stay on top
            gif_hid_avatar_ = avatar_img_ != nullptr && !lv_obj_has_flag(avatar_img_, LV_OBJ_FLAG_HIDDEN);
            if (gif_hid_avatar_) lv_obj_add_flag(avatar_img_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(gif_img_, LV_OBJ_FLAG_HIDDEN);
            if (gif_hid_avatar_) lv_obj_clear_flag(avatar_img_, LV_OBJ_FLAG_HIDDEN);
            gif_hid_avatar_ = false;
        }
    }

    std::string PlayGif(const GifJob& job) {
        if (display_ == nullptr) return "no_display";
        std::unique_ptr<uint8_t, void (*)(void*)> data(nullptr, heap_caps_free);
        std::string error = DownloadGif(job, data);
        if (!error.empty()) return error;
        gd_GIF* gif = gd_open_gif_data(data.get());
        if (gif == nullptr) return "not_a_gif";
        {
            DisplayLockGuard lock(display_);
            if (!EnsureGifBuffersLocked()) {
                gd_close_gif(gif);
                return "psram_oom";
            }
            memset(gif_buf_, 0, kOverlayImageBytes);
            lv_image_set_src(gif_img_, &gif_dsc_);
        }
        HideOverlayImage();
        gif_playing_.store(true);
        HandleScreenActivity();
        ESP_LOGI(TAG, "show_gif: %ux%u, seconds=%d loops=%d fast=%d", gif->width, gif->height,
                 job.seconds, job.loops, job.fast);

        const int gw = std::min<int>(gif->width, kOverlayImageW);
        const int gh = std::min<int>(gif->height, kOverlayImageH);
        const int ox = (kOverlayImageW - gw) / 2, oy = (kOverlayImageH - gh) / 2;
        const int64_t start_us = esp_timer_get_time();
        const int64_t end_us = job.seconds > 0 ? start_us + int64_t(job.seconds) * 1000000 : INT64_MAX;
        int loops_done = 0, frames = 0;
        int64_t decode_us = 0, convert_us = 0, refresh_us = 0, slept_at = start_us;
        int64_t dirty_px = 0;
        bool shown = false, full = true;
        GifRect prev;  // previous frame's rect, if its disposal changes it now
        while (!gif_stop_.load() && esp_timer_get_time() < end_us) {
            const int64_t t0 = esp_timer_get_time();
            // For looping GIFs gifdec silently seeks back at the end; with
            // loop_count 1 it reports the end (0) so loops can be counted.
            gif->loop_count = 1;
            const int got = gd_get_frame(gif);
            if (got < 0) {
                error = "decode_failed";
                break;
            }
            if (got == 0) {  // trailer: one loop finished
                if (++loops_done >= job.loops && job.loops > 0) break;
                gd_rewind(gif);
                full = true;  // the first frame rebuilds the whole canvas
                continue;
            }
            // gd_get_frame only decodes; draw the frame onto the canvas, as
            // LVGL's own GIF widget does. (Without this the canvas lags a
            // frame and the partial redraw below copies stale pixels.)
            gd_render_frame(gif, gif->canvas);
            const int64_t t1 = esp_timer_get_time();
            // gd_get_frame first applied the previous frame's disposal (only
            // modes 2 and 3 change pixels), then this frame's rect was drawn.
            GifRect dirty = prev;
            dirty.add(gif->fx, gif->fy, gif->fw, gif->fh);
            prev = {};
            if (gif->gce.disposal == 2 || gif->gce.disposal == 3) {
                prev.add(gif->fx, gif->fy, gif->fw, gif->fh);
            } else {
                // gifdec's dispose() would draw this frame onto the canvas a
                // second time at the start of the next frame; it is already
                // there, so make that a no-op ("restore previous" in gifdec).
                gif->gce.disposal = 3;
            }
            if (full) dirty = {0, 0, gw - 1, gh - 1};
            full = false;
            int64_t t2 = t1;
            if (!dirty.empty()) {
                DisplayLockGuard lock(display_);
                GifCanvasToRgb565(gif, dirty, gif_buf_);
                t2 = esp_timer_get_time();
                if (!shown) {
                    SetGifVisibleLocked(true);
                    lv_obj_invalidate(gif_img_);
                    shown = true;
                } else {
                    lv_area_t area = {static_cast<int32_t>(ox + dirty.x0), static_cast<int32_t>(oy + dirty.y0),
                                      static_cast<int32_t>(ox + dirty.x1), static_cast<int32_t>(oy + dirty.y1)};
                    lv_obj_invalidate_area(gif_img_, &area);
                }
                lv_refr_now(nullptr);  // draw this frame now; none get skipped
                dirty_px += int64_t(dirty.x1 - dirty.x0 + 1) * (dirty.y1 - dirty.y0 + 1);
            }
            const int64_t t3 = esp_timer_get_time();
            decode_us += t1 - t0;
            convert_us += t2 - t1;
            refresh_us += t3 - t2;
            frames++;
            NoteActivity();  // no clock screensaver mid-GIF
            // GIF delays are in 1/100 s; browsers treat 0-1 as 100 ms.
            const int delay_ms = job.fast ? 0 : (gif->gce.delay < 2 ? 100 : gif->gce.delay * 10);
            const int64_t wait_us = t0 + int64_t(delay_ms) * 1000 - esp_timer_get_time();
            if (wait_us >= 1000) {
                vTaskDelay(std::max<TickType_t>(1, pdMS_TO_TICKS(wait_us / 1000)));
                slept_at = esp_timer_get_time();
            } else if (esp_timer_get_time() - slept_at > 50000) {
                // Flat out: still let LVGL input and the idle task (watchdog)
                // on this core run, but only once per 50 ms (a tick is 10 ms).
                vTaskDelay(1);
                slept_at = esp_timer_get_time();
            }
        }
        const int64_t elapsed_us = esp_timer_get_time() - start_us;
        gif_fps_x10_.store(elapsed_us > 0 ? static_cast<int>(int64_t(frames) * 10000000 / elapsed_us) : 0);
        if (frames > 0) {
            char stats[160];
            snprintf(stats, sizeof(stats),
                     "%d frames, per frame: decode %d ms, convert %d ms, refresh %d ms, %d%% of screen",
                     frames, static_cast<int>(decode_us / frames / 1000),
                     static_cast<int>(convert_us / frames / 1000), static_cast<int>(refresh_us / frames / 1000),
                     static_cast<int>(dirty_px * 100 / frames / (kOverlayImageW * kOverlayImageH)));
            gif_last_stats_ = stats;
        }
        gd_close_gif(gif);
        {
            DisplayLockGuard lock(display_);
            SetGifVisibleLocked(false);
        }
        gif_playing_.store(false);
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        {
            // The face task skipped frames while the GIF played: redraw now.
            DisplayLockGuard lock(display_);
            if (current_avatar_face_ != "off" && !avatar_set_.is_loaded()) RenderProceduralFaceLocked();
        }
#endif
        ESP_LOGI(TAG, "show_gif: %.1f fps, %s", gif_fps_x10_.load() / 10.0, gif_last_stats_.c_str());
        return error;
    }

    bool ShowOverlayText(const std::string& text, int seconds, const std::string& position) {
        if (display_ == nullptr) return false;
        {
            DisplayLockGuard lock(display_);
            if (overlay_text_ == nullptr) {
                overlay_text_ = lv_label_create(lv_layer_top());
                if (overlay_text_ == nullptr) return false;
                lv_obj_set_width(overlay_text_, 300);
                lv_label_set_long_mode(overlay_text_, LV_LABEL_LONG_WRAP);
                lv_obj_set_style_text_align(overlay_text_, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_set_style_text_color(overlay_text_, lv_color_white(), 0);
                lv_obj_set_style_bg_color(overlay_text_, lv_color_black(), 0);
                lv_obj_set_style_bg_opa(overlay_text_, LV_OPA_80, 0);
                lv_obj_set_style_pad_all(overlay_text_, 8, 0);
                lv_obj_set_style_radius(overlay_text_, 10, 0);
                auto* theme = static_cast<LvglTheme*>(display_->GetTheme());
                if (theme != nullptr && theme->text_font() != nullptr) {
                    lv_obj_set_style_text_font(overlay_text_, theme->text_font()->font(), 0);
                }
            }
            lv_label_set_text(overlay_text_, text.c_str());
            lv_align_t align = position == "top"      ? LV_ALIGN_TOP_MID
                               : position == "center" ? LV_ALIGN_CENTER
                                                      : LV_ALIGN_BOTTOM_MID;
            int dy = position == "top" ? 8 : (position == "center" ? 0 : -8);
            lv_obj_align(overlay_text_, align, 0, dy);
            lv_obj_clear_flag(overlay_text_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(overlay_text_);
        }
        HandleScreenActivity();
        ArmOverlayTimer(overlay_text_timer_, "overlay_txt",
                        [](void* arg) { static_cast<StackChanBoard*>(arg)->HideOverlayText(); },
                        seconds);
        return true;
    }

    // ---- Status icons (Wi-Fi / battery) ----------------------------------

    static lv_obj_t* CreateStatusLabel(const lv_font_t* font) {
        lv_obj_t* label = lv_label_create(lv_layer_top());
        if (label == nullptr) return nullptr;
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_text_opa(label, LV_OPA_70, 0);
        if (font != nullptr) lv_obj_set_style_text_font(label, font, 0);
        lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
        return label;
    }

    void UpdateStatusIcons() {
        if (display_ == nullptr) return;
        const char* wifi_icon = GetNetworkStateIcon();
        int level = 0;
        bool charging = false, discharging = false;
        const bool have_battery = GetBatteryLevel(level, charging, discharging);
        level = std::max(0, std::min(level, 100));
        static const char* const kLevels[] = {
            FONT_AWESOME_BATTERY_EMPTY, FONT_AWESOME_BATTERY_QUARTER,
            FONT_AWESOME_BATTERY_HALF,  FONT_AWESOME_BATTERY_THREE_QUARTERS,
            FONT_AWESOME_BATTERY_FULL,  FONT_AWESOME_BATTERY_FULL,
        };
        const char* battery_icon = charging ? FONT_AWESOME_BATTERY_BOLT : kLevels[level / 20];
        const std::string percent = std::to_string(level) + "%";

        DisplayLockGuard lock(display_);
        if (status_wifi_label_ == nullptr) {
            auto* theme = static_cast<LvglTheme*>(display_->GetTheme());
            const lv_font_t* icon_font =
                theme != nullptr && theme->icon_font() != nullptr ? theme->icon_font()->font() : nullptr;
            const lv_font_t* text_font =
                theme != nullptr && theme->text_font() != nullptr ? theme->text_font()->font() : nullptr;
            if (icon_font == nullptr || text_font == nullptr) {
                return;  // UI fonts not set up yet; retry on the next tick
            }
            status_wifi_label_ = CreateStatusLabel(icon_font);
            status_battery_label_ = CreateStatusLabel(icon_font);
            status_battery_pct_label_ = CreateStatusLabel(text_font);
            if (status_wifi_label_ == nullptr || status_battery_label_ == nullptr ||
                status_battery_pct_label_ == nullptr) {
                return;
            }
            lv_obj_align(status_wifi_label_, LV_ALIGN_TOP_LEFT, 6, 4);
            lv_obj_align(status_battery_label_, LV_ALIGN_TOP_RIGHT, -6, 4);
        }
        // Wi-Fi icon colour shows the whole link: red = no Wi-Fi or no
        // gateway, orange = gateway up but no Claude client, white = ready.
        const bool gateway_up = Application::GetInstance().IsGatewayConnected();
        lv_color_t link_color = !gateway_up                 ? lv_color_hex(0xFF4040)
                                : !claude_connected_.load() ? lv_color_hex(0xFFA030)
                                                            : lv_color_white();
        lv_obj_set_style_text_color(status_wifi_label_, link_color, 0);
        lv_obj_set_style_text_opa(status_wifi_label_, gateway_up && claude_connected_.load() ? LV_OPA_70 : LV_OPA_COVER, 0);
        lv_label_set_text(status_wifi_label_, wifi_icon != nullptr ? wifi_icon : "");
        if (have_battery) {
            const bool low = level < 20 && !charging;
            const lv_color_t battery_color = low ? lv_color_hex(0xFF4040) : lv_color_white();
            const lv_opa_t battery_opa = low ? LV_OPA_COVER : LV_OPA_70;
            lv_obj_set_style_text_color(status_battery_label_, battery_color, 0);
            lv_obj_set_style_text_color(status_battery_pct_label_, battery_color, 0);
            lv_obj_set_style_text_opa(status_battery_label_, battery_opa, 0);
            lv_obj_set_style_text_opa(status_battery_pct_label_, battery_opa, 0);
            lv_label_set_text(status_battery_label_, battery_icon);
            lv_label_set_text(status_battery_pct_label_, percent.c_str());
            lv_obj_clear_flag(status_battery_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(status_battery_pct_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_update_layout(status_battery_label_);
            lv_obj_align_to(status_battery_pct_label_, status_battery_label_,
                            LV_ALIGN_OUT_LEFT_MID, -4, 0);
        } else {
            lv_obj_add_flag(status_battery_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(status_battery_pct_label_, LV_OBJ_FLAG_HIDDEN);
        }
        lv_obj_move_foreground(status_wifi_label_);
        lv_obj_move_foreground(status_battery_label_);
        lv_obj_move_foreground(status_battery_pct_label_);
    }

    // Re-sends the ILI9342 state that a corrupted SPI byte can knock out
    // (sleep, display on, pixel format, inversion, address mode). Busy
    // full-screen animation at high SPI clock occasionally turned the panel
    // dark until a re-init; this makes any such glitch heal within seconds.
    // Runs under the display lock so it never interleaves with a flush.
    void RefreshLcdState() {
        if (display_ == nullptr || lcd_panel_ == nullptr || lcd_panel_io_ == nullptr) return;
        if (!lcd_refresh_enabled_.load()) return;
        DisplayLockGuard lock(display_);
        esp_lcd_panel_io_tx_param(lcd_panel_io_, 0x11, nullptr, 0);  // SLPOUT
        vTaskDelay(pdMS_TO_TICKS(5));
        esp_lcd_panel_io_tx_param(lcd_panel_io_, 0x13, nullptr, 0);  // NORON
        esp_lcd_panel_io_tx_param(lcd_panel_io_, 0x38, nullptr, 0);  // IDMOFF
        const uint8_t colmod = 0x55;                                  // 16 bpp
        esp_lcd_panel_io_tx_param(lcd_panel_io_, 0x3A, &colmod, 1);
        esp_lcd_panel_invert_color(lcd_panel_, true);
        esp_lcd_panel_swap_xy(lcd_panel_, DISPLAY_SWAP_XY);           // re-sends MADCTL
        esp_lcd_panel_mirror(lcd_panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(lcd_panel_, true);                  // DISPON
    }

    void StartLcdRefresh() {
        esp_timer_create_args_t args = {
            .callback = [](void* arg) { static_cast<StackChanBoard*>(arg)->RefreshLcdState(); },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "lcd_refresh",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &lcd_refresh_timer_) == ESP_OK) {
            esp_timer_start_periodic(lcd_refresh_timer_, 5 * 1000 * 1000);
        }
    }

    void StartStatusIcons() {
        esp_timer_create_args_t args = {
            .callback = [](void* arg) { static_cast<StackChanBoard*>(arg)->UpdateStatusIcons(); },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "status_icons",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &status_icons_timer_) == ESP_OK) {
            esp_timer_start_periodic(status_icons_timer_, 5 * 1000 * 1000);
        }
    }

    // ---- Bottom strip: ticker + subtitles ---------------------------------

    static constexpr int kStripH = 30;

    bool EnsureStripLocked() {
        if (strip_bar_ != nullptr) return true;
        strip_bar_ = lv_obj_create(lv_layer_top());
        if (strip_bar_ == nullptr) return false;
        lv_obj_remove_style_all(strip_bar_);
        lv_obj_set_size(strip_bar_, 320, kStripH);
        lv_obj_align(strip_bar_, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_bg_color(strip_bar_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(strip_bar_, LV_OPA_70, 0);
        lv_obj_clear_flag(strip_bar_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(strip_bar_, LV_OBJ_FLAG_CLICKABLE);
        strip_label_ = lv_label_create(strip_bar_);
        if (strip_label_ == nullptr) return false;
        lv_obj_set_width(strip_label_, 308);
        lv_label_set_long_mode(strip_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_align(strip_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(strip_label_, lv_color_white(), 0);
        auto* theme = static_cast<LvglTheme*>(display_->GetTheme());
        if (theme != nullptr && theme->text_font() != nullptr) {
            lv_obj_set_style_text_font(strip_label_, theme->text_font()->font(), 0);
        }
        lv_obj_center(strip_label_);
        lv_obj_add_flag(strip_bar_, LV_OBJ_FLAG_HIDDEN);
        return true;
    }

    // Subtitles scroll through once, slowly: the first words stay put for
    // kSubtitleLeadMs, then the line moves left until its end is on screen
    // and stops there. Readable beats keeping pace with fast speech, so a
    // long line can still be moving after the audio ends; a clear that
    // arrives then waits for the scroll (see SetSubtitle).
    static constexpr int kSubtitleMaxPxS = 45;
    static constexpr int kSubtitleMinPxS = 20;
    static constexpr int kSubtitleLeadMs = 1000;
    static constexpr int kSubtitleHoldMs = 1500;
    static constexpr int kStripTextW = 308;

    // Shows the subtitle if one is active, else the ticker, else nothing.
    // Caller holds the display lock.
    void ApplyStripLocked() {
        if (!EnsureStripLocked()) return;
        lv_anim_delete(strip_label_, nullptr);
        subtitle_scroll_end_us_ = 0;
        const bool subtitle = !subtitle_text_.empty();
        const std::string& text = subtitle ? subtitle_text_ : ticker_text_;
        if (text.empty()) {
            lv_obj_add_flag(strip_bar_, LV_OBJ_FLAG_HIDDEN);
            return;
        }
        const lv_font_t* font = lv_obj_get_style_text_font(strip_label_, LV_PART_MAIN);
        int text_w = 0;
        if (font != nullptr) {
            lv_point_t size = {0, 0};
            lv_text_get_size(&size, text.c_str(), font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
            text_w = static_cast<int>(size.x);
        }
        lv_obj_set_style_text_color(strip_label_,
                                    subtitle ? lv_color_hex(0xFFE680) : lv_color_white(), 0);
        lv_label_set_text(strip_label_, text.c_str());
        if (subtitle) {
            ShowSubtitleLocked(text_w);
        } else {
            ShowTickerLocked(font, text_w);
        }
        lv_obj_clear_flag(strip_bar_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(strip_bar_);
    }

    void ShowSubtitleLocked(int text_w) {
        lv_label_set_long_mode(strip_label_, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(strip_label_, LV_SIZE_CONTENT);
        const int margin = (320 - kStripTextW) / 2;
        if (text_w <= kStripTextW) {
            lv_obj_align(strip_label_, LV_ALIGN_CENTER, 0, 0);
            return;
        }
        const int travel = text_w - kStripTextW;
        const int scroll_ms = std::max(subtitle_duration_ms_ - kSubtitleLeadMs, 1);
        const int speed = std::clamp(static_cast<int>(int64_t(travel) * 1000 / scroll_ms),
                                     kSubtitleMinPxS, kSubtitleMaxPxS);
        const uint32_t duration = static_cast<uint32_t>(int64_t(travel) * 1000 / speed);
        lv_obj_align(strip_label_, LV_ALIGN_LEFT_MID, margin, 0);
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, strip_label_);
        lv_anim_set_exec_cb(&a, [](void* obj, int32_t x) {
            lv_obj_set_x(static_cast<lv_obj_t*>(obj), x);
        });
        lv_anim_set_values(&a, margin, margin - travel);
        lv_anim_set_delay(&a, kSubtitleLeadMs);
        lv_anim_set_duration(&a, duration);
        lv_anim_set_repeat_count(&a, 0);
        lv_anim_start(&a);
        subtitle_scroll_end_us_ =
            esp_timer_get_time() + int64_t(kSubtitleLeadMs + duration) * 1000;
    }

    void ShowTickerLocked(const lv_font_t* font, int text_w) {
        lv_obj_set_width(strip_label_, kStripTextW);
        lv_label_set_long_mode(strip_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_center(strip_label_);
        const int speed = ticker_speed_px_s_;
        uint32_t anim_duration = lv_anim_speed(speed);
        if (font != nullptr) {
            // Pass a plain loop time in ms rather than lv_anim_speed*():
            // LVGL caps encoded speeds at ~10 s per loop, so long ticker
            // text raced past at hundreds of px/s whatever speed was asked.
            // Travel matches lv_label's circular scroll (text + wait chars).
            int travel = text_w + lv_font_get_glyph_width(font, ' ', ' ') * LV_LABEL_WAIT_CHAR_COUNT;
            anim_duration = std::max<uint32_t>(200, static_cast<uint32_t>(travel) * 1000U / speed);
        }
        lv_obj_set_style_anim_duration(strip_label_, anim_duration, 0);
    }

    void SetTicker(const std::string& text, int seconds, int speed) {
        if (display_ == nullptr) return;
        ticker_text_ = text;
        ticker_speed_px_s_ = std::max(10, std::min(speed, 400));
        {
            DisplayLockGuard lock(display_);
            ApplyStripLocked();
        }
        ArmOverlayTimer(ticker_timer_, "ticker",
                        [](void* arg) {
                            auto* self = static_cast<StackChanBoard*>(arg);
                            self->SetTicker("", 0, self->ticker_speed_px_s_);
                        },
                        text.empty() ? 0 : seconds);
    }

    // An empty text clears the subtitle, but not before a running scroll has
    // reached the end and been readable for kSubtitleHoldMs.
    void SetSubtitle(const std::string& text, int duration_ms) {
        if (display_ == nullptr) return;
        if (subtitle_clear_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = [](void* arg) {
                    auto* self = static_cast<StackChanBoard*>(arg);
                    DisplayLockGuard lock(self->display_);
                    self->subtitle_text_.clear();
                    self->ApplyStripLocked();
                },
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "subtitle_clear",
                .skip_unhandled_events = true,
            };
            esp_timer_create(&args, &subtitle_clear_timer_);
        }
        if (subtitle_clear_timer_ != nullptr) esp_timer_stop(subtitle_clear_timer_);
        DisplayLockGuard lock(display_);
        if (text.empty() && !subtitle_text_.empty() && subtitle_clear_timer_ != nullptr) {
            const int64_t wait_us = subtitle_scroll_end_us_ - esp_timer_get_time();
            if (wait_us > 0) {
                esp_timer_start_once(subtitle_clear_timer_, wait_us + kSubtitleHoldMs * 1000);
                return;
            }
        }
        subtitle_text_ = text;
        subtitle_duration_ms_ = duration_ms;
        ApplyStripLocked();
    }

#if CONFIG_STACKCHAN_PROCEDURAL_FACE
    static constexpr const char* kFaceStyleNvsKey = "face_style";

    void LoadFaceStyle() {
        Settings settings(STACKCHAN_SCREEN_OFF_NVS_NAMESPACE, false);
        int idx = procedural_face::StyleFromName(settings.GetString(kFaceStyleNvsKey, "stackchan").c_str());
        face_style_ = static_cast<procedural_face::Style>(idx < 0 ? 0 : idx);
        ESP_LOGI(TAG, "Face style: %s", procedural_face::StyleName(face_style_));
    }

    bool SetFaceStyle(const std::string& name) {
        int idx = procedural_face::StyleFromName(name.c_str());
        if (idx < 0) return false;
        face_style_ = static_cast<procedural_face::Style>(idx);
        {
            Settings settings(STACKCHAN_SCREEN_OFF_NVS_NAMESPACE, true);
            settings.SetString(kFaceStyleNvsKey, procedural_face::StyleName(face_style_));
        }
        if (display_ != nullptr) {
            DisplayLockGuard lock(display_);
            if (avatar_img_ != nullptr && current_avatar_face_ != "off") RenderAvatarLocked();
        }
        return true;
    }
#endif

    // ---- Clock screensaver ------------------------------------------------

    void NoteActivity() { last_activity_us_.store(esp_timer_get_time()); }

    void StartSntpOnce() {
        if (sntp_started_) return;
        sntp_started_ = true;
#ifdef CONFIG_STACKCHAN_TIMEZONE
        setenv("TZ", CONFIG_STACKCHAN_TIMEZONE, 1);
        tzset();
#endif
        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        if (esp_netif_sntp_init(&config) != ESP_OK) {
            ESP_LOGW(TAG, "SNTP init failed");
            sntp_started_ = false;
            return;
        }
        ESP_LOGI(TAG, "SNTP started (pool.ntp.org)");
    }

    static bool LocalTimeValid(struct tm* out) {
        time_t now = time(nullptr);
        localtime_r(&now, out);
        return out->tm_year >= 2024 - 1900;
    }

    bool EnsureClockObjects() {
        if (clock_buf_ == nullptr) {
            const size_t bytes = 320 * 240 * 2;
            clock_buf_ = static_cast<uint16_t*>(
                heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (clock_buf_ == nullptr) return false;
            clock_dsc_.header.magic = LV_IMAGE_HEADER_MAGIC;
            clock_dsc_.header.cf = LV_COLOR_FORMAT_RGB565;
            clock_dsc_.header.w = 320;
            clock_dsc_.header.h = 240;
            clock_dsc_.header.stride = 320 * 2;
            clock_dsc_.data_size = bytes;
            clock_dsc_.data = reinterpret_cast<const uint8_t*>(clock_buf_);
        }
        if (clock_img_ == nullptr) {
            clock_img_ = lv_image_create(lv_layer_top());
            if (clock_img_ == nullptr) return false;
            lv_obj_center(clock_img_);
            lv_obj_add_flag(clock_img_, LV_OBJ_FLAG_HIDDEN);
            clock_date_ = lv_label_create(lv_layer_top());
            if (clock_date_ != nullptr) {
                lv_obj_set_style_text_color(clock_date_, lv_color_hex(0x909090), 0);
                auto* theme = static_cast<LvglTheme*>(display_->GetTheme());
                if (theme != nullptr && theme->text_font() != nullptr) {
                    lv_obj_set_style_text_font(clock_date_, theme->text_font()->font(), 0);
                }
                lv_obj_add_flag(clock_date_, LV_OBJ_FLAG_HIDDEN);
            }
        }
        return true;
    }

    // Caller holds the display lock.
    void DrawClockLocked(const struct tm& tm) {
        procedural_face::Theme theme;
        theme.foreground = 0xC618;  // soft grey, easy on the eyes
        procedural_face::RenderClock(tm.tm_hour, tm.tm_min, true, theme, clock_buf_, 320, 240);
        lv_image_set_src(clock_img_, &clock_dsc_);
        lv_obj_invalidate(clock_img_);
        if (clock_date_ != nullptr) {
            char date[32];
            strftime(date, sizeof(date), "%A %d %B", &tm);
            lv_label_set_text(clock_date_, date);
            lv_obj_align(clock_date_, LV_ALIGN_BOTTOM_MID, 0, -36);
        }
        clock_drawn_minute_ = tm.tm_hour * 60 + tm.tm_min;
    }

    void ShowClock(const struct tm& tm) {
        if (display_ == nullptr) return;
        DisplayLockGuard lock(display_);
        if (!EnsureClockObjects()) return;
        DrawClockLocked(tm);
        lv_obj_clear_flag(clock_img_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(clock_img_);
        if (clock_date_ != nullptr) {
            lv_obj_clear_flag(clock_date_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(clock_date_);
        }
        // Keep the Wi-Fi / battery icons visible on top of the clock.
        if (status_wifi_label_ != nullptr) lv_obj_move_foreground(status_wifi_label_);
        if (status_battery_label_ != nullptr) lv_obj_move_foreground(status_battery_label_);
        if (status_battery_pct_label_ != nullptr) lv_obj_move_foreground(status_battery_pct_label_);
        clock_visible_ = true;
        clock_shown_us_ = esp_timer_get_time();
        ESP_LOGI(TAG, "Clock screensaver shown");
    }

    void HideClock() {
        if (!clock_visible_ || display_ == nullptr) return;
        DisplayLockGuard lock(display_);
        if (clock_img_ != nullptr) lv_obj_add_flag(clock_img_, LV_OBJ_FLAG_HIDDEN);
        if (clock_date_ != nullptr) lv_obj_add_flag(clock_date_, LV_OBJ_FLAG_HIDDEN);
        clock_visible_ = false;
        clock_hidden_us_ = esp_timer_get_time();
    }

    void ClockTick() {
        auto& app = Application::GetInstance();
        const DeviceState state = app.GetDeviceState();
        if (!sntp_started_ && state == kDeviceStateIdle) {
            StartSntpOnce();
        }
        if (state != kDeviceStateIdle) {
            NoteActivity();  // listening, speaking, connecting ...
        }
#if CONFIG_STACKCHAN_CLOCK_SCREENSAVER_SECONDS > 0
        const int64_t now = esp_timer_get_time();
        if (clock_visible_) {
            if (last_activity_us_.load() > clock_shown_us_) {
                HideClock();
                return;
            }
            struct tm tm;
            if (LocalTimeValid(&tm) && tm.tm_hour * 60 + tm.tm_min != clock_drawn_minute_) {
                DisplayLockGuard lock(display_);
                DrawClockLocked(tm);
            }
            return;
        }
        if (avatar_img_ == nullptr || current_avatar_face_ == "off" || IsOverlayVisible()) {
            return;  // face not up yet (e.g. Wi-Fi setup) or a picture is showing
        }
        if (now - last_activity_us_.load() <
            int64_t(CONFIG_STACKCHAN_CLOCK_SCREENSAVER_SECONDS) * 1000 * 1000) {
            return;
        }
        struct tm tm;
        if (LocalTimeValid(&tm)) {
            ShowClock(tm);
        }
#endif
    }

    void StartClockTimer() {
        NoteActivity();
        esp_timer_create_args_t args = {
            .callback = [](void* arg) { static_cast<StackChanBoard*>(arg)->ClockTick(); },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "clock_saver",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &clock_tick_timer_) == ESP_OK) {
            esp_timer_start_periodic(clock_tick_timer_, 1000 * 1000);
        }
    }

    // ---- Avatar fetch pending machinery (intent doc invariant #6) -------

    // Lazily create avatar_pending_lock_. Safe to call repeatedly.
    void EnsureAvatarPendingLock() {
        if (avatar_pending_lock_ == nullptr) {
            avatar_pending_lock_ = xSemaphoreCreateMutex();
        }
    }

    // Record the request as pending if a fetch is currently in progress.
    // Returns true when the request was captured (caller should NOT proceed
    // with the live LVGL write); false when no fetch is active and the
    // caller should run its normal path.
    //
    // Each helper writes the relevant subset of avatar_pending_ — a later
    // call within the same fetch window wins (the user's most recent
    // intent is what we apply when the fetch completes). set_avatar(face)
    // and set_avatar("off") are mutually exclusive on the face axis, so
    // they clear each other; mouth and blink axes are independent.
    bool DeferAvatarFaceIfFetching(const char* face) {
        if (!avatar_fetch_in_progress_.load(std::memory_order_acquire)) return false;
        EnsureAvatarPendingLock();
        if (avatar_pending_lock_ == nullptr) return false;
        if (xSemaphoreTake(avatar_pending_lock_, portMAX_DELAY) == pdTRUE) {
            avatar_pending_.has_off = false;
            avatar_pending_.has_face = true;
            avatar_pending_.face_name = (face != nullptr) ? face : "";
            xSemaphoreGive(avatar_pending_lock_);
        }
        ESP_LOGI(TAG, "SetAvatarExpression('%s') deferred (avatar fetch in progress)",
                 face != nullptr ? face : "(null)");
        return true;
    }

    bool DeferAvatarOffIfFetching() {
        if (!avatar_fetch_in_progress_.load(std::memory_order_acquire)) return false;
        EnsureAvatarPendingLock();
        if (avatar_pending_lock_ == nullptr) return false;
        if (xSemaphoreTake(avatar_pending_lock_, portMAX_DELAY) == pdTRUE) {
            avatar_pending_.has_face = false;
            avatar_pending_.face_name.clear();
            avatar_pending_.has_off = true;
            xSemaphoreGive(avatar_pending_lock_);
        }
        ESP_LOGI(TAG, "SetAvatarOff deferred (avatar fetch in progress)");
        return true;
    }

    bool DeferAvatarMouthIfFetching(const char* shape) {
        if (!avatar_fetch_in_progress_.load(std::memory_order_acquire)) return false;
        EnsureAvatarPendingLock();
        if (avatar_pending_lock_ == nullptr) return false;
        if (xSemaphoreTake(avatar_pending_lock_, portMAX_DELAY) == pdTRUE) {
            avatar_pending_.has_mouth = true;
            avatar_pending_.mouth_shape = (shape != nullptr) ? shape : "";
            xSemaphoreGive(avatar_pending_lock_);
        }
        ESP_LOGI(TAG, "SetMouthShape('%s') deferred (avatar fetch in progress)",
                 shape != nullptr ? shape : "(null)");
        return true;
    }

    bool DeferAvatarBlinkIfFetching(bool enabled) {
        if (!avatar_fetch_in_progress_.load(std::memory_order_acquire)) return false;
        EnsureAvatarPendingLock();
        if (avatar_pending_lock_ == nullptr) return false;
        if (xSemaphoreTake(avatar_pending_lock_, portMAX_DELAY) == pdTRUE) {
            avatar_pending_.has_blink = true;
            avatar_pending_.blink_enabled = enabled;
            xSemaphoreGive(avatar_pending_lock_);
        }
        ESP_LOGI(TAG, "set_blink(%d) deferred (avatar fetch in progress)", (int)enabled);
        return true;
    }

    // Drain avatar_pending_ and apply it. Called from the avatar_fetch
    // worker task after AvatarSet::AdoptOwnedBuffer returns (regardless of success);
    // the caller must have already cleared avatar_fetch_in_progress_ so
    // that the public SetAvatarExpression / SetMouthShape / set_blink
    // paths invoked here run their live LVGL writes instead of looping
    // back through the defer helpers.
    void ApplyPendingAvatarAfterFetch() {
        PendingAvatarState pending;
        EnsureAvatarPendingLock();
        if (avatar_pending_lock_ == nullptr) return;
        if (xSemaphoreTake(avatar_pending_lock_, portMAX_DELAY) == pdTRUE) {
            pending = avatar_pending_;
            avatar_pending_ = PendingAvatarState{};
            xSemaphoreGive(avatar_pending_lock_);
        }
        if (pending.has_off) {
            SetAvatarOff();
            return;
        }
        if (pending.has_face) {
            SetAvatarExpression(pending.face_name.c_str());
        }
        if (pending.has_mouth) {
            SetMouthShape(pending.mouth_shape.c_str());
        }
        if (pending.has_blink) {
            if (pending.blink_enabled) {
                StartBlinkTimer();
            } else {
                StopBlinkTimer();
            }
        }
    }

    // Create avatar_img_ on the active LVGL screen, scaled to fill the LCD.
    // Caller must hold the LVGL/display lock. Returns true on success or
    // when avatar_img_ already exists.
    bool EnsureAvatarObject() {
        if (avatar_img_ != nullptr) {
            return true;
        }
        lv_obj_t* screen = lv_screen_active();
        if (screen == nullptr) {
            return false;
        }
        avatar_img_ = lv_image_create(screen);
        if (avatar_img_ == nullptr) {
            return false;
        }
        // Center on the 320x240 LCD and upscale 160x120 -> ~320x240 (2x).
        // lv_image_set_scale uses 256 = 1.0x; 512 = 2.0x.
        lv_image_set_scale(avatar_img_, 512);
        lv_obj_align(avatar_img_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(avatar_img_, LV_OBJ_FLAG_SCROLLABLE);
        // Keep the avatar visually on top of the chat UI's emoji_label_,
        // chat bubbles, etc. The status bar (clock/battery) lives on a
        // separate sibling and is moved to foreground later if needed.
        lv_obj_move_foreground(avatar_img_);
        ESP_LOGI(TAG, "Avatar lv_image created on active screen");
        return true;
    }

    // Apply the requested face to avatar_img_. Returns false if the face is
    // unknown or the avatar object cannot be created yet.
    bool SetAvatarExpressionLocked(const char* face) {
        const int idx = FaceNameToIndex(face);
        if (idx < 0) return false;
        current_face_index_ = idx;
        active_layer_ = ActiveLayer::FACE;
        if (!RenderAvatarLocked()) return false;
        current_avatar_face_ = face;
        return true;
    }

    // Public-style entry that takes the display lock. Used by the MCP tool
    // and by the deferred init timer. Always safe to call from any task.
    //
    // Also handles the "resume from off" path: if the previous face was
    // "off" (avatar layer hidden), the layer is unhidden here, and if blink
    // was enabled before SetAvatarOff() ran it is restored automatically.
    bool SetAvatarExpression(const char* face) {
        NoteActivity();
        if (display_ == nullptr) {
            ESP_LOGW(TAG, "SetAvatarExpression('%s') ignored: display_ not ready", face);
            return false;
        }
        // Avatar set fetch in progress — record the request and return
        // success. ApplyPendingAvatarAfterFetch() will replay the latest
        // captured face when the fetch completes.
        if (DeferAvatarFaceIfFetching(face)) {
            return true;
        }
        bool was_off = (current_avatar_face_ == "off");
        bool ok;
        {
            DisplayLockGuard lock(display_);
            if (avatar_img_ != nullptr) {
                // Restore visibility if a previous SetAvatarOff() hid the
                // layer. Cheap no-op when the flag is already clear.
                lv_obj_clear_flag(avatar_img_, LV_OBJ_FLAG_HIDDEN);
            }
            ok = SetAvatarExpressionLocked(face);
        }
        if (!ok) {
            ESP_LOGW(TAG, "SetAvatarExpression('%s') deferred (face unknown or screen not ready)", face);
            return ok;
        }
        // Coming back from "off": if blink was on before going off, restart
        // it so the user does not have to re-issue set_blink. The default
        // experience is "blink follows the avatar".
        if (was_off && blink_enabled_before_off_) {
            StartBlinkTimer();
            ESP_LOGI(TAG, "Blink restored after avatar resume from OFF");
        }
        blink_enabled_before_off_ = false;
        return ok;
    }

    // Hide the avatar layer and disable blink so the underlying
    // xiaozhi-esp32 screens (WiFi config UI, OTA, settings) become visible.
    // The avatar lv_obj is kept allocated so a subsequent
    // SetAvatarExpression(<other face>) can re-show it cheaply, and the
    // previous blink state is remembered for restoration.
    bool SetAvatarOff() {
        if (display_ == nullptr) {
            ESP_LOGW(TAG, "SetAvatarOff() ignored: display_ not ready");
            return false;
        }
        if (DeferAvatarOffIfFetching()) {
            return true;
        }
        // Capture the previous blink state only on the first transition
        // into "off". A repeated set_avatar("off") while already off must
        // not overwrite the saved state with the post-off (always-false)
        // blink_enabled_.
        if (current_avatar_face_ != "off") {
            blink_enabled_before_off_ = blink_enabled_;
        }
        // StopBlinkTimer() takes the display lock internally for the
        // resting-face restore step, so call it before grabbing our lock.
        StopBlinkTimer();
        {
            DisplayLockGuard lock(display_);
            if (avatar_img_ != nullptr) {
                lv_obj_add_flag(avatar_img_, LV_OBJ_FLAG_HIDDEN);
            }
        }
        current_avatar_face_ = "off";
        ESP_LOGI(TAG, "Avatar OFF: hidden + blink disabled (was %s)",
                 blink_enabled_before_off_ ? "ON" : "OFF");
        return true;
    }

    // Internal-only entry used by autonomous animations (touch reactions,
    // idle revert, etc.). Skips the face change while the avatar is in
    // the user-requested "off" state, so the underlying xiaozhi-esp32
    // screens remain visible. Returns true if the face was applied.
    bool SetAvatarExpressionIfActive(const char* face) {
        if (current_avatar_face_ == "off") {
            ESP_LOGD(TAG, "SetAvatarExpressionIfActive('%s') skipped: avatar is OFF", face);
            return false;
        }
        return SetAvatarExpression(face);
    }

    // Schedule a one-shot/periodic timer that keeps trying to install the
    // initial avatar image until the LVGL screen tree is ready (i.e. after
    // Application::Start() has run Display::SetupUI()).
    void InitializeAvatar() {
        ESP_LOGI(TAG, "Schedule avatar init (deferred until SetupUI completes)");
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                StackChanBoard* board = static_cast<StackChanBoard*>(arg);
                if (board->SetAvatarExpression("idle")) {
                    ESP_LOGI(TAG, "Initial avatar (idle) installed");
                    if (board->avatar_init_timer_ != nullptr) {
                        esp_timer_stop(board->avatar_init_timer_);
                    }
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "avatar_init",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &avatar_init_timer_));
        // Retry every 500 ms; SetupUI() typically completes within a few
        // hundred ms after Application::Start(). Once installed the callback
        // stops the timer.
        ESP_ERROR_CHECK(esp_timer_start_periodic(avatar_init_timer_, 500 * 1000));
    }

    // ---- Phase 2: parts (eyes / mouth) and blink state machine ----------
    //
    // Eye and mouth axes share the unified rendering state machine above —
    // each operation updates current_*_index_ + active_layer_ and asks
    // RenderAvatarLocked() to redraw. In layered mode that produces the
    // upstream Phase 2 behaviour (one image at a time, with blink
    // temporarily replacing the face); in matrix mode the same state
    // change drives a composite (face, eyes, mouth) frame.

    // Restore the resting expression after a part overlay (= blink end /
    // explicit stop). Eyes return to 0 (open); the mouth index is preserved
    // so a Phase 4 lip-sync shape kept after the sequence ends continues to
    // be composited in matrix mode and remains the last frame the next
    // mouth call will replace in layered mode.
    bool RestoreCurrentFaceLocked() {
        current_eyes_index_ = 0;
        active_layer_ = ActiveLayer::FACE;
        return RenderAvatarLocked();
    }

    // Public mouth setter: wraps lock + look-up.
    bool SetMouthShape(const char* shape) {
        if (display_ == nullptr) {
            ESP_LOGW(TAG, "SetMouthShape('%s') ignored: display_ not ready", shape);
            return false;
        }
        const int idx = MouthShapeToIndex(shape);
        if (idx < 0) return false;
        if (DeferAvatarMouthIfFetching(shape)) {
            // Fetch in progress — record the request in avatar_pending_ and
            // let the post-fetch replay path apply it. Matches the face / off
            // / blink deferral paths so set_mouth doesn't race the AvatarSet
            // buffer swap.
            return true;
        }
        DisplayLockGuard lock(display_);
        current_mouth_index_ = idx;
        active_layer_ = ActiveLayer::MOUTH;
        return RenderAvatarLocked();
    }

    // Step callback for the four-phase blink sequence. Each invocation
    // advances blink_state_, applies the corresponding image, and re-arms
    // blink_step_timer_ unless we're returning to the resting face.
    static void BlinkStepCb(void* arg) {
        StackChanBoard* self = static_cast<StackChanBoard*>(arg);
        self->BlinkStepAdvance();
    }

    void BlinkStepAdvance() {
        if (display_ == nullptr) {
            blink_state_ = BlinkState::IDLE;
            return;
        }
        DisplayLockGuard lock(display_);
        switch (blink_state_) {
            case BlinkState::EYES_HALF_DOWN:
                current_eyes_index_ = 2;  // closed
                active_layer_ = ActiveLayer::EYES;
                RenderAvatarLocked();
                blink_state_ = BlinkState::EYES_CLOSED;
                esp_timer_start_once(blink_step_timer_, BLINK_STEP_MS * 1000);
                break;
            case BlinkState::EYES_CLOSED:
                current_eyes_index_ = 1;  // half
                active_layer_ = ActiveLayer::EYES;
                RenderAvatarLocked();
                blink_state_ = BlinkState::EYES_HALF_UP;
                esp_timer_start_once(blink_step_timer_, BLINK_STEP_MS * 1000);
                break;
            case BlinkState::EYES_HALF_UP:
                // Final: restore the resting state. In layered mode this
                // repaints the face image (the Phase 2 trade-off: any
                // active mouth overlay is replaced by the face). In matrix
                // mode the mouth index is preserved so the composite frame
                // keeps the user's lip-sync state.
                RestoreCurrentFaceLocked();
                blink_state_ = BlinkState::IDLE;
                break;
            case BlinkState::IDLE:
            default:
                // Stale callback; nothing to do.
                break;
        }
    }

    // Schedule callback: fires roughly every BLINK_MIN_GAP_MS..BLINK_MAX_GAP_MS.
    // Starts a new blink if enabled and not already blinking, then re-arms
    // itself with a fresh random interval.
    static void BlinkScheduleCb(void* arg) {
        StackChanBoard* self = static_cast<StackChanBoard*>(arg);
        self->BlinkScheduleTick();
    }

    void BlinkScheduleTick() {
        if (blink_enabled_ && blink_state_ == BlinkState::IDLE && display_ != nullptr) {
            // Begin the blink: half-down now, full-closed at next step.
            DisplayLockGuard lock(display_);
            current_eyes_index_ = 1;  // half
            active_layer_ = ActiveLayer::EYES;
            if (RenderAvatarLocked()) {
                blink_state_ = BlinkState::EYES_HALF_DOWN;
                esp_timer_start_once(blink_step_timer_, BLINK_STEP_MS * 1000);
            }
        }
        // Re-arm scheduler with a fresh random interval, even if we skipped
        // this blink (e.g. avatar not yet on screen). This keeps the cadence
        // organic instead of clumping after a long pause.
        if (blink_enabled_) {
            uint32_t span_ms = BLINK_MAX_GAP_MS - BLINK_MIN_GAP_MS;
            uint32_t next_ms = BLINK_MIN_GAP_MS + (esp_random() % span_ms);
            esp_timer_start_once(blink_schedule_timer_, (uint64_t)next_ms * 1000);
        }
    }

    void EnsureBlinkTimers() {
        if (blink_step_timer_ == nullptr) {
            esp_timer_create_args_t step_args = {
                .callback = &StackChanBoard::BlinkStepCb,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "blink_step",
                .skip_unhandled_events = true,
            };
            ESP_ERROR_CHECK(esp_timer_create(&step_args, &blink_step_timer_));
        }
        if (blink_schedule_timer_ == nullptr) {
            esp_timer_create_args_t sched_args = {
                .callback = &StackChanBoard::BlinkScheduleCb,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "blink_sched",
                .skip_unhandled_events = true,
            };
            ESP_ERROR_CHECK(esp_timer_create(&sched_args, &blink_schedule_timer_));
        }
    }

    void StartBlinkTimer() {
        EnsureBlinkTimers();
        blink_enabled_ = true;
        // Make sure no leftover schedule timer is running, then arm one with
        // a fresh random interval.
        esp_timer_stop(blink_schedule_timer_);
        uint32_t span_ms = BLINK_MAX_GAP_MS - BLINK_MIN_GAP_MS;
        uint32_t first_ms = BLINK_MIN_GAP_MS + (esp_random() % span_ms);
        esp_timer_start_once(blink_schedule_timer_, (uint64_t)first_ms * 1000);
        ESP_LOGI(TAG, "Blink ENABLED (first blink in %u ms)", (unsigned)first_ms);
    }

    void StopBlinkTimer() {
        blink_enabled_ = false;
        if (blink_schedule_timer_ != nullptr) {
            esp_timer_stop(blink_schedule_timer_);
        }
        if (blink_step_timer_ != nullptr) {
            esp_timer_stop(blink_step_timer_);
        }
        // If we stopped mid-sequence, snap back to the resting face so the
        // user is not left staring at half-closed eyes.
        if (blink_state_ != BlinkState::IDLE && display_ != nullptr) {
            DisplayLockGuard lock(display_);
            RestoreCurrentFaceLocked();
        }
        blink_state_ = BlinkState::IDLE;
        ESP_LOGI(TAG, "Blink DISABLED");
    }

    // ---- Phase 4 audio (Issue #76): TTS state-driven lip-sync ----------
    //
    // While the gateway is playing TTS audio (tts.start..tts.stop), cycle
    // the mouth shape through CLOSED -> HALF -> OPEN -> HALF on a fixed
    // TTS_LIPSYNC_STEP_MS cadence. This is the (A) state-driven approach
    // proposed in Issue #76; the (B) audio-envelope-driven follow-up will
    // replace this cycle with a per-frame amplitude mapping in a separate
    // change.
    //
    // Concurrency:
    //   - Single esp_timer self-rearming on ESP_TIMER_TASK.
    //   - Coexists with the mouth-sequence playback task: when
    //     mouth_seq_active_ is true (the user issued a set_mouth_sequence
    //     while we were animating), the lip-sync step yields its frame and
    //     re-arms; the user-issued sequence wins until it completes, then
    //     lip-sync resumes naturally on the next tick.
    //   - Pauses autonomous blink while active (same Phase 2 reasoning as
    //     the mouth-sequence task: BlinkStepAdvance()'s
    //     RestoreCurrentFaceLocked() would overwrite the mouth overlay).
    //     Restores blink at stop based on blink_desired_ so a set_blink
    //     issued mid-playback is honoured.
    static void TtsLipSyncStepCb(void* arg) {
        static_cast<StackChanBoard*>(arg)->TtsLipSyncStepAdvance();
    }

    void TtsLipSyncStepAdvance() {
        if (!tts_lipsync_active_.load(std::memory_order_acquire)) {
            return;
        }
        if (display_ == nullptr) {
            return;
        }
        // Yield to an in-flight user-issued mouth sequence; re-arm so we
        // resume on our cadence as soon as the sequence finishes.
        if (mouth_seq_active_.load(std::memory_order_acquire)) {
            esp_timer_start_once(tts_lipsync_timer_,
                                 (uint64_t)TTS_LIPSYNC_STEP_MS * 1000);
            return;
        }
        const char* shape = nullptr;
        switch (tts_lipsync_shape_) {
            case TtsLipSyncShape::CLOSED:
                shape = "half";
                tts_lipsync_shape_ = TtsLipSyncShape::HALF_RISING;
                break;
            case TtsLipSyncShape::HALF_RISING:
                shape = "open";
                tts_lipsync_shape_ = TtsLipSyncShape::OPEN;
                break;
            case TtsLipSyncShape::OPEN:
                shape = "half";
                tts_lipsync_shape_ = TtsLipSyncShape::HALF_FALLING;
                break;
            case TtsLipSyncShape::HALF_FALLING:
            default:
                shape = "closed";
                tts_lipsync_shape_ = TtsLipSyncShape::CLOSED;
                break;
        }
        SetMouthShape(shape);
        if (tts_lipsync_active_.load(std::memory_order_acquire)) {
            esp_timer_start_once(tts_lipsync_timer_,
                                 (uint64_t)TTS_LIPSYNC_STEP_MS * 1000);
        }
    }

    void EnsureTtsLipSyncTimer() {
        if (tts_lipsync_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = &StackChanBoard::TtsLipSyncStepCb,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "tts_lipsync",
                .skip_unhandled_events = true,
            };
            ESP_ERROR_CHECK(esp_timer_create(&args, &tts_lipsync_timer_));
        }
    }

    void StartTtsLipSync() {
        if (display_ == nullptr) {
            ESP_LOGD(TAG, "StartTtsLipSync ignored: display_ not ready");
            return;
        }
        EnsureTtsLipSyncTimer();
        if (tts_lipsync_active_.exchange(true, std::memory_order_acq_rel)) {
            // Already active (e.g. duplicate tts.start); nothing to do.
            return;
        }
        // Pause autonomous blink so BlinkStepAdvance()'s
        // RestoreCurrentFaceLocked() does not overwrite the mouth overlay.
        // blink_desired_ remembers the user's intent for restore at stop.
        StopBlinkTimer();
        // Start the cycle from a known resting position so the first audible
        // frame opens the mouth from closed.
        tts_lipsync_shape_ = TtsLipSyncShape::CLOSED;
        SetMouthShape("closed");
        esp_timer_start_once(tts_lipsync_timer_,
                             (uint64_t)TTS_LIPSYNC_STEP_MS * 1000);
        ESP_LOGI(TAG, "TTS lip-sync STARTED (cycle=%d ms)",
                 TTS_LIPSYNC_STEP_MS);
    }

    void StopTtsLipSync() {
        if (!tts_lipsync_active_.exchange(false, std::memory_order_acq_rel)) {
            return;  // already stopped
        }
        if (tts_lipsync_timer_ != nullptr) {
            esp_timer_stop(tts_lipsync_timer_);
        }
        // If a user-issued mouth sequence is in flight (we were yielding
        // our frames to it via the mouth_seq_active_ guard in
        // TtsLipSyncStepAdvance), let the sequence task own both the
        // mouth shape and the blink restore at sequence end. Touching
        // either here would race the sequence:
        //   - SetMouthShape("closed") would clobber the user's current
        //     frame mid-sequence;
        //   - StartBlinkTimer() would let BlinkStepAdvance()'s
        //     RestoreCurrentFaceLocked() overwrite the mouth overlay
        //     before the sequence finishes drawing (Phase 2 trade-off).
        // The sequence task already restores blink from blink_desired_
        // at its own end (see MouthSequenceTaskLoop), so deferring is
        // safe and idempotent.
        if (mouth_seq_active_.load(std::memory_order_acquire)) {
            ESP_LOGI(TAG,
                     "TTS lip-sync STOPPED (mouth_seq active; deferring "
                     "mouth + blink restore to sequence end)");
            return;
        }
        // Snap back to a closed mouth so the device does not freeze on a
        // half-open frame.
        if (display_ != nullptr) {
            SetMouthShape("closed");
        }
        // Restore blink based on the user's most recent intent (mirrors the
        // mouth-sequence playback task's restore semantics).
        if (blink_desired_.load(std::memory_order_acquire)) {
            StartBlinkTimer();
        }
        ESP_LOGI(TAG, "TTS lip-sync STOPPED");
    }

    // ---- Phase 2: lip-sync sequence playback (Issue #5) ----------------
    //
    // set_mouth_sequence accepts a list of {shape, duration_ms} pairs and
    // walks through it on a dedicated FreeRTOS task. Each step swaps the
    // mouth-only image and waits duration_ms before advancing. Walking the
    // queue locally avoids the per-step WebSocket RTT jitter that callers
    // see when issuing many set_mouth calls back-to-back from a TTS loop.
    //
    // Concurrency model:
    //   - mouth_seq_lock_ protects mouth_seq_pending_.
    //   - mouth_seq_signal_ is a binary semaphore that wakes the task when
    //     a new sequence has been enqueued.
    //   - mouth_seq_active_ / mouth_seq_cancel_requested_ are volatile flags
    //     read by the task between steps. Setting cancel_requested while the
    //     task is sleeping in vTaskDelay simply means the task picks up the
    //     cancel at the next slice boundary (kMouthCancelSliceMs apart).
    //   - Re-entry semantics: a fresh set_mouth_sequence call replaces the
    //     pending queue and marks cancel_requested so the task drops the
    //     remainder of the current sequence and starts the new one.
    //   - Interrupt sources: set_mouth, set_avatar, and set_mouth_sequence
    //     all call RequestMouthSequenceCancel() before mutating display
    //     state, so a sequence in flight is cleanly preempted.
    //
    // Trade-offs:
    //   - The MCP Property type system does not support array values, so
    //     the gateway serialises `steps` to a JSON string and passes it as
    //     `steps_json`. Validation happens here once, atomically: if any
    //     step is malformed the whole call is rejected and nothing is
    //     queued (no half-played sequences).
    //   - Autonomous blink is paused while a sequence plays because the
    //     blink state machine ends by calling RestoreCurrentFaceLocked(),
    //     which would replace the active mouth overlay with the resting
    //     face image (see Phase 2 comment near BlinkStepAdvance()).
    //   - The final shape is held after the sequence finishes; callers
    //     that want the mouth to close at the end should append a
    //     {"closed", N} step explicitly. This keeps the primitive composable
    //     with future expression-style use cases (e.g. ending on an open
    //     smile).
    static constexpr int kMaxMouthSequenceSteps = 256;
    static constexpr int kMouthStepMinMs = 10;
    static constexpr int kMouthStepMaxMs = 10000;
    static constexpr uint32_t kMouthCancelSliceMs = 20;

    struct MouthStep {
        std::string shape;
        uint32_t duration_ms;
    };

    struct MouthSequenceEnqueueResult {
        bool ok;
        std::string error;
        int queued_steps;
        uint32_t total_duration_ms;
    };

    TaskHandle_t mouth_seq_task_ = nullptr;
    SemaphoreHandle_t mouth_seq_lock_ = nullptr;     // protects mouth_seq_pending_ + generation
    SemaphoreHandle_t mouth_seq_signal_ = nullptr;   // binary semaphore: wake the task
    std::vector<MouthStep> mouth_seq_pending_;
    std::atomic<bool> mouth_seq_active_{false};
    std::atomic<bool> mouth_seq_cancel_requested_{false};
    // Generation counter bumped under mouth_seq_lock_ by every preemption
    // path (set_mouth, set_avatar, fresh set_mouth_sequence). The playback
    // task latches a snapshot at sequence start and re-checks before every
    // SetMouthShape() call, so a preempt issued in the 0..kMouthCancelSliceMs
    // window between the last cancel-flag check and the next SetMouthShape
    // call still aborts the current frame draw. Without this, the task
    // could draw one stale mouth frame after the user-issued set_mouth /
    // set_avatar handler had already returned.
    std::atomic<uint32_t> mouth_seq_generation_{0};
    // User's explicitly-requested blink state, independent of whether
    // a mouth sequence is currently suppressing the timer. set_blink
    // updates this; the playback task restores StartBlinkTimer() at
    // sequence end iff this is true. Without this split a set_blink
    // call issued during a sequence is silently overwritten by the
    // pre-sequence snapshot when the task finishes.
    std::atomic<bool> blink_desired_{false};

    // Mark any in-flight or pending sequence for cancellation. Safe to
    // call from any thread. Takes mouth_seq_lock_ so that:
    //   - mouth_seq_pending_ is cleared atomically (callers that issue
    //     set_mouth / set_avatar in the brief window between
    //     EnqueueMouthSequence() returning and the task waking up don't
    //     get overwritten by the queued-but-not-yet-active sequence);
    //   - mouth_seq_cancel_requested_ is set so the task aborts at the
    //     next slice boundary if it is already active;
    //   - mouth_seq_generation_ is bumped under release ordering so the
    //     task observes a stale generation at its next per-step check
    //     and skips the remaining SetMouthShape() calls.
    // Idempotent under repeated calls.
    void RequestMouthSequenceCancel() {
        if (mouth_seq_lock_ == nullptr) {
            return;
        }
        if (xSemaphoreTake(mouth_seq_lock_, portMAX_DELAY) == pdTRUE) {
            mouth_seq_pending_.clear();
            mouth_seq_cancel_requested_.store(true, std::memory_order_release);
            mouth_seq_generation_.fetch_add(1, std::memory_order_release);
            xSemaphoreGive(mouth_seq_lock_);
        }
    }

    // Parse and validate a JSON-serialised sequence, then atomically
    // replace mouth_seq_pending_ and signal the playback task. Returns
    // a populated MouthSequenceEnqueueResult; on validation failure
    // nothing is queued.
    MouthSequenceEnqueueResult EnqueueMouthSequence(const std::string& steps_json) {
        MouthSequenceEnqueueResult r{false, std::string(), 0, 0};

        cJSON* root = cJSON_Parse(steps_json.c_str());
        if (root == nullptr) {
            r.error = "steps must be a JSON array (parse failed)";
            return r;
        }
        if (!cJSON_IsArray(root)) {
            r.error = "steps must be a JSON array";
            cJSON_Delete(root);
            return r;
        }
        int n = cJSON_GetArraySize(root);
        if (n < 1 || n > kMaxMouthSequenceSteps) {
            r.error = std::string("steps length out of range (1..") +
                      std::to_string(kMaxMouthSequenceSteps) + ")";
            cJSON_Delete(root);
            return r;
        }

        std::vector<MouthStep> parsed;
        parsed.reserve(static_cast<size_t>(n));
        uint32_t total = 0;
        for (int i = 0; i < n; ++i) {
            cJSON* item = cJSON_GetArrayItem(root, i);
            if (!cJSON_IsObject(item)) {
                r.error = std::string("step[") + std::to_string(i) + "] must be an object";
                cJSON_Delete(root);
                return r;
            }
            cJSON* shape = cJSON_GetObjectItem(item, "shape");
            cJSON* dur = cJSON_GetObjectItem(item, "duration_ms");
            if (!cJSON_IsString(shape) || shape->valuestring == nullptr) {
                r.error = std::string("step[") + std::to_string(i) + "].shape must be a string";
                cJSON_Delete(root);
                return r;
            }
            if (!cJSON_IsNumber(dur)) {
                r.error = std::string("step[") + std::to_string(i) + "].duration_ms must be an integer";
                cJSON_Delete(root);
                return r;
            }
            if (MouthShapeToIndex(shape->valuestring) < 0) {
                r.error = std::string("step[") + std::to_string(i) +
                          "].shape unknown: '" + shape->valuestring +
                          "' (allowed: closed, half, open, e, u)";
                cJSON_Delete(root);
                return r;
            }
            int d = dur->valueint;
            if (d < kMouthStepMinMs || d > kMouthStepMaxMs) {
                r.error = std::string("step[") + std::to_string(i) +
                          "].duration_ms out of range (" +
                          std::to_string(kMouthStepMinMs) + ".." +
                          std::to_string(kMouthStepMaxMs) + ")";
                cJSON_Delete(root);
                return r;
            }
            parsed.push_back({std::string(shape->valuestring),
                              static_cast<uint32_t>(d)});
            total += static_cast<uint32_t>(d);
        }
        cJSON_Delete(root);

        if (mouth_seq_lock_ == nullptr || mouth_seq_signal_ == nullptr ||
            mouth_seq_task_ == nullptr) {
            r.error = "mouth sequence task not initialised";
            return r;
        }

        // Atomically replace the pending queue and mark any in-flight
        // sequence for cancellation so it stops at the next slice. The
        // generation bump is what makes a fresh enqueue preempt the
        // currently-playing sequence even between cancel-flag checks
        // and SetMouthShape() calls (per-step generation re-check in
        // MouthSequenceTaskLoop).
        if (xSemaphoreTake(mouth_seq_lock_, portMAX_DELAY) == pdTRUE) {
            mouth_seq_pending_ = std::move(parsed);
            if (mouth_seq_active_.load(std::memory_order_acquire)) {
                mouth_seq_cancel_requested_.store(true, std::memory_order_release);
            }
            mouth_seq_generation_.fetch_add(1, std::memory_order_release);
            xSemaphoreGive(mouth_seq_lock_);
        }
        // Wake the task. If the task is already running through a previous
        // sequence, it will pick up the new pending queue after observing
        // cancel_requested at the next slice and looping back.
        xSemaphoreGive(mouth_seq_signal_);

        r.ok = true;
        r.queued_steps = n;
        r.total_duration_ms = total;
        return r;
    }

    static void MouthSequenceTaskTrampoline(void* arg) {
        static_cast<StackChanBoard*>(arg)->MouthSequenceTaskLoop();
    }

    void MouthSequenceTaskLoop() {
        for (;;) {
            // Wait until something is enqueued (or self-signaled at the
            // tail of a previous run when more pending was discovered).
            xSemaphoreTake(mouth_seq_signal_, portMAX_DELAY);

            // Drain whatever is pending right now into a local copy so
            // we can release the lock before walking the sequence. Latch
            // the generation under the same lock so we can reject any
            // newer preempt at the next per-step check.
            std::vector<MouthStep> seq;
            uint32_t my_generation = 0;
            if (xSemaphoreTake(mouth_seq_lock_, portMAX_DELAY) == pdTRUE) {
                seq = std::move(mouth_seq_pending_);
                mouth_seq_pending_.clear();
                mouth_seq_cancel_requested_.store(false, std::memory_order_release);
                mouth_seq_active_.store(!seq.empty(), std::memory_order_release);
                my_generation = mouth_seq_generation_.load(std::memory_order_acquire);
                xSemaphoreGive(mouth_seq_lock_);
            }
            if (seq.empty()) {
                continue;
            }

            // Pause autonomous blink for the duration of the sequence so
            // BlinkStepAdvance()'s RestoreCurrentFaceLocked() does not
            // overwrite the active mouth overlay. Note: we no longer
            // snapshot blink_enabled_ here — the user's intent is read
            // from blink_desired_ at sequence end so calls to set_blink
            // made during playback are honoured.
            StopBlinkTimer();

            for (const auto& step : seq) {
                // Re-check cancel + generation right before each frame
                // draw. Any preempt issued between this check and the
                // previous SetMouthShape() will be observed here, so we
                // never draw a frame after a newer set_mouth / set_avatar
                // / set_mouth_sequence handler has returned to the caller.
                if (mouth_seq_cancel_requested_.load(std::memory_order_acquire) ||
                    mouth_seq_generation_.load(std::memory_order_acquire) != my_generation) {
                    break;
                }
                SetMouthShape(step.shape.c_str());
                // Sleep in small slices so cancel is observed quickly.
                uint32_t remaining = step.duration_ms;
                while (remaining > 0 &&
                       !mouth_seq_cancel_requested_.load(std::memory_order_acquire) &&
                       mouth_seq_generation_.load(std::memory_order_acquire) == my_generation) {
                    uint32_t slice = remaining > kMouthCancelSliceMs
                                         ? kMouthCancelSliceMs
                                         : remaining;
                    vTaskDelay(pdMS_TO_TICKS(slice));
                    remaining -= slice;
                }
            }

            // Restore blink according to the user's most recent intent,
            // not a snapshot taken before the sequence started. This way
            // a set_blink(true/false) issued during the sequence is the
            // one that wins at the end.
            if (blink_desired_.load(std::memory_order_acquire)) {
                StartBlinkTimer();
            }

            // If a fresh sequence was enqueued during playback, the
            // cancel path above will have left it in mouth_seq_pending_.
            // Self-signal so the next outer-loop iteration picks it up
            // immediately rather than parking on the semaphore.
            bool has_more = false;
            if (xSemaphoreTake(mouth_seq_lock_, portMAX_DELAY) == pdTRUE) {
                mouth_seq_active_.store(false, std::memory_order_release);
                has_more = !mouth_seq_pending_.empty();
                xSemaphoreGive(mouth_seq_lock_);
            }
            if (has_more) {
                xSemaphoreGive(mouth_seq_signal_);
            }
        }
    }

    void InitializeMouthSequenceTask() {
        if (mouth_seq_task_ != nullptr) {
            return;
        }
        mouth_seq_lock_ = xSemaphoreCreateMutex();
        mouth_seq_signal_ = xSemaphoreCreateBinary();
        if (mouth_seq_lock_ == nullptr || mouth_seq_signal_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create mouth sequence sync primitives");
            if (mouth_seq_lock_ != nullptr) {
                vSemaphoreDelete(mouth_seq_lock_);
                mouth_seq_lock_ = nullptr;
            }
            if (mouth_seq_signal_ != nullptr) {
                vSemaphoreDelete(mouth_seq_signal_);
                mouth_seq_signal_ = nullptr;
            }
            return;
        }
        BaseType_t ok = xTaskCreate(&StackChanBoard::MouthSequenceTaskTrampoline,
                                    "mouth_seq", 4096, this,
                                    tskIDLE_PRIORITY + 2,
                                    &mouth_seq_task_);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "Failed to create mouth_seq task");
            vSemaphoreDelete(mouth_seq_lock_);
            mouth_seq_lock_ = nullptr;
            vSemaphoreDelete(mouth_seq_signal_);
            mouth_seq_signal_ = nullptr;
            mouth_seq_task_ = nullptr;
        } else {
            ESP_LOGI(TAG, "Mouth sequence task ready");
        }
    }

    static bool IsGatewayUrlForced() {
#if defined(CONFIG_FORCE_DEFAULT_WEBSOCKET_URL) && defined(CONFIG_DEFAULT_WEBSOCKET_URL)
        return CONFIG_DEFAULT_WEBSOCKET_URL[0] != '\0';
#else
        return false;
#endif
    }

    static bool IsGatewayFallbackUrlForced() {
#if defined(CONFIG_FORCE_DEFAULT_WEBSOCKET_URL) && defined(CONFIG_DEFAULT_WEBSOCKET_FALLBACK_URL)
        return CONFIG_DEFAULT_WEBSOCKET_FALLBACK_URL[0] != '\0';
#else
        return false;
#endif
    }

    static bool IsGatewayTokenForced() {
#if defined(CONFIG_FORCE_DEFAULT_WEBSOCKET_URL) && defined(CONFIG_DEFAULT_WEBSOCKET_TOKEN)
        return CONFIG_DEFAULT_WEBSOCKET_TOKEN[0] != '\0';
#else
        return false;
#endif
    }

    static bool IsGatewayForceMode() {
        return IsGatewayUrlForced() || IsGatewayFallbackUrlForced() || IsGatewayTokenForced();
    }

    static bool IsGatewayDiscoveryCompiledIn() {
#if defined(CONFIG_STACKCHAN_MDNS_DISCOVERY) && CONFIG_STACKCHAN_MDNS_DISCOVERY
        return true;
#else
        return false;
#endif
    }

    static bool IsGatewayDiscoveryEnabled(const std::string& url) {
        // Only a forced primary URL suppresses the NVS/mDNS candidate path.
        return IsGatewayDiscoveryCompiledIn() && !IsGatewayUrlForced() && url.empty();
    }

    static void AddGatewayForcedKeys(cJSON* root) {
        cJSON* forced_keys = cJSON_CreateArray();
        if (IsGatewayUrlForced()) {
            cJSON_AddItemToArray(forced_keys, cJSON_CreateString("url"));
        }
        if (IsGatewayFallbackUrlForced()) {
            cJSON_AddItemToArray(forced_keys, cJSON_CreateString("fallback_url"));
        }
        if (IsGatewayTokenForced()) {
            cJSON_AddItemToArray(forced_keys, cJSON_CreateString("token"));
        }
        cJSON_AddItemToObject(root, "forced_keys", forced_keys);
    }

    static void AddGatewayForcedKeyNote(cJSON* notes,
                                        const char* key,
                                        bool provided,
                                        bool forced) {
        if (!provided || !forced) {
            return;
        }
        std::string note = std::string(key) +
            " is overridden by the Kconfig default at connect time until a non-force build is flashed.";
        cJSON_AddItemToArray(notes, cJSON_CreateString(note.c_str()));
    }

    static void AddGatewayRuntimeContext(cJSON* root, const std::string& url) {
        bool force_mode = IsGatewayForceMode();
        cJSON_AddBoolToObject(root, "force_mode", force_mode);
        AddGatewayForcedKeys(root);
        cJSON_AddBoolToObject(root, "discovery_compiled_in", IsGatewayDiscoveryCompiledIn());
        cJSON_AddBoolToObject(root, "discovery_enabled", IsGatewayDiscoveryEnabled(url));
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        ESP_LOGI(TAG, "Registering StackChan MCP tools...");

        mcp_server.AddTool(
            "self.display.show_image",
            "Download a 320x240 RGB565 picture (153600 bytes) from url and show...",
            PropertyList({
                Property("url", kPropertyTypeString),
                Property("token", kPropertyTypeString, std::string("")),
                Property("seconds", kPropertyTypeInteger, 10, 0, 3600),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                if (overlay_image_loading_.exchange(true)) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", "busy: previous image still loading");
                    return root;
                }
                auto* job = new OverlayImageJob{
                    this,
                    properties["url"].value<std::string>(),
                    properties["token"].value<std::string>(),
                    properties["seconds"].value<int>(),
                };
                if (xTaskCreate(OverlayImageTask, "overlay_img", 6144, job, 4, nullptr) != pdPASS) {
                    delete job;
                    overlay_image_loading_.store(false);
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", "task_create_failed");
                    return root;
                }
                cJSON_AddBoolToObject(root, "ok", true);
                cJSON_AddStringToObject(root, "status", "loading");
                if (!overlay_image_last_error_.empty()) {
                    cJSON_AddStringToObject(root, "previous_error", overlay_image_last_error_.c_str());
                }
                return root;
            });

        mcp_server.AddTool(
            "self.display.show_gif",
            "Download an animated GIF (prepared by the gateway) and play it full screen.",
            PropertyList({
                Property("url", kPropertyTypeString),
                Property("token", kPropertyTypeString, std::string("")),
                Property("seconds", kPropertyTypeInteger, 15, 0, 3600),
                Property("loops", kPropertyTypeInteger, 0, 0, 1000),
                Property("fast", kPropertyTypeBoolean, false),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto job = std::make_unique<GifJob>();
                job->url = properties["url"].value<std::string>();
                job->token = properties["token"].value<std::string>();
                job->seconds = properties["seconds"].value<int>();
                job->loops = properties["loops"].value<int>();
                job->fast = properties["fast"].value<bool>();
                cJSON* root = cJSON_CreateObject();
                const bool ok = QueueGif(std::move(job));
                cJSON_AddBoolToObject(root, "ok", ok);
                cJSON_AddStringToObject(root, "status", ok ? "loading" : "task_create_failed");
                if (gif_fps_x10_.load() > 0) {
                    cJSON_AddNumberToObject(root, "previous_fps", gif_fps_x10_.load() / 10.0);
                    cJSON_AddStringToObject(root, "previous_stats", gif_last_stats_.c_str());
                }
                if (!gif_last_error_.empty()) {
                    cJSON_AddStringToObject(root, "previous_error", gif_last_error_.c_str());
                }
                return root;
            });

        mcp_server.AddTool(
            "self.display.show_text",
            "Show a caption above the face.",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("seconds", kPropertyTypeInteger, 10, 0, 3600),
                Property("position", kPropertyTypeString, std::string("bottom")),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                bool ok = ShowOverlayText(properties["text"].value<std::string>(),
                                          properties["seconds"].value<int>(),
                                          properties["position"].value<std::string>());
                cJSON_AddBoolToObject(root, "ok", ok);
                return root;
            });

#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        mcp_server.AddTool(
            "self.display.set_animation",
            "Diagnostics: set the face animation period in ms (0 = no timed...",
            PropertyList({Property("interval_ms", kPropertyTypeInteger, 200, 0, 5000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                proc_face_anim_ms_.store(properties["interval_ms"].value<int>());
                if (proc_face_anim_task_ != nullptr) xTaskNotifyGive(proc_face_anim_task_);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                cJSON_AddNumberToObject(root, "interval_ms", proc_face_anim_ms_.load());
                return root;
            });

#if CONFIG_STACKCHAN_DIAGNOSTIC_TOOLS
        mcp_server.AddTool(
            "self.display.get_face_stats",
            "Diagnostics: face frames drawn since the last call, with average and...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                const uint32_t frames = face_stat_frames_.exchange(0);
                const uint32_t rsum = face_stat_render_us_sum_.exchange(0);
                const uint32_t lsum = face_stat_lock_us_sum_.exchange(0);
                const uint32_t rmax = face_stat_render_us_max_.exchange(0);
                const uint32_t lmax = face_stat_lock_us_max_.exchange(0);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "frames", frames);
                cJSON_AddNumberToObject(root, "render_avg_ms", frames ? rsum / 1000.0 / frames : 0);
                cJSON_AddNumberToObject(root, "render_max_ms", rmax / 1000.0);
                cJSON_AddNumberToObject(root, "lock_wait_avg_ms", frames ? lsum / 1000.0 / frames : 0);
                cJSON_AddNumberToObject(root, "lock_wait_max_ms", lmax / 1000.0);
                cJSON_AddNumberToObject(root, "interval_ms", proc_face_anim_ms_.load());
                cJSON_AddNumberToObject(root, "free_internal_kb", heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
                cJSON_AddNumberToObject(root, "min_free_internal_kb", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024);
                return root;
            });
#endif  // CONFIG_STACKCHAN_DIAGNOSTIC_TOOLS
#endif

#if CONFIG_STACKCHAN_DIAGNOSTIC_TOOLS
        mcp_server.AddTool(
            "self.system.get_task_stats",
            "Diagnostics: CPU share of every task over a 2 s window, with the...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                struct Sample { std::string name; uint32_t runtime; BaseType_t core; UBaseType_t prio; };
                auto snapshot = [](std::vector<Sample>& out, uint32_t& total) {
                    UBaseType_t n = uxTaskGetNumberOfTasks() + 4;
                    std::vector<TaskStatus_t> st(n);
                    n = uxTaskGetSystemState(st.data(), n, &total);
                    out.clear();
                    for (UBaseType_t i = 0; i < n; i++) {
                        out.push_back({st[i].pcTaskName, st[i].ulRunTimeCounter,
                                       xTaskGetCoreID(st[i].xHandle), st[i].uxCurrentPriority});
                    }
                };
                std::vector<Sample> a, b;
                uint32_t ta = 0, tb = 0;
                snapshot(a, ta);
                vTaskDelay(pdMS_TO_TICKS(2000));
                snapshot(b, tb);
                const double window = static_cast<double>(tb - ta);
                cJSON* root = cJSON_CreateObject();
                cJSON* tasks = cJSON_AddArrayToObject(root, "tasks");
                for (const auto& s2 : b) {
                    uint32_t before = 0;
                    for (const auto& s1 : a) if (s1.name == s2.name) { before = s1.runtime; break; }
                    double pct = window > 0 ? (s2.runtime - before) * 100.0 / window : 0;
                    if (pct < 0.5) continue;
                    cJSON* t = cJSON_CreateObject();
                    cJSON_AddStringToObject(t, "task", s2.name.c_str());
                    cJSON_AddNumberToObject(t, "cpu_pct_of_one_core", static_cast<int>(pct * 10) / 10.0);
                    cJSON_AddNumberToObject(t, "core", s2.core > 1 ? -1 : s2.core);
                    cJSON_AddNumberToObject(t, "prio", s2.prio);
                    cJSON_AddItemToArray(tasks, t);
                }
                return root;
            });

        mcp_server.AddTool(
            "self.audio.set_wake_word",
            "Diagnostics: turn wake-word detection on or off until the next state...",
            PropertyList({Property("enabled", kPropertyTypeBoolean)}),
            [](const PropertyList& properties) -> ReturnValue {
                bool en = properties["enabled"].value<bool>();
                Application::GetInstance().GetAudioService().EnableWakeWordDetection(en);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

        mcp_server.AddTool(
            "self.display.set_lcd_refresh",
            "Diagnostics: enable/disable the periodic LCD state re-send.",
            PropertyList({Property("enabled", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                lcd_refresh_enabled_.store(properties["enabled"].value<bool>());
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

#endif  // CONFIG_STACKCHAN_DIAGNOSTIC_TOOLS
#if CONFIG_STACKCHAN_BODY_SENSORS
        mcp_server.AddTool(
            "self.sensors.get_motion",
            "IMU reading: accel (g), gyro (deg/s), and whether it is being held.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                float a[3], g[3];
                taskENTER_CRITICAL(&body_mux_);
                std::copy(body_accel_, body_accel_ + 3, a);
                std::copy(body_gyro_, body_gyro_ + 3, g);
                taskEXIT_CRITICAL(&body_mux_);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", imu_ != nullptr);
                cJSON_AddItemToObject(root, "accel_g", cJSON_CreateFloatArray(a, 3));
                cJSON_AddItemToObject(root, "gyro_dps", cJSON_CreateFloatArray(g, 3));
                cJSON_AddBoolToObject(root, "held", motion_detector_.held());
                cJSON_AddNumberToObject(root, "knock_threshold_g", motion_detector_.knock_threshold());
                // Biggest jolt since the last read: knock, then read it to
                // see what threshold would catch your knocks.
                cJSON_AddNumberToObject(root, "peak_jolt_g", motion_detector_.TakePeakJerk());
                return root;
            });

        mcp_server.AddTool(
            "self.sensors.set_knock_threshold",
            "Set how hard a knock must be, in milli-g of sample-to-sample jolt "
            "(default 250). Lower = more sensitive. Knock, then read peak_jolt_g "
            "from get_motion to choose. Saved across reboots.",
            PropertyList({Property("threshold_mg", kPropertyTypeInteger, 250, 50, 3000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                const int mg = properties["threshold_mg"].value<int>();
                motion_detector_.set_knock_threshold(mg / 1000.0f);
                Settings settings("body", true);
                settings.SetInt("knock_mg", mg);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "knock_threshold_g", mg / 1000.0f);
                return root;
            });

        mcp_server.AddTool(
            "self.sensors.get_proximity",
            "Proximity (raw 0..2047, bigger = closer) and whether something is near.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                taskENTER_CRITICAL(&body_mux_);
                const int raw = body_proximity_;
                taskEXIT_CRITICAL(&body_mux_);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", prox_ != nullptr);
                cJSON_AddNumberToObject(root, "proximity", raw);
                cJSON_AddNumberToObject(root, "baseline", proximity_detector_.baseline());
                cJSON_AddBoolToObject(root, "near", proximity_detector_.near());
                return root;
            });
#endif

        mcp_server.AddTool(
            "self.display.set_ticker",
            "Scroll text along the bottom of the screen like a news ticker.",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("seconds", kPropertyTypeInteger, 0, 0, 86400),
                Property("speed", kPropertyTypeInteger, 60, 10, 400),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                SetTicker(properties["text"].value<std::string>(),
                          properties["seconds"].value<int>(),
                          properties["speed"].value<int>());
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

        mcp_server.AddTool(
            "self.display.set_subtitle",
            "Show a subtitle in the bottom strip (temporarily replacing any...",
            PropertyList({
                Property("text", kPropertyTypeString),
                Property("duration_ms", kPropertyTypeInteger, 0, 0, 600000),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                SetSubtitle(properties["text"].value<std::string>(),
                            properties["duration_ms"].value<int>());
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        mcp_server.AddTool(
            "self.display.set_face_style",
            "Switch the drawn face's look: stackchan (white dots and line) or...",
            PropertyList({Property("style", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                bool ok = SetFaceStyle(properties["style"].value<std::string>());
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) cJSON_AddStringToObject(root, "error", "unknown style; use stackchan, bender or hal");
                return root;
            });
#endif

        mcp_server.AddTool(
            "self.display.set_link_status",
            "Tell the robot whether a Claude client is connected to the gateway.",
            PropertyList({Property("claude_connected", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                claude_connected_.store(properties["claude_connected"].value<bool>());
                UpdateStatusIcons();
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

        mcp_server.AddTool(
            "self.display.clear_overlay",
            "Hide any picture or caption shown with show_image / show_text.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                HideOverlayImage();
                HideOverlayText();
                HideCanvas();
                StopGif();
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

        mcp_server.AddTool(
            "self.display.canvas",
            "Draw a JSON array of shapes on the canvas above the face.",
            PropertyList({Property("ops", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                int drawn = 0;
                std::string error = DrawCanvas(properties["ops"].value<std::string>(), &drawn);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", error.empty());
                cJSON_AddNumberToObject(root, "drawn", drawn);
                if (!error.empty()) cJSON_AddStringToObject(root, "error", error.c_str());
                return root;
            });

        mcp_server.AddTool(
            "self.display.canvas_hide",
            "Hide the drawing canvas and show the face again.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                HideCanvas();
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                return root;
            });

#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        mcp_server.AddTool(
            "self.display.set_gaze",
            "Point the face's eyes without moving the head.",
            PropertyList({
                Property("x", kPropertyTypeInteger, 0, -100, 100),
                Property("y", kPropertyTypeInteger, 0, -100, 100),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                gaze_x_ = properties["x"].value<int>() * 0.3f;
                gaze_y_ = properties["y"].value<int>() * 0.2f;
                bool ok = false;
                if (display_ != nullptr) {
                    DisplayLockGuard lock(display_);
                    ok = avatar_img_ != nullptr && current_avatar_face_ != "off" &&
                         RenderAvatarLocked();
                }
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", ok);
                return root;
            });
#endif

        mcp_server.AddTool(
            "self.screen.set_off_timeout",
            "Set the StackChan idle screen-off timeout in seconds.",
            PropertyList({
                Property("seconds", kPropertyTypeInteger, 0,
                         STACKCHAN_SCREEN_OFF_MAX_TIMEOUT_SECONDS)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int timeout_seconds = properties["seconds"].value<int>();
                bool applied = SetScreenOffTimeoutSeconds(timeout_seconds);
                return GetScreenOffTimeoutJson(applied);
            });

        mcp_server.AddTool(
            "self.screen.get_off_timeout",
            "Get the effective NVS-backed StackChan idle screen-off timeout...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                return GetScreenOffTimeoutJson();
            });

        mcp_server.AddTool(
            "self.gateway_config.get",
            "Read the NVS-backed WebSocket gateway connection settings.",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                Settings settings("websocket", false);
                std::string url = settings.GetString("url");
                std::string fallback_url = settings.GetString("fallback_url");
                bool token_set = !settings.GetString("token").empty();
                std::string connected_url = Application::GetInstance().GetConnectedGatewayUrl();

                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "url", url.c_str());
                cJSON_AddStringToObject(root, "fallback_url", fallback_url.c_str());
                cJSON_AddBoolToObject(root, "token_set", token_set);
                AddGatewayRuntimeContext(root, url);
                if (connected_url.empty()) {
                    cJSON_AddNullToObject(root, "connected_url");
                } else {
                    cJSON_AddStringToObject(root, "connected_url", connected_url.c_str());
                }
                return root;
            });

        mcp_server.AddTool(
            "self.gateway_config.set",
            "Update the NVS-backed WebSocket gateway connection settings.",
            PropertyList({Property("url", kPropertyTypeString, std::string()),
                          Property("fallback_url", kPropertyTypeString, std::string()),
                          Property("token", kPropertyTypeString, std::string())}),
            [](const PropertyList& properties) -> ReturnValue {
                const auto& url_property = properties["url"];
                const auto& fallback_url_property = properties["fallback_url"];
                const auto& token_property = properties["token"];
                bool url_provided = url_property.was_provided();
                bool fallback_url_provided = fallback_url_property.was_provided();
                bool token_provided = token_property.was_provided();
                if (!url_provided && !fallback_url_provided && !token_provided) {
                    throw std::invalid_argument(
                        "At least one of url, fallback_url, or token must be provided");
                }

                Settings settings("websocket", true);
                std::string url = settings.GetString("url");
                std::string fallback_url = settings.GetString("fallback_url");
                std::string token = settings.GetString("token");

                cJSON* updated_keys = cJSON_CreateArray();
                auto apply_string = [&](const char* response_key,
                                        const char* nvs_key,
                                        bool provided,
                                        const std::string& requested_value,
                                        std::string& current_value) {
                    if (!provided) {
                        return;
                    }
                    if (requested_value.empty()) {
                        settings.EraseKey(nvs_key);
                        current_value.clear();
                    } else {
                        settings.SetString(nvs_key, requested_value);
                        current_value = requested_value;
                    }
                    cJSON_AddItemToArray(updated_keys, cJSON_CreateString(response_key));
                };

                apply_string("url", "url", url_provided,
                             url_property.value<std::string>(), url);
                apply_string("fallback_url", "fallback_url", fallback_url_provided,
                             fallback_url_property.value<std::string>(), fallback_url);
                apply_string("token", "token", token_provided,
                             token_property.value<std::string>(), token);

                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                cJSON_AddItemToObject(root, "updated_keys", updated_keys);
                cJSON_AddStringToObject(root, "url", url.c_str());
                cJSON_AddStringToObject(root, "fallback_url", fallback_url.c_str());
                cJSON_AddBoolToObject(root, "token_set", !token.empty());
                AddGatewayRuntimeContext(root, url);
                cJSON_AddStringToObject(root, "takes_effect", "next_reconnect");

                cJSON* notes = cJSON_CreateArray();
                if (!url.empty() && !IsGatewayUrlForced()) {
                    cJSON_AddItemToArray(
                        notes,
                        cJSON_CreateString(
                            "mDNS discovery is disabled until websocket.url is cleared."));
                }
                AddGatewayForcedKeyNote(notes, "url", url_provided, IsGatewayUrlForced());
                AddGatewayForcedKeyNote(notes, "fallback_url", fallback_url_provided,
                                        IsGatewayFallbackUrlForced());
                AddGatewayForcedKeyNote(notes, "token", token_provided, IsGatewayTokenForced());
                cJSON_AddItemToObject(root, "notes", notes);
                return root;
            });

        mcp_server.AddTool(
            "self.robot.get_touch_sensor_enabled",
            "Read the NVS-backed head-touch sensor enable flag.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "enabled",
                                      touch_sensor_enabled_.load(std::memory_order_acquire));
                return root;
            });

        mcp_server.AddTool(
            "self.robot.set_touch_sensor_enabled",
            "Update the NVS-backed head-touch sensor enable flag.",
            PropertyList({Property("enabled", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = properties["enabled"].value<bool>();
                Settings settings("touch", true);
                settings.SetBool("enabled", enabled);
                touch_sensor_enabled_.store(enabled, std::memory_order_release);

                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", true);
                cJSON_AddBoolToObject(root, "enabled", enabled);
                cJSON_AddStringToObject(root, "takes_effect", "immediate");
                cJSON_AddStringToObject(root, "persistence", "nvs");
                return root;
            });

        // Set head angles (yaw, pitch in degrees)
        // SCS0009: 1 step = 0.3125 degrees, so 1 degree = 3.2 steps (= 16/5)
        // yaw: -90..90 degrees (no hardware restriction). pitch: two-tier
        // guard — see SAFE_PITCH_MIN/MAX (hard clamp for mechanical safety)
        // and RECOMMENDED_PITCH_MIN/MAX (M5Stack-documented operating sweet
        // spot) above, plus Issue #80 / #98.
        mcp_server.AddTool(
            "self.robot.set_head_angles",
            "Move the head: yaw -90..90, pitch clamped to the safe range; optional speed_dps.",
            // Pitch schema range is intentionally permissive across the
            // entire `int` value range (std::numeric_limits<int>::min/max):
            // the authoritative Tier 1 enforcement lives in the handler
            // below (silent clamp to [SAFE_PITCH_MIN, SAFE_PITCH_MAX] with
            // ESP_LOGW). Any narrower range would cause McpServer::Property
            // to reject sufficiently-extreme requests (e.g. pitch=200 or
            // pitch=INT_MIN) before the handler can run, leaving the
            // Tier 1 clamp / log unreachable for those callers and
            // contradicting the tool-description / README claim that
            // out-of-range requests are silently clamped with ESP_LOGW —
            // see Issue #98 (three adversarial-review rounds zeroed in on
            // this exact contract, including the int-boundary corners) and
            // PR #81's defense-in-depth requirement that every servo-write
            // boundary be guarded inside the firmware regardless of
            // caller behavior.
            PropertyList({Property("yaw", kPropertyTypeInteger, 0, -90, 90),
                          Property("pitch", kPropertyTypeInteger, 0,
                                   std::numeric_limits<int>::min(),
                                   std::numeric_limits<int>::max()),
                          Property("speed_dps", kPropertyTypeInteger, 0,
                                   std::numeric_limits<int>::min(),
                                   std::numeric_limits<int>::max())}),
            [this](const PropertyList& properties) -> ReturnValue {
                int yaw = properties["yaw"].value<int>();
                int pitch = properties["pitch"].value<int>();
                int speed_dps = properties["speed_dps"].value<int>();
                // Issue #80 / #98: two-tier pitch guard.
                //
                // Tier 1 (hard clamp): silently clamp to [SAFE_PITCH_MIN,
                // SAFE_PITCH_MAX] and ESP_LOGW. PitchDegToPos() clamps
                // again at the servo-write boundary (defense-in-depth);
                // doing it here lets us log the original out-of-range
                // value. See the SAFE_PITCH_MIN/MAX comment block above.
                if (pitch < SAFE_PITCH_MIN) {
                    ESP_LOGW(TAG, "set_head_angles: pitch=%d below SAFE_PITCH_MIN=%d, clamping (servo end-stop protection)",
                             pitch, SAFE_PITCH_MIN);
                    pitch = SAFE_PITCH_MIN;
                }
                if (pitch > SAFE_PITCH_MAX) {
                    ESP_LOGW(TAG, "set_head_angles: pitch=%d above SAFE_PITCH_MAX=%d, clamping (servo end-stop protection)",
                             pitch, SAFE_PITCH_MAX);
                    pitch = SAFE_PITCH_MAX;
                }
                // Tier 2 (recommended-range soft signal): inside the hard
                // clamp but outside the M5Stack-documented operating
                // range — accept the value and emit an ESP_LOGI so callers
                // can notice the deviation without blocking the motion.
                if (pitch < RECOMMENDED_PITCH_MIN || pitch > RECOMMENDED_PITCH_MAX) {
                    ESP_LOGI(TAG, "set_head_angles: pitch=%d outside M5Stack-recommended range %d..%d (within hard clamp %d..%d); acceptable but not officially endorsed",
                             pitch, RECOMMENDED_PITCH_MIN, RECOMMENDED_PITCH_MAX,
                             SAFE_PITCH_MIN, SAFE_PITCH_MAX);
                }
                int yaw_pos = YawDegToPos(yaw);
                int pitch_pos = PitchDegToPos(pitch);
                if (speed_dps > 0) {
                    WriteHeadAngles(yaw, pitch, speed_dps);
                } else {
                    WriteHeadAngles(yaw, pitch);
                }
                bool yaw_motion_started = false;
                bool pitch_motion_started = false;
                if (servo_ok_) {
                    xSemaphoreTake(motion_mutex_, portMAX_DELAY);
                    yaw_motion_started = yaw_motion_.moving;
                    pitch_motion_started = pitch_motion_.moving;
                    xSemaphoreGive(motion_mutex_);
                }
                ESP_LOGI(TAG, "set_head_angles: yaw=%d (pos=%d) motion_started=%d, pitch=%d (pos=%d) motion_started=%d, uart=%d, servo_ok=%d",
                         yaw, yaw_pos, yaw_motion_started, pitch, pitch_pos, pitch_motion_started, (int)SERVO_UART_NUM, servo_ok_);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "servo_init_ok", servo_ok_);
                cJSON_AddNumberToObject(root, "uart_num", (int)SERVO_UART_NUM);
                cJSON_AddNumberToObject(root, "yaw_pos", yaw_pos);
                cJSON_AddNumberToObject(root, "pitch_pos", pitch_pos);
                cJSON_AddNumberToObject(root, "yaw_motion_started", yaw_motion_started ? 1 : 0);
                cJSON_AddNumberToObject(root, "pitch_motion_started", pitch_motion_started ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.wifi.set_power_save",
            "Set the ESP32 WiFi power-save mode at runtime.",
            PropertyList({Property("mode", kPropertyTypeString)}),
            [](const PropertyList& properties) -> ReturnValue {
                std::string mode_str = properties["mode"].value<std::string>();
                wifi_ps_type_t target;
                if (mode_str == "none") {
                    target = WIFI_PS_NONE;
                } else if (mode_str == "min_modem") {
                    target = WIFI_PS_MIN_MODEM;
                } else if (mode_str == "max_modem") {
                    target = WIFI_PS_MAX_MODEM;
                } else {
                    cJSON* root = cJSON_CreateObject();
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", "invalid mode (expected 'none' | 'min_modem' | 'max_modem')");
                    return root;
                }

                auto ps_str = [](wifi_ps_type_t mode) -> const char* {
                    switch (mode) {
                    case WIFI_PS_NONE:
                        return "none";
                    case WIFI_PS_MIN_MODEM:
                        return "min_modem";
                    case WIFI_PS_MAX_MODEM:
                        return "max_modem";
                    default:
                        return "unknown";
                    }
                };

                wifi_ps_type_t previous = WIFI_PS_MIN_MODEM;
                esp_err_t get_result = esp_wifi_get_ps(&previous);
                esp_err_t set_result = esp_wifi_set_ps(target);
                const char* previous_str = get_result == ESP_OK ? ps_str(previous) : "unknown";

                ESP_LOGI(TAG, "wifi.set_power_save: target=%s previous=%s set_result=%d get_result=%d",
                         ps_str(target), previous_str, (int)set_result, (int)get_result);

                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", set_result == ESP_OK);
                cJSON_AddStringToObject(root, "previous", previous_str);
                cJSON_AddStringToObject(root, "current", set_result == ESP_OK ? ps_str(target) : ps_str(previous));
                if (set_result != ESP_OK) {
                    cJSON_AddNumberToObject(root, "esp_err", (int)set_result);
                }
                return root;
            });

        // Get current head angles
        mcp_server.AddTool(
            "self.robot.get_head_angles",
            "Get the current head angles (yaw, pitch) of the robot in degrees.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                // Issue #123: retry ReadPos a few times before falling back
                // to an explicit error reply. Single-call ReadPos failures
                // (e.g. servo mid-motion, transient bus contention) are a
                // known transient mode that InitializeServo() already treats
                // as warning-and-continue; the previous behaviour of running
                // `ReadPos==-1` through the same `(pos-zero)*5/16` math as a
                // valid position produced sentinel `{-144,-194}` that was
                // indistinguishable from a genuine bus hang at the MCP layer
                // (see #1 / #100 / #118 hang judgments).
                constexpr int kReadPosRetryMax = 3;
                constexpr uint32_t kReadPosRetryDelayMs = 50;

                int yaw_pos = -1;
                int pitch_pos = -1;
                int yaw_attempts = 0;
                int pitch_attempts = 0;
                if (servo_ok_) {
                    xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);
                    for (int i = 0; i < kReadPosRetryMax; i++) {
                        yaw_attempts = i + 1;
                        yaw_pos = scs_bus_.ReadPos(SERVO_YAW_ID);
                        if (yaw_pos >= 0) break;
                        if (i + 1 < kReadPosRetryMax) {
                            vTaskDelay(pdMS_TO_TICKS(kReadPosRetryDelayMs));
                        }
                    }
                    for (int i = 0; i < kReadPosRetryMax; i++) {
                        pitch_attempts = i + 1;
                        pitch_pos = scs_bus_.ReadPos(SERVO_PITCH_ID);
                        if (pitch_pos >= 0) break;
                        if (i + 1 < kReadPosRetryMax) {
                            vTaskDelay(pdMS_TO_TICKS(kReadPosRetryDelayMs));
                        }
                    }
                    xSemaphoreGive(scs_bus_mutex_);
                }

                cJSON* root = cJSON_CreateObject();
                const bool yaw_ok = yaw_pos >= 0;
                const bool pitch_ok = pitch_pos >= 0;
                if (yaw_ok && pitch_ok) {
                    int yaw = (yaw_pos - 460) * 5 / 16;
                    int pitch = (pitch_pos - 620) * 5 / 16;
                    cJSON_AddNumberToObject(root, "yaw", yaw);
                    cJSON_AddNumberToObject(root, "pitch", pitch);
                } else {
                    cJSON_AddNullToObject(root, "yaw");
                    cJSON_AddNullToObject(root, "pitch");
                    char err[160];
                    snprintf(err, sizeof(err),
                             "ReadPos failed: yaw_raw=%d (attempts=%d) "
                             "pitch_raw=%d (attempts=%d) servo_ok=%d",
                             yaw_pos, yaw_attempts,
                             pitch_pos, pitch_attempts,
                             servo_ok_ ? 1 : 0);
                    cJSON_AddStringToObject(root, "error", err);
                    cJSON_AddBoolToObject(root, "servo_ok", servo_ok_);
                    cJSON_AddNumberToObject(root, "yaw_attempts", yaw_attempts);
                    cJSON_AddNumberToObject(root, "pitch_attempts", pitch_attempts);
                }
                char* str = cJSON_PrintUnformatted(root);
                std::string result(str);
                cJSON_free(str);
                cJSON_Delete(root);
                ESP_LOGI(TAG,
                         "get_head_angles: servo_ok=%d yaw_raw=%d (attempts=%d) "
                         "pitch_raw=%d (attempts=%d) result=%s",
                         servo_ok_ ? 1 : 0,
                         yaw_pos, yaw_attempts,
                         pitch_pos, pitch_attempts,
                         result.c_str());
                return result;
            });

        mcp_server.AddTool(
            "self.robot.set_servo_torque",
            "Enable or disable SCS0009 servo torque on the yaw / pitch axes...",
            PropertyList({Property("yaw_enabled", kPropertyTypeBoolean),
                          Property("pitch_enabled", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                bool yaw_enabled = properties["yaw_enabled"].value<bool>();
                bool pitch_enabled = properties["pitch_enabled"].value<bool>();
                ServoTorqueResult torque_result = InternalSetServoTorque(
                    yaw_enabled, pitch_enabled, ReleaseReason::kManual);

                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "yaw_enabled", yaw_enabled);
                cJSON_AddBoolToObject(root, "pitch_enabled", pitch_enabled);
                cJSON_AddNumberToObject(root, "yaw_bus_return",
                                        torque_result.yaw_bus_return);
                cJSON_AddNumberToObject(root, "pitch_bus_return",
                                        torque_result.pitch_bus_return);
                cJSON_AddBoolToObject(root, "servo_ok", servo_ok_);
                // Issue #171: ok counts an idempotent no-op as success but a
                // wait-budget exhaustion as failure (the requested torque
                // transition did not actually happen on the bus).
                cJSON_AddBoolToObject(
                    root, "ok",
                    servo_ok_ && (torque_result.idempotent_short_circuit ||
                                  (torque_result.yaw_ok &&
                                   torque_result.pitch_ok)));
                // Issue #171: the old single `short_circuited` field is
                // removed (no alias). These two orthogonal, mutually
                // exclusive flags let callers distinguish a degraded-bus
                // wait-exhaustion from an idempotent no-op success.
                cJSON_AddBoolToObject(root, "idempotent_short_circuit",
                                      torque_result.idempotent_short_circuit);
                cJSON_AddBoolToObject(root, "wait_exhausted",
                                      torque_result.wait_exhausted);
                if (!servo_ok_) {
                    cJSON_AddStringToObject(root, "error",
                                            "Servo bus not initialized.");
                }
                return root;
            });

        mcp_server.AddTool(
            "self.robot.set_auto_torque_release",
            "Enable or disable automatic SCS0009 torque release after motion idle...",
            PropertyList({Property("enabled", kPropertyTypeBoolean),
                          Property("timeout_ms", kPropertyTypeInteger,
                                   (int)AUTO_TORQUE_RELEASE_DEFAULT_MS)}),
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = properties["enabled"].value<bool>();
                int requested_timeout_ms = properties["timeout_ms"].value<int>();
                bool clamped = false;
                uint32_t timeout_ms = 0;

                if (requested_timeout_ms <
                    static_cast<int>(AUTO_TORQUE_RELEASE_MIN_MS)) {
                    timeout_ms = AUTO_TORQUE_RELEASE_MIN_MS;
                    clamped = true;
                    ESP_LOGW(TAG,
                             "set_auto_torque_release: timeout_ms=%d below "
                             "minimum %u, clamping",
                             requested_timeout_ms,
                             (unsigned)AUTO_TORQUE_RELEASE_MIN_MS);
                } else if (requested_timeout_ms >
                           static_cast<int>(AUTO_TORQUE_RELEASE_MAX_MS)) {
                    timeout_ms = AUTO_TORQUE_RELEASE_MAX_MS;
                    clamped = true;
                    ESP_LOGW(TAG,
                             "set_auto_torque_release: timeout_ms=%d above "
                             "maximum %u, clamping",
                             requested_timeout_ms,
                             (unsigned)AUTO_TORQUE_RELEASE_MAX_MS);
                } else {
                    timeout_ms = static_cast<uint32_t>(requested_timeout_ms);
                }

                bool torque_released_at_call =
                    torque_state_.load(std::memory_order_acquire) ==
                    TorqueState::kReleased;
                auto_release_timeout_ms_.store(timeout_ms,
                                               std::memory_order_release);
                auto_release_enabled_.store(enabled,
                                            std::memory_order_release);

                ESP_LOGI(TAG,
                         "set_auto_torque_release: enabled=%d "
                         "timeout_ms=%u clamped=%d "
                         "torque_released_at_call=%d",
                         enabled ? 1 : 0, (unsigned)timeout_ms,
                         clamped ? 1 : 0,
                         torque_released_at_call ? 1 : 0);

                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "enabled", enabled);
                cJSON_AddNumberToObject(root, "timeout_ms", timeout_ms);
                cJSON_AddBoolToObject(root, "clamped", clamped);
                cJSON_AddBoolToObject(root, "torque_released_at_call",
                                      torque_released_at_call);
                return root;
            });

#if CONFIG_STACKCHAN_DIAGNOSTIC_TOOLS
        // Diagnostic: toggle GPIO6 (servo TX) HIGH/LOW to verify physical signal
        mcp_server.AddTool(
            "self.robot.gpio_test",
            "Diagnostic: toggle GPIO6 (servo TX pin) HIGH/LOW 5 times at 100ms...",
            PropertyList(),
            [](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();

                gpio_num_t pin = static_cast<gpio_num_t>(SERVO_TX_PIN);

                esp_err_t err_dir = gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                cJSON_AddStringToObject(root, "set_direction", esp_err_to_name(err_dir));
                cJSON_AddNumberToObject(root, "pin", SERVO_TX_PIN);

                cJSON* toggles = cJSON_CreateArray();
                for (int i = 0; i < 5; i++) {
                    esp_err_t err_h = gpio_set_level(pin, 1);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_err_t err_l = gpio_set_level(pin, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    cJSON* item = cJSON_CreateObject();
                    cJSON_AddNumberToObject(item, "iter", i);
                    cJSON_AddStringToObject(item, "high", esp_err_to_name(err_h));
                    cJSON_AddStringToObject(item, "low", esp_err_to_name(err_l));
                    cJSON_AddItemToArray(toggles, item);
                }
                cJSON_AddItemToObject(root, "toggles", toggles);

                // Restore UART pin assignment after raw GPIO toggling
                esp_err_t err_restore = uart_set_pin(SERVO_UART_NUM, SERVO_TX_PIN, SERVO_RX_PIN,
                                                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
                cJSON_AddStringToObject(root, "uart_pin_restore", esp_err_to_name(err_restore));

                char* str = cJSON_PrintUnformatted(root);
                std::string result(str);
                cJSON_free(str);
                cJSON_Delete(root);
                ESP_LOGI(TAG, "gpio_test: %s", result.c_str());
                return result;
            });

        // Diagnostic: send raw bytes via uart_write_bytes, equivalent to WritePos(1, 1000, 0, 0)
        mcp_server.AddTool(
            "self.robot.uart_diag",
            "Diagnostic: send raw 8 bytes (FF FF 01 04 03 E8 00 00) directly via...",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();

                size_t buf_before = 0;
                esp_err_t err_b = ESP_ERR_INVALID_STATE;
                int written = -1;
                esp_err_t err_wait = ESP_ERR_INVALID_STATE;
                esp_err_t err_a = ESP_ERR_INVALID_STATE;
                size_t buf_after = 0;
                const uint8_t bytes[] = {0xFF, 0xFF, 0x01, 0x04, 0x03, 0xE8, 0x00, 0x00};

                if (servo_ok_) {
                    xSemaphoreTake(scs_bus_mutex_, portMAX_DELAY);

                    err_b = uart_get_buffered_data_len(SERVO_UART_NUM, &buf_before);

                    written = uart_write_bytes(SERVO_UART_NUM, (const char*)bytes, sizeof(bytes));

                    // Wait for TX FIFO drain
                    err_wait = uart_wait_tx_done(SERVO_UART_NUM, pdMS_TO_TICKS(100));

                    vTaskDelay(pdMS_TO_TICKS(20));

                    err_a = uart_get_buffered_data_len(SERVO_UART_NUM, &buf_after);

                    xSemaphoreGive(scs_bus_mutex_);
                }
                cJSON_AddStringToObject(root, "buf_before_status", esp_err_to_name(err_b));
                cJSON_AddNumberToObject(root, "buf_before", buf_before);

                cJSON_AddNumberToObject(root, "written", written);
                cJSON_AddNumberToObject(root, "expected", (int)sizeof(bytes));

                cJSON_AddStringToObject(root, "tx_done_status", esp_err_to_name(err_wait));

                cJSON_AddStringToObject(root, "buf_after_status", esp_err_to_name(err_a));
                cJSON_AddNumberToObject(root, "buf_after", buf_after);

                char* str = cJSON_PrintUnformatted(root);
                std::string result(str);
                cJSON_free(str);
                cJSON_Delete(root);
                ESP_LOGI(TAG, "uart_diag: %s", result.c_str());
                return result;
            });

        // Diagnostic: read PY32 REG_GPIO_O_L (output low byte) and report
        // whether VM EN (pin 0) is HIGH. Used to investigate "servo stops
        // moving after the first move_head" — if VM EN drops to LOW under
        // load, the servo loses power even though the I2C write succeeds.
        mcp_server.AddTool(
            "self.robot.check_vm_en",
            "Diagnostic: read PY32 REG_GPIO_O_L and report whether VM EN (pin 0 =...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                bool present = (io_expander_ != nullptr);
                cJSON_AddBoolToObject(root, "io_expander_present", present);
                if (present) {
                    uint8_t out_low = 0;
                    bool ok = io_expander_->ReadOutputLow(&out_low);
                    cJSON_AddBoolToObject(root, "i2c_read_ok", ok);
                    if (ok) {
                        cJSON_AddNumberToObject(root, "raw", out_low);
                        cJSON_AddBoolToObject(root, "vm_en_high", (out_low & 0x01) != 0);
                    }
                }
                ESP_LOGI(TAG, "check_vm_en queried");
                return root;
            });

#endif  // CONFIG_STACKCHAN_DIAGNOSTIC_TOOLS

        // Set the avatar face (one of: idle, happy, thinking, sad, surprised,
        // embarrassed, off).
        // The image is rendered as a 320x240 overlay on top of the chat UI's
        // emoji_label_ / emoji_image_; LVGL theme/Application emotion updates
        // will keep happening underneath but are visually masked.
        // 'off' hides the avatar lv_obj and disables blink so the underlying
        // xiaozhi-esp32 screens (WiFi config UI, OTA, settings) become visible.
        // A subsequent set_avatar with any other face brings it back, and
        // restores blink to whatever state it was in before going off.
        mcp_server.AddTool(
            "self.display.set_avatar",
            "Set the face (an expression name, or off to show the system UI).",
            PropertyList({Property("face", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string face = properties["face"].value<std::string>();
                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "face", face.c_str());

                bool applied = false;
                if (face == "off") {
                    // Any avatar transition supersedes an in-flight mouth
                    // sequence (per Issue #5 acceptance: "set_avatar() takes
                    // effect after the queued sequence finishes (or
                    // interrupts cleanly)").
                    RequestMouthSequenceCancel();
                    applied = SetAvatarOff();
                } else if (FaceNameToIndex(face.c_str()) >= 0) {
                    RequestMouthSequenceCancel();
                    applied = SetAvatarExpression(face.c_str());
                } else {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                        "Unknown face. Allowed: " STACKCHAN_FACE_NAMES ", off.");
                    ESP_LOGW(TAG, "set_avatar rejected: unknown face '%s'", face.c_str());
                    return root;
                }
                cJSON_AddBoolToObject(root, "ok", applied);
                if (!applied) {
                    cJSON_AddStringToObject(root, "error",
                        "Display not ready yet; retry after a moment.");
                }
                ESP_LOGI(TAG, "set_avatar: face=%s applied=%d", face.c_str(), applied);
                return root;
            });

        // Phase 2: lip-sync. Swap the avatar to one of the mouth-only frames.
        // The shape is held until the next set_avatar / set_mouth / blink, so
        // callers should drive it from their TTS / audio level loop.
        mcp_server.AddTool(
            "self.display.set_mouth",
            "Set the avatar mouth shape.",
            PropertyList({Property("mouth", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string mouth = properties["mouth"].value<std::string>();
                bool valid = (MouthShapeToIndex(mouth.c_str()) >= 0);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "mouth", mouth.c_str());
                if (!valid) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                        "Unknown mouth. Allowed: closed, half, open, e, u.");
                    ESP_LOGW(TAG, "set_mouth rejected: unknown shape '%s'", mouth.c_str());
                    return root;
                }
                // Any explicit mouth set supersedes an in-flight sequence
                // (Issue #5: set_mouth("closed") doubles as the cancellation
                // path so we don't need a separate cancel_mouth_sequence
                // tool).
                RequestMouthSequenceCancel();
                bool applied = SetMouthShape(mouth.c_str());
                cJSON_AddBoolToObject(root, "ok", applied);
                if (!applied) {
                    cJSON_AddStringToObject(root, "error",
                        "Display not ready yet; retry after a moment.");
                }
                ESP_LOGI(TAG, "set_mouth: mouth=%s applied=%d", mouth.c_str(), applied);
                return root;
            });

        // Phase 2: lip-sync sequence. Queue and play a list of
        // {shape, duration_ms} pairs locally so a TTS-driven caller can
        // ship one MCP call per utterance instead of N back-to-back
        // set_mouth calls (which suffer per-step WebSocket RTT jitter).
        // The MCP Property type system has no array kind, so the gateway
        // serialises `steps` to a JSON string and sends it as `steps_json`.
        // See Phase 2 lip-sync sequence playback comment block above for
        // the concurrency model and trade-offs (blink pause, atomic queue
        // replacement, final shape held).
        mcp_server.AddTool(
            "self.display.set_mouth_sequence",
            "Queue a lip-sync sequence and play it locally.",
            PropertyList({Property("steps_json", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string steps_json = properties["steps_json"].value<std::string>();
                MouthSequenceEnqueueResult r = EnqueueMouthSequence(steps_json);
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", r.ok);
                if (!r.ok) {
                    cJSON_AddStringToObject(root, "error", r.error.c_str());
                    ESP_LOGW(TAG, "set_mouth_sequence rejected: %s", r.error.c_str());
                } else {
                    cJSON_AddNumberToObject(root, "queued_steps", r.queued_steps);
                    cJSON_AddNumberToObject(root, "estimated_duration_ms",
                                            static_cast<double>(r.total_duration_ms));
                    ESP_LOGI(TAG, "set_mouth_sequence queued %d steps (%u ms)",
                             r.queued_steps, (unsigned)r.total_duration_ms);
                }
                return root;
            });

        // Phase 2: enable/disable autonomous blinking. When enabled, a
        // background timer fires every 3-6 s (random) and runs the four-step
        // blink sequence (half -> closed -> half -> face). Also captures
        // the user's intent into blink_desired_ so that a call issued while
        // a mouth sequence is suppressing blink is honoured when the
        // sequence finishes.
        mcp_server.AddTool(
            "self.display.set_blink",
            "Enable or disable autonomous eye blinking on the avatar.",
            PropertyList({Property("enabled", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = properties["enabled"].value<bool>();
                // blink_desired_ stays in sync with the user's intent
                // regardless of which deferral path applies, so the
                // mouth-sequence task and the avatar-fetch apply-pending
                // path both see the latest value at their respective
                // restore points.
                blink_desired_.store(enabled, std::memory_order_release);
                bool deferred_by_fetch =
                    DeferAvatarBlinkIfFetching(enabled);
                bool deferred_by_mouth_seq =
                    mouth_seq_active_.load(std::memory_order_acquire);
                if (!deferred_by_fetch && !deferred_by_mouth_seq) {
                    if (enabled) {
                        StartBlinkTimer();
                    } else {
                        StopBlinkTimer();
                    }
                }
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "enabled", enabled);
                cJSON_AddBoolToObject(root, "ok", true);
                if (deferred_by_fetch || deferred_by_mouth_seq) {
                    cJSON_AddBoolToObject(root, "deferred", true);
                }
                ESP_LOGI(TAG,
                         "set_blink: enabled=%d deferred_by_fetch=%d "
                         "deferred_by_mouth_seq=%d",
                         (int)enabled,
                         deferred_by_fetch ? 1 : 0,
                         deferred_by_mouth_seq ? 1 : 0);
                return root;
            });

        // Phase 7: head-touch (Si12T). Returns the latest debounced zone
        // states plus the most recent gesture event. Polled by the MCP client
        // to notice TAP/STROKE on the head without holding open a stream.
        mcp_server.AddTool(
            "self.touch.get_touch_state",
            "Get the current head-touch sensor state and last gesture event...",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", si12t_ok_);
                cJSON_AddBoolToObject(root, "zone0", last_zone_snapshot_[0]);
                cJSON_AddBoolToObject(root, "zone1", last_zone_snapshot_[1]);
                cJSON_AddBoolToObject(root, "zone2", last_zone_snapshot_[2]);
                cJSON_AddNumberToObject(root, "raw", last_output1_raw_);
                const char* ev = "idle";
                switch (last_event_) {
                    case TouchEvent::TAP:    ev = "tap";    break;
                    case TouchEvent::STROKE: ev = "stroke"; break;
                    case TouchEvent::IDLE:
                    default:                 ev = "idle";   break;
                }
                cJSON_AddStringToObject(root, "last_event", ev);
                int64_t age_ms = -1;
                if (last_event_us_ != 0) {
                    int64_t now_us = (int64_t)esp_timer_get_time();
                    age_ms = (now_us - (int64_t)last_event_us_) / 1000;
                    if (age_ms < 0) age_ms = 0;
                }
                cJSON_AddNumberToObject(root, "last_event_age_ms", (double)age_ms);
                return root;
            });

        // ---- LED tools (12x WS2812C on the StackChan base) ----
        // The strip is driven by the PY32 IO expander on its pin 13, not by
        // an ESP32 GPIO. Updates are non-latching writes into the PY32 LED
        // RAM followed by a single RefreshLeds() to strobe the strip. All
        // four tools refresh implicitly so the LLM gets WYSIWYG behaviour.
        mcp_server.AddTool(
            "self.led.set_color",
            "Set a single RGB LED on the StackChan base.",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, RGB_LED_COUNT - 1),
                Property("r", kPropertyTypeInteger, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 255),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", rgb_ok_);
                if (!rgb_ok_) {
                    cJSON_AddStringToObject(root, "error", "RGB strip not available (PY32 init failed?)");
                    return root;
                }
                int index = properties["index"].value<int>();
                uint8_t r = ClampByte(properties["r"].value<int>());
                uint8_t g = ClampByte(properties["g"].value<int>());
                uint8_t b = ClampByte(properties["b"].value<int>());
                bool ok_w = io_expander_->SetLedColor((uint8_t)index, r, g, b);
                bool ok_r = ok_w ? io_expander_->RefreshLeds() : false;
                cJSON_AddBoolToObject(root, "ok", ok_w && ok_r);
                cJSON_AddNumberToObject(root, "index", index);
                ESP_LOGI(TAG, "set_led: index=%d rgb=(%u,%u,%u) ok=%d", index, r, g, b, ok_w && ok_r);
                return root;
            });

        mcp_server.AddTool(
            "self.led.set_all",
            "Set all 12 RGB LEDs on the StackChan base to the same color.",
            PropertyList({
                Property("r", kPropertyTypeInteger, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 255),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", rgb_ok_);
                if (!rgb_ok_) {
                    cJSON_AddStringToObject(root, "error", "RGB strip not available (PY32 init failed?)");
                    return root;
                }
                uint8_t r = ClampByte(properties["r"].value<int>());
                uint8_t g = ClampByte(properties["g"].value<int>());
                uint8_t b = ClampByte(properties["b"].value<int>());
                uint8_t buf[RGB_LED_COUNT * 2];
                uint8_t pair[2];
                PackRgb565(r, g, b, pair);
                for (int i = 0; i < RGB_LED_COUNT; i++) {
                    buf[i * 2 + 0] = pair[0];
                    buf[i * 2 + 1] = pair[1];
                }
                bool ok_w = io_expander_->SetLedData(buf, sizeof(buf));
                bool ok_r = ok_w ? io_expander_->RefreshLeds() : false;
                cJSON_AddBoolToObject(root, "ok", ok_w && ok_r);
                ESP_LOGI(TAG, "set_all_leds: rgb=(%u,%u,%u) ok=%d", r, g, b, ok_w && ok_r);
                return root;
            });

        // Batch set: accepts a JSON-encoded array of 12 [r,g,b] triples.
        // Single I2C burst + one refresh — use this for animations or any
        // multi-color pattern to avoid 12x round-trips. Missing trailing
        // entries are left at their previous color (PY32 RAM is sticky).
        mcp_server.AddTool(
            "self.led.set_many",
            "Set multiple RGB LEDs in one shot.",
            PropertyList({Property("colors", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", rgb_ok_);
                if (!rgb_ok_) {
                    cJSON_AddStringToObject(root, "error", "RGB strip not available (PY32 init failed?)");
                    return root;
                }
                std::string json = properties["colors"].value<std::string>();
                cJSON* arr = cJSON_Parse(json.c_str());
                if (arr == nullptr || !cJSON_IsArray(arr)) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                        "colors must be a JSON array of [r,g,b] triples");
                    if (arr != nullptr) cJSON_Delete(arr);
                    return root;
                }
                int n = cJSON_GetArraySize(arr);
                if (n > RGB_LED_COUNT) n = RGB_LED_COUNT;

                // Validate every entry FIRST and pack into a local buffer.
                // Only after the whole array is known good do we touch the
                // PY32 — that way a malformed entry at i=5 cannot leave
                // LEDs 0..4 mutated (atomic semantics, same as
                // set_mouth_sequence). cJSON_IsNumber is required because
                // valueint silently returns 0 for non-number nodes (string,
                // null, bool), so without the guard a payload like
                // [["255",0,0]] would write black and report ok=true.
                uint8_t buf[RGB_LED_COUNT * 2];   // 24 bytes, fits the cap
                bool parse_ok = true;
                for (int i = 0; i < n; i++) {
                    cJSON* triple = cJSON_GetArrayItem(arr, i);
                    if (!cJSON_IsArray(triple) || cJSON_GetArraySize(triple) < 3) {
                        parse_ok = false;
                        break;
                    }
                    cJSON* jr = cJSON_GetArrayItem(triple, 0);
                    cJSON* jg = cJSON_GetArrayItem(triple, 1);
                    cJSON* jb = cJSON_GetArrayItem(triple, 2);
                    if (!cJSON_IsNumber(jr) || !cJSON_IsNumber(jg) || !cJSON_IsNumber(jb)) {
                        parse_ok = false;
                        break;
                    }
                    PackRgb565(ClampByte(jr->valueint),
                               ClampByte(jg->valueint),
                               ClampByte(jb->valueint),
                               &buf[i * 2]);
                }
                cJSON_Delete(arr);

                // Single I2C burst for the validated prefix, then one latch.
                // n=0 is treated as success (gateway schema enforces
                // minItems=1, but a direct device caller could hit this).
                bool ok_w = false, ok_r = false;
                if (parse_ok && n > 0) {
                    ok_w = io_expander_->SetLedData(buf, (size_t)(n * 2));
                    ok_r = ok_w ? io_expander_->RefreshLeds() : false;
                }
                bool ok = parse_ok && (n == 0 || (ok_w && ok_r));
                cJSON_AddBoolToObject(root, "ok", ok);
                cJSON_AddNumberToObject(root, "written", ok ? n : 0);
                if (!parse_ok) {
                    cJSON_AddStringToObject(root, "error",
                        "Each entry must be a [r,g,b] triple of integers");
                }
                ESP_LOGI(TAG, "set_many_leds: written=%d/%d ok=%d",
                         ok ? n : 0, n, ok);
                return root;
            });

        mcp_server.AddTool(
            "self.led.clear",
            "Turn off all 12 RGB LEDs on the StackChan base.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", rgb_ok_);
                if (!rgb_ok_) {
                    cJSON_AddStringToObject(root, "error", "RGB strip not available (PY32 init failed?)");
                    return root;
                }
                uint8_t buf[RGB_LED_COUNT * 2] = {0};
                bool ok_w = io_expander_->SetLedData(buf, sizeof(buf));
                bool ok_r = ok_w ? io_expander_->RefreshLeds() : false;
                cJSON_AddBoolToObject(root, "ok", ok_w && ok_r);
                ESP_LOGI(TAG, "clear_leds: ok=%d", ok_w && ok_r);
                return root;
            });

        // ---- Generic I2C bus tools (Grove Port A) ----
        // Expose the external Port A I2C bus to the MCP client so that
        // attached M5Stack Unit modules (ENV III, ToF, gas sensor, PaHub,
        // etc.) can be driven from the gateway / host side without
        // recompiling and re-flashing per Unit. The on-board IC bus (PMIC,
        // touch, IMU, AW9523, audio codec) is on a physically separate I2C
        // controller and is NOT reachable from these tools by construction.

        mcp_server.AddTool(
            "self.i2c.scan",
            "Scan the external I2C bus on Grove Port A and return all 7-bit...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON* addrs = cJSON_CreateArray();
                int found = 0;
                // Probe 0x08..0x77 (skip I2C reserved 0x00-0x07 / 0x78-0x7F).
                // 200 ms per-probe timeout matches the boot-time I2cDetect()
                // and reliably catches slower Units (RCWL-9620 etc.).
                for (uint8_t addr = 0x08; addr < 0x78; addr++) {
                    esp_err_t ret = i2c_master_probe(port_a_i2c_bus_, addr, pdMS_TO_TICKS(200));
                    if (ret == ESP_OK) {
                        cJSON_AddItemToArray(addrs, cJSON_CreateNumber(addr));
                        found++;
                    }
                }
                cJSON_AddBoolToObject(root, "ok", true);
                cJSON_AddItemToObject(root, "addresses", addrs);
                ESP_LOGI(TAG, "i2c.scan: found %d device(s) on Port A", found);
                return root;
            });

        mcp_server.AddTool(
            "self.i2c.read",
            "Read n_bytes from an I2C device at 7-bit address `addr` on Grove Port A.",
            PropertyList({
                Property("addr", kPropertyTypeInteger, 0x08, 0x77),
                Property("n_bytes", kPropertyTypeInteger, 1, 256),
                Property("scl_speed_hz", kPropertyTypeInteger, 400000, 100000, 1000000)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                uint8_t addr = static_cast<uint8_t>(props["addr"].value<int>());
                int n = props["n_bytes"].value<int>();

                i2c_device_config_t cfg = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = addr,
                    .scl_speed_hz = static_cast<uint32_t>(props["scl_speed_hz"].value<int>()),
                };
                i2c_master_dev_handle_t dev;
                esp_err_t err = i2c_master_bus_add_device(port_a_i2c_bus_, &cfg, &dev);
                if (err != ESP_OK) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                    ESP_LOGW(TAG, "i2c.read addr=0x%02X add_device failed: %s",
                             addr, esp_err_to_name(err));
                    return root;
                }

                std::vector<uint8_t> buf(static_cast<size_t>(n));
                err = i2c_master_receive(dev, buf.data(), buf.size(), 100);
                i2c_master_bus_rm_device(dev);

                if (err == ESP_OK) {
                    cJSON* bytes = cJSON_CreateArray();
                    for (uint8_t b : buf) {
                        cJSON_AddItemToArray(bytes, cJSON_CreateNumber(b));
                    }
                    cJSON_AddBoolToObject(root, "ok", true);
                    cJSON_AddItemToObject(root, "bytes", bytes);
                } else {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "i2c.read addr=0x%02X n=%d ok=%d",
                         addr, n, err == ESP_OK);
                return root;
            });

        Property i2c_write_bytes_prop(
            "bytes", kPropertyTypeArray, kPropertyElementTypeInteger, 0, 255
        );
        i2c_write_bytes_prop.set_max_items(256);  // 対称: n_bytes の read 上限と同じ
        mcp_server.AddTool(
            "self.i2c.write",
            "Write bytes to an I2C device at 7-bit address `addr` on Grove Port A.",
            PropertyList({
                Property("addr", kPropertyTypeInteger, 0x08, 0x77),
                i2c_write_bytes_prop,
                Property("scl_speed_hz", kPropertyTypeInteger, 400000, 100000, 1000000)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                uint8_t addr = static_cast<uint8_t>(props["addr"].value<int>());
                auto bytes_int = props["bytes"].value<std::vector<int>>();

                i2c_device_config_t cfg = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = addr,
                    .scl_speed_hz = static_cast<uint32_t>(props["scl_speed_hz"].value<int>()),
                };
                i2c_master_dev_handle_t dev;
                esp_err_t err = i2c_master_bus_add_device(port_a_i2c_bus_, &cfg, &dev);
                if (err != ESP_OK) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                    ESP_LOGW(TAG, "i2c.write addr=0x%02X add_device failed: %s",
                             addr, esp_err_to_name(err));
                    return root;
                }

                std::vector<uint8_t> buf;
                buf.reserve(bytes_int.size());
                for (int b : bytes_int) buf.push_back(static_cast<uint8_t>(b));

                err = i2c_master_transmit(dev, buf.data(), buf.size(), 100);
                i2c_master_bus_rm_device(dev);

                if (err == ESP_OK) {
                    cJSON_AddBoolToObject(root, "ok", true);
                } else {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "i2c.write addr=0x%02X n=%d ok=%d",
                         addr, (int)buf.size(), err == ESP_OK);
                return root;
            });

        Property i2c_wr_write_bytes_prop(
            "write_bytes", kPropertyTypeArray, kPropertyElementTypeInteger, 0, 255
        );
        i2c_wr_write_bytes_prop.set_max_items(256);  // 対称: n_bytes の read 上限と同じ
        mcp_server.AddTool(
            "self.i2c.write_read",
            "Write `write_bytes` to an I2C device at 7-bit address `addr` on...",
            PropertyList({
                Property("addr", kPropertyTypeInteger, 0x08, 0x77),
                i2c_wr_write_bytes_prop,
                Property("n_bytes", kPropertyTypeInteger, 1, 256),
                Property("scl_speed_hz", kPropertyTypeInteger, 400000, 100000, 1000000)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                uint8_t addr = static_cast<uint8_t>(props["addr"].value<int>());
                auto write_bytes_int = props["write_bytes"].value<std::vector<int>>();
                int n = props["n_bytes"].value<int>();

                i2c_device_config_t cfg = {
                    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                    .device_address = addr,
                    .scl_speed_hz = static_cast<uint32_t>(props["scl_speed_hz"].value<int>()),
                };
                i2c_master_dev_handle_t dev;
                esp_err_t err = i2c_master_bus_add_device(port_a_i2c_bus_, &cfg, &dev);
                if (err != ESP_OK) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                    ESP_LOGW(TAG, "i2c.write_read addr=0x%02X add_device failed: %s",
                             addr, esp_err_to_name(err));
                    return root;
                }

                std::vector<uint8_t> write_buf;
                write_buf.reserve(write_bytes_int.size());
                for (int b : write_bytes_int) write_buf.push_back(static_cast<uint8_t>(b));

                std::vector<uint8_t> read_buf(static_cast<size_t>(n));
                err = i2c_master_transmit_receive(dev,
                                                   write_buf.data(), write_buf.size(),
                                                   read_buf.data(), read_buf.size(),
                                                   100);
                i2c_master_bus_rm_device(dev);

                if (err == ESP_OK) {
                    cJSON* bytes = cJSON_CreateArray();
                    for (uint8_t b : read_buf) {
                        cJSON_AddItemToArray(bytes, cJSON_CreateNumber(b));
                    }
                    cJSON_AddBoolToObject(root, "ok", true);
                    cJSON_AddItemToObject(root, "bytes", bytes);
                } else {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "i2c.write_read addr=0x%02X w=%d r=%d ok=%d",
                         addr, (int)write_buf.size(), n, err == ESP_OK);
                return root;
            });

        // ---- Generic Port B WS2812 strip tools ----
        // Expose the CoreS3 Port B digital output (GPIO 9) as a generic
        // WS2812-compatible strip driver. This is independent from self.led.*,
        // which drives the 12-LED base strip through the PY32 I2C path.

        mcp_server.AddTool(
            "self.port_b.ws2812.init",
            "Initialize a WS2812-compatible LED strip connected to Port B (CoreS3...",
            PropertyList({
                Property("led_count", kPropertyTypeInteger, 1, PORT_B_WS2812_MAX_LEDS)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                uint16_t led_count = static_cast<uint16_t>(props["led_count"].value<int>());
                esp_err_t err = InitPortBWs2812(led_count);
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "available", ok);
                cJSON_AddBoolToObject(root, "ok", ok);
                cJSON_AddNumberToObject(root, "led_count", led_count);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_b.ws2812.init led_count=%u ok=%d",
                         (unsigned)led_count, ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_b.ws2812.set_pixel",
            "Set one LED in the Port B WS2812 strip buffer.",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, PORT_B_WS2812_MAX_LEDS - 1),
                Property("r", kPropertyTypeInteger, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 255),
                Property("refresh", kPropertyTypeBoolean, false)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", ws2812_ok_);
                if (!ws2812_ok_ || ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                                            "Port B WS2812 strip not initialized.");
                    return root;
                }
                int index = props["index"].value<int>();
                if (index >= ws2812_led_count_) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", "index out of range");
                    ESP_LOGW(TAG, "port_b.ws2812.set_pixel index=%d out of range (led_count=%u)",
                             index, (unsigned)ws2812_led_count_);
                    return root;
                }
                uint8_t r = ClampByte(props["r"].value<int>());
                uint8_t g = ClampByte(props["g"].value<int>());
                uint8_t b = ClampByte(props["b"].value<int>());
                bool refresh = props["refresh"].value<bool>();
                esp_err_t err = led_strip_set_pixel(ws2812_handle_, index, r, g, b);
                if (err == ESP_OK && refresh) {
                    err = led_strip_refresh(ws2812_handle_);
                }
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_b.ws2812.set_pixel index=%d rgb=(%u,%u,%u) refresh=%d ok=%d",
                         index, r, g, b, refresh ? 1 : 0, ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_b.ws2812.set_strip",
            "Set multiple LEDs in the Port B WS2812 strip and refresh immediately.",
            PropertyList({Property("colors", kPropertyTypeString)}),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", ws2812_ok_);
                if (!ws2812_ok_ || ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddNumberToObject(root, "written", 0);
                    cJSON_AddStringToObject(root, "error",
                                            "Port B WS2812 strip not initialized.");
                    return root;
                }

                std::string json = props["colors"].value<std::string>();
                cJSON* arr = cJSON_Parse(json.c_str());
                if (arr == nullptr || !cJSON_IsArray(arr)) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddNumberToObject(root, "written", 0);
                    cJSON_AddStringToObject(root, "error",
                                            "colors must be a JSON array of [r,g,b] triples");
                    if (arr != nullptr) cJSON_Delete(arr);
                    return root;
                }

                int n = cJSON_GetArraySize(arr);
                if (n > ws2812_led_count_) n = ws2812_led_count_;
                std::vector<uint8_t> rgb;
                rgb.reserve(static_cast<size_t>(n) * 3);
                bool parse_ok = true;
                for (int i = 0; i < n; i++) {
                    cJSON* triple = cJSON_GetArrayItem(arr, i);
                    if (!cJSON_IsArray(triple) || cJSON_GetArraySize(triple) != 3) {
                        parse_ok = false;
                        break;
                    }
                    uint8_t r = 0, g = 0, b = 0;
                    if (!JsonByte(cJSON_GetArrayItem(triple, 0), &r) ||
                        !JsonByte(cJSON_GetArrayItem(triple, 1), &g) ||
                        !JsonByte(cJSON_GetArrayItem(triple, 2), &b)) {
                        parse_ok = false;
                        break;
                    }
                    rgb.push_back(r);
                    rgb.push_back(g);
                    rgb.push_back(b);
                }
                cJSON_Delete(arr);

                if (!parse_ok) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddNumberToObject(root, "written", 0);
                    cJSON_AddStringToObject(root, "error",
                                            "Each entry must be a [r,g,b] triple of integers 0..255");
                    ESP_LOGW(TAG, "port_b.ws2812.set_strip rejected malformed colors payload");
                    return root;
                }

                esp_err_t err = ESP_OK;
                for (int i = 0; i < n; i++) {
                    size_t offset = static_cast<size_t>(i) * 3;
                    err = led_strip_set_pixel(ws2812_handle_, i,
                                              rgb[offset + 0],
                                              rgb[offset + 1],
                                              rgb[offset + 2]);
                    if (err != ESP_OK) {
                        break;
                    }
                }
                if (err == ESP_OK) {
                    err = led_strip_refresh(ws2812_handle_);
                }

                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                cJSON_AddNumberToObject(root, "written", ok ? n : 0);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_b.ws2812.set_strip written=%d ok=%d",
                         ok ? n : 0, ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_b.ws2812.refresh",
            "Refresh the Port B WS2812 strip, latching the current buffered...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", ws2812_ok_);
                if (!ws2812_ok_ || ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                                            "Port B WS2812 strip not initialized.");
                    return root;
                }
                esp_err_t err = led_strip_refresh(ws2812_handle_);
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_b.ws2812.refresh ok=%d", ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_b.ws2812.clear",
            "Turn off every LED in the Port B WS2812 strip and refresh...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", ws2812_ok_);
                if (!ws2812_ok_ || ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                                            "Port B WS2812 strip not initialized.");
                    return root;
                }
                esp_err_t err = led_strip_clear(ws2812_handle_);
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_b.ws2812.clear ok=%d", ok ? 1 : 0);
                return root;
            });

        // ---- Generic Port C WS2812 strip tools ----
        // Expose the CoreS3 Port C signal 1 (GPIO 17) as a generic
        // WS2812-compatible strip driver. This is independent from self.led.*,
        // which drives the 12-LED base strip through the PY32 I2C path.

        mcp_server.AddTool(
            "self.port_c.ws2812.init",
            "Initialize a WS2812-compatible LED strip connected to Port C (CoreS3...",
            PropertyList({
                Property("led_count", kPropertyTypeInteger, 1, PORT_C_WS2812_MAX_LEDS)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                uint16_t led_count = static_cast<uint16_t>(props["led_count"].value<int>());
                esp_err_t err = InitPortCWs2812(led_count);
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "available", ok);
                cJSON_AddBoolToObject(root, "ok", ok);
                cJSON_AddNumberToObject(root, "led_count", led_count);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_c.ws2812.init led_count=%u ok=%d",
                         (unsigned)led_count, ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_c.ws2812.set_pixel",
            "Set one LED in the Port C WS2812 strip buffer.",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, PORT_C_WS2812_MAX_LEDS - 1),
                Property("r", kPropertyTypeInteger, 0, 255),
                Property("g", kPropertyTypeInteger, 0, 255),
                Property("b", kPropertyTypeInteger, 0, 255),
                Property("refresh", kPropertyTypeBoolean, false)
            }),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", port_c_ws2812_ok_);
                if (!port_c_ws2812_ok_ || port_c_ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                                            "Port C WS2812 strip not initialized.");
                    return root;
                }
                int index = props["index"].value<int>();
                if (index >= port_c_ws2812_led_count_) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error", "index out of range");
                    ESP_LOGW(TAG, "port_c.ws2812.set_pixel index=%d out of range (led_count=%u)",
                             index, (unsigned)port_c_ws2812_led_count_);
                    return root;
                }
                uint8_t r = ClampByte(props["r"].value<int>());
                uint8_t g = ClampByte(props["g"].value<int>());
                uint8_t b = ClampByte(props["b"].value<int>());
                bool refresh = props["refresh"].value<bool>();
                esp_err_t err = led_strip_set_pixel(port_c_ws2812_handle_, index, r, g, b);
                if (err == ESP_OK && refresh) {
                    err = led_strip_refresh(port_c_ws2812_handle_);
                }
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_c.ws2812.set_pixel index=%d rgb=(%u,%u,%u) refresh=%d ok=%d",
                         index, r, g, b, refresh ? 1 : 0, ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_c.ws2812.set_strip",
            "Set multiple LEDs in the Port C WS2812 strip and refresh immediately.",
            PropertyList({Property("colors", kPropertyTypeString)}),
            [this](const PropertyList& props) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", port_c_ws2812_ok_);
                if (!port_c_ws2812_ok_ || port_c_ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddNumberToObject(root, "written", 0);
                    cJSON_AddStringToObject(root, "error",
                                            "Port C WS2812 strip not initialized.");
                    return root;
                }

                std::string json = props["colors"].value<std::string>();
                cJSON* arr = cJSON_Parse(json.c_str());
                if (arr == nullptr || !cJSON_IsArray(arr)) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddNumberToObject(root, "written", 0);
                    cJSON_AddStringToObject(root, "error",
                                            "colors must be a JSON array of [r,g,b] triples");
                    if (arr != nullptr) cJSON_Delete(arr);
                    return root;
                }

                int n = cJSON_GetArraySize(arr);
                if (n > port_c_ws2812_led_count_) n = port_c_ws2812_led_count_;
                std::vector<uint8_t> rgb;
                rgb.reserve(static_cast<size_t>(n) * 3);
                bool parse_ok = true;
                for (int i = 0; i < n; i++) {
                    cJSON* triple = cJSON_GetArrayItem(arr, i);
                    if (!cJSON_IsArray(triple) || cJSON_GetArraySize(triple) != 3) {
                        parse_ok = false;
                        break;
                    }
                    uint8_t r = 0, g = 0, b = 0;
                    if (!JsonByte(cJSON_GetArrayItem(triple, 0), &r) ||
                        !JsonByte(cJSON_GetArrayItem(triple, 1), &g) ||
                        !JsonByte(cJSON_GetArrayItem(triple, 2), &b)) {
                        parse_ok = false;
                        break;
                    }
                    rgb.push_back(r);
                    rgb.push_back(g);
                    rgb.push_back(b);
                }
                cJSON_Delete(arr);

                if (!parse_ok) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddNumberToObject(root, "written", 0);
                    cJSON_AddStringToObject(root, "error",
                                            "Each entry must be a [r,g,b] triple of integers 0..255");
                    ESP_LOGW(TAG, "port_c.ws2812.set_strip rejected malformed colors payload");
                    return root;
                }

                esp_err_t err = ESP_OK;
                for (int i = 0; i < n; i++) {
                    size_t offset = static_cast<size_t>(i) * 3;
                    err = led_strip_set_pixel(port_c_ws2812_handle_, i,
                                              rgb[offset + 0],
                                              rgb[offset + 1],
                                              rgb[offset + 2]);
                    if (err != ESP_OK) {
                        break;
                    }
                }
                if (err == ESP_OK) {
                    err = led_strip_refresh(port_c_ws2812_handle_);
                }

                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                cJSON_AddNumberToObject(root, "written", ok ? n : 0);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_c.ws2812.set_strip written=%d ok=%d",
                         ok ? n : 0, ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_c.ws2812.refresh",
            "Refresh the Port C WS2812 strip, latching the current buffered...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", port_c_ws2812_ok_);
                if (!port_c_ws2812_ok_ || port_c_ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                                            "Port C WS2812 strip not initialized.");
                    return root;
                }
                esp_err_t err = led_strip_refresh(port_c_ws2812_handle_);
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_c.ws2812.refresh ok=%d", ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.port_c.ws2812.clear",
            "Turn off every LED in the Port C WS2812 strip and refresh...",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "available", port_c_ws2812_ok_);
                if (!port_c_ws2812_ok_ || port_c_ws2812_handle_ == nullptr) {
                    cJSON_AddBoolToObject(root, "ok", false);
                    cJSON_AddStringToObject(root, "error",
                                            "Port C WS2812 strip not initialized.");
                    return root;
                }
                esp_err_t err = led_strip_clear(port_c_ws2812_handle_);
                bool ok = (err == ESP_OK);
                cJSON_AddBoolToObject(root, "ok", ok);
                if (!ok) {
                    cJSON_AddStringToObject(root, "error", esp_err_to_name(err));
                }
                ESP_LOGI(TAG, "port_c.ws2812.clear ok=%d", ok ? 1 : 0);
                return root;
            });

        mcp_server.AddTool(
            "self.power.set_charge_enabled",
            "Enable or disable AXP2101 cell battery charging (register 0x18 bit1...",
            PropertyList({Property("enabled", kPropertyTypeBoolean)}),
            [this](const PropertyList& properties) -> ReturnValue {
                bool enabled = properties["enabled"].value<bool>();
                pmic_->SetChargeEnabled(enabled);
                bool actual = pmic_->IsChargeEnabled();

                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "ok", actual == enabled);
                cJSON_AddBoolToObject(root, "charge_enabled", actual);
                cJSON_AddNumberToObject(root, "reg_0x18", pmic_->GetChargeControlRegister());
                cJSON_AddStringToObject(root, "takes_effect", "immediate");
                cJSON_AddStringToObject(root, "persistence", "volatile_resets_on_boot");
                cJSON_AddBoolToObject(root, "auto_enabled",
                                      STACKCHAN_CHARGE_AUTO != 0);
                cJSON_AddBoolToObject(root,
                                      "manual_may_be_overridden_by_auto",
                                      STACKCHAN_CHARGE_AUTO != 0);
                cJSON_AddStringToObject(
                    root, "automatic_control_note",
                    (STACKCHAN_CHARGE_AUTO != 0)
                        ? "Manual setting may be overwritten by the next automatic decision."
                        : "Automatic charge control is disabled.");
                ESP_LOGI(TAG, "set_charge_enabled: requested=%d actual=%d",
                         enabled, actual);
                return root;
            });

        mcp_server.AddTool(
            "self.power.get_charge_state",
            "Read the current AXP2101 charge-control state.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                cJSON* root = cJSON_CreateObject();
                cJSON_AddBoolToObject(root, "charge_enabled", pmic_->IsChargeEnabled());
                cJSON_AddNumberToObject(root, "reg_0x18", pmic_->GetChargeControlRegister());
                cJSON_AddBoolToObject(root, "charging", pmic_->IsCharging());
                cJSON_AddBoolToObject(root, "discharging", pmic_->IsDischarging());
                cJSON_AddBoolToObject(root, "charge_done", pmic_->IsChargingDone());
                int battery_level = -1;
                bool battery_level_available =
                    ReadBatteryLevel(&battery_level);
                cJSON_AddNumberToObject(root, "battery_level", battery_level);
                cJSON_AddBoolToObject(root, "battery_level_available",
                                      battery_level_available);
                cJSON_AddBoolToObject(root, "auto_enabled",
                                      STACKCHAN_CHARGE_AUTO != 0);
                cJSON_AddNumberToObject(root, "auto_on_below",
                                        STACKCHAN_CHARGE_ON_BELOW);
                cJSON_AddNumberToObject(root, "auto_off_above",
                                        STACKCHAN_CHARGE_OFF_ABOVE);
                cJSON_AddNumberToObject(root, "auto_interval_seconds", 60);
                ChargeAutoDecision last_decision =
                    last_charge_auto_decision_.load(
                        std::memory_order_acquire);
                cJSON_AddStringToObject(
                    root, "last_auto_decision",
                    ChargeAutoDecisionName(last_decision));
                cJSON_AddNumberToObject(
                    root, "last_auto_battery_level",
                    last_charge_auto_battery_level_.load(
                        std::memory_order_relaxed));
                cJSON_AddBoolToObject(root,
                                      "manual_may_be_overridden_by_auto",
                                      STACKCHAN_CHARGE_AUTO != 0);
                return root;
            });

        ESP_LOGI(TAG, "StackChan MCP tools registered");
    }

public:
    StackChanBoard() {
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializePortAI2c();
        InitializeAxp2101();
        InitializeAw9523();
        // I2cDetect() moved AFTER all I2C device initializations.
        // The 128-address probe (i2c_master_probe over the whole bus) was
        // leaving PY32 (0x6F) in a half-finished slave state, so the
        // following transmit_receive (REG_VERSION via Repeated Start)
        // timed out (0x103). Doing the scan after IOExpander/Si12T init
        // preserves the boot-log debug info without poisoning subsequent
        // register reads. Si12T (0x68) is unaffected on the same bus,
        // but moving the scan is safer for any future I2C peripheral too.
        InitializeSpi();
        InitializeIli9342Display();
        InitializeCamera();
        InitializeFt6336TouchPad();
        GetBacklight()->RestoreBrightness();
        InitializeScreenOffTimer();
        InitializeIOExpander();
        InitializeServo();
        InitializeTouchSettings();
        InitializeSi12tTouch();
#if CONFIG_STACKCHAN_BODY_SENSORS
        InitializeBodySensors();
#endif
        I2cDetect();
        // Avatar auto-display disabled: WiFi config UI needs to be visible.
        // Avatar is shown on-demand via MCP set_avatar command.
        // InitializeAvatar();
        InitializeMouthSequenceTask();
        RegisterMcpTools();
        // These used to hang off InitializeAvatar(), which this board no
        // longer calls (the avatar appears on the first set_avatar).
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        StartProceduralFaceAnimation();
#endif
#if CONFIG_STACKCHAN_PROCEDURAL_FACE
        LoadFaceStyle();
#endif
        StartStatusIcons();
        StartClockTimer();
        StartLcdRefresh();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static CoreS3AudioCodec audio_codec(i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_AW88298_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual bool GetBatteryLevel(int &level, bool& charging, bool& discharging) override {
        level = -1;
        bool level_available = ReadBatteryLevel(&level);
        MaybeApplyAutomaticChargePolicy(level, level_available);
        if (!level_available) {
            return false;
        }

        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }

    virtual void OnUserActivity() override {
        power_save_timer_->WakeUp();
        HandleScreenActivity();
    }

    // Phase 4 audio (Issue #76): drive avatar mouth animation alongside TTS
    // playback. The gateway's tts.start / tts.stop notifications reach this
    // board via Application::OnIncomingJson() -> Board::OnTtsStart/Stop().
    virtual void OnTtsStart() override {
        StartTtsLipSync();
    }

    virtual void OnTtsStop() override {
        StopTtsLipSync();
    }

    // Phase 4.5 avatar (saiverse-stackchan-addon): handle the gateway's
    // `avatar_set_fetch` WS message. Parse url/token/mode/checksum/
    // expected_size, spawn a worker task that performs HTTP GET + SHA256
    // verify + AvatarSet::AdoptOwnedBuffer, then send `avatar_set_loaded` back via
    // the protocol. Runs on the protocol receive task; the actual fetch
    // is delegated to a FreeRTOS task to avoid blocking the receive loop
    // while the LCD-sized payload flows in.
    virtual void OnAvatarSetFetch(const cJSON* root) override {
        if (root == nullptr) {
            ESP_LOGW(TAG, "OnAvatarSetFetch: root is null");
            return;
        }
        auto url      = cJSON_GetObjectItem(root, "url");
        auto token    = cJSON_GetObjectItem(root, "token");
        auto mode_j   = cJSON_GetObjectItem(root, "mode");
        auto checksum = cJSON_GetObjectItem(root, "checksum");
        auto size_j   = cJSON_GetObjectItem(root, "expected_size");

        // The gateway correlates avatar_set_loaded replies by checksum
        // (see ESP32Connection._avatar_set_waiters). Reply with the
        // requested checksum on every error path so a failure can wake
        // the waiter promptly instead of timing out.
        const std::string req_checksum =
            cJSON_IsString(checksum) ? checksum->valuestring : "";

        if (!cJSON_IsString(url) || !cJSON_IsString(token) ||
            !cJSON_IsString(mode_j) || !cJSON_IsNumber(size_j)) {
            ESP_LOGW(TAG, "OnAvatarSetFetch: missing required fields");
            SendAvatarSetLoadedError(req_checksum, "missing_fields");
            return;
        }

        AvatarSet::Mode mode_enum;
        if (strcmp(mode_j->valuestring, "layered") == 0) {
            mode_enum = AvatarSet::Mode::kLayered;
        } else if (strcmp(mode_j->valuestring, "matrix") == 0) {
            mode_enum = AvatarSet::Mode::kMatrix;
        } else {
            ESP_LOGW(TAG, "OnAvatarSetFetch: unknown mode '%s'", mode_j->valuestring);
            SendAvatarSetLoadedError(req_checksum, "unknown_mode");
            return;
        }

        // Take the in-progress guard. exchange(true) returns the previous
        // value, so if another fetch was already running we reject this
        // request rather than racing on avatar_set_'s PSRAM swap. The
        // pending lock is created lazily (the defer helpers do the same;
        // create it here so both producer and consumer share the same
        // mutex instance).
        if (avatar_fetch_in_progress_.exchange(true, std::memory_order_acq_rel)) {
            ESP_LOGW(TAG, "OnAvatarSetFetch: another fetch already in progress");
            SendAvatarSetLoadedError(req_checksum, "fetch_in_progress");
            return;
        }
        EnsureAvatarPendingLock();
        if (avatar_pending_lock_ != nullptr &&
            xSemaphoreTake(avatar_pending_lock_, portMAX_DELAY) == pdTRUE) {
            avatar_pending_ = PendingAvatarState{};
            xSemaphoreGive(avatar_pending_lock_);
        }
        // Quiesce every autonomous LVGL writer so no set_src lands while
        // AvatarSet::AdoptOwnedBuffer atomically swaps the PSRAM buffer backing
        // each lv_image_dsc_t. The schedule timers / state machines restart
        // from ApplyPendingAvatarAfterFetch (blink) or the next tts.start
        // (TTS lipsync) once the fetch resolves.
        StopTtsLipSync();
        RequestMouthSequenceCancel();
        StopBlinkTimer();

        auto* context = new AvatarFetchContext;
        context->board = this;
        context->url = url->valuestring;
        context->token = token->valuestring;
        context->mode = mode_enum;
        context->expected_size = static_cast<size_t>(size_j->valuedouble);
        context->expected_sha256 = cJSON_IsString(checksum) ? checksum->valuestring : "";

        BaseType_t ok = xTaskCreate(
            &StackChanBoard::AvatarFetchTaskTrampoline,
            "avatar_fetch",
            8192,
            context,
            tskIDLE_PRIORITY + 2,
            nullptr);
        if (ok != pdPASS) {
            ESP_LOGE(TAG, "OnAvatarSetFetch: failed to create avatar_fetch task");
            delete context;
            avatar_fetch_in_progress_.store(false, std::memory_order_release);
            SendAvatarSetLoadedError(req_checksum, "task_create_failed");
        }
    }

    // ---- Phase 4.5 avatar helpers --------------------------------------

    struct AvatarFetchContext {
        StackChanBoard* board;
        std::string url;
        std::string token;
        AvatarSet::Mode mode;
        size_t expected_size;
        std::string expected_sha256;
    };

    static void AvatarFetchTaskTrampoline(void* arg) {
        auto* ctx = static_cast<AvatarFetchContext*>(arg);
        ctx->board->RunAvatarFetch(ctx);
        delete ctx;
        vTaskDelete(nullptr);
    }

    void RunAvatarFetch(const AvatarFetchContext* ctx) {
        // Capture expected_sha256 by value so the callback can fall back
        // to it when AvatarSetFetcher reports an error before computing
        // the actual checksum (HTTP error, size mismatch, allocation
        // failure, etc.). The gateway's _avatar_set_waiters dict is keyed
        // by checksum; replying with an empty key means the failure
        // cannot resolve any waiter and the caller waits until timeout.
        const std::string expected_sha256 = ctx->expected_sha256;
        AvatarSetFetcher::Fetch(
            avatar_set_,
            ctx->url, ctx->token,
            ctx->mode, ctx->expected_size, ctx->expected_sha256,
            [expected_sha256](bool ok,
                              const std::string& actual_checksum,
                              const std::string& error_code) {
                const std::string& correlation =
                    actual_checksum.empty() ? expected_sha256 : actual_checksum;
                SendAvatarSetLoaded(ok, correlation, error_code);
            });

        // Fetch finished (success or failure). Clear the in-progress flag
        // BEFORE replaying pending state — otherwise the public
        // SetAvatarExpression / SetMouthShape / StartBlinkTimer calls
        // inside ApplyPendingAvatarAfterFetch would loop back into the
        // defer helpers and the pending state would never be drained.
        avatar_fetch_in_progress_.store(false, std::memory_order_release);

        // After a successful adoption the previously displayed face is still
        // pointing into the freed static-table data via avatar_img_; force a
        // refresh so the new AvatarSet entry is picked up by the next
        // RenderAvatarLocked() call (driven by SetAvatarExpressionIfActive
        // below). Skipped on failure (the old image is still valid).
        if (avatar_set_.is_loaded()) {
            SetAvatarExpressionIfActive(current_avatar_face_.c_str());
        }

        // Replay the latest face / mouth / blink intent the user expressed
        // while the fetch was running. Order: "off" wins over a face if
        // both were issued (mutually exclusive on the face axis); blink
        // restoration happens last so a successful fetch doesn't restart
        // blink if the user disabled it mid-fetch.
        ApplyPendingAvatarAfterFetch();
    }

    static void SendAvatarSetLoaded(
        bool ok, const std::string& checksum, const std::string& error_code) {
        cJSON* root = cJSON_CreateObject();
        if (root == nullptr) return;
        cJSON_AddStringToObject(root, "type", "avatar_set_loaded");
        cJSON_AddStringToObject(root, "checksum", checksum.c_str());
        cJSON_AddBoolToObject(root, "ok", ok);
        if (ok || error_code.empty()) {
            cJSON_AddNullToObject(root, "error");
        } else {
            cJSON_AddStringToObject(root, "error", error_code.c_str());
        }
        char* str = cJSON_PrintUnformatted(root);
        if (str != nullptr) {
            Application::GetInstance().SendJsonString(std::string(str));
            cJSON_free(str);
        }
        cJSON_Delete(root);
    }

    static void SendAvatarSetLoadedError(
        const std::string& checksum, const std::string& error_code) {
        SendAvatarSetLoaded(false, checksum, error_code);
    }

    // --------------------------------------------------------------------

    virtual Backlight *GetBacklight() override {
        static CustomBacklight backlight(pmic_);
        return &backlight;
    }
};

DECLARE_BOARD(StackChanBoard);
