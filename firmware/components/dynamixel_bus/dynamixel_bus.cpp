/*
 * SPDX-License-Identifier: MIT
 *
 * DYNAMIXEL Protocol 2.0 driver for Stack-chan RT ver. See dynamixel_bus.h.
 */
#include "dynamixel_bus.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cmath>
#include <cstring>

static const char* TAG = "DynamixelBus";

namespace {

constexpr uint8_t INST_READ = 0x02;
constexpr uint8_t INST_WRITE = 0x03;
constexpr uint8_t INST_STATUS = 0x55;

constexpr uint16_t REG_DRIVE_MODE = 10;
constexpr uint16_t REG_OPERATING_MODE = 11;
constexpr uint16_t REG_TORQUE_ENABLE = 64;
constexpr uint16_t REG_GOAL_CURRENT = 102;
constexpr uint16_t REG_PROFILE_ACCELERATION = 108;
constexpr uint16_t REG_PROFILE_VELOCITY = 112;
constexpr uint16_t REG_GOAL_POSITION = 116;
constexpr uint16_t REG_MOVING = 122;
constexpr uint16_t REG_PRESENT_POSITION = 132;

constexpr uint8_t DRIVE_MODE_TIME_BASED = 0x04;
constexpr uint8_t OPERATING_MODE_CURRENT_BASED_POSITION = 5;

constexpr int kDxlCenter = 2048;
constexpr int kDxlPerRev = 4096;
// Time-based Profile Velocity is limited to 32767 ms by the XL330.
constexpr uint32_t kMaxProfileMs = 32767;
// Status packets arrive well within 1 ms at 1 Mbps; the budget also covers
// the echo of our own instruction packet and the servo's return delay.
constexpr int64_t kRxTimeoutUs = 20 * 1000;

uint16_t Crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x8005)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Builds an instruction packet into buf and returns its length.
size_t BuildPacket(uint8_t* buf, uint8_t id, uint8_t inst, const uint8_t* params, uint16_t n) {
    const uint16_t len = n + 3;
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0xFD;
    buf[3] = 0x00;
    buf[4] = id;
    buf[5] = static_cast<uint8_t>(len & 0xFF);
    buf[6] = static_cast<uint8_t>(len >> 8);
    buf[7] = inst;
    if (n) std::memcpy(&buf[8], params, n);
    const size_t body = 8 + n;
    const uint16_t crc = Crc16(buf, body);
    buf[body] = static_cast<uint8_t>(crc & 0xFF);
    buf[body + 1] = static_cast<uint8_t>(crc >> 8);
    return body + 2;
}

// Wraps a raw reading into [-2048, 2048) around the centre so a servo that
// reports an extended (multi-turn) position still maps to a head angle.
int WrapRelative(int32_t raw) {
    int rel = static_cast<int>((raw - kDxlCenter) % kDxlPerRev);
    if (rel < -kDxlPerRev / 2) rel += kDxlPerRev;
    if (rel >= kDxlPerRev / 2) rel -= kDxlPerRev;
    return rel;
}

}  // namespace

void DynamixelBus::ConfigureAxis(const Axis& axis) {
    for (auto& st : axes_) {
        if (st.axis.id == axis.id || st.axis.id == 0) {
            st = AxisState{};
            st.axis = axis;
            return;
        }
    }
    ESP_LOGE(TAG, "ConfigureAxis: no free slot for id=%u", axis.id);
}

DynamixelBus::AxisState* DynamixelBus::Find(uint8_t id) {
    for (auto& st : axes_) {
        if (st.axis.id == id && id != 0) return &st;
    }
    return nullptr;
}

void DynamixelBus::begin(uart_port_t uart, int baud, int tx_pin, int rx_pin) {
    if (ready_ && uart_ == uart) return;
    uart_ = uart;
    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(uart, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    if (!uart_is_driver_installed(uart)) {
        ESP_ERROR_CHECK(uart_driver_install(uart, 512, 0, 0, nullptr, 0));
    }
    ready_ = true;
    ESP_LOGI(TAG, "begin uart=%d baud=%d tx=%d rx=%d", (int)uart, baud, tx_pin, rx_pin);
}

int DynamixelBus::Transact(const uint8_t* tx, size_t tx_len, uint8_t id, uint8_t* params,
                           uint16_t want) {
    if (!ready_) return -1;
    uart_flush_input(uart_);
    if (uart_write_bytes(uart_, reinterpret_cast<const char*>(tx), tx_len) != (int)tx_len) {
        return -1;
    }
    if (uart_wait_tx_done(uart_, pdMS_TO_TICKS(100)) != ESP_OK) return -1;

    uint8_t buf[64];
    size_t have = 0;
    const int64_t deadline = esp_timer_get_time() + kRxTimeoutUs;
    while (esp_timer_get_time() < deadline) {
        int got = uart_read_bytes(uart_, buf + have, sizeof(buf) - have, pdMS_TO_TICKS(2));
        if (got > 0) have += got;

        // Parse as many complete packets as the buffer holds.
        while (true) {
            size_t start = 0;
            while (start + 4 <= have &&
                   !(buf[start] == 0xFF && buf[start + 1] == 0xFF && buf[start + 2] == 0xFD &&
                     buf[start + 3] == 0x00)) {
                start++;
            }
            if (start > 0) {
                std::memmove(buf, buf + start, have - start);
                have -= start;
            }
            if (have < 7) break;
            const uint16_t len = buf[5] | (buf[6] << 8);
            const size_t total = 7u + len;
            if (total > sizeof(buf) || len < 3) {
                // Corrupt length: drop the header and resync.
                std::memmove(buf, buf + 1, have - 1);
                have -= 1;
                continue;
            }
            if (have < total) break;

            const uint16_t crc = buf[total - 2] | (buf[total - 1] << 8);
            const bool crc_ok = Crc16(buf, total - 2) == crc;
            const uint8_t pkt_id = buf[4];
            const uint8_t inst = buf[7];
            if (crc_ok && inst == INST_STATUS && pkt_id == id) {
                const uint8_t err = buf[8];
                const uint16_t n = len - 4;  // minus INST, ERR, CRC x2
                if (err & 0x7F) {
                    ESP_LOGW(TAG, "id=%u status error 0x%02X", id, err);
                    return -1;
                }
                if (n < want) return -1;
                if (want) std::memcpy(params, &buf[9], want);
                return 0;
            }
            // Our own echoed instruction (or noise): skip it.
            std::memmove(buf, buf + total, have - total);
            have -= total;
        }
        if (have == sizeof(buf)) have = 0;
    }
    return -1;
}

int DynamixelBus::Write(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t n) {
    uint8_t params[2 + 8];
    if (n > 8) return -1;
    params[0] = static_cast<uint8_t>(addr & 0xFF);
    params[1] = static_cast<uint8_t>(addr >> 8);
    std::memcpy(&params[2], data, n);
    uint8_t pkt[8 + sizeof(params) + 2];
    const size_t len = BuildPacket(pkt, id, INST_WRITE, params, 2 + n);
    return Transact(pkt, len, id, nullptr, 0);
}

int DynamixelBus::Read(uint8_t id, uint16_t addr, uint16_t n, uint8_t* out) {
    const uint8_t params[4] = {
        static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>(addr >> 8),
        static_cast<uint8_t>(n & 0xFF), static_cast<uint8_t>(n >> 8),
    };
    uint8_t pkt[8 + 4 + 2];
    const size_t len = BuildPacket(pkt, id, INST_READ, params, 4);
    return Transact(pkt, len, id, out, n);
}

int DynamixelBus::Write1(uint8_t id, uint16_t addr, uint8_t v) { return Write(id, addr, &v, 1); }

int DynamixelBus::Write2(uint8_t id, uint16_t addr, uint16_t v) {
    const uint8_t b[2] = {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>(v >> 8)};
    return Write(id, addr, b, 2);
}

int DynamixelBus::Write4(uint8_t id, uint16_t addr, uint32_t v) {
    const uint8_t b[4] = {
        static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>((v >> 8) & 0xFF),
        static_cast<uint8_t>((v >> 16) & 0xFF), static_cast<uint8_t>((v >> 24) & 0xFF),
    };
    return Write(id, addr, b, 4);
}

// Puts the servo into current-based position mode with a time-based
// profile, the same operating mode rt-net's firmware uses. Mode registers
// live in EEPROM and can only change with torque off, so they are read
// first and only rewritten when different.
int DynamixelBus::EnsureConfigured(AxisState& st) {
    if (st.configured) return 0;
    const uint8_t id = st.axis.id;

    uint8_t drive = 0, mode = 0;
    if (Read(id, REG_DRIVE_MODE, 1, &drive) != 0) return -1;
    if (Read(id, REG_OPERATING_MODE, 1, &mode) != 0) return -1;
    const uint8_t want_drive = drive | DRIVE_MODE_TIME_BASED;
    if (drive != want_drive || mode != OPERATING_MODE_CURRENT_BASED_POSITION) {
        if (Write1(id, REG_TORQUE_ENABLE, 0) != 0) return -1;
        if (drive != want_drive && Write1(id, REG_DRIVE_MODE, want_drive) != 0) return -1;
        if (mode != OPERATING_MODE_CURRENT_BASED_POSITION &&
            Write1(id, REG_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION) != 0) {
            return -1;
        }
        ESP_LOGI(TAG, "id=%u configured: drive 0x%02X->0x%02X, mode %u->%u", id, drive,
                 want_drive, mode, OPERATING_MODE_CURRENT_BASED_POSITION);
    }
    if (st.axis.goal_current_ma > 0 &&
        Write2(id, REG_GOAL_CURRENT, st.axis.goal_current_ma) != 0) {
        return -1;
    }
    // Current-based position mode accepts multi-turn goals. If the servo
    // reports a position outside one turn (e.g. after being spun while
    // unpowered), keep goals in that same turn instead of unwinding it.
    uint8_t b[4];
    if (Read(id, REG_PRESENT_POSITION, 4, b) != 0) return -1;
    const int32_t present = static_cast<int32_t>(b[0] | (b[1] << 8) | (b[2] << 16) |
                                                 (static_cast<uint32_t>(b[3]) << 24));
    st.turn_offset = present - (kDxlCenter + WrapRelative(present));
    if (st.turn_offset != 0) {
        ESP_LOGW(TAG, "id=%u present=%ld outside one turn; offset=%ld", id, (long)present,
                 (long)st.turn_offset);
    }
    st.last_profile_ms = -1;
    st.configured = true;
    return 0;
}

int DynamixelBus::WritePos(uint8_t id, uint16_t scs_position, uint16_t time_ms, uint16_t /*speed*/) {
    AxisState* st = Find(id);
    if (st == nullptr || EnsureConfigured(*st) != 0) return -1;

    const float scs_deg = (static_cast<int>(scs_position) - st->axis.scs_zero) * 5.0f / 16.0f;
    const float dxl_deg = st->axis.scs_deg_to_dxl_deg(scs_deg);
    int raw = kDxlCenter + static_cast<int>(std::lround(dxl_deg * kDxlPerRev / 360.0f));
    raw = std::clamp(raw, 0, kDxlPerRev - 1);

    const uint32_t profile_ms = std::min<uint32_t>(time_ms, kMaxProfileMs);
    if (static_cast<int32_t>(profile_ms) != st->last_profile_ms) {
        // Time-based profile: Profile Velocity is the whole move time and
        // Profile Acceleration the ramp time (must not exceed half of it).
        if (Write4(id, REG_PROFILE_ACCELERATION, profile_ms / 4) != 0) return -1;
        if (Write4(id, REG_PROFILE_VELOCITY, profile_ms) != 0) return -1;
        st->last_profile_ms = static_cast<int32_t>(profile_ms);
    }
    // XL330 ignores goal writes while torque is off, whereas the SCS0009
    // the board code was written for engages on the first goal write.
    if (Write1(id, REG_TORQUE_ENABLE, 1) != 0) return -1;
    return Write4(id, REG_GOAL_POSITION, static_cast<uint32_t>(raw + st->turn_offset));
}

int DynamixelBus::ReadPos(uint8_t id) {
    AxisState* st = Find(id);
    if (st == nullptr) return -1;
    uint8_t b[4];
    if (Read(id, REG_PRESENT_POSITION, 4, b) != 0) return -1;
    const int32_t raw = static_cast<int32_t>(b[0] | (b[1] << 8) | (b[2] << 16) |
                                             (static_cast<uint32_t>(b[3]) << 24));
    const float dxl_deg = WrapRelative(raw) * 360.0f / kDxlPerRev;
    const float scs_deg = st->axis.dxl_deg_to_scs_deg(dxl_deg);
    return st->axis.scs_zero + static_cast<int>(std::lround(scs_deg * 16.0f / 5.0f));
}

int DynamixelBus::ReadMove(uint8_t id) {
    uint8_t b = 0;
    if (Read(id, REG_MOVING, 1, &b) != 0) return -1;
    return b;
}

int DynamixelBus::EnableTorque(uint8_t id, uint8_t enable) {
    AxisState* st = Find(id);
    if (st == nullptr) return -1;
    if (enable && EnsureConfigured(*st) != 0) return -1;
    return Write1(id, REG_TORQUE_ENABLE, enable ? 1 : 0);
}
