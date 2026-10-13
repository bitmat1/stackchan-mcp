/*
 * SPDX-License-Identifier: MIT
 *
 * Minimal DYNAMIXEL Protocol 2.0 driver for the Stack-chan RT ver.
 * (rt-net/stack-chan: M5Stack CoreS3 + DYNAMIXEL XL330-M288-T x2).
 *
 * The board code in boards/stackchan/stackchan.cc was written against the
 * Feetech SCS0009 API (FeetechScs / SCSCL) and works in SCS0009 raw position
 * units throughout (yaw: 460 + deg * 16 / 5, pitch: 620 + deg * 16 / 5).
 * DynamixelBus exposes the same five calls the board uses -- begin, WritePos,
 * ReadPos, ReadMove, EnableTorque -- and translates SCS0009 raw units to and
 * from XL330 raw units per axis, so the motion code does not change.
 *
 * Protocol 2.0 packet:
 *   FF FF FD 00 ID LEN_L LEN_H INST PARAMS... CRC_L CRC_H
 *   LEN = params + 3 (INST + 2 CRC bytes); CRC-16 poly 0x8005 over all
 *   preceding bytes.
 *
 * The RT board ties TX and RX to a single half-duplex DYNAMIXEL line through
 * a buffer, so every instruction packet is echoed back on RX. The reader
 * skips any packet that is not a status packet (INST 0x55) from the
 * addressed ID, matching the echo handling in rt-net's dynamixel.ts.
 *
 * XL330 control table (Protocol 2.0, RAM/EEPROM):
 *   10  Drive Mode          u8   bit 2 = time-based profile
 *   11  Operating Mode      u8   5 = current-based position
 *   64  Torque Enable       u8
 *   102 Goal Current        u16  mA (XL330: 1 unit = 1 mA)
 *   108 Profile Accel       u32  (time-based: ms)
 *   112 Profile Velocity    u32  (time-based: total move time, ms)
 *   116 Goal Position       u32  0..4095 = 0..360 deg, 2048 = centre
 *   122 Moving              u8
 *   132 Present Position    i32
 */
#pragma once

#include <cstdint>
#include <driver/uart.h>

class DynamixelBus {
public:
    // Per-axis translation between SCS0009 raw units (what stackchan.cc
    // speaks) and XL330 raw units. scs_zero is the SCS raw value at the
    // axis' 0 deg (yaw 460, pitch 620).
    struct Axis {
        uint8_t id = 0;
        int scs_zero = 0;
        // Maps an SCS-convention angle (deg) to an XL330 angle relative to
        // 2048 (deg). Implemented by the board so it can own the geometry.
        float (*scs_deg_to_dxl_deg)(float) = nullptr;
        float (*dxl_deg_to_scs_deg)(float) = nullptr;
        uint16_t goal_current_ma = 0;  // 0 = leave the servo default
    };

    void ConfigureAxis(const Axis& axis);

    // Same call shapes as FeetechScs so stackchan.cc is driver-agnostic.
    void begin(uart_port_t uart, int baud, int tx_pin, int rx_pin);
    int WritePos(uint8_t id, uint16_t scs_position, uint16_t time_ms, uint16_t speed);
    int ReadPos(uint8_t id);
    int ReadMove(uint8_t id);
    int EnableTorque(uint8_t id, uint8_t enable);

private:
    static constexpr int kMaxAxes = 2;

    struct AxisState {
        Axis axis;
        bool configured = false;      // operating/drive mode applied
        int32_t last_profile_ms = -1; // last Profile Velocity written
        int32_t turn_offset = 0;      // whole turns reported by the servo
    };

    uart_port_t uart_ = UART_NUM_1;
    bool ready_ = false;
    AxisState axes_[kMaxAxes];

    AxisState* Find(uint8_t id);
    int EnsureConfigured(AxisState& st);

    int Write(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t n);
    int Read(uint8_t id, uint16_t addr, uint16_t n, uint8_t* out);
    int Transact(const uint8_t* tx, size_t tx_len, uint8_t id, uint8_t* params, uint16_t want);
    int Write1(uint8_t id, uint16_t addr, uint8_t v);
    int Write2(uint8_t id, uint16_t addr, uint16_t v);
    int Write4(uint8_t id, uint16_t addr, uint32_t v);
};
