// Copyright (c) Formula Slug 2016. All Rights Reserved.

#pragma once

#include <stdint.h>

#include "ch.h"
#include "hal.h"

// COB-IDs: MAX LENGTH of 12 bits, only the LSB 12 should be used
// IDs specified in format 0x<system-id><node-id><function-id>
// System IDs: 6 = FS System
// Node IDs:
//   1 = Primary Controller, (node 3)
//   2 = Secondary Controller, (node 4)
//   3 = Cell Temp Monitor
//     // Function IDs
//     2 = ADC Chip 1 Reading
//     3 = ADC Chip 2 Reading
//     4 = ADC Chip 3 Reading
//     5 = ADC Chip 4 Reading
// Universal Function IDs:
//   1 = Heartbeat
constexpr uint32_t kSysid_fs = 0x600;
constexpr uint32_t kNodeid_primary = 0x010;
constexpr uint32_t kNodeid_secondary = 0x020;
constexpr uint32_t kNodeid_cellTemp = 0x030;
constexpr uint32_t kFuncid_cellTemp_adc1 = 0x002;
constexpr uint32_t kFuncid_cellTemp_adc2 = 0x003;
constexpr uint32_t kFuncid_cellTemp_adc3 = 0x004;
constexpr uint32_t kFuncid_cellTemp_adc4 = 0x005;

constexpr uint32_t kCobid_TPDO5 = 0x241;  // including throttle voltage payload
constexpr uint32_t kCobid_node3Heartbeat = 0x611; // changed from 0x003
constexpr uint32_t kCobid_node4Heartbeat = 0x621; // changed from 0x004
constexpr uint32_t kCobid_cellTempHeartbeat = 0x631;
constexpr uint32_t kCobid_p2s = 0x613; // changed from 0x013
constexpr uint32_t kCobid_s2p = 0x622; // changed from 0x014

// Payload constants
constexpr uint32_t kPayloadHeartbeat = 0x1;

struct HeartbeatMessage : public CANTxFrame {
  HeartbeatMessage(uint32_t id);
};

struct CellTempMessage : public CANTxFrame {
  CellTempMessage(uint32_t adc_chip_id, uint8_t cell_module_readings[7]);
};

struct ThrottleMessage : public CANTxFrame {
  /**
   * @param throttleVoltage The current, cleaned throttle voltage to be sent to
   *                        Master
   * @param forwardSwitch If true, enables forward drive
   */
  ThrottleMessage(uint16_t throttleVoltage, bool forwardSwitch);
};

/**
 * TPDO sent from Teensy to master motor controller
 */
struct[[gnu::packed]] TPDO5 {
  uint16_t throttleInputVoltage;
  uint16_t maxBatteryDischargeCurrent;
  uint16_t maxBatteryRechargeCurrent;
  uint8_t forwardSwitch : 1;
  uint8_t driveSelect1Switch : 1;
  uint8_t driveSelect2Switch : 1;
  uint8_t reverseSwitch : 1;
  uint8_t padding : 4;
};
