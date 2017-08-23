// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#pragma once

#include <stdint.h>

#include "ch.h"
#include "hal.h"

// COB-IDs: MAX LENGTH of 12 bits, only the LSB 12 should be used
// IDs specified in format 0x<system-id><node-id><function-id>
// NOTE: The constexprs below are now bit-shifted, there OR'd as-is so
//       the values must be in the correct hex digit.
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
//     6 = Fault Statuses for Temp, BMS, and IMD
// Universal Function IDs:
//   1 = Heartbeat
constexpr uint32_t kSysIdFs = 0x600;
constexpr uint32_t kNodeIdPrimary = 0x010;
constexpr uint32_t kNodeIdSecondary = 0x020;
constexpr uint32_t kNodeIdCellTemp = 0x030;
constexpr uint32_t kFuncIdCellTempAdc[4] = {0x002, 0x003, 0x004, 0x005};
constexpr uint32_t kFuncIdFaultStatuses = 0x006;

constexpr uint32_t kCobIdTPDO5 = 0x241;  // including throttle voltage payload
constexpr uint32_t kCobIdNode3Heartbeat = 0x611;  // changed from 0x003
constexpr uint32_t kCobIdNode4Heartbeat = 0x621;  // changed from 0x004
constexpr uint32_t kCobIdCellTempHeartbeat = 0x631;
constexpr uint32_t kCobIdP2s = 0x613;  // changed from 0x013
constexpr uint32_t kCobIdS2p = 0x622;  // changed from 0x014

// Payload constants
constexpr uint32_t kPayloadHeartbeat = 0x1;

struct HeartbeatMessage : public CANTxFrame {
  HeartbeatMessage(uint32_t id);
};

struct CellTempMessage : public CANTxFrame {
  CellTempMessage(uint32_t adcChipId, uint8_t cellModuleReadings[7]);
};

struct FaultStatusesMessage : public CANTxFrame {
  FaultStatusesMessage(uint8_t tempFaultStatus,
                       uint8_t bmsFaultStatus,
                       uint8_t imdFaultStatus);
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
