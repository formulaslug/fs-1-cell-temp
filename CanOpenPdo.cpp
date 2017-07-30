// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include "CanOpenPdo.h"

#include <cstring>

HeartbeatMessage::HeartbeatMessage(uint32_t id) {
  IDE = CAN_IDE_EXT;
  EID = id;
  RTR = CAN_RTR_DATA;
  DLC = 2;

  data8[0] = (kPayloadHeartbeat >> 8) & 0xFF;  // MSB (32's 3rd byte)
  data8[1] = kPayloadHeartbeat & 0xFF;         // LSB (32's 4th byte)
}

ThrottleMessage::ThrottleMessage(uint16_t throttleVoltage, bool forwardSwitch) {
  IDE = CAN_IDE_EXT;
  EID = kCobid_TPDO5;
  RTR = CAN_RTR_DATA;
  DLC = 8;

  TPDO5 tpdo5;
  tpdo5.throttleInputVoltage = throttleVoltage;
  tpdo5.maxBatteryDischargeCurrent = 400;
  tpdo5.maxBatteryRechargeCurrent = 400;
  tpdo5.forwardSwitch = forwardSwitch;
  tpdo5.driveSelect1Switch = false;
  tpdo5.driveSelect2Switch = false;
  tpdo5.reverseSwitch = false;

  std::memcpy(data8, &tpdo5, sizeof(TPDO5));
}
