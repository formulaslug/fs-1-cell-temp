// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include "CanOpenPdo.h"

#include <cstring>

FaultStatusesMessage::FaultStatusesMessage(uint8_t tempDidFault,
                                           uint8_t bmsDidFault,
                                           uint8_t imdDidFault) {
  IDE = CAN_IDE_EXT;
  EID = kSysIdFs | kNodeIdCellTemp | kFuncIdFaultStatuses;
  RTR = CAN_RTR_DATA;
  DLC = 1;  // num data bytes in frame

  // clean fault statuses in case more than LS bit set
  tempDidFault = tempDidFault & 0x1;
  bmsDidFault = bmsDidFault & 0x1;
  imdDidFault = imdDidFault & 0x1;

  // copy passed data into frame's data byte
  data8[0] = (tempDidFault << 2) | (bmsDidFault << 1) | (imdDidFault);
}

CellTempMessage::CellTempMessage(uint32_t adcChipId,
                                 uint8_t cellModuleReadings[7]) {
  IDE = CAN_IDE_EXT;
  EID = kSysIdFs | kNodeIdCellTemp | adcChipId;
  RTR = CAN_RTR_DATA;
  DLC = 7;  // num data bytes in frame

  // copy passed data into frame's data bytes
  std::memcpy(data8, cellModuleReadings, 7 * sizeof(uint8_t));
}

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
  EID = kCobIdTPDO5;
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
