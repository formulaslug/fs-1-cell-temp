// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#pragma once

#include <initializer_list>

#include "hal.h"

constexpr uint8_t kSPImaxSlaves = 8;

enum class SpiBusBaudRate : uint8_t {
  k140k,  // 140.625 kHz
};

class SpiBus {
 public:
  SpiBus(SpiBusBaudRate baud, const uint8_t* slavePins,
         const uint8_t numSlaves);
  ~SpiBus();
  void send(uint8_t length, const void* txbuf);
  void recv(uint8_t length, void* rxbuf);
  void acquireSlave(uint8_t ssPinIndex);
  void releaseSlave();

 private:
  uint8_t m_numSlaves;
  SPIConfig m_slaveConfigs[kSPImaxSlaves];
};
