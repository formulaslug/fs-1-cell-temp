// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#pragma once

#include <initializer_list>

#include "hal.h"

enum class SpiBusBaudRate : uint8_t {
  k140k,  // 140.625 kHz
};

class MuxedSpiBus {
 public:
  MuxedSpiBus(SpiBusBaudRate baud, const uint8_t mssPin,
              const uint8_t* chipSelectors, const uint8_t numChips,
              const uint8_t numChipSelectors);
  ~MuxedSpiBus();
  void send(uint8_t length, const void* txbuf);
  void recv(uint8_t length, void* rxbuf);
  void acquireSlave(uint8_t ssPinIndex);
  void releaseSlave();

 private:
  uint8_t m_numChips;
  uint8_t m_numChipSelectors;
  const uint8_t* m_chipSelectors;
  SPIConfig m_chipConfig;
  void setSelectors(uint8_t chipIndex);
};
