// Copyright (c) Formula Slug 2016. All Rights Reserved.

#pragma once

#include <initializer_list>

#include "hal.h"

constexpr uint8_t kSPImaxSlaves = 8;

enum class SpiBusBaudRate : uint8_t {
  k140k,  // 140.625 kHz
};

class SpiBus {
 public:
  SpiBus(SpiBusBaudRate baud, uint8_t *ssPins, uint8_t numSlaves);
  ~SpiBus();
  void send(uint8_t length, const void * txbuf);
  void recv(uint8_t length, void * rxbuf);
  void acquireSlave();
  void releaseSlave();

 private:
  // private member functions
  // void tickSlaveSelect();

  // private vars
  uint8_t *m_ssPins;
  uint8_t m_numSlaves;
  SPIConfig m_config;
};
