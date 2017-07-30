// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include "SpiBus.h"

#include <cmath>
#include <cstdio>

/* Hard coded to baudrate=140.625kHz, CPHA=0, CPOL=0, MSb first
 * MCP3008's baud maxes out around 3-4MHz
 */
constexpr SPIConfig MakeConfig(SpiBusBaudRate baud, uint8_t ssPin) {
  uint16_t cr1 = 0, cr2 = 0;

  switch (baud) {
    case SpiBusBaudRate::k140k:
      cr1 = SPI_CR1_BR_2 | SPI_CR1_BR_1;
      cr2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0;
      break;
    default:
      break;
  }

  // slave selects hard-coded to port A
  return { NULL, GPIOA, ssPin, cr1, cr2 };
}

SpiBus::SpiBus(SpiBusBaudRate baud, const uint8_t *slave_pins, const uint8_t num_slaves) {
  // init private vars from params
  m_numSlaves = num_slaves;

  // init SPI pins
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // SCK
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // MISO
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST); // MOSI
  palSetPadMode(GPIOA, slave_pins[0], PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, slave_pins[1], PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, slave_pins[2], PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOA, slave_pins[3], PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

  // create all config structs for each slave
  for (uint8_t slave_index = 0; slave_index < m_numSlaves; ++slave_index) {
    m_slaveConfigs[slave_index] = MakeConfig(baud, slave_pins[slave_index]);
  }
}

SpiBus::~SpiBus() {
  spiUnselect(&SPID1); // de-assert chip select
  spiReleaseBus(&SPID1); // release ownership of bus as master
}

/*
 * @desc switches the slave select signal to the next device in the originally
 *       passed ssPins
 */
// SpiBus::tickSlaveSelect() {
//   ;
// }

/*
 * @desc Reads `length` bytes from txbuf and sends over SPI bus
 */
void SpiBus::send(uint8_t length, const void * txbuf) {
  spiSend(&SPID1, length, txbuf); // transmit
}

/*
 * @desc Writes `length` bytes into rxbuf from SPI bus
 */
void SpiBus::recv(uint8_t length, void *rxbuf) {
  spiReceive(&SPID1, length, rxbuf); // receive
}

/*
 * @desc Acquire bus for the passed slave slave pin index
 */
void SpiBus::acquireSlave(uint8_t ssPinIndex) {
  spiAcquireBus(&SPID1); // acquire ownership of the bus.
  spiStart(&SPID1, &(m_slaveConfigs[ssPinIndex])); // setup transfer parameters.
  spiSelect(&SPID1); // slave Select assertion.
}

/*
 * @desc Release bus
 */
void SpiBus::releaseSlave() {
  spiUnselect(&SPID1); // de-assert chip select
  spiReleaseBus(&SPID1); // release ownership of bus as master
}
