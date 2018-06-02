// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include "MuxedSpiBus.h"

#include <cmath>
#include <cstdio>

/* Hard coded to baudrate=140.625kHz, CPHA=0, CPOL=0, MSb first
 * MCP3008's baud maxes out around 3-4MHz
 */
constexpr SPIConfig MakeConfig(SpiBusBaudRate baud, uint8_t mssPin) {
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
  return {NULL, GPIOA, mssPin, cr1, cr2};
}

MuxedSpiBus::MuxedSpiBus(SpiBusBaudRate baud, const uint8_t mssPin,
                         const uint8_t* chipSelectors, const uint8_t numChips,
                         const uint8_t numChipSelectors) {
  // init private vars from params
  m_numChips = numChips;
  m_numChipSelectors = numChipSelectors;
  m_chipSelectors = chipSelectors;
  // Note: ChibiOS HAL SPI lib will assert MSS properly as if it's a
  //       regular SS pin, however this module will be asserted to MUX
  //       selector inputs for the actual SPI chips (as opposed to the
  //       isolator).
  m_chipConfig = MakeConfig(baud, mssPin);

  // init SPI pins
  // SCK
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  // MISO
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  // MOSI
  palSetPadMode(GPIOA, 7, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  // selector pins
  for (uint8_t i = 0; i < m_numChipSelectors; i++) {
    palSetPadMode(GPIOA, m_chipSelectors[i],
                  PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  }
  // mss pin —- needed to manually drive the mssPin
  // TODO: parameterize the mss pin
  palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
}

MuxedSpiBus::~MuxedSpiBus() {
  spiUnselect(&SPID1);    // de-assert chip select
  spiReleaseBus(&SPID1);  // release ownership of bus as master
}

/*
 * @brief Reads `length` bytes from txbuf and sends over SPI bus
 */
void MuxedSpiBus::send(uint8_t length, const void* txbuf) {
  spiSend(&SPID1, length, txbuf);  // transmit
}

/*
 * @brief Writes `length` bytes into rxbuf from SPI bus
 */
void MuxedSpiBus::recv(uint8_t length, void* rxbuf) {
  spiReceive(&SPID1, length, rxbuf);  // receive
}

/*
 * @brief Acquire bus for the passed slave select pin index
 */
void MuxedSpiBus::acquireSlave(uint8_t chipIndex) {
  setSelectors(chipIndex);  // set selector pins to correct pattern
  // TODO: parameterize the mss pin
  palWriteLine(LINE_ARD_A3, PAL_LOW); // need to manually drive the mss pin
  spiAcquireBus(&SPID1);  // acquire ownership of the bus with MSS
  // drive chip selects low through multiplixed selectors
  spiStart(&SPID1, &m_chipConfig);  // setup transfer parameters.
  spiSelect(&SPID1);  // slave Select assertion.
}

/*
 * @brief Release bus
 */
void MuxedSpiBus::releaseSlave() {
  spiUnselect(&SPID1);  // de-assert chip select
  // TODO: parameterize the mss pin
  palWriteLine(LINE_ARD_A3, PAL_LOW); // need to manually drive the mss pin
  spiReleaseBus(&SPID1);  // release ownership of bus as master
}

/*
 * @brief Drive selector pins to select the chip corresponding to
 *        passed chip index
 * @note No need to clear or reset this value, since all 0b00 through
 *       0b11 correspond to a SPI chip. Deselection is done by driving
 *       the MSS pin high. This function should be called before
 *       acquiring the SPI bus so that MSS is driving low AFTER the
 *       selector pins are setup so that the chip has time to
 *       multiplex SPI chip I/O.
 * @note Hard-coded to having 2 selector pins:
 *       chip index -> C | 0 1 <- mux selector pins for chips
 *                     - + ---
 *                     0 | 0 0
 *                     1 | 0 1
 *                     2 | 1 0
 *                     3 | 1 1
 */
void MuxedSpiBus::setSelectors(uint8_t chipIndex) {
  // first selector line
  if (chipIndex == 0 || chipIndex == 1) {
    palWriteLine(LINE_ARD_A2, PAL_LOW);
  } else {
    // LINE_LED_GREEN
    palWriteLine(LINE_ARD_A2, PAL_HIGH);
  }
  // second selector line
  if (chipIndex == 0 || chipIndex == 2) {
    palWriteLine(LINE_ARD_A1, PAL_LOW);
  } else {
    palWriteLine(LINE_ARD_A1, PAL_HIGH);
  }
}
