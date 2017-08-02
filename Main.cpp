// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include <array>
#include <cmath>
#include <mutex>

#include "CanBus.h"
#include "CanOpenPdo.h"
#include "SpiBus.h"
#include "ch.hpp"
#include "hal.h"
#include "thread.h"

// This is very, very temporary
#define CAN_BUS (*(CanBus*)((*(std::vector<void*>*)arg)[0]))
#define CAN_BUS_MUT *(chibios_rt::Mutex*)((*(std::vector<void*>*)arg)[1])

// module function prototypes
uint8_t adc_to_temp(uint8_t* rxbuf);

// TODO: remove lookup from global namespace and put in put in thread (?)
// index constants for lookup table
static constexpr uint8_t g_kSegmentStartIndex = 0, g_kSegmentLengthIndex = 1,
                         g_kBaseTempIndex = 2;
static constexpr uint8_t g_kVoltageToTempLength = 70;
static constexpr uint8_t g_kMaxVoltageIndex = g_kVoltageToTempLength - 1;

// Lookup table for voltage->temperature conversion. Used to find which linearly
// interpolable temperature range a given voltage falls in.
// Entries have the form { voltage-bound * 100,
//                         length-of-voltage-range * 100,
//                         base-temperature-of-voltage-range }
// TODO: use vectors
static uint8_t g_voltage_to_temp[g_kVoltageToTempLength][3] = {
    {148, 3, 65}, {148, 3, 65}, {148, 3, 65},                // 65C range
    {151, 4, 60}, {151, 4, 60}, {151, 4, 60}, {151, 4, 60},  // 60C range
    {155, 4, 55}, {155, 4, 55}, {155, 4, 55}, {155, 4, 55},  // 55C range
    {159, 4, 50}, {159, 4, 50}, {159, 4, 50}, {159, 4, 50},  // 50C range
    {163, 5, 45}, {163, 5, 45}, {163, 5, 45}, {163, 5, 45},  // 45C range
    {163, 5, 45}, {168, 6, 40}, {168, 6, 40}, {168, 6, 40},  // 40C range
    {168, 6, 40}, {168, 6, 40}, {168, 6, 40},
    {174, 6, 35}, {174, 6, 35}, {174, 6, 35}, {174, 6, 35},  // 35C range
    {174, 6, 35}, {174, 6, 35},
    {180, 6, 30}, {180, 6, 30}, {180, 6, 30}, {180, 6, 30},  // 30C range
    {180, 6, 30}, {180, 6, 30},
    {186, 6, 25}, {186, 6, 25}, {186, 6, 25}, {186, 6, 25},  // 25C range
    {186, 6, 25}, {186, 6, 25},
    {192, 6, 20}, {192, 6, 20}, {192, 6, 20}, {192, 6, 20},  // 20C range
    {192, 6, 20}, {192, 6, 20}, {192, 6, 20},
    {199, 6, 15}, {199, 6, 15}, {199, 6, 15}, {199, 6, 15},  // 15C range
    {199, 6, 15}, {199, 6, 15},
    {205, 6, 10}, {205, 6, 10}, {205, 6, 10}, {205, 6, 10},  // 10C range
    {205, 6, 10}, {205, 6, 10},
    {211, 6, 5}, {211, 6, 5},  {211, 6, 5}, {211, 6, 5},     // 5C range
    {211, 6, 5},  {211, 6, 5},
    {217, 6, 0}                                              // 0C range
};

/*
 * SPI TX and RX buffers.
 */
static uint8_t rxbuf[2];

/*
 * SPI bus thread
 */
// static THD_WORKING_AREA(spi_thread_2_wa, 256);
static THD_WORKING_AREA(spi_thread_2_wa, 256);
static THD_FUNCTION(spi_thread_2, arg) {
  chRegSetThreadName("SPI thread 2");

  // setup SPI comm...command format is
  // 0b00110000=<null><null><start><single-ended><d2><d1><d0><null>

  constexpr uint8_t kNumAdcSlaves = 4;
  constexpr uint8_t kNumAdcChannels = 7;  // ignoring 8th channel on all chips
  constexpr uint8_t kBaseCommand = 0x30;  // null, null, start, single_ended
  constexpr uint8_t kSsPins[kNumAdcSlaves] = {4, 3, 1, 0};

  SpiBus* spiBus = new SpiBus(SpiBusBaudRate::k140k, kSsPins, kNumAdcSlaves);

  uint8_t txbuf[1];  // empty buff for commands to external ADC chips
  uint8_t cell_module_readings[kNumAdcChannels];

  while (true) {
    // TODO: Make an iterator for the SpiBus based on subsets of slaves
    // Read analog values on 7 of the 8 channels on the 4 ADC chips and dump
    // them on the CAN network as 4 separate frames for each chip (set of 7 cell
    // modules)
    for (uint8_t chip_index = 0; chip_index < kNumAdcSlaves; ++chip_index) {
      // spiBus->acquireSlave(2); // chip 2 not working
      // for each channel on the current chip
      for (uint8_t channel_index = 0; channel_index < kNumAdcChannels;
           ++channel_index) {
        // acquire for current chip, current channel
        spiBus->acquireSlave(chip_index);

        // set the channel address in LS nibble (LS bit is null, so << 1)
        txbuf[0] = kBaseCommand | (channel_index << 1);

        // send command word then read in data
        spiBus->send(1, txbuf);
        spiBus->recv(2, rxbuf);

        // convert SPI bytes to an 8b compressed temperature
        cell_module_readings[channel_index] = adc_to_temp(rxbuf);

        // release for next chip, channel
        spiBus->releaseSlave();
      }

      // pack 7 module temp readings into CAN frame
      const CellTempMessage cellTempMessage(kFuncid_cellTemp_adc[chip_index],
                                            cell_module_readings);

      // queue CAN frame for transmission in thread-safe block
      {
        // Lock from simultaneous thread access
        std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);

        // queue CAN message for one set of 7 cell modules
        (CAN_BUS).queueTxMessage(cellTempMessage);
      }
    }

    // throttle back thread runloop to prevent overconsumption of resources
    chThdSleepMilliseconds(200);
  }
}

/*
 * CAN TX thread
 */
static THD_WORKING_AREA(wa_canTxThreadFunc, 128);
static THD_FUNCTION(canTxThreadFunc, arg) {
  chRegSetThreadName("CAN TX");

  while (true) {
    {
      // Lock from simultaneous thread access
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      // Process all messages to transmit from the message transmission queue
      (CAN_BUS).processTxMessages();
    }
    // throttle back thread runloop to prevent overconsumption of resources
    chThdSleepMilliseconds(10);
  }
}

/*
 * CAN RX thread
 */
static THD_WORKING_AREA(wa_canRxThreadFunc, 128);
static THD_FUNCTION(canRxThreadFunc, arg) {
  event_listener_t el;

  chRegSetThreadName("CAN RX");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0) {
      continue;
    }
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).processRxMessages();
    }
  }

  chEvtUnregister(&CAND1.rxfull_event, &el);
}

/**
 * @desc Performs periodic tasks every second
 */
static THD_WORKING_AREA(wa_heartbeatThreadFunc, 128);
static THD_FUNCTION(heartbeatThreadFunc, arg) {
  chRegSetThreadName("NODE HEARTBEAT");

  while (1) {
    // enqueue heartbeat message to g_canTxQueue
    const HeartbeatMessage heartbeatMessage(kNodeid_cellTemp);
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).queueTxMessage(heartbeatMessage);
    }
    // transmit node's (self) heartbeat every 1s
    chThdSleepMilliseconds(1000);
  }
}

int main() {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // INIT INTERFACES
  // Activate CAN driver 1 (PA11 = CANRX, PA12 = CANTX)
  CanBus canBus(kNodeid_cellTemp, CanBusBaudRate::k250k, false);
  chibios_rt::Mutex canBusMut;
  chibios_rt::Mutex spiBusMut;

  // create void* compatible obj
  std::vector<void*> args = {&canBus, &canBusMut};

  // start the CAN TX/RX threads
  chThdCreateStatic(wa_canTxThreadFunc, sizeof(wa_canTxThreadFunc), NORMALPRIO,
                    canTxThreadFunc, &args);
  chThdCreateStatic(wa_canRxThreadFunc, sizeof(wa_canRxThreadFunc), NORMALPRIO,
                    canRxThreadFunc, &args);
  // start the CAN heartbeat thread
  chThdCreateStatic(wa_heartbeatThreadFunc, sizeof(wa_heartbeatThreadFunc),
                    NORMALPRIO, heartbeatThreadFunc, &args);
  // Start SPI thread
  chThdCreateStatic(spi_thread_2_wa, sizeof(spi_thread_2_wa), NORMALPRIO + 1,
                    spi_thread_2, &args);

  // Successful startup indicator
  for (int i = 0; i < 4; ++i) {
    palWriteLine(LINE_LED_GREEN, PAL_HIGH);
    chThdSleepMilliseconds(50);
    palWriteLine(LINE_LED_GREEN, PAL_LOW);
    chThdSleepMilliseconds(50);
  }

  while (1) {
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);

      // // print all transmitted messages
      // canBus.printTxAll();
      // // print all received messages
      // canBus.printRxAll();
    }

    chThdSleepMilliseconds(10000);
  }
}

// @brief Convert bytes in RX buffer from ADC reading to temperature
// @return temp Temperature in degrees C, resolution of .25
// TODO: Add in per-channel moving average
uint8_t adc_to_temp(uint8_t* rxbuf) {
  static uint16_t raw{};
  static double voltage{};
  static int lookup_index{};
  static uint8_t* interpolation_segment{};
  static float interpolated_base_offset{};
  static uint16_t temp{};
  static uint8_t compressed_temp{};

  // voltage at max supported temperature (63.75C)
  static constexpr float kVref = 3.315;
  static constexpr double kAdcMax = 1023.0;
  static constexpr uint8_t kMaxTemp = 255;  // max of 63.75C

  // Pack bits from SPI bytes into single 10b ADC reading:
  // (upper 7 bits of the first byte, in the upper 7 position) |
  //    (upper 3 bits of the second byte, in the lower 3 position)
  raw = ((rxbuf[0] & 0x7f) << 3) | ((rxbuf[1] & 0xe0) >> 5);

  // convert raw reading to voltage based on ADC resolution and Vref
  voltage = (raw / kAdcMax) * kVref;

  // Convert fp voltage to integer (with 0.01V resolution), then normalize to
  // 0=148 (1.48V) and 69=217 (2.17V)
  lookup_index = (voltage * 100) - 148;

  // clamp index between 0 and 69
  // TODO: Try std::clamp (-std=c++1z didn't work)
  if (lookup_index < 0) {
    lookup_index = 0;
  } else if (lookup_index > g_kMaxVoltageIndex) {
    lookup_index = g_kMaxVoltageIndex;
  }

  // fetch linearly interpolable range (e.g. 0-5 degrees C)
  interpolation_segment = g_voltage_to_temp[lookup_index];

  // compute temperature offset from upper bound of range
  interpolated_base_offset =
      5 * (((voltage * 100) - interpolation_segment[g_kSegmentStartIndex]) /
           (interpolation_segment[g_kSegmentLengthIndex]));

  // compute fp temperature from base and base offset
  temp =
      4 * (interpolation_segment[g_kBaseTempIndex] - interpolated_base_offset);

  // convert to uint8 with resolution of 0.25 degrees C
  // clamp to max temp that can be represented by single uint8
  compressed_temp = temp > kMaxTemp ? kMaxTemp : temp;

  // divide this returned integer temp by 4 to get fp temp
  return compressed_temp;
}
