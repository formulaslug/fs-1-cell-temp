// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include <array>
#include <mutex>
#include <vector>

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
// uint8_t adc_to_temp(uint8_t * rxbuf, std::vector<std::vector<float>> &voltage_to_temp);
uint8_t adc_to_temp(uint8_t * rxbuf);

// TODO: remove from global namespace
// TODO: use vectors
// static std::vector<std::vector<float>> g_voltage_to_temp {
static constexpr uint8_t g_kVoltageToTempLength = 73;
static constexpr uint8_t g_kMaxVoltageIndex = g_kVoltageToTempLength - 1;
static float g_voltage_to_temp[g_kVoltageToTempLength][3] = {
  {1.48, .03, 65}, {1.48, .03, 65}, {1.48, .03, 65},  // 65C range
  {1.51, .04, 60}, {1.51, .04, 60}, {1.51, .04, 60},  // 60C range
  {1.51, .04, 60},
  {1.55, .04, 55}, {1.55, .04, 55}, {1.55, .04, 55},  // 55C range
  {1.55, .04, 55},
  {1.59, .04, 50}, {1.59, .04, 50}, {1.59, .04, 50},  // 50C range
  {1.59, .04, 50},
  {1.63, .04, 45}, {1.63, .04, 45}, {1.63, .04, 45},  // 45C range
  {1.63, .04, 45},
  {1.68, .05, 40}, {1.68, .05, 40}, {1.68, .05, 40},  // 40C range
  {1.68, .05, 40}, {1.68, .05, 40},
  {1.74, .06, 35}, {1.74, .06, 35}, {1.74, .06, 35},  // 35C range
  {1.74, .06, 35}, {1.74, .06, 35}, {1.74, .06, 35},
  {1.80, .06, 30}, {1.80, .06, 30}, {1.80, .06, 30},  // 30C range
  {1.80, .06, 30}, {1.80, .06, 30}, {1.80, .06, 30},
  {1.86, .06, 25}, {1.86, .06, 25}, {1.86, .06, 25},  // 25C range
  {1.86, .06, 25}, {1.86, .06, 25}, {1.86, .06, 25},
  {1.92, .06, 20}, {1.92, .06, 20}, {1.92, .06, 20},  // 20C range
  {1.92, .06, 20}, {1.92, .06, 20}, {1.92, .06, 20},
  {1.99, .07, 15}, {1.99, .07, 15}, {1.99, .07, 15},  // 15C range
  {1.99, .07, 15}, {1.99, .07, 15}, {1.99, .07, 15},
  {1.99, .07, 15},
  {2.05, .06, 10}, {2.05, .06, 10}, {2.05, .06, 10},  // 10C range
  {2.05, .06, 10}, {2.05, .06, 10}, {2.05, .06, 10},
  {2.11, .06, 5}, {2.11, .06, 5}, {2.11, .06, 5},  // 5C range
  {2.11, .06, 5}, {2.11, .06, 5}, {2.11, .06, 5},
  {2.17, .06, 0}, {2.17, .06, 0}, {2.17, .06, 0},  // 0C range
  {2.17, .06, 0}, {2.17, .06, 0}, {2.17, .06, 0}
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

  // Setup SPI comm
  // command format
  // 0b00110000=<null><null><start><single-ended><d2><d1><d0><null>

  constexpr uint8_t kNumAdcSlaves = 4;
  constexpr uint8_t kNumAdcChannels = 7;  // ignoring 8th channel on all chips
  constexpr uint8_t kBaseCommand = 0x30;  // null, null, start, single_ended
  constexpr uint8_t kSsPins[kNumAdcSlaves] = {4, 3, 1, 0};

  SpiBus* spiBus = new SpiBus(SpiBusBaudRate::k140k, kSsPins, kNumAdcSlaves);

  uint8_t txbuf[1];  // empty buff for commands to external ADC chips
  uint8_t cell_module_readings[kNumAdcChannels];

  // Setup lookup table for voltage->temperature conversion
  // Lookup table to first find which linearly interpolable temperature range
  // a given voltage falls in

  // TODO: Make lookup table a const
  // should be able to convert all to unsigned 8b integers
  // constexpr float voltage_to_temp[kNumLookupEntries][kNumLookupEntryItems] = {
  // std::vector<std::vector<float>> voltage_to_temp {
  //   {1.48, .03, 65}, {1.48, .03, 65}, {1.48, .03, 65},  // 65C range
  //   {1.51, .04, 60}, {1.51, .04, 60}, {1.51, .04, 60},  // 60C range
  //   {1.51, .04, 60},
  //   {1.55, .04, 55}, {1.55, .04, 55}, {1.55, .04, 55},  // 55C range
  //   {1.55, .04, 55},
  //   {1.59, .04, 50}, {1.59, .04, 50}, {1.59, .04, 50},  // 50C range
  //   {1.59, .04, 50},
  //   {1.63, .04, 45}, {1.63, .04, 45}, {1.63, .04, 45},  // 45C range
  //   {1.63, .04, 45},
  //   {1.68, .05, 40}, {1.68, .05, 40}, {1.68, .05, 40},  // 40C range
  //   {1.68, .05, 40},
  //   {1.74, .06, 35}, {1.74, .06, 35}, {1.74, .06, 35},  // 35C range
  //   {1.74, .06, 35},
  //   {1.80, .06, 30}, {1.80, .06, 30}, {1.80, .06, 30},  // 30C range
  //   {1.80, .06, 30},
  //   {1.86, .06, 25}, {1.86, .06, 25}, {1.86, .06, 25},  // 25C range
  //   {1.86, .06, 25},
  //   {1.92, .06, 20}, {1.92, .06, 20}, {1.92, .06, 20},  // 20C range
  //   {1.92, .06, 20},
  //   {1.99, .07, 15}, {1.99, .07, 15}, {1.99, .07, 15},  // 15C range
  //   {1.99, .07, 15},
  //   {2.05, .06, 10}, {2.05, .06, 10}, {2.05, .06, 10},  // 10C range
  //   {2.05, .06, 10},
  //   {2.11, .06, 5}, {2.11, .06, 5}, {2.11, .06, 5},  // 5C range
  //   {2.11, .06, 5},
  //   {2.17, .06, 0}, {2.17, .06, 0}, {2.17, .06, 0},  // 0C range
  //   {2.17, .06, 0}
  // };

  while (true) {
    // TODO: Make an iterator for the SpiBus based on subsets of slaves
    // Read analog analog values on 7 of the 8 channels on the 4 ADC chips
    // and dump them on CAN network as 4 separate frames for each chip (set of
    // 7 cell modules)
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

        // temporarily just dropping the lower 2 bits to pack into single byte
        // cell_module_readings[channel_index] = adc_to_temp(rxbuf, voltage_to_temp);
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
    chThdSleepMilliseconds(50);
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
    chThdSleepMilliseconds(50);
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

// module utility functions
// @brief Convert bytes in RX buffer from ADC reading to temperature
// @return temp Temperature in degrees C, resolution of .25
// uint8_t adc_to_temp(uint8_t * rxbuf, std::vector<std::vector<float>> &voltage_to_temp) {
uint8_t adc_to_temp(uint8_t * rxbuf) {
  static uint16_t raw{};
  // static float norm{};
  static float voltage{};
  static int lookup_index{};
  static float * interpolation_segment{};
  static float interpolated_subtraction{};
  static float temp{}, compressed_temp{};

  // voltage at max supported temperature (63.75C)
  // static constexpr float kMaxTempVoltage = 1.4875;
  // static constexpr float kMinTempVoltage = 2.17;
  static constexpr float kVref = 3.316;
  static constexpr float kAdcMax = 1023.0;
  // index constants for lookup table
  constexpr uint8_t kSegmentStartIndex = 0,
                    kSegmentLengthIndex = 1,
                    kBaseTempIndex = 2;

  // true resolution of ~ 211.5 values across 0-63.75C
  // static constexpr float kMaxTempValue =
  //     (kMaxTempVoltage / kVref) * kAdcMax;  // ~ 461.125
  // static constexpr float kMinTempValue =
  //     (kMinTempVoltage / kVref) * kAdcMax;  // ~ 672.7
  // static constexpr float kMaxIntegerValue =
  //     kMinTempValue - kMaxTempValue;  // ~ 211.575

  // unpack raw reading to continuous 10 bits
  // A) ((lower 7 of byte 0) << 3) | (lower 3 of byte 1)
  // B) (upper 7 bits of the reading, in the upper 7 position) |
  //      (lower 3 bits of the second byte, in the lower 3 position)
  //
  raw = ((rxbuf[0] & 0x7f) << 3) | (rxbuf[1] & 0x7);

  // convert from raw reading to 0-[Vref] voltage
  voltage = (raw / kAdcMax) * kVref;
  // return voltage * 100;

  // Convert raw voltage to 0-69 corresponding to 1.48V to 2.17V
  // with .01V resolution
  lookup_index = (voltage * 100) - 148;

  // TODO: Try std::clamp (-std=c++1z didn't work)
  if (lookup_index < 0) {
    lookup_index = 0;
  } else if (lookup_index > g_kMaxVoltageIndex) {
    lookup_index = g_kMaxVoltageIndex;
  }

  // interpolation_segment = voltage_to_temp[lookup_index];
  interpolation_segment = g_voltage_to_temp[lookup_index];
  // return g_voltage_to_temp[lookup_index][kBaseTempIndex];

  interpolated_subtraction = ((voltage -
        interpolation_segment[kSegmentStartIndex]) /
      interpolation_segment[kSegmentLengthIndex]) * 5;

  temp = interpolation_segment[kBaseTempIndex] - interpolated_subtraction;

  compressed_temp = temp * 4;

  return compressed_temp;
}
