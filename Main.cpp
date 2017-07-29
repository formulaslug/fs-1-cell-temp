// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include <array>
#include <mutex>

#include "SpiBus.h"
#include "CanBus.h"
#include "CanOpenPdo.h"
#include "ch.hpp"
#include "hal.h"
#include "thread.h"

// This is very, very temporary
#define CAN_BUS (*(CanBus *)((*(std::vector<void*> *)arg)[0]))
#define CAN_BUS_MUT *(chibios_rt::Mutex *)((*(std::vector<void*> *)arg)[1])

/*
 * SPI TX and RX buffers.
 */
// 0b00110000=<null><null><start><single-ended><d2><d1><d0><null>
static uint8_t txbuf[1] = {0x30};
static uint8_t rxbuf[2];

/*
 * SPI bus thread
 */
static THD_WORKING_AREA(spi_thread_2_wa, 256);
static THD_FUNCTION(spi_thread_2, arg) {

  chRegSetThreadName("SPI thread 2");

  uint8_t ssPins[1] = { 4 };
  SpiBus *spiBus = new SpiBus(SpiBusBaudRate::k140k, ssPins, 1);

  while (true) {
    spiBus->acquireSlave(0);
    spiBus->send(1, txbuf);
    spiBus->recv(2, rxbuf);
    spiBus->releaseSlave();
    uint32_t r;
    // A) ((lower 7 of byte 0) << 3) | (lower 3 of byte 1)
    // B) (upper 7 bits of the reading, in the upper 7 position) |
    //      (lower 3 bits of the second byte, in the lower 3 position)
    r = ((rxbuf[0] & 0x7f) << 3) | (rxbuf[1] & 0x7);
    uint8_t cell_module_readings[7];
    cell_module_readings[0] = r >> 2; // drop the lowest two bits
    const CellTempMessage cellTempMessage(kFuncid_cellTemp_adc1,
                                          cell_module_readings);
    {
      // Lock from simultaneous thread access
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      // queue CAN message for one set of 7 cell modules
      (CAN_BUS).queueTxMessage(cellTempMessage);
    }
    chThdSleepMilliseconds(100);
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

    chThdSleepMilliseconds(50); // changed from 50->200ms
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

    chThdSleepMilliseconds(1000); // change from 1000ms to 1ms
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
  // SpiBus spiBus(SpiBusBaudRate::k140k, ssPins, 1);
  chibios_rt::Mutex spiBusMut;

  // create void* compatible obj
  std::vector<void*> args = {&canBus, &canBusMut}; //, &spiBus, &spiBusMut};
  // start the CAN TX/RX threads
  chThdCreateStatic(wa_canTxThreadFunc, sizeof(wa_canTxThreadFunc), NORMALPRIO, canTxThreadFunc, &args);
  chThdCreateStatic(wa_canRxThreadFunc, sizeof(wa_canRxThreadFunc), NORMALPRIO, canRxThreadFunc, &args);
  // start the CAN heartbeat thread
  chThdCreateStatic(wa_heartbeatThreadFunc, sizeof(wa_heartbeatThreadFunc), NORMALPRIO, heartbeatThreadFunc, &args);

  // Start SPI thread
  chThdCreateStatic(spi_thread_2_wa, sizeof(spi_thread_2_wa),
                    NORMALPRIO + 1, spi_thread_2, &args);

  // Successful startup indicator
  for (int i = 0; i < 4; i++) {
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

    chThdSleepMilliseconds(50);
  }
}
