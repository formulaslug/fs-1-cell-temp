// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include <array>
#include <mutex>

#include "SpiBus.h"
#include "CanBus.h"
#include "CanOpenPdo.h"
#include "ch.hpp"
#include "hal.h"
#include "thread.h"
// #include "spi.h"

// This is very, very temporary
#define CAN_BUS (*(CanBus *)((*(std::vector<void*> *)arg)[0]))
#define CAN_BUS_MUT *(chibios_rt::Mutex *)((*(std::vector<void*> *)arg)[1])

/*
 * SPI TX and RX buffers.
 */
static uint8_t txbuf[1] = {0xc0};
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
    spiBus->acquireSlave();
    spiBus->send(1, txbuf);
    spiBus->recv(2, rxbuf);
    spiBus->releaseSlave();
    chThdSleepMilliseconds(10);
  }
}

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
    // const HeartbeatMessage heartbeatMessage(kCobid_cellTempHeartbeat);
    const HeartbeatMessage heartbeatMessage((uint32_t)rxbuf);
    {
      std::lock_guard<chibios_rt::Mutex> lock(CAN_BUS_MUT);
      (CAN_BUS).queueTxMessage(heartbeatMessage);
      // write test byte to the SPI bus
      // writeByteSPI(0x20, 0xcF);
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
  // Activate SPI driver 1
  uint8_t ssPins[1] = { 4 };
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
                    NORMALPRIO + 1, spi_thread_2, NULL);

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
