// Copyright (c) Formula Slug 2016. All Rights Reserved.

#include <array>
#include <mutex>

#include "CanBus.h"
#include "CanOpenPdo.h"
#include "ch.hpp"
#include "hal.h"
#include "thread.h"

#define CAN_SELF_NODE_ID 0x681

void heartbeatThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);
void canRxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);
void canTxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut);

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

  // Activate CAN driver 1 (PA11 = CANRX, PA12 = CANTX)
  CanBus canBus(CAN_SELF_NODE_ID, CanBusBaudRate::k250k, true);
  chibios_rt::Mutex canBusMut;

  thread heartbeatThread(NORMALPRIO + 3, heartbeatThreadFunc, canBus,
                         canBusMut);

  // Start receiver thread
  thread canRxThread(NORMALPRIO + 7, canRxThreadFunc, canBus, canBusMut);

  // Start transmitter thread
  thread canTxThread(NORMALPRIO + 7, canTxThreadFunc, canBus, canBusMut);

  while (1) {
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);

      // print all transmitted messages
      canBus.printTxAll();
      // print all received messages
      canBus.printRxAll();
    }

    chThdSleepMilliseconds(50);
  }
}

/**
 * @desc Performs period tasks every second
 */
void heartbeatThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  while (1) {
    // enqueue heartbeat message to g_canTxQueue
    const HeartbeatMessage heartbeatMessage(kCobid_node3Heartbeat);
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.queueTxMessage(heartbeatMessage);
    }

    chThdSleepMilliseconds(1000);
  }
}

void canRxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  event_listener_t el;

  chRegSetThreadName("CAN RX");
  chEvtRegister(&CAND1.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0) {
      continue;
    }
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.processRxMessages();
    }
  }

  chEvtUnregister(&CAND1.rxfull_event, &el);
}

void canTxThreadFunc(CanBus& canBus, chibios_rt::Mutex& canBusMut) {
  chRegSetThreadName("CAN TX");

  while (true) {
    {
      std::lock_guard<chibios_rt::Mutex> lock(canBusMut);
      canBus.send(0x00FF00FF55AA55AA);
      canBus.processTxMessages();
    }

    chThdSleepMilliseconds(50);
  }
}
