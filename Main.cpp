// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include <stdint.h>

#include <vector>

#include "CanBus.h"
#include "CanOpenPdo.h"
#include "Threads.h"
#include "ch.hpp"
#include "hal.h"

/*
 * SPI bus thread
 */
static THD_WORKING_AREA(spiThread2Wa, 256);

/*
 * CAN TX thread
 */
static THD_WORKING_AREA(canTxThreadFuncWa, 128);

/*
 * CAN RX thread
 */
static THD_WORKING_AREA(canRxThreadFuncWa, 128);

/**
 * @desc Performs periodic tasks every second
 */
static THD_WORKING_AREA(heartbeatThreadFuncWa, 128);

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
  CanBus canBus(kNodeIdCellTemp, CanBusBaudRate::k250k, false);
  chibios_rt::Mutex canBusMut;
  chibios_rt::Mutex spiBusMut;

  // create void* compatible obj
  std::vector<void*> args = {&canBus, &canBusMut};

  // start the CAN TX/RX threads
  chThdCreateStatic(canTxThreadFuncWa, sizeof(canTxThreadFuncWa), NORMALPRIO,
                    canTxThreadFunc, &args);
  chThdCreateStatic(canRxThreadFuncWa, sizeof(canRxThreadFuncWa), NORMALPRIO,
                    canRxThreadFunc, &args);
  // start the CAN heartbeat thread
  chThdCreateStatic(heartbeatThreadFuncWa, sizeof(heartbeatThreadFuncWa),
                    NORMALPRIO, heartbeatThreadFunc, &args);
  // Start SPI thread
  chThdCreateStatic(spiThread2Wa, sizeof(spiThread2Wa), NORMALPRIO + 1,
                    spiThread2, &args);

  // Successful startup indicator
  for (int32_t i = 0; i < 4; ++i) {
    palWriteLine(LINE_LED_GREEN, PAL_HIGH);
    chThdSleepMilliseconds(50);
    palWriteLine(LINE_LED_GREEN, PAL_LOW);
    chThdSleepMilliseconds(50);
  }

  while (1) {
    // Sleep 24 hours
    chThdSleepMilliseconds(1000 * 60 * 60 * 24);
  }
}
