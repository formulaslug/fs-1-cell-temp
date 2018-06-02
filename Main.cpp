// Copyright (c) 2016-2017 Formula Slug. All Rights Reserved.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>

#include "ch.hpp"
#include "hal.h"

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

  while (1) {
    // Blink continusouly
    palWriteLine(LINE_LED_GREEN, PAL_HIGH);
    chThdSleepMilliseconds(50);
    palWriteLine(LINE_LED_GREEN, PAL_LOW);
    chThdSleepMilliseconds(50);
  }
}
