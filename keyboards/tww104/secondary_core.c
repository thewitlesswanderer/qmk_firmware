#include "ch.h"
#include "hal.h"

#include "chprintf.h"


// From chibios rp pico demo
/**
 * Core 1 entry point.
 */
void c1_main(void) {
    /*
     * Starting a new OS instance running on this core, we need to wait for
     * system initialization on the other side.
     */
  chSysWaitSystemState(ch_sys_running);
  chInstanceObjectInit(&ch1, &ch_core1_cfg);

    /* It is alive now.*/
  chSysUnlock();

  palSetLineMode(0U, PAL_MODE_ALTERNATE_UART);
  palSetLineMode(1U, PAL_MODE_ALTERNATE_UART);

  sioStart(&SIOD0, NULL);

  sioAsyncWrite(&SIOD0, (uint8_t *)"Second Core Start\n", 18);

  while (true) {
    chThdSleepMilliseconds(500);
  }

}
