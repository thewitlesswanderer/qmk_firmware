/*
 * HAL driver system settings.
 */
#pragma once

#include_next "mcuconf.h"
// Unset default core1 start false, now needs to provide function c1_main
#undef RP_CORE1_START
#define RP_CORE1_START TRUE

#define RP_SIO_USE_UART0 TRUE
#define SIO_DEFAULT_BITRATE 115200

