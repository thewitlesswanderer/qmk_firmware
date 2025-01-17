#include QMK_KEYBOARD_H

#include "tusb.h"

static void process_mouse_report(uint8_t dev_addr, hid_mouse_report_t const * report);
static void process_kbd_report(uint8_t dev_addr, hid_mouse_report_t const * report);
