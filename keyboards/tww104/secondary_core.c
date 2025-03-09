#include QMK_KEYBOARD_H

#include "ch.h"
#include "hal.h"

//#include "chprintf.h"

#include "tusb.h"
#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "hardware/sync.h"

#define CONSTRAIN_HID_XY(amt) ((amt) < XY_REPORT_MIN ? XY_REPORT_MIN : ((amt) > XY_REPORT_MAX ? XY_REPORT_MAX : (amt)))

// 1ms repeat timer for USB frame
extern void pio_usb_host_frame(void);

static virtual_timer_t vt;

void alarm_pool_add_repeating_timer_us(void) {}
void alarm_pool_create(void) {}

static void __no_inline_not_in_flash_func(timer_cb)(virtual_timer_t *_vt, void *_) {
  // Start USB frame
  pio_usb_host_frame();

}

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

  hal_lld_peripheral_unreset(RESETS_ALLREG_PIO1);
  hal_lld_peripheral_unreset(RESETS_ALLREG_DMA);

  palSetLineMode(0U, PAL_MODE_ALTERNATE_UART);
  palSetLineMode(1U, PAL_MODE_ALTERNATE_UART);

  //sioStart(&SIOD0, NULL);

  //sioAsyncWrite(&SIOD0, (uint8_t *)"Second Core Start\n", 18);

  // To run USB in core1, config in core1.
  static pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
  config.pin_dp = 29;
  config.pio_tx_num = 1;
  config.tx_ch = 10;
  config.pio_rx_num = 1;
  config.skip_alarm_pool = true;
  config.pinout = PIO_USB_PINOUT_DMDP;

  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &config);

  tuh_init(1);

  // Start 1ms timer
  chVTObjectInit(&vt);
  chVTSetContinuous(&vt, TIME_US2I(1000), timer_cb, NULL);

  while (true) {
    tuh_task();
  }
}

void tuh_mount_cb(uint8_t dev_addr) {

   //chprintf((BaseSequentialStream*)&SIOD0,"USB device is mounted:%d\n", dev_addr);

}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  //chprintf((BaseSequentialStream*)&SIOD0,"VID:%x PID:%x\n", vid, pid);

  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  if (vid == 0x046D && (pid == 0xC52B || pid == 0xC548)) { //MX Master 3 or 3S
    if (itf_protocol == HID_ITF_PROTOCOL_MOUSE) {
      tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_REPORT); //Set Mouse to Report Protocol get pan
      //chprintf((BaseSequentialStream*)&SIOD0,"REPORT PROTOCOL\n");
    }
  }

  tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
    //chprintf((BaseSequentialStream*)&SIOD0,"HID unmounted:%d:%d\n", dev_addr, instance);
}

static void process_mouse_report(uint8_t dev_addr, uint8_t instance, uint8_t const * report)
{
 //chprintf((BaseSequentialStream*)&SIOD0,"%02x %d %d %d %d\r\n", report->buttons, report->x, report->y, report->wheel, report->pan);

  if (tuh_hid_get_protocol(dev_addr, instance) == HID_PROTOCOL_BOOT ) {
    hid_mouse_report_t const * boot_mouse_report = (hid_mouse_report_t const*) report;
    report_mouse_t mouse;

    mouse.buttons = boot_mouse_report->buttons;

    mouse.x = boot_mouse_report->x;
    mouse.y = boot_mouse_report->y;
    mouse.v = boot_mouse_report->wheel;
    mouse.h = boot_mouse_report->pan;

    chSysLock();
    pointing_device_set_report(mouse);
    chSysUnlock();

  } else {
    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    if (vid == 0x046D && (pid == 0xC52B || pid == 0xC548)) { //MX Master 3 or 3S
      if (report[0] == 0x02) {
        report_mouse_t mouse;
        if (pid == 0xC52B) { //MX Master 3

            int8_t ext_x = report[3];
            int8_t ext_y = (report[4]>>4) | (report[5]<<4);

            //chprintf((BaseSequentialStream*)&SIOD0,"X:%d Y:%d\n", ext_x, ext_y);
            //chprintf((BaseSequentialStream*)&SIOD0,"X:%d Y:%d\n", CONSTRAIN_HID_XY(ext_x), CONSTRAIN_HID_XY(ext_y));


            mouse.buttons = report[1];

            mouse.x = CONSTRAIN_HID_XY(ext_x);
            mouse.y = CONSTRAIN_HID_XY(ext_y);
            mouse.v = report[6];
            mouse.h = -report[7];

        } else if (pid == 0xC548) { //MX Master 3S

            int16_t ext_x = (report[3]) | (report[4]<<8);
            int16_t ext_y = (report[5]) | (report[6]<<8);

            //chprintf((BaseSequentialStream*)&SIOD0,"X:%d Y:%d\n", ext_x, ext_y);
            //chprintf((BaseSequentialStream*)&SIOD0,"X:%d Y:%d\n", CONSTRAIN_HID_XY(ext_x), CONSTRAIN_HID_XY(ext_y));

            int8_t prev_buttons = pointing_device_get_report().buttons;
            //chprintf((BaseSequentialStream*)&SIOD0,"B:%x\n", prev_buttons);
            mouse.buttons = report[1];
            if (!(prev_buttons & 0x20) && (mouse.buttons & 0x20)) {
                //chprintf((BaseSequentialStream*)&SIOD0,"Switch\n"); //Thumb button
            }

            mouse.x = CONSTRAIN_HID_XY(ext_x);
            mouse.y = CONSTRAIN_HID_XY(ext_y);
            mouse.v = report[7];
            mouse.h = -report[8];

        }


        chSysLock();
        pointing_device_set_report(mouse);
        chSysUnlock();

      }
    }
  }


}

static void process_kbd_report(uint8_t dev_addr, uint8_t const * report)
{
  //chprintf((BaseSequentialStream*)&SIOD0,"KBD ");
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if (vid == 0x046D && pid == 0xC52B) { //MX Master 3
    if (report[0] == 0x08 && report[2] == 0x2B) {
      //chprintf((BaseSequentialStream*)&SIOD0,"Switch\n"); //Thumb button
    }
  }


}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len) {
  //chprintf((BaseSequentialStream*)&SIOD0,"HID Interface:%u Report received\n", instance);
  for (uint16_t i=0; i<len+2; i++) {
    //chprintf((BaseSequentialStream*)&SIOD0,"%02x ", report[i]);
  }
  //chprintf((BaseSequentialStream*)&SIOD0,"\n");

  (void) len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  switch(itf_protocol)
  {
    case HID_ITF_PROTOCOL_KEYBOARD:
      process_kbd_report(dev_addr, report );
    break;

    case HID_ITF_PROTOCOL_MOUSE:
      process_mouse_report(dev_addr, instance, report );
    break;

    default: break;
  }
    tuh_hid_receive_report(dev_addr, instance);
}

