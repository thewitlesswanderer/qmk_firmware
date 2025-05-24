/* Copyright 2021 QMK
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "switch_util.h"
#include "matrix.h"
#include "keyboard.h"
#include "timer.h"
#include "transport.h"
#include "wait.h"
#include "debug.h"
#include "usb_util.h"
#include "bootloader.h"

// Max number of consecutive failed communications (one per scan cycle) before the communication is seen as disconnected.
// Set to 0 to disable the disconnection check altogether.
#ifndef SWITCH_MAX_CONNECTION_ERRORS
#    define SWITCH_MAX_CONNECTION_ERRORS 10
#endif // SPLIT_MAX_CONNECTION_ERRORS

// How long (in milliseconds) to block all connection attempts after the communication has been flagged as disconnected.
// One communication attempt will be allowed everytime this amount of time has passed since the last attempt. If that attempt succeeds, the communication is seen as working again.
// Set to 0 to disable communication throttling while disconnected
#ifndef SWITCH_CONNECTION_CHECK_TIMEOUT
#    define SWITCH_CONNECTION_CHECK_TIMEOUT 500
#endif // SPLIT_CONNECTION_CHECK_TIMEOUT

static uint8_t connection_errors = 0;

static inline bool usb_bus_detected(void) {
    return usb_vbus_state();
}


// this code runs before the keyboard is fully initialized
void switch_pre_init(void) {
#if defined(SWITCH_MASTER)
    transport_master_init();
#endif
}

// this code runs after the keyboard is fully initialized
//   - avoids race condition during matrix_init_quantum where slave can start
//     receiving before the init process has completed
void switch_post_init(void) {
#if defined(SWITCH_SLAVE)
    transport_slave_init();
#endif
}

bool is_transport_connected(void) {
    return connection_errors < SWITCH_MAX_CONNECTION_ERRORS;
}

bool transport_master_if_connected(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
#if SPLIT_MAX_CONNECTION_ERRORS > 0 && SPLIT_CONNECTION_CHECK_TIMEOUT > 0
    // Throttle transaction attempts if target doesn't seem to be connected
    // Without this, a solo half becomes unusable due to constant read timeouts
    static uint16_t connection_check_timer = 0;
    const bool      is_disconnected        = !is_transport_connected();
    if (is_disconnected && timer_elapsed(connection_check_timer) < SPLIT_CONNECTION_CHECK_TIMEOUT) {
        return false;
    }
#endif // SPLIT_MAX_CONNECTION_ERRORS > 0 && SPLIT_CONNECTION_CHECK_TIMEOUT > 0

    __attribute__((unused)) bool okay = transport_master(master_matrix, slave_matrix);
#if SPLIT_MAX_CONNECTION_ERRORS > 0
    if (!okay) {
        if (connection_errors < UINT8_MAX) {
            connection_errors++;
        }
#    if SPLIT_CONNECTION_CHECK_TIMEOUT > 0
        bool connected = is_transport_connected();
        if (!connected) {
            connection_check_timer = timer_read();
            dprintln("Target disconnected, throttling connection attempts");
        }
        return connected;
    } else if (is_disconnected) {
        dprintln("Target connected");
#    endif // SPLIT_CONNECTION_CHECK_TIMEOUT > 0
    }

    connection_errors = 0;
#endif // SPLIT_MAX_CONNECTION_ERRORS > 0
    return true;
}
