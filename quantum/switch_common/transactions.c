/* Copyright 2021 QMK
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
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

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "crc.h"
#include "debug.h"
#include "matrix.h"
#include "host.h"
#include "action_util.h"
#include "sync_timer.h"
#include "wait.h"
#include "transactions.h"
#include "transport.h"
#include "transaction_id_define.h"
#include "switch_util.h"
#include "synchronization_util.h"

#ifdef POINTING_DEVICE_ENABLE
#    include "pointing_device.h"
#endif

#define SYNC_TIMER_OFFSET 2

#ifndef FORCED_SYNC_THROTTLE_MS
#    define FORCED_SYNC_THROTTLE_MS 100
#endif // FORCED_SYNC_THROTTLE_MS

#define sizeof_member(type, member) sizeof(((type *)NULL)->member)

#define trans_initiator2target_initializer_cb(member, cb) \
    { sizeof_member(split_shared_memory_t, member), offsetof(split_shared_memory_t, member), 0, 0, cb }
#define trans_initiator2target_initializer(member) trans_initiator2target_initializer_cb(member, NULL)

#define trans_target2initiator_initializer_cb(member, cb) \
    { 0, 0, sizeof_member(split_shared_memory_t, member), offsetof(split_shared_memory_t, member), cb }
#define trans_target2initiator_initializer(member) trans_target2initiator_initializer_cb(member, NULL)

#define trans_initiator2target_cb(cb) \
    { 0, 0, 0, 0, cb }

#define transport_write(id, data, length) transport_execute_transaction(id, data, length, NULL, 0)
#define transport_read(id, data, length) transport_execute_transaction(id, NULL, 0, data, length)
#define transport_exec(id) transport_execute_transaction(id, NULL, 0, NULL, 0)

#if defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)
// Forward-declare the RPC callback handlers
void slave_rpc_info_callback(uint8_t initiator2target_buffer_size, const void *initiator2target_buffer, uint8_t target2initiator_buffer_size, void *target2initiator_buffer);
void slave_rpc_exec_callback(uint8_t initiator2target_buffer_size, const void *initiator2target_buffer, uint8_t target2initiator_buffer_size, void *target2initiator_buffer);
#endif // defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)

////////////////////////////////////////////////////
// Helpers

static bool transaction_handler_master(matrix_row_t master_matrix[], matrix_row_t slave_matrix[], const char *prefix, bool (*handler)(matrix_row_t master_matrix[], matrix_row_t slave_matrix[])) {
    int num_retries = is_transport_connected() ? 10 : 1;
    for (int iter = 1; iter <= num_retries; ++iter) {
        if (iter > 1) {
            for (int i = 0; i < iter * iter; ++i) {
                wait_us(10);
            }
        }
        bool this_okay = true;
        this_okay      = handler(master_matrix, slave_matrix);
        if (this_okay) return true;
    }
    dprintf("Failed to execute %s\n", prefix);
    return false;
}

#define TRANSACTION_HANDLER_MASTER(prefix)                                                                              \
    do {                                                                                                                \
        if (!transaction_handler_master(master_matrix, slave_matrix, #prefix, &prefix##_handlers_master)) return false; \
    } while (0)

/**
 * @brief Constructs a transaction handler that doesn't acquire a lock to the
 * split shared memory. Therefore the locking and unlocking has to be done
 * manually inside the handler. Use this macro only if the handler is
 * non-deterministic in runtime and thus needs a manual lock unlock
 * implementation to hold the lock for the shortest possible time.
 */
#define TRANSACTION_HANDLER_SLAVE(prefix)                     \
    do {                                                      \
        prefix##_handlers_slave(master_matrix, slave_matrix); \
    } while (0)

/**
 * @brief Constructs a transaction handler that automatically acquires a lock to
 * safely access the split shared memory and releases the lock again after
 * processing the handler. Use this macro if the handler is fast and
 * deterministic in runtime and thus holds the lock only for a very short time.
 * If not fallback to manually locking and unlocking inside the handler.
 */
#define TRANSACTION_HANDLER_SLAVE_AUTOLOCK(prefix)            \
    do {                                                      \
        split_shared_memory_lock();                           \
        prefix##_handlers_slave(master_matrix, slave_matrix); \
        split_shared_memory_unlock();                         \
    } while (0)

inline static bool read_if_checksum_mismatch(int8_t trans_id_checksum, int8_t trans_id_retrieve, uint32_t *last_update, void *destination, const void *equiv_shmem, size_t length) {
    uint8_t curr_checksum;
    bool    okay = transport_read(trans_id_checksum, &curr_checksum, sizeof(curr_checksum));
    if (okay && (timer_elapsed32(*last_update) >= FORCED_SYNC_THROTTLE_MS || curr_checksum != crc8(equiv_shmem, length))) {
        okay &= transport_read(trans_id_retrieve, destination, length);
        okay &= curr_checksum == crc8(equiv_shmem, length);
        if (okay) {
            *last_update = timer_read32();
        }
    } else {
        memcpy(destination, equiv_shmem, length);
    }
    return okay;
}

inline static bool send_if_condition(int8_t trans_id, uint32_t *last_update, bool condition, void *source, size_t length) {
    bool okay = true;
    if (timer_elapsed32(*last_update) >= FORCED_SYNC_THROTTLE_MS || condition) {
        okay &= transport_write(trans_id, source, length);
        if (okay) {
            *last_update = timer_read32();
        }
    }
    return okay;
}

inline static bool send_if_data_mismatch(int8_t trans_id, uint32_t *last_update, void *source, const void *equiv_shmem, size_t length) {
    // Just run a memcmp to compare the source and equivalent shmem location
    return send_if_condition(trans_id, last_update, (memcmp(source, equiv_shmem, length) != 0), source, length);
}

////////////////////////////////////////////////////
// Matrix

static bool master_matrix_handlers_master(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    static uint32_t last_update = 0;

    split_shmem->matrix.checksum = crc8(split_shmem->matrix.matrix, sizeof(split_shmem->matrix.matrix));
    bool okay = send_if_data_mismatch(PUT_MASTER_MATRIX_CHECKSUM, &last_update, master_matrix, &split_shmem->matrix.checksum, sizeof(split_shmem->matrix.checksum));
     if (okay) {
        okay &= send_if_condition(PUT_MASTER_MATRIX, &last_update, master_matrix, split_shmem->matrix.matrix, sizeof(split_shmem->matrix.matrix));
    }
    return okay;
}

static void master_matrix_handlers_slave(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    static uint32_t     last_update                    = 0;
    static matrix_row_t last_matrix[MATRIX_ROWS] = {0}; // last successfully-read matrix, so we can replicate if there are checksum errors
    matrix_row_t        temp_matrix[MATRIX_ROWS];       // holding area while we test whether or not checksum is correct

    bool okay = read_if_checksum_mismatch(PUT_MASTER_MATRIX_CHECKSUM, PUT_MASTER_MATRIX, &last_update, temp_matrix, split_shmem->matrix.matrix, sizeof(split_shmem->matrix.matrix));
    if (okay) {
        // Checksum matches the received data, save as the last matrix state
        memcpy(last_matrix, temp_matrix, sizeof(temp_matrix));
    }
    // Copy out the last-known-good matrix state to the matrix
    memcpy(slave_matrix, last_matrix, sizeof(last_matrix));
}

// clang-format off
#define TRANSACTIONS_MASTER_MATRIX_MASTER() TRANSACTION_HANDLER_MASTER(master_matrix)
#define TRANSACTIONS_MASTER_MATRIX_SLAVE() TRANSACTION_HANDLER_SLAVE_AUTOLOCK(master_matrix)
#define TRANSACTIONS_MASTER_MATRIX_REGISTRATIONS \
    [PUT_MASTER_MATRIX_CHECKSUM] = trans_initiator2target_initializer(matrix.checksum), \
    [PUT_MASTER_MATRIX]     = trans_initiator2target_initializer(matrix.matrix),
// clang-format on

////////////////////////////////////////////////////
// POINTING

extern const pointing_device_driver_t pointing_device_driver;

static bool pointing_handlers_master(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    static uint32_t last_update = 0;

    split_pointing_sync_t pointing;
    memcpy(&pointing, &split_shmem->pointing, sizeof(split_pointing_sync_t));

    pointing.report = pointing_device_driver.get_report((report_mouse_t){0});
    // Now update the checksum given that the pointing has been written to
    pointing.checksum = crc8(&pointing.report, sizeof(report_mouse_t));

    memcpy(&split_shmem->pointing, &pointing, sizeof(split_pointing_sync_t));

    bool okay = send_if_data_mismatch(PUT_POINTING_CHECKSUM, &last_update, master_matrix, &split_shmem->pointing.checksum, sizeof(split_shmem->pointing.checksum));
     if (okay) {
        okay &= send_if_condition(PUT_POINTING_DATA, &last_update, master_matrix, &split_shmem->pointing.report, sizeof(split_shmem->pointing.report));
    }
    return okay;
}

static void pointing_handlers_slave(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    static uint32_t last_update     = 0;
    report_mouse_t  temp_state;
    bool            okay = read_if_checksum_mismatch(PUT_POINTING_CHECKSUM, PUT_POINTING_DATA, &last_update, &temp_state, &split_shmem->pointing.report, sizeof(temp_state));
    if (okay) pointing_device_set_report(temp_state);
}

#    define TRANSACTIONS_POINTING_MASTER() TRANSACTION_HANDLER_MASTER(pointing)
#    define TRANSACTIONS_POINTING_SLAVE() TRANSACTION_HANDLER_SLAVE_AUTOLOCK(pointing)
#    define TRANSACTIONS_POINTING_REGISTRATIONS [PUT_POINTING_CHECKSUM] = trans_initiator2target_initializer(pointing.checksum), [PUT_POINTING_DATA] = trans_initiator2target_initializer(pointing.report),


////////////////////////////////////////////////////
// LED state

static bool led_state_handlers_master(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    void set_split_host_keyboard_leds(uint8_t led_state);
    set_split_host_keyboard_leds(split_shmem->led_state);
    return true;
}


static void led_state_handlers_slave(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    split_shmem->led_state = host_keyboard_leds();
    memcpy(&split_shmem->led_state, slave_matrix, sizeof(split_shmem->led_state));

}

#    define TRANSACTIONS_LED_STATE_MASTER() TRANSACTION_HANDLER_MASTER(led_state)
#    define TRANSACTIONS_LED_STATE_SLAVE() TRANSACTION_HANDLER_SLAVE_AUTOLOCK(led_state)
#    define TRANSACTIONS_LED_STATE_REGISTRATIONS [GET_LED_STATE] = trans_target2initiator_initializer(led_state),

////////////////////////////////////////////////////


split_transaction_desc_t split_transaction_table[NUM_TOTAL_TRANSACTIONS] = {
    // Set defaults
    [0 ...(NUM_TOTAL_TRANSACTIONS - 1)] = {0, 0, 0, 0, 0},

#ifdef USE_I2C
    [I2C_EXECUTE_CALLBACK] = trans_initiator2target_initializer(transaction_id),
#endif // USE_I2C

    // clang-format off
    TRANSACTIONS_MASTER_MATRIX_REGISTRATIONS
    TRANSACTIONS_LED_STATE_REGISTRATIONS
    TRANSACTIONS_POINTING_REGISTRATIONS
    // clang-format on

#if defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)
        [PUT_RPC_INFO]  = trans_initiator2target_initializer_cb(rpc_info, slave_rpc_info_callback),
    [PUT_RPC_REQ_DATA]  = trans_initiator2target_initializer(rpc_m2s_buffer),
    [EXECUTE_RPC]       = trans_initiator2target_initializer_cb(rpc_info.payload.transaction_id, slave_rpc_exec_callback),
    [GET_RPC_RESP_DATA] = trans_target2initiator_initializer(rpc_s2m_buffer),
#endif // defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)
};

bool transactions_master(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    TRANSACTIONS_MASTER_MATRIX_MASTER();
    TRANSACTIONS_LED_STATE_MASTER();
    TRANSACTIONS_POINTING_MASTER();
    return true;
}

void transactions_slave(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]) {
    TRANSACTIONS_MASTER_MATRIX_SLAVE();
    TRANSACTIONS_LED_STATE_SLAVE();
    TRANSACTIONS_POINTING_SLAVE();
}

#if defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)

void transaction_register_rpc(int8_t transaction_id, slave_callback_t callback) {
    // Prevent invoking RPC on QMK core sync data
    if (transaction_id <= GET_RPC_RESP_DATA) return;

    // Set the callback
    split_transaction_table[transaction_id].slave_callback          = callback;
    split_transaction_table[transaction_id].initiator2target_offset = offsetof(split_shared_memory_t, rpc_m2s_buffer);
    split_transaction_table[transaction_id].target2initiator_offset = offsetof(split_shared_memory_t, rpc_s2m_buffer);
}

bool transaction_rpc_exec(int8_t transaction_id, uint8_t initiator2target_buffer_size, const void *initiator2target_buffer, uint8_t target2initiator_buffer_size, void *target2initiator_buffer) {
    // Prevent transaction attempts while transport is disconnected
    if (!is_transport_connected()) {
        return false;
    }
    // Prevent invoking RPC on QMK core sync data
    if (transaction_id <= GET_RPC_RESP_DATA) return false;
    // Prevent sizing issues
    if (initiator2target_buffer_size > RPC_M2S_BUFFER_SIZE) return false;
    if (target2initiator_buffer_size > RPC_S2M_BUFFER_SIZE) return false;

    // Prepare the metadata block
    rpc_sync_info_t info = {.payload = {.transaction_id = transaction_id, .m2s_length = initiator2target_buffer_size, .s2m_length = target2initiator_buffer_size}};
    info.checksum        = crc8(&info.payload, sizeof(info.payload));

    // Make sure the local side knows that we're not sending the full block of data
    split_transaction_table[PUT_RPC_REQ_DATA].initiator2target_buffer_size  = initiator2target_buffer_size;
    split_transaction_table[GET_RPC_RESP_DATA].target2initiator_buffer_size = target2initiator_buffer_size;

    // Run through the sequence:
    // * set the transaction ID and lengths
    // * send the request data
    // * execute RPC callback
    // * retrieve the response data
    if (!transport_write(PUT_RPC_INFO, &info, sizeof(info))) {
        return false;
    }
    if (!transport_write(PUT_RPC_REQ_DATA, initiator2target_buffer, initiator2target_buffer_size)) {
        return false;
    }
    if (!transport_write(EXECUTE_RPC, &transaction_id, sizeof(transaction_id))) {
        return false;
    }
    if (!transport_read(GET_RPC_RESP_DATA, target2initiator_buffer, target2initiator_buffer_size)) {
        return false;
    }
    return true;
}

void slave_rpc_info_callback(uint8_t initiator2target_buffer_size, const void *initiator2target_buffer, uint8_t target2initiator_buffer_size, void *target2initiator_buffer) {
    // The RPC info block contains the intended transaction ID, as well as the sizes for both inbound and outbound data.
    // Ignore the args -- the `split_shmem` already has the info, we just need to act upon it.
    // We must keep the `split_transaction_table` non-const, so that it is able to be modified at runtime.

    split_transaction_table[PUT_RPC_REQ_DATA].initiator2target_buffer_size  = split_shmem->rpc_info.payload.m2s_length;
    split_transaction_table[GET_RPC_RESP_DATA].target2initiator_buffer_size = split_shmem->rpc_info.payload.s2m_length;
}

void slave_rpc_exec_callback(uint8_t initiator2target_buffer_size, const void *initiator2target_buffer, uint8_t target2initiator_buffer_size, void *target2initiator_buffer) {
    // We can assume that the buffer lengths are correctly set, now, given that sequentially the rpc_info callback was already executed.
    // Go through the rpc_info and execute _that_ transaction's callback, with the scratch buffers as inputs.
    // As a safety precaution we check that the received payload matches its checksum first.
    if (crc8(&split_shmem->rpc_info.payload, sizeof(split_shmem->rpc_info.payload)) != split_shmem->rpc_info.checksum) {
        return;
    }

    int8_t transaction_id = split_shmem->rpc_info.payload.transaction_id;
    if (transaction_id < NUM_TOTAL_TRANSACTIONS) {
        split_transaction_desc_t *trans = &split_transaction_table[transaction_id];
        if (trans->slave_callback) {
            trans->slave_callback(split_shmem->rpc_info.payload.m2s_length, split_shmem->rpc_m2s_buffer, split_shmem->rpc_info.payload.s2m_length, split_shmem->rpc_s2m_buffer);
        }
    }
}

#endif // defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)
