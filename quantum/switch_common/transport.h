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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "progmem.h"
#include "action_layer.h"
#include "matrix.h"

#ifndef RPC_M2S_BUFFER_SIZE
#    define RPC_M2S_BUFFER_SIZE 32
#endif // RPC_M2S_BUFFER_SIZE

#ifndef RPC_S2M_BUFFER_SIZE
#    define RPC_S2M_BUFFER_SIZE 32
#endif // RPC_S2M_BUFFER_SIZE

void transport_master_init(void);
void transport_slave_init(void);

// returns false if valid data not received from slave
bool transport_master(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]);
void transport_slave(matrix_row_t master_matrix[], matrix_row_t slave_matrix[]);

bool transport_execute_transaction(int8_t id, const void *initiator2target_buf, uint16_t initiator2target_length, void *target2initiator_buf, uint16_t target2initiator_length);

typedef struct _split_matrix_sync_t {
    uint8_t      checksum;
    matrix_row_t matrix[MATRIX_ROWS];
} split_matrix_sync_t;

#    include "pointing_device.h"
typedef struct _split_pointing_sync_t {
    uint8_t        checksum;
    report_mouse_t report;
} split_pointing_sync_t;


#if defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)
typedef struct _rpc_sync_info_t {
    uint8_t checksum;
    struct {
        int8_t  transaction_id;
        uint8_t m2s_length;
        uint8_t s2m_length;
    } payload;
} rpc_sync_info_t;
#endif // defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)


typedef struct _split_shared_memory_t {
#ifdef USE_I2C
    int8_t transaction_id;
#endif // USE_I2C

    split_matrix_sync_t matrix;
    split_pointing_sync_t pointing;
    uint8_t switch_cmd;
    uint8_t led_state;

#if defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)
    rpc_sync_info_t rpc_info;
    uint8_t         rpc_m2s_buffer[RPC_M2S_BUFFER_SIZE];
    uint8_t         rpc_s2m_buffer[RPC_S2M_BUFFER_SIZE];
#endif // defined(SPLIT_TRANSACTION_IDS_KB) || defined(SPLIT_TRANSACTION_IDS_USER)

} split_shared_memory_t;

extern split_shared_memory_t *const split_shmem;
