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

enum serial_transaction_id {

    PUT_MASTER_MATRIX,
    PUT_MASTER_MATRIX_CHECKSUM,
    PUT_POINTING_DATA,
    PUT_POINTING_CHECKSUM,
    PUT_CMD,
    GET_LED_STATE,

    NUM_TOTAL_TRANSACTIONS
};

// Ensure we only use 5 bits for transaction
_Static_assert(NUM_TOTAL_TRANSACTIONS <= (1 << 5), "Max number of usable transactions exceeded");
