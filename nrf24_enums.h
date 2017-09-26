/*
 * nRF24L01 device driver.
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.cm>
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

#ifndef NRF24_ENUMS_H
#define NRF24_ENUMS_H

enum nrf24_pipe_num {
        NRF24_PIPE0,
        NRF24_PIPE1,
        NRF24_PIPE2,
        NRF24_PIPE3,
        NRF24_PIPE4,
        NRF24_PIPE5,
        NRF24_TX,
        NRF24_PIPE_ALL = 0xFF
};

enum nrf24_crc_mode {
        NRF24_CRC_OFF,
        NRF24_CRC_8BIT = 2,
        NRF24_CRC_16BIT
};

enum nrf24_address_width {
        NRF24_AW_3 = 3,
        NRF24_AW_4,
        NRF24_AW_5
};

enum nrf24_pload {
        NRF24_TX_PLOAD = 7,
        NRF24_TX_PLOAD_NOACK,
        NRF24_RX_PLOAD,
        NRF24_ACK_PLOAD
};

enum nrf24_datarate {
        NRF24_DATARATE_1MBPS,
        NRF24_DATARATE_2MBPS,
        NRF24_DATARATE_256KBPS
};

enum nrf24_mode {
        NRF24_MODE_TX,
        NRF24_MODE_RX
};

enum nrf24_rf_power {
        NRF24_POWER_18DBM,
        NRF24_POWER_12DBM,
        NRF24_POWER_6DBM,
        NRF24_POWER_0DBM
};

#endif

