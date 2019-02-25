/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
 *
 */

#ifndef NRF24L01_H
#define NRF24L01_H

/* nRF24L01 Register map */

#define CONFIG			0x00
#define EN_AA			0x01
#define EN_RXADDR		0x02
#define SETUP_AW		0x03
#define SETUP_RETR		0x04
#define RF_CH			0x05
#define RF_SETUP		0x06
#define STATUS			0x07
#define OBSERVE_TX		0x08
#define	CD			0x09
#define RX_ADDR_P0		0x0A
#define RX_ADDR_P1		0x0B
#define RX_ADDR_P2		0x0C
#define RX_ADDR_P3		0x0D
#define RX_ADDR_P4		0x0E
#define RX_ADDR_P5		0x0F
#define TX_ADDR			0x10
#define RX_PW_P0		0x11
#define RX_PW_P1		0x12
#define RX_PW_P2		0x13
#define RX_PW_P3		0x14
#define RX_PW_P4		0x15
#define RX_PW_P5		0x16
#define FIFO_STATUS		0x17
#define DYNPD			0x1C
#define FEATURE			0x1D

/* nRF24L01 Instruction Definitions */
#define W_REGISTER		0x20
#define R_RX_PL_WID		0x60
#define R_RX_PAYLOAD		0x61
#define W_TX_PAYLOAD		0xA0
#define W_ACK_PAYLOAD		0xA8
#define W_TX_PAYLOAD_NOACK	0xB0
#define FLUSH_TX		0xE1
#define FLUSH_RX		0xE2
#define REUSE_TX_PL		0xE3
#define LOCK_UNLOCK		0x50
#define NOP			0xFF

/* CONFIG 0x00 */
#define MASK_RX_DR		0x40
#define MASK_TX_DS		0x20
#define MASK_MAX_RT		0x10
#define EN_CRC			0x08
#define CRCO			0x04
#define PWR_UP			0x02
#define PRIM_RX			0x01

/* RF_SETUP 0x06 */
#define RF_DR_LO		0x20
#define PLL_LOCK		0x10
#define RF_DR_HI		0x08
#define RF_PWR1			0x04
#define RF_PWR0			0x02

/* STATUS 0x07 */
#define RX_DR			0x40
#define TX_DS			0x20
#define MAX_RT			0x10
#define TX_FULL			0x01

/* FEATURE 0x1D */
#define EN_DPL			0x04
#define EN_ACK_PAY		0x02
#define EN_DYN_ACK		0x01

#define PLOAD_MAX		32

#endif
