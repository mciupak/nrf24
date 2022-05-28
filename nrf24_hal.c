// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
 *
 */

#include <linux/types.h>
#include <linux/spi/spi.h>

#include "nrf24_hal.h"

static ssize_t nrf24_read_reg(struct spi_device *spi, u8 addr)
{
	ssize_t ret;

	ret = spi_w8r8(spi, addr);

	if (ret < 0)
		dev_dbg(&spi->dev, "%s: read 0x%X FAILED\n", __func__, addr);

	return ret;
}

static ssize_t nrf24_write_reg(struct spi_device *spi, u8 addr, u8 val)
{
	ssize_t ret;
	u8 buffer[2];

	buffer[0] = addr;
	buffer[1] = val;

	if (addr < W_REGISTER) {
		buffer[0] = buffer[0] + W_REGISTER;
		ret = spi_write(spi, buffer, 2);
	} else if (addr != FLUSH_TX &&
		   addr != FLUSH_RX &&
		   addr != REUSE_TX_PL) {
		ret = spi_write(spi, buffer, 2);
	} else {
		ret = spi_write(spi, buffer, 1);
	}
	if (ret < 0)
		dev_dbg(&spi->dev, "%s: write 0x%X to 0x%X  FAILED\n",
			__func__, val, addr);

	return ret;
}

static ssize_t nrf24_write_multireg(struct spi_device *spi,
				    u8 reg,
				    u8 *buf,
				    u8 length)
{
	u8 buffer[PLOAD_MAX + 1];

	if (!length)
		return -EINVAL;

	switch (reg) {
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_TX:
		buffer[0] = W_REGISTER + RX_ADDR_P0 + reg;
		break;
	case NRF24_TX_PLOAD:
		buffer[0] = W_TX_PAYLOAD;
		break;
	case NRF24_TX_PLOAD_NOACK:
		buffer[0] = W_TX_PAYLOAD_NOACK;
		break;
	default:
		dev_dbg(&spi->dev, "%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

	memcpy(buffer + 1, buf, length);

	return spi_write(spi, buffer, length + 1);
}

static ssize_t nrf24_read_multireg(struct spi_device *spi, u8 reg, u8 *buf)
{
	ssize_t ret;
	u8 reg_addr;
	ssize_t length = 0;

	switch (reg) {
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_TX:
		length = nrf24_get_address_width(spi);
		if (length < 0)
			return length;
		reg_addr = RX_ADDR_P0 + reg;
		break;
	case NRF24_RX_PLOAD:
		ret = nrf24_get_rx_data_source(spi);
		if (ret < 0) {
			dev_dbg(&spi->dev, "%s: invalid rx data source (%zd)\n", __func__, ret);
			return ret;
		}

		if (ret < NRF24_TX_PLOAD) {
			length = nrf24_get_rx_pl_w(spi);
			if (length < 0)
				return length;
			reg_addr = R_RX_PAYLOAD;
		}
		break;
	default:
		dev_dbg(&spi->dev, "%s: invalid parameter\n", __func__);
		return -EINVAL;
	}

	if (length > 0 && length <= PLOAD_MAX) {
		dev_dbg(&spi->dev, "%s: length = %zd\n", __func__, length);

		ret = spi_write_then_read(spi,
					  &reg_addr,
					  1,
					  buf,
					  length);

		if (ret < 0)
			return ret;
	}

	return length;
}

ssize_t nrf24_get_dynamic_pl(struct spi_device *spi)
{
	ssize_t feature;

	feature = nrf24_read_reg(spi, FEATURE);
	if (feature < 0)
		return feature;

	return (feature & EN_DPL) == EN_DPL;
}

ssize_t nrf24_enable_dynamic_pl(struct spi_device *spi)
{
	ssize_t feature;

	feature = nrf24_read_reg(spi, FEATURE);
	if (feature < 0)
		return feature;
	return nrf24_write_reg(spi, FEATURE, feature | EN_DPL);
}

ssize_t nrf24_disable_dynamic_pl(struct spi_device *spi)
{
	ssize_t feature;

	feature = nrf24_read_reg(spi, FEATURE);
	if (feature < 0)
		return feature;
	return nrf24_write_reg(spi, FEATURE, feature & ~EN_DPL);
}

static ssize_t nrf24_setup_dynamic_pl(struct spi_device *spi,
				      u8 pipe,
				      bool enable)
{
	ssize_t dynpd;
	ssize_t ret;

	if (pipe != NRF24_PIPE0 && enable) {
		ret = nrf24_setup_dynamic_pl(spi, NRF24_PIPE0, enable);
		if (ret < 0)
			return ret;
	}
	dynpd = nrf24_read_reg(spi, DYNPD);
	if (dynpd < 0)
		return dynpd;

	if (enable) {
		ret = nrf24_setup_auto_ack(spi, pipe, enable);
		if (ret < 0)
			return ret;

		dynpd |= BIT(pipe);
	} else {
		dynpd &= ~BIT(pipe);
	}
	ret = nrf24_write_reg(spi, DYNPD, dynpd);
	if (ret < 0)
		return ret;

	if (dynpd)
		ret = nrf24_enable_dynamic_pl(spi);
	else
		ret = nrf24_disable_dynamic_pl(spi);

	return ret;
}

ssize_t nrf24_setup_auto_ack(struct spi_device *spi, u8 pipe, bool enable)
{
	ssize_t aa;

	aa = nrf24_read_reg(spi, EN_AA);
	if (aa < 0)
		return aa;
	if (enable)
		aa |= BIT(pipe);
	else
		aa &= ~BIT(pipe);

	return nrf24_write_reg(spi, EN_AA, aa);
}

static ssize_t nrf24_enable_pipe(struct spi_device *spi, u8 pipe)
{
	ssize_t rxaddr;

	rxaddr = nrf24_read_reg(spi, EN_RXADDR);
	if (rxaddr < 0)
		return rxaddr;
	return nrf24_write_reg(spi, EN_RXADDR, rxaddr | BIT(pipe));
}

static ssize_t nrf24_disable_pipe(struct spi_device *spi, u8 pipe)
{
	ssize_t rxaddr;

	rxaddr = nrf24_read_reg(spi, EN_RXADDR);
	if (rxaddr < 0)
		return rxaddr;
	return nrf24_write_reg(spi, EN_RXADDR, rxaddr & ~BIT(pipe));
}

ssize_t nrf24_open_pipe(struct spi_device *spi, enum nrf24_pipe_num pipe)
{
	ssize_t ret = -EINVAL;

	switch (pipe) {
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_PIPE2:
	case NRF24_PIPE3:
	case NRF24_PIPE4:
	case NRF24_PIPE5:
		ret = nrf24_enable_pipe(spi, pipe);
		break;
	case NRF24_PIPE_ALL:
		ret = nrf24_write_reg(spi, EN_RXADDR, 0x3F);
		break;
	default:
		dev_dbg(&spi->dev, "%s: invalid parameter\n", __func__);
	}

	return ret;
}

ssize_t nrf24_close_pipe(struct spi_device *spi, enum nrf24_pipe_num pipe)
{
	ssize_t ret = -EINVAL;

	switch (pipe) {
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_PIPE2:
	case NRF24_PIPE3:
	case NRF24_PIPE4:
	case NRF24_PIPE5:
		ret = nrf24_disable_pipe(spi, pipe);
		if (ret < 0)
			break;
		ret = nrf24_setup_auto_ack(spi, pipe, false);
		break;
	case NRF24_PIPE_ALL:
		ret = nrf24_write_reg(spi, EN_RXADDR, 0x00);
		if (ret)
			break;
		ret = nrf24_write_reg(spi, EN_AA, 0x00);
		break;
	default:
		dev_dbg(&spi->dev, "%s: invalid parameter\n", __func__);
	}

	return ret;
}

ssize_t nrf24_set_address(struct spi_device *spi,
			  enum nrf24_pipe_num pipe,
			  u8 *addr)
{
	ssize_t ret = -EINVAL;
	ssize_t length;

	switch (pipe) {
	case NRF24_TX:
	case NRF24_PIPE0:
	case NRF24_PIPE1:
		length = nrf24_get_address_width(spi);
		if (length < 0)
			return length;
		ret = nrf24_write_multireg(spi, pipe, addr, length);
		break;
	case NRF24_PIPE2:
	case NRF24_PIPE3:
	case NRF24_PIPE4:
	case NRF24_PIPE5:
		ret = nrf24_write_reg(spi, RX_ADDR_P0 + pipe, *addr);
		break;
	default:
		dev_dbg(&spi->dev, "%s: invalid parameter\n", __func__);
	}

	return ret;
}

ssize_t nrf24_get_address(struct spi_device *spi,
			  enum nrf24_pipe_num pipe,
			  u8 *addr)
{
	ssize_t ret;
	ssize_t length;

	switch (pipe) {
	case NRF24_PIPE0:
	case NRF24_PIPE1:
	case NRF24_TX:
		ret = nrf24_read_multireg(spi, pipe, addr);
		if (ret < 0)
			return ret;
		break;
	default:
		length = nrf24_read_multireg(spi, NRF24_PIPE1, addr);
		if (length < 0)
			return length;
		ret = nrf24_read_reg(spi, RX_ADDR_P0 + pipe);
		if (ret < 0)
			return ret;
		*(addr) = ret;
		break;
	}

	return nrf24_get_address_width(spi);
}

ssize_t nrf24_set_crc_mode(struct spi_device *spi, enum nrf24_crc_mode mode)
{
	ssize_t config;

	config = nrf24_read_reg(spi, CONFIG);
	if (config < 0)
		return config;
	config &= ~(EN_CRC | CRCO);
	config |= (mode << 2);

	config = nrf24_write_reg(spi, CONFIG, config);

	return config;
}

ssize_t nrf24_get_crc_mode(struct spi_device *spi)
{
	ssize_t config;

	config = nrf24_read_reg(spi, CONFIG);
	if (config < 0)
		return config;
	config &= (EN_CRC | CRCO);
	config >>= 2;

	return config;
}

ssize_t nrf24_set_auto_retr_delay(struct spi_device *spi, u16 delay)
{
	ssize_t retr;

	retr = nrf24_read_reg(spi, SETUP_RETR);
	if (retr < 0)
		return retr;

	retr &= 0x0F;
	retr |= (((delay / 250) - 1) << 4);

	return nrf24_write_reg(spi, SETUP_RETR, retr);
}

ssize_t nrf24_get_auto_retr_delay(struct spi_device *spi)
{
	ssize_t retr;

	retr = nrf24_read_reg(spi, SETUP_RETR);
	if (retr < 0)
		return retr;

	return ((retr >> 4) + 1) * 250;
}

ssize_t nrf24_set_auto_retr_count(struct spi_device *spi, u8 count)
{
	ssize_t retr;

	retr = nrf24_read_reg(spi, SETUP_RETR);
	if (retr < 0)
		return retr;

	retr &= 0xF0;
	retr |= (count & 0x0F);

	return nrf24_write_reg(spi, SETUP_RETR, retr);
}

ssize_t nrf24_get_auto_retr_count(struct spi_device *spi)
{
	ssize_t retr;

	retr = nrf24_read_reg(spi, SETUP_RETR);

	if (retr < 0)
		return retr;

	return retr & 0x0F;
}

ssize_t nrf24_set_address_width(struct spi_device *spi,
				enum nrf24_address_width aw)
{
	return nrf24_write_reg(spi, SETUP_AW, aw - 2);
}

ssize_t nrf24_get_address_width(struct spi_device *spi)
{
	return nrf24_read_reg(spi, SETUP_AW) + 2;
}

ssize_t nrf24_lock_unlock(struct spi_device *spi)
{
	return nrf24_write_reg(spi, LOCK_UNLOCK, 0x73);
}

ssize_t nrf24_set_datarate(struct spi_device *spi, enum nrf24_datarate datarate)
{
	ssize_t rf;

	rf = nrf24_read_reg(spi, RF_SETUP);
	if (rf < 0)
		return rf;
	if (datarate == NRF24_DATARATE_1MBPS)
		rf &= ~RF_DR_HI;
	else
		rf |= RF_DR_HI;

	return nrf24_write_reg(spi, RF_SETUP, rf);
}

ssize_t nrf24_get_datarate(struct spi_device *spi)
{
	ssize_t rf;
	ssize_t lo;
	ssize_t hi;

	rf = nrf24_read_reg(spi, RF_SETUP);
	if (rf < 0)
		return rf;

	lo = rf & RF_DR_LO;
	hi = rf & RF_DR_HI;

	if (lo && hi)
		return -EINVAL;
	if (lo)
		return NRF24_DATARATE_256KBPS;
	if (hi)
		return NRF24_DATARATE_2MBPS;
	return NRF24_DATARATE_1MBPS;
}

ssize_t nrf24_set_mode(struct spi_device *spi, enum nrf24_mode mode)
{
	ssize_t config;

	config = nrf24_read_reg(spi, CONFIG);

	if (config < 0)
		return config;

	if (mode == NRF24_MODE_RX)
		config |= PRIM_RX;
	else
		config &= ~PRIM_RX;

	return nrf24_write_reg(spi, CONFIG, config);
}

ssize_t nrf24_set_rf_power(struct spi_device *spi, enum nrf24_rf_power rf_pwr)
{
	ssize_t rf_setup;

	rf_setup = nrf24_read_reg(spi, RF_SETUP);

	if (rf_setup < 0)
		return rf_setup;

	rf_setup &= ~(RF_PWR1 | RF_PWR0);
	rf_setup |= (rf_pwr << 1);

	return nrf24_write_reg(spi, RF_SETUP, rf_setup);
}

ssize_t nrf24_get_rf_power(struct spi_device *spi)
{
	ssize_t rf;

	rf = nrf24_read_reg(spi, RF_SETUP);

	if (rf < 0)
		return rf;

	rf &= (RF_PWR1 | RF_PWR0);
	rf >>= 1;

	return rf;
}

//plw = 0 -> dynamic
ssize_t nrf24_set_rx_pload_width(struct spi_device *spi, u8 pipe, u8 plw)
{
	ssize_t ret;

	if (plw > PLOAD_MAX)
		return -EINVAL;

	ret = nrf24_write_reg(spi, RX_PW_P0 + pipe, plw);
	if (ret < 0)
		return ret;

	return nrf24_setup_dynamic_pl(spi, pipe, plw == 0);
}

ssize_t nrf24_set_rf_channel(struct spi_device *spi, u8 channel)
{
	return nrf24_write_reg(spi, RF_CH, channel);
}

ssize_t nrf24_get_rf_channel(struct spi_device *spi)
{
	return nrf24_read_reg(spi, RF_CH);
}

ssize_t nrf24_power_up(struct spi_device *spi)
{
	ssize_t config;

	config = nrf24_read_reg(spi, CONFIG);

	if (config < 0)
		return config;

	return nrf24_write_reg(spi, CONFIG, config | PWR_UP);
}

ssize_t nrf24_write_tx_pload(struct spi_device *dev, u8 *buf, u8 length)
{
	return nrf24_write_multireg(dev, NRF24_TX_PLOAD, buf, length);
}

ssize_t nrf24_write_tx_pload_noack(struct spi_device *dev, u8 *buf, u8 length)
{
	return nrf24_write_multireg(dev, NRF24_TX_PLOAD_NOACK, buf, length);
}

ssize_t nrf24_read_rx_pload(struct spi_device *spi, u8 *buf)
{
	return nrf24_read_multireg(spi, NRF24_RX_PLOAD, buf);
}

ssize_t nrf24_get_status(struct spi_device *spi)
{
	return nrf24_read_reg(spi, STATUS);
}

ssize_t nrf24_get_rx_data_source(struct spi_device *spi)
{
	ssize_t status;

	status = nrf24_get_status(spi);
	if (status < 0)
		return status;
	return (status & 0x0E) >> 1;
}

ssize_t nrf24_get_rx_pload_width(struct spi_device *spi, u8 pipe)
{
	return nrf24_read_reg(spi, RX_PW_P0 + pipe);
}

ssize_t nrf24_get_rx_pl_w(struct spi_device *spi)
{
	return nrf24_read_reg(spi, R_RX_PL_WID);
}

ssize_t nrf24_soft_reset(struct spi_device *spi)
{
	ssize_t ret;
	u8 addr0[5] = {0xf0, 0xf0, 0xf0, 0xf0, 0xe1};
	u8 addr1[5] = {0xf0, 0xf0, 0xf0, 0xf0, 0xe1};

	ret = nrf24_write_reg(spi, CONFIG, 0x08);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, EN_AA, 0x3F);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, EN_RXADDR, 0x03);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, SETUP_AW, 0x03);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, SETUP_RETR, 0x03);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RF_CH, 0x02);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RF_SETUP, 0x07);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, STATUS, 0x70);
	if (ret < 0)
		return ret;
	ret = nrf24_set_address(spi, NRF24_PIPE0, addr0);
	if (ret < 0)
		return ret;
	ret = nrf24_set_address(spi, NRF24_PIPE1, addr1);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_ADDR_P2, 0xC3);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_ADDR_P3, 0xC4);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_ADDR_P4, 0xC5);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_ADDR_P5, 0xC6);
	if (ret < 0)
		return ret;
	ret = nrf24_set_address(spi, NRF24_TX, addr0);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_PW_P0, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_PW_P1, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_PW_P2, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_PW_P3, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_PW_P4, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, RX_PW_P5, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, DYNPD, 0x00);
	if (ret < 0)
		return ret;
	ret = nrf24_write_reg(spi, FEATURE, 0x00);
	if (ret < 0)
		return ret;

	return ret;
}

ssize_t nrf24_clear_irq(struct spi_device *spi, u8 irq)
{
	return nrf24_write_reg(spi, STATUS, irq);
}

ssize_t nrf24_flush_rx_fifo(struct spi_device *spi)
{
	return nrf24_write_reg(spi, FLUSH_RX, 0);
}

ssize_t nrf24_flush_tx_fifo(struct spi_device *spi)
{
	return nrf24_write_reg(spi, FLUSH_TX, 0);
}

ssize_t nrf24_flush_fifo(struct spi_device *spi)
{
	ssize_t ret;

	ret = nrf24_flush_rx_fifo(spi);
	if (ret < 0)
		return ret;
	return nrf24_flush_tx_fifo(spi);
}

ssize_t nrf24_print_status(struct spi_device *spi)
{
const u8 nrf_reg[] = {
	CONFIG,
	EN_AA,
	EN_RXADDR,
	SETUP_AW,
	SETUP_RETR,
	RF_CH,
	RF_SETUP,
	STATUS,
	OBSERVE_TX,
	CD,
	FIFO_STATUS,
	DYNPD,
	FEATURE
};

char *nrf_reg_name[] = {
	"CONFIG",
	"EN_AA",
	"EN_RXADDR",
	"SETUP_AW",
	"SETUP_RETR",
	"RF_CH",
	"RF_SETUP",
	"STATUS",
	"OBSERVE_TX",
	"CD",
	"FIFO_STATUS",
	"DYNPD",
	"FEATURE"
};

	ssize_t loop;
	ssize_t ret;

	for (loop = 0; loop < 13; loop++) {
		ret = spi_w8r8(spi, nrf_reg[loop]);
		if (ret < 0)
			return ret;

		dev_dbg(&spi->dev,
			"%s: %s = 0%02zx\n",
			__func__,
			nrf_reg_name[loop],
			ret);
	}

	return 0;
}

ssize_t nrf24_get_auto_ack(struct spi_device *spi, u8 pipe)
{
	ssize_t aa;

	aa = nrf24_read_reg(spi, EN_AA);
	if (aa < 0)
		return aa;

	return (aa & BIT(pipe)) == BIT(pipe);
}

ssize_t nrf24_is_rx_fifo_empty(struct spi_device *spi)
{
	ssize_t fifo;

	fifo = nrf24_read_reg(spi, FIFO_STATUS);
	if (fifo < 0)
		return fifo;

	return fifo & 0x01;
}

ssize_t nrf24_reuse_tx_pl(struct spi_device *spi)
{
	return nrf24_write_reg(spi, REUSE_TX_PL, 0);
}

