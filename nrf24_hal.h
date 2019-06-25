/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
 *
 */

#ifndef NRF24_HAL_H
#define NRF24_HAL_H

#include "nRF24L01.h"
#include "nrf24_enums.h"

ssize_t nrf24_open_pipe(struct spi_device *spi, enum nrf24_pipe_num pipe);
ssize_t nrf24_close_pipe(struct spi_device *spi, enum nrf24_pipe_num pipe);
ssize_t nrf24_set_address(struct spi_device *spi, enum nrf24_pipe_num pipe, u8 *addr);
ssize_t nrf24_get_address(struct spi_device *spi, enum nrf24_pipe_num pipe, u8 *addr);
ssize_t nrf24_set_crc_mode(struct spi_device *spi, enum nrf24_crc_mode mode);
ssize_t nrf24_get_crc_mode(struct spi_device *spi);
ssize_t nrf24_set_auto_retr_count(struct spi_device *spi, u8 count);
ssize_t nrf24_get_auto_retr_count(struct spi_device *spi);
ssize_t nrf24_set_auto_retr_delay(struct spi_device *spi, u16 delay);
ssize_t nrf24_get_auto_retr_delay(struct spi_device *spi);
ssize_t nrf24_set_address_width(struct spi_device *spi, enum nrf24_address_width aw);
ssize_t nrf24_get_address_width(struct spi_device *spi);
ssize_t nrf24_lock_unlock(struct spi_device *spi);
ssize_t nrf24_set_datarate(struct spi_device *spi, enum nrf24_datarate datarate);
ssize_t nrf24_get_datarate(struct spi_device *spi);
ssize_t nrf24_set_mode(struct spi_device *spi, enum nrf24_mode mode);
ssize_t nrf24_set_rf_power(struct spi_device *spi, enum nrf24_rf_power rf_pwr);
ssize_t nrf24_get_rf_power(struct spi_device *spi);
ssize_t nrf24_set_rx_pload_width(struct spi_device *spi, u8 pipe_no, u8 plw);
ssize_t nrf24_set_rf_channel(struct spi_device *spi, u8 channel);
ssize_t nrf24_write_tx_pload(struct spi_device *dev, u8 *buf, u8 length);
ssize_t nrf24_write_tx_pload_noack(struct spi_device *dev, u8 *buf, u8 length);
ssize_t nrf24_read_rx_pload(struct spi_device *spi, u8 *buf);
ssize_t nrf24_power_up(struct spi_device *spi);
ssize_t nrf24_get_status(struct spi_device *spi);
ssize_t nrf24_get_rx_data_source(struct spi_device *spi);
ssize_t nrf24_get_rx_pload_width(struct spi_device *spi, u8 pipe);
ssize_t nrf24_soft_reset(struct spi_device *spi);
ssize_t nrf24_clear_irq(struct spi_device *spi, u8 irq);
ssize_t nrf24_flush_fifo(struct spi_device *spi);
ssize_t nrf24_flush_rx_fifo(struct spi_device *spi);
ssize_t nrf24_flush_tx_fifo(struct spi_device *spi);
ssize_t nrf24_get_rx_pl_w(struct spi_device *spi);
ssize_t nrf24_print_status(struct spi_device *spi);
ssize_t nrf24_get_auto_ack(struct spi_device *spi, u8 pipe);
ssize_t nrf24_setup_auto_ack(struct spi_device *spi, u8 pipe, bool enable);
ssize_t nrf24_is_rx_fifo_empty(struct spi_device *spi);
ssize_t nrf24_reuse_tx_pl(struct spi_device *spi);
ssize_t nrf24_get_dynamic_pl(struct spi_device *spi);
ssize_t nrf24_enable_dynamic_pl(struct spi_device *spi);
ssize_t nrf24_disable_dynamic_pl(struct spi_device *spi);

#endif
