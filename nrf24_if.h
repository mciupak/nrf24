/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
 *
 */

#ifndef NRF24_IF_H
#define NRF24_IF_H

#define FIFO_SIZE			65536

struct nrf24_pipe_cfg {
	u64			address;
	u8			ack;
	ssize_t			plw;
};

struct nrf24_pipe {
	dev_t			devt;
	struct device		*dev;
	struct cdev		cdev;
	int			id;
	struct nrf24_pipe_cfg	cfg;

	DECLARE_KFIFO(rx_fifo, u8, FIFO_SIZE);
	wait_queue_head_t	read_wait_queue;
	u8			rx_active;

	struct list_head	list;

	struct timer_list	rx_active_timer;
};

struct nrf24_device_cfg {
	u16			data_rate;
	u8			crc;
	u8			retr_count;
	u16			retr_delay;
	u8			rf_power;
	u8			address_width;
};

struct nrf24_device {
	u32			id;
	struct device		dev;
	struct spi_device	*spi;
	struct list_head	pipes;

	struct gpio_desc	*ce;

	struct nrf24_device_cfg	cfg;

	/* for irqsave */
	spinlock_t		lock;

	struct work_struct	isr_work;

	/* tx */
	STRUCT_KFIFO_REC_2(FIFO_SIZE) tx_fifo;
	struct mutex		tx_fifo_mutex;
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_wait_queue;
	wait_queue_head_t	tx_done_wait_queue;
	wait_queue_head_t	write_wait_queue;
	u8			tx_done;
	u8			write_done;
	struct timer_list	tx_active_timer;

	/* rx */
	struct task_struct	*rx_task_struct;
	wait_queue_head_t	rx_wait_queue;

};

#define to_nrf24_device(device)	container_of(device, struct nrf24_device, dev)

#endif /* NRF24_IF_H */
