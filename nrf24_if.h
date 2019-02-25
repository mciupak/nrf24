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
	ssize_t			rx_size;

	struct list_head list;
};

struct nrf24_device {
	u32			id;
	struct device		dev;
	struct spi_device	*spi;
	struct list_head	pipes;

	struct gpio_desc	*ce;

	/* for irqsave */
	spinlock_t		lock;

	struct work_struct	isr_work;

	/* tx */
	STRUCT_KFIFO_REC_2(FIFO_SIZE) tx_fifo;

	/* tx fifo lock */
	struct mutex		tx_fifo_mutex;
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_wait_queue;
	wait_queue_head_t	tx_done_wait_queue;

	struct task_struct	*rx_task_struct;
	wait_queue_head_t	rx_wait_queue;

	u8			tx_done;
};

#define to_nrf24_device(device)	container_of(device, struct nrf24_device, dev)

#endif /* NRF24_IF_H */
