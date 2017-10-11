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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/wait.h>
#include <linux/poll.h>

#include "nrf24_hal.h"

#define N_NRF24_MINORS                  BIT(MINORBITS)
#define N_NRF24_PIPES			6
#define FIFO_SIZE			65536

static dev_t nrf24_dev;
static DEFINE_IDR(nrf24_idr);
static DEFINE_IDA(nrf24_ida);
static DEFINE_MUTEX(minor_mutex);
static struct class *nrf24_class;

struct nrf24_pipe_cfg {
	u64			address;
	bool			ack;
};

struct nrf24_pipe {
	dev_t			devt;
	struct device		*dev;
	int			minor;
	int			pipe;
	struct nrf24_pipe_cfg	cfg;

	STRUCT_KFIFO_REC_1(FIFO_SIZE) rx_fifo;
	wait_queue_head_t	rx_wait_queue;
	bool			rx_active;
	ssize_t			rx_size;
};

struct nrf24_device {
	u32			id;
	struct device		dev;
	struct cdev		*cdev;
	struct spi_device	*spi;
	struct nrf24_pipe	*pipes[N_NRF24_PIPES];

	struct gpio_desc	*ce;
	spinlock_t		lock;

	struct work_struct	isr_work;

	//tx
	STRUCT_KFIFO_REC_2(FIFO_SIZE) tx_fifo;
	struct mutex		tx_fifo_mutex;
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_fifo_wait_queue;
	wait_queue_head_t	tx_wait_queue;

	struct task_struct	*rx_task_struct;
	wait_queue_head_t	rx_wait_queue;

	bool			tx_active;
	bool			tx_done;
	bool			tx_requested;
	bool			rx_active;
};

static bool nrf24_is_rx_active(struct nrf24_device *device)
{
	int i;

	device->rx_active = false;

	for (i = 0; i < N_NRF24_PIPES; i++)
		device->rx_active |= device->pipes[i]->rx_active;

	return device->rx_active;
}

static void nrf24_ce_hi(struct nrf24_device *device)
{
	gpiod_set_value(device->ce, 1);
}

static void nrf24_ce_lo(struct nrf24_device *device)
{
	gpiod_set_value(device->ce, 0);
}

#define to_nrf24_device(device)	container_of(device, struct nrf24_device, dev)

static int nrf24_find_pipe_ptr(struct device *dev)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int i;

	for (i = 0; i < N_NRF24_PIPES; i++)
		if (device->pipes[i]->dev == dev)
			return i;

	return -ENODEV;
}

static ssize_t ack_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	int pipe;


	pipe = nrf24_find_pipe_ptr(dev);
	if (pipe < 0)
		return pipe;

	ret = nrf24_get_auto_ack(device->spi, pipe);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t ack_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	int pipe;
	u8 new;


	pipe = nrf24_find_pipe_ptr(dev);
	if (pipe < 0)
		return pipe;

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, pipe, new);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t plw_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	int pipe;


	pipe = nrf24_find_pipe_ptr(dev);
	if (pipe < 0)
		return pipe;

	ret = nrf24_get_rx_pload_width(device->spi, pipe);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int pipe;
	int ret;
	u8 new;
	u8 old;

	pipe = nrf24_find_pipe_ptr(dev);
	if (pipe < 0)
		return pipe;
	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, pipe);
	if (old < 0)
		return old;

	if (old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, pipe, new);
		if (ret < 0)
			return ret;
	}

	return count;
}
static ssize_t address_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int pipe;
	int ret;
	int count;
	int i;

	pipe = nrf24_find_pipe_ptr(dev);
	if (pipe < 0)
		return pipe;

	ret = nrf24_get_address(device->spi, pipe, addr);
	if (ret < 0)
		return ret;

	count = snprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += snprintf(buf + count, PAGE_SIZE, "%02X", addr[i]);
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t address_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int pipe;
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	pipe = nrf24_find_pipe_ptr(dev);
	if (pipe < 0)
		return pipe;

	ret = nrf24_set_address(device->spi, pipe, (u8 *)&address);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(ack);
static DEVICE_ATTR_RW(plw);
static DEVICE_ATTR_RW(address);

static struct attribute *nrf24_pipe_attrs[] = {
	&dev_attr_ack.attr,
	&dev_attr_plw.attr,
	&dev_attr_address.attr,
	NULL,
};

ATTRIBUTE_GROUPS(nrf24_pipe);

static ssize_t tx_address_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev);
	u8 addr[16];
	int ret;
	int count;
	int i;


	ret = nrf24_get_address(device->spi, NRF24_TX, addr);
	if (ret < 0)
		return ret;

	count = snprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += snprintf(buf + count, PAGE_SIZE, "%02X", addr[i]);
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t tx_address_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev);
	int ret;
	u64 address;
	int len;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	ret = nrf24_set_address(device->spi, NRF24_TX, (u8 *)&address);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	nrf24_print_status(device->spi);
	ret = nrf24_get_status(device->spi);
	if (ret < 0)
		return ret;
	return snprintf(buf, PAGE_SIZE, "STATUS = 0x%02X\n", ret);
}

static ssize_t available_crc_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0 8 16\n");
}

static ssize_t crc_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_crc_mode(device->spi);
	if (ret < 0)
		return ret;

	switch (ret) {
	case NRF24_CRC_OFF:
		ret = snprintf(buf, PAGE_SIZE, "0\n");
		break;
	case NRF24_CRC_8BIT:
		ret = snprintf(buf, PAGE_SIZE, "8\n");
		break;
	case NRF24_CRC_16BIT:
		ret = snprintf(buf, PAGE_SIZE, "16\n");
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t crc_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	int ret;
	u8 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	switch (new) {
	case 0:
		new = NRF24_CRC_OFF;
		break;
	case 8:
		new = NRF24_CRC_8BIT;
		break;
	case 16:
		new = NRF24_CRC_16BIT;
		break;
	default:
		return -EINVAL;
	}

	ret = nrf24_get_crc_mode(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_crc_mode(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new crc mode = %d", __func__, new);
	}
	return count;
}

static ssize_t available_address_width_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	return snprintf(buf, PAGE_SIZE, "3 4 5\n");
}

static ssize_t address_width_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_address_width(device->spi);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t address_width_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	int ret;
	u8 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new != NRF24_AW_3 &&
	    new != NRF24_AW_4 &&
	    new != NRF24_AW_5)
		return -EINVAL;

	ret = nrf24_get_address_width(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_address_width(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new address width = %d", __func__, new);
	}
	return count;
}

static ssize_t available_output_power_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0 -6 -12 -18\n");
}

static ssize_t rf_power_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_rf_power(device->spi);
	if (ret < 0)
		return ret;

	switch (ret) {
	case NRF24_POWER_0DBM:
		ret = snprintf(buf, PAGE_SIZE, "0\n");
		break;
	case NRF24_POWER_6DBM:
		ret = snprintf(buf, PAGE_SIZE, "-6\n");
		break;
	case NRF24_POWER_12DBM:
		ret = snprintf(buf, PAGE_SIZE, "-12\n");
		break;
	case NRF24_POWER_18DBM:
		ret = snprintf(buf, PAGE_SIZE, "-18\n");
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t rf_power_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int ret;
	u8 new;
	s8 tmp;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtos8(buf, 10, &tmp);
	if (ret < 0)
		return ret;

	switch (abs(tmp)) {
	case 0:
		new = NRF24_POWER_0DBM;
		break;
	case 6:
		new = NRF24_POWER_6DBM;
		break;
	case 12:
		new = NRF24_POWER_12DBM;
		break;
	case 18:
		new = NRF24_POWER_18DBM;
		break;
	default:
		return -EINVAL;
	}

	ret = nrf24_get_rf_power(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_rf_power(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new rf power level = %d", __func__, new);
	}
	return count;
}

static ssize_t available_data_rate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "256 1024 2048\n");
}

static ssize_t data_rate_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_datarate(device->spi);
	if (ret < 0)
		return ret;

	switch (ret) {
	case NRF24_DATARATE_256KBPS:
		ret = snprintf(buf, PAGE_SIZE, "256\n");
		break;
	case NRF24_DATARATE_1MBPS:
		ret = snprintf(buf, PAGE_SIZE, "1024\n");
		break;
	case NRF24_DATARATE_2MBPS:
		ret = snprintf(buf, PAGE_SIZE, "2048\n");
		break;
	}

	return ret;
}

static ssize_t data_rate_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	int ret;
	u8 new;
	u16 tmp;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou16(buf, 10, &tmp);
	if (ret < 0)
		return ret;

	switch (tmp) {
	case 256:
		new = NRF24_DATARATE_256KBPS;
		break;
	case 1024:
		new = NRF24_DATARATE_256KBPS;
		break;
	case 2048:
		new = NRF24_DATARATE_256KBPS;
		break;
	default:
		return -EINVAL;
	}

	ret = nrf24_get_datarate(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_datarate(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new datarate = %d", __func__, new);
	}
	return count;
}

static ssize_t available_retr_delay_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i;
	int count = 0;

	for (i = 1; i <= 16; i++)
		count += snprintf(buf + count, PAGE_SIZE, "%d ", i * 250);
	buf[count - 1] = '\n';

	return count;
}

static ssize_t retr_delay_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_auto_retr_delay(device->spi);
	if (ret < 0)
		return ret;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t retr_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret;
	u16 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou16(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 250 || new > 4000 || new % 250)
		return -EINVAL;

	ret = nrf24_get_auto_retr_delay(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_auto_retr_delay(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new autr retr delay = %d", __func__, new);
	}
	return count;
}

static ssize_t available_retr_count_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i;
	int count = 0;

	for (i = 0; i < 16; i++)
		count += snprintf(buf + count, PAGE_SIZE, "%d ", i);
	buf[count - 1] = '\n';

	return count;
}

static ssize_t retr_count_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	struct nrf24_device *device = to_nrf24_device(dev);

	ret = nrf24_get_auto_retr_count(device->spi);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t retr_count_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret;
	u16 new;
	struct nrf24_device *device;

	device = to_nrf24_device(dev);

	ret = kstrtou16(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > 15)
		return -EINVAL;

	ret = nrf24_get_auto_retr_count(device->spi);
	if (ret < 0)
		return ret;

	if (new != ret) {
		ret = nrf24_set_auto_retr_count(device->spi, new);
		if (ret < 0)
			return ret;
		dev_dbg(dev, "%s: new autr retr count = %d", __func__, new);
	}
	return count;
}

static DEVICE_ATTR_RW(tx_address);
static DEVICE_ATTR_RO(status);
static DEVICE_ATTR_RO(available_crc);
static DEVICE_ATTR_RW(crc);
static DEVICE_ATTR_RO(available_address_width);
static DEVICE_ATTR_RW(address_width);
static DEVICE_ATTR_RO(available_output_power);
static DEVICE_ATTR_RW(rf_power);
static DEVICE_ATTR_RO(available_data_rate);
static DEVICE_ATTR_RW(data_rate);
static DEVICE_ATTR_RO(available_retr_delay);
static DEVICE_ATTR_RW(retr_delay);
static DEVICE_ATTR_RO(available_retr_count);
static DEVICE_ATTR_RW(retr_count);

static struct attribute *nrf24_attrs[] = {
	&dev_attr_tx_address.attr,
	&dev_attr_status.attr,
	&dev_attr_crc.attr,
	&dev_attr_available_crc.attr,
	&dev_attr_address_width.attr,
	&dev_attr_available_address_width.attr,
	&dev_attr_rf_power.attr,
	&dev_attr_available_output_power.attr,
	&dev_attr_data_rate.attr,
	&dev_attr_available_data_rate.attr,
	&dev_attr_retr_delay.attr,
	&dev_attr_available_retr_delay.attr,
	&dev_attr_retr_count.attr,
	&dev_attr_available_retr_count.attr,
	NULL,
};

ATTRIBUTE_GROUPS(nrf24);

static ssize_t nrf24_tx_thread(void *data)
{
	struct nrf24_device *device = data;
	struct nrf24_pipe *p;
	int len;
	u8 pload[PLOAD_MAX];
	int ret;
	ssize_t plw;
	ssize_t dpl;
	ssize_t size;
	ssize_t n;
	ssize_t sent = 0;
	u8 *buf;

	while (true) {
		dev_dbg(&device->dev,
			"%s: waiting for new messages",
			__func__);
		wait_event_interruptible(device->tx_fifo_wait_queue,
					 (!kfifo_is_empty(&device->tx_fifo) ||
					  kthread_should_stop()));
		if (kthread_should_stop())
			return 0;

		//set active flag
		device->tx_active = false;

		//clear flag
		device->tx_done = false;

		//lock fifo
		//this is needed as write to tx fifo may be done by 6 pipes
		mutex_lock(&device->tx_fifo_mutex);

		//take address of pipe which is sending
		ret = kfifo_out(&device->tx_fifo, &p, sizeof(p));
		if (ret != sizeof(p)) {
			dev_warn(&device->dev, "get pipe from fifo failed");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		//take out size of data
		ret = kfifo_out(&device->tx_fifo, &size, sizeof(size));
		if (ret != sizeof(size)) {
			dev_warn(&device->dev, "get size from fifo failed");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		//alloc space for data
		buf = kzalloc(size, GFP_KERNEL);
		if (!buf) {
			//unlock tx fifo
			mutex_unlock(&device->tx_fifo_mutex);
			dev_warn(&device->dev, "buf alloc failed");
			continue;
		}

		//take out size of data
		ret = kfifo_out(&device->tx_fifo, buf, size);
		if (ret != size) {
			dev_warn(&device->dev, "get buf from fifo failed");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		//unlock tx fifo
		mutex_unlock(&device->tx_fifo_mutex);

		//if rx is active set tx_requested flag
		device->tx_requested = nrf24_is_rx_active(device);
		if (device->tx_requested)
			dev_dbg(p->dev, "RX is active, waiting to finish...");
		else
			dev_dbg(p->dev, "RX NOT active, sending");


		wait_event_interruptible(device->tx_fifo_wait_queue,
					 (!nrf24_is_rx_active(device) ||
					  kthread_should_stop()));
		if (kthread_should_stop())
			return 0;

		if (device->tx_requested)
			dev_dbg(p->dev, "RX done, sending...");

		device->tx_requested = nrf24_is_rx_active(device);

		//enter Standby-I mode
		nrf24_ce_lo(device);

		//set active flag
		device->tx_active = true;

		//check pipe's payload width
		plw = nrf24_get_rx_pload_width(device->spi, p->pipe);
		if (plw < 0) {
			dev_warn(&device->dev,
				 "get rx pload width failed (%d)",
				 plw);
			continue;
		}
		//check if pipe uses dynamic payload
		if (plw) {
			len = plw;
			//check if dynamic payload is enabled
			dpl = nrf24_get_dynamic_pl(device->spi);
			if (dpl < 0) {
				dev_warn(&device->dev,
					 "get dpl failed (%d)",
					 dpl);
				continue;
			}
		} else {
			len = PLOAD_MAX;
			dpl = 0;
		}

		//disable dynamic payload
		if (dpl) {
			ret = nrf24_disable_dynamic_pl(device->spi);
			if (ret < 0) {
				dev_warn(&device->dev,
					 "disable dpl failed (%d)",
					 ret);
				continue;
			}
		}

		//set TX MODE
		ret = nrf24_set_mode(device->spi, NRF24_MODE_TX);
		if (ret < 0)
			continue;

		//set PIPE0 address
		//this is needed to receive ACK
		ret = nrf24_set_address(device->spi,
					NRF24_PIPE0,
					(u8 *)&p->cfg.address);
		if (ret < 0) {
			dev_warn(&device->dev,
				 "set PIPE0 address failed (%d)",
				 ret);
			continue;
		}

		//set TX address
		ret = nrf24_set_address(device->spi,
					NRF24_TX,
					(u8 *)&p->cfg.address);
		if (ret < 0) {
			dev_warn(&device->dev,
				 "set TX address failed (%d)",
				 ret);
			continue;
		}

		memset(pload, 0, PLOAD_MAX);
		memcpy(pload, &size, sizeof(size));

		//send size
		nrf24_write_tx_pload(device->spi, pload, len);
		if (ret < 0) {
			dev_warn(&device->dev,
				 "set TX PLOAD  failed (%d)",
				 ret);
			continue;
		}

		//enter TX MODE and start transmission
		nrf24_ce_hi(device);

		//wait for ACK
		wait_event_interruptible(device->tx_wait_queue,
					 (device->tx_done ||
					 kthread_should_stop()));

		if (kthread_should_stop())
			return 0;

		//clear counter
		sent = 0;

		while (size > sent) {

			n = min_t(ssize_t, size - sent, len);
			memset(pload, 0, PLOAD_MAX);
			memcpy(pload, buf + sent, n);

			sent += n;

			//write PLOAD to nRF FIFO
			ret = nrf24_write_tx_pload(device->spi,
					     pload,
					     plw ? len : n);
			if (ret < 0)
				break;

			device->tx_done = false;

			//wait for ACK
			wait_event_interruptible(device->tx_wait_queue,
						 (device->tx_done ||
						 kthread_should_stop()));

			if (kthread_should_stop())
				return 0;
		}

		//free data buffer
		kfree(buf);

		//restore dynamic payload feature
		if (dpl)
			ret = nrf24_enable_dynamic_pl(device->spi);

		//if all send enter RX MODE
		if (kfifo_is_empty(&device->tx_fifo)) {

			dev_dbg(&device->dev, "%s: NRF24_MODE_RX", __func__);
			device->tx_active = false;

			device->tx_done = false;

			//enter Standby-I
			nrf24_ce_lo(device);

			//restore PIPE0 address
			nrf24_set_address(device->spi,
					  NRF24_PIPE0,
					  (u8 *)&device->pipes[0]->cfg.address);
			//set RX MODE
			nrf24_set_mode(device->spi, NRF24_MODE_RX);

			//enter TX MODE and start receiving
			nrf24_ce_hi(device);
		}

	}
}

static ssize_t nrf24_rx_thread(void *data)
{
	struct nrf24_device *device = data;
	ssize_t pipe;
	ssize_t length;
	u8 pload[PLOAD_MAX];
	int ret;
	ssize_t n;
	ssize_t plw;
	struct nrf24_pipe *p;

	while (true) {

		wait_event_interruptible(device->rx_wait_queue,
					 (!nrf24_is_rx_fifo_empty(device->spi) ||
					  kthread_should_stop()));
		if (kthread_should_stop())
			return 0;

		pipe = nrf24_get_rx_data_source(device->spi);
		if (pipe < 0) {
			dev_warn(&device->dev,
				 "%s: no datasource (err: %d)",
				 __func__,
				 pipe);
			continue;
		}

		if (pipe > N_NRF24_PIPES) {
			dev_warn(&device->dev, "%s: pipe is TX!", __func__);
			continue;
		}

		p = device->pipes[pipe];

		length = nrf24_get_rx_pl_w(device->spi);
		if (length < 0) {
			dev_warn(&device->dev,
				 "%s: get pload length failed (err: %d)",
				 __func__,
				 length);
			continue;
		}

		//check pipe's payload width
		plw = nrf24_get_rx_pload_width(device->spi, p->pipe);
		if (plw < 0) {
			dev_warn(&device->dev,
				 "get rx pload width failed (%d)",
				 plw);
			continue;
		}

		memset(pload, 0, PLOAD_MAX);
		ret = nrf24_read_rx_pload(device->spi, pload);
		if (ret < 0) {
			dev_warn(&device->dev,
				 "%s: could not read pload (err = %d)",
				 __func__,
				 ret);
			continue;
		}

		if (!p->rx_active) {
			p->rx_active = true;
			memcpy(&p->rx_size, pload, sizeof(p->rx_size));
			dev_dbg(p->dev, "rx active");
		} else {
			if (p->rx_size < plw)
				length = p->rx_size;
			n = kfifo_in(&p->rx_fifo, &pload, length);
			p->rx_size -= n;
			if (p->rx_size <= 0) {
				dev_dbg(p->dev, "rx done");
				p->rx_active = false;
				wake_up_interruptible(&p->rx_wait_queue);
			}
		}

		if (!nrf24_is_rx_active(device) && device->tx_requested) {
			dev_dbg(&device->dev, "wake up tx...");
			wake_up_interruptible(&device->tx_fifo_wait_queue);
		}
	}
}

static void nrf24_isr_work_handler(struct work_struct *work)
{
	struct nrf24_device *device;
	ssize_t status;

	device = container_of(work, struct nrf24_device, isr_work);

	status = nrf24_get_status(device->spi);
	if (status < 0)
		return;

	if (status & RX_DR) {
		dev_dbg(&device->dev, "%s: RX_DR", __func__);
		nrf24_clear_irq(device->spi, RX_DR);
		wake_up_interruptible(&device->rx_wait_queue);
	}

	if (status & TX_DS) {
		dev_dbg(&device->dev, "%s: TX_DS", __func__);
		nrf24_clear_irq(device->spi, TX_DS);
		device->tx_done = true;
		wake_up_interruptible(&device->tx_wait_queue);

	}

	if (status & MAX_RT) {
		nrf24_ce_lo(device);
		dev_dbg(&device->dev,
			"%s: MAX_RT, tx req: %d, rx act: %d",
			__func__,
			device->tx_requested,
			nrf24_is_rx_active(device));
		nrf24_clear_irq(device->spi, MAX_RT);
		nrf24_reuse_tx_pl(device->spi);
		nrf24_ce_hi(device);
	}
}

static irqreturn_t nrf24_isr(int irq, void *dev_id)
{
	unsigned long flags;
	struct nrf24_device *device = dev_id;

	spin_lock_irqsave(&device->lock, flags);

	schedule_work(&device->isr_work);

	spin_unlock_irqrestore(&device->lock, flags);

	return IRQ_HANDLED;
}

static ssize_t nrf24_read(struct file *filp,
			  char __user *buf,
			  size_t size,
			  loff_t *f_pos)
{
	struct nrf24_pipe *p;
	ssize_t copied;
	ssize_t n;

	p = filp->private_data;

	if (kfifo_is_empty(&p->rx_fifo) && (filp->f_flags & O_NONBLOCK))
		return -EAGAIN;

	n = kfifo_to_user(&p->rx_fifo, buf, size, &copied);
	if (n)
		return n;
	return copied;
}

static ssize_t nrf24_write(struct file *filp,
			   const char __user *buf,
			   size_t size,
			   loff_t *f_pos)
{
	struct nrf24_device *device;
	struct nrf24_pipe *p;
	ssize_t n;
	ssize_t copied;

	p = filp->private_data;
	device = to_nrf24_device(p->dev->parent);

	dev_dbg(p->dev, "write (%d)", size);

	mutex_lock(&device->tx_fifo_mutex);

	//put pipe pointer in fifo
	n = kfifo_in(&device->tx_fifo, &p, sizeof(p));
	if (n != sizeof(p))
		goto error;

	//put size in fifo
	n = kfifo_in(&device->tx_fifo, &size, sizeof(size));
	if (n != sizeof(size))
		goto error;

	//put data to be sent into fifo
	n = kfifo_from_user(&device->tx_fifo,
			    buf,
			    size,
			    &copied);
	if (n || size != copied)
		goto error;

	mutex_unlock(&device->tx_fifo_mutex);

	wake_up_interruptible(&device->tx_fifo_wait_queue);

	return copied;
error:
	kfifo_reset(&device->tx_fifo);
	mutex_unlock(&device->tx_fifo_mutex);
	return -EAGAIN;
}

static int nrf24_open(struct inode *inode, struct file *filp)
{
	struct nrf24_pipee *pipe;

	mutex_lock(&minor_mutex);
	pipe = idr_find(&nrf24_idr, iminor(inode));
	mutex_unlock(&minor_mutex);

	if (!pipe) {
		pr_err("device: minor %d unknown.\n", iminor(inode));
		return -ENODEV;
	}

	filp->private_data = pipe;
	nonseekable_open(inode, filp);

	return 0;
}

static int nrf24_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;

	return 0;
}

static unsigned int nrf24_poll(struct file *filp,
			       struct poll_table_struct *wait)
{
	struct nrf24_device *device;
	struct nrf24_pipe *p;

	p = filp->private_data;
	device = to_nrf24_device(p->dev->parent);

	dev_dbg(p->dev, "%s: waiting...", __func__);
	poll_wait(filp, &p->rx_wait_queue, wait);
	if (!kfifo_is_empty(&p->rx_fifo)) {
		dev_dbg(p->dev, "%s: got data!", __func__);
		return POLLIN | POLLRDNORM;
	}
	dev_dbg(p->dev, "%s: no data!", __func__);
	return 0;
}

static int nrf24_get_minor(struct nrf24_pipe *pipe)
{
	int ret;

	mutex_lock(&minor_mutex);
	ret = idr_alloc(&nrf24_idr, pipe, 0, N_NRF24_MINORS, GFP_KERNEL);
	mutex_unlock(&minor_mutex);

	return ret;
}

static void nrf24_free_minor(int minor)
{
	mutex_lock(&minor_mutex);
	idr_remove(&nrf24_idr, minor);
	mutex_unlock(&minor_mutex);
}

static void nrf24_destroy_devices(struct nrf24_device *device)
{
	int i;

	for (i = 0; i < N_NRF24_PIPES; i++) {
		device_destroy(nrf24_class, device->pipes[i]->devt);
		nrf24_free_minor(device->pipes[i]->minor);
		kfree(device->pipes[i]);
	}
}

static int nrf24_create_device(struct nrf24_device *device, int i)
{
	int ret;
	char name[16];

	//sets flags to false as well
	device->pipes[i] = kzalloc(sizeof(*device->pipes[i]), GFP_KERNEL);
	if (!device->pipes[i]) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	ret = nrf24_get_minor(device->pipes[i]);
	if (ret < 0) {
		dev_err(&device->dev, "%s: get_minor failed", __func__);
		goto error_minor;
	}

	device->pipes[i]->minor = ret;
	device->pipes[i]->pipe = i;
	INIT_KFIFO(device->pipes[i]->rx_fifo);
	init_waitqueue_head(&device->pipes[i]->rx_wait_queue);

	snprintf(name, sizeof(name), "%s.%d", dev_name(&device->dev), i);
	device->pipes[i]->devt = MKDEV(MAJOR(nrf24_dev),
				       device->pipes[i]->minor);

	device->pipes[i]->dev = device_create_with_groups(
					nrf24_class,
					&device->dev,
					device->pipes[i]->devt,
					device->pipes[i],
					nrf24_pipe_groups,
					name);

	if (IS_ERR(device->pipes[i]->dev)) {
		dev_err(&device->dev,
			"%s: device_create of '%s' failed",
			__func__,
			name);
		ret = PTR_ERR(device->pipes[i]->dev);
		goto error_device;
	}

	dev_dbg(&device->dev,
		"%s: device created: major(%d), minor(%d)",
		__func__,
		MAJOR(nrf24_dev),
		device->pipes[i]->minor
		);

	return 0;
error_device:
	nrf24_free_minor(device->pipes[i]->minor);
error_minor:
	kfree(device->pipes[i]);
error_alloc:
	return ret;
}

static void nrf24_gpio_free(struct nrf24_device *device)
{
	if (!IS_ERR(device->ce))
		gpiod_put(device->ce);

	free_irq(device->spi->irq, device);
}

static int nrf24_gpio_setup(struct nrf24_device *device)
{
	int ret;

	device->ce = gpiod_get(&device->spi->dev, "ce", 0);

	if (device->ce == ERR_PTR(-ENOENT))
		dev_dbg(&device->dev, "%s: no entry for CE", __func__);
	else if (device->ce == ERR_PTR(-EBUSY))
		dev_dbg(&device->dev, "%s: CE is busy", __func__);

	if (IS_ERR(device->ce)) {
		ret = PTR_ERR(device->ce);
		dev_err(&device->dev, "%s: CE gpio setup error", __func__);
		return ret;
	}

	nrf24_ce_lo(device);

	//irq
	ret = request_irq(device->spi->irq,
			  nrf24_isr,
			  0,
			  dev_name(&device->dev),
			  device);
	if (ret < 0) {
		free_irq(device->spi->irq, device);
		goto err;
	}

	return 0;

err:
	gpiod_put(device->ce);
	return ret;
}

static void nrf24_dev_release(struct device *dev)
{
	struct nrf24_device *device = to_nrf24_device(dev);

	ida_simple_remove(&nrf24_ida, device->id);
	kfree(device);
}

static struct device_type nrf24_dev_type = {
	.name = "nrf24_device",
	.release = nrf24_dev_release,
};

static struct nrf24_device *nrf24_dev_init(struct spi_device *spi)
{
	int ret;
	struct nrf24_device *device;
	int id;

	id = ida_simple_get(&nrf24_ida, 0, 0, GFP_KERNEL);
	if (id < 0)
		return ERR_PTR(id);

	//sets flags to false as well
	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		ida_simple_remove(&nrf24_ida, id);
		return ERR_PTR(-ENOMEM);
	}
	device->spi = spi;

	dev_set_name(&device->dev, "nrf%d", id);
	device->id = id;
	device->dev.parent = &spi->dev;
	device->dev.class = nrf24_class;
	device->dev.type = &nrf24_dev_type;
	device->dev.groups = nrf24_groups;
	ret = device_register(&device->dev);
	if (ret < 0) {
		put_device(&device->dev);
		return ERR_PTR(ret);
	}

	init_waitqueue_head(&device->tx_fifo_wait_queue);
	init_waitqueue_head(&device->tx_wait_queue);
	init_waitqueue_head(&device->rx_wait_queue);

	INIT_WORK(&device->isr_work, nrf24_isr_work_handler);
	INIT_KFIFO(device->tx_fifo);
	spin_lock_init(&device->lock);
	mutex_init(&device->tx_fifo_mutex);

	return device;
}

static int nrf24_hal_init(struct nrf24_device *device)
{
	int ret;
	int i;
	struct spi_device *spi = device->spi;


	ret = nrf24_soft_reset(spi);
	if (ret < 0)
		return ret;

	for (i = 0; i < N_NRF24_PIPES; i++) {
		ret = nrf24_get_address(spi,
					i,
					(u8 *)&device->pipes[i]->cfg.address);
		if (ret < 0)
			return ret;
		ret = nrf24_get_auto_ack(spi, i);
		if (ret < 0)
			return ret;
		device->pipes[i]->cfg.ack = ret;
	}

	ret = nrf24_flush_fifo(spi);
	if (ret < 0)
		return ret;
	//ret = nrf24_close_pipe(spi, NRF24_PIPE_ALL);
	//if (ret < 0)
	//	return ret;
	ret = nrf24_open_pipe(spi, NRF24_PIPE_ALL);
	if (ret < 0)
		return ret;
	ret = nrf24_lock_unlock(spi);
	if (ret < 0)
		return ret;
	ret = nrf24_set_mode(spi, NRF24_MODE_RX);
	if (ret < 0)
		return ret;
	//0 -> dynamic pload
	ret = nrf24_set_rx_pload_width(spi, NRF24_PIPE0, 0);
	if (ret < 0)
		return ret;
	//0 -> dynamic pload
	ret = nrf24_set_rx_pload_width(spi, NRF24_PIPE1, 0);
	if (ret < 0)
		return ret;
	//0 -> dynamic pload
	ret = nrf24_set_rx_pload_width(spi, NRF24_PIPE2, 0);
	if (ret < 0)
		return ret;
	//0 -> dynamic pload
	ret = nrf24_set_rx_pload_width(spi, NRF24_PIPE3, 0);
	if (ret < 0)
		return ret;
	//0 -> dynamic pload
	ret = nrf24_set_rx_pload_width(spi, NRF24_PIPE4, 0);
	if (ret < 0)
		return ret;
	//0 -> dynamic pload
	ret = nrf24_set_rx_pload_width(spi, NRF24_PIPE5, 0);
	if (ret < 0)
		return ret;
	ret = nrf24_set_crc_mode(spi, NRF24_CRC_16BIT);
	if (ret < 0)
		return ret;
	ret = nrf24_set_auto_retr_count(spi, 15);
	if (ret < 0)
		return ret;
	ret = nrf24_set_auto_retr_delay(spi, 4000);
	if (ret < 0)
		return ret;
	ret = nrf24_set_rf_power(spi, NRF24_POWER_0DBM);
	if (ret < 0)
		return ret;
	ret = nrf24_set_datarate(spi, NRF24_DATARATE_2MBPS);
	if (ret < 0)
		return ret;
	ret = nrf24_power_up(spi);
	if (ret < 0)
		return ret;

	nrf24_ce_hi(device);

	return ret;

}

static const struct file_operations nrf24_fops = {
	.owner = THIS_MODULE,
	.open = nrf24_open,
	.release = nrf24_release,
	.read = nrf24_read,
	.write = nrf24_write,
	.llseek = no_llseek,
	.poll = nrf24_poll,
};

static int nrf24_probe(struct spi_device *spi)
{
	int ret;
	struct nrf24_device *device;
	int i;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: spi_setup failed", __func__);
		return ret;
	}

	device = nrf24_dev_init(spi);
	if (IS_ERR(device)) {
		dev_err(&device->spi->dev, "%s: dev_init failed", __func__);
		return PTR_ERR(device);
	}

	ret = nrf24_gpio_setup(device);
	if (ret < 0) {
		dev_err(&device->dev, "%s: gpio_setup failed", __func__);
		goto gpio_setup_err;
	}

	for (i = 0; i < N_NRF24_PIPES; i++) {
		ret = nrf24_create_device(device, i);
		if (ret < 0)
			goto device_err;
	}

	ret = nrf24_hal_init(device);
	if (ret < 0)
		goto hal_init_err;

	/* start rx thread */
	device->rx_task_struct = kthread_run(nrf24_rx_thread,
					     device,
					     "nrf%d_rx_thread", device->id);
	if (IS_ERR(device->rx_task_struct)) {
		dev_err(&device->dev, "start of tx thread failed");
		goto rx_thread_err;
	}

	/* start tx thread */
	device->tx_task_struct = kthread_run(nrf24_tx_thread,
					     device,
					     "nrf%d_tx_thread", device->id);
	if (IS_ERR(device->tx_task_struct)) {
		dev_err(&device->dev, "start of tx thread failed");
		goto tx_thread_err;
	}

	device->cdev = cdev_alloc();
	device->cdev->owner = THIS_MODULE;
	cdev_init(device->cdev, &nrf24_fops);
	ret = cdev_add(device->cdev, nrf24_dev, N_NRF24_MINORS);
	if (ret < 0) {
		dev_err(&device->dev, "%s: cdev failed", __func__);
		goto cdev_err;
	}

	spi_set_drvdata(spi, device);

	return 0;

cdev_err:
	kthread_stop(device->tx_task_struct);
tx_thread_err:
	kthread_stop(device->rx_task_struct);
rx_thread_err:
hal_init_err:
device_err:
	for (i--; i >= 0; --i) {
		device_destroy(nrf24_class, device->pipes[i]->devt);
		nrf24_free_minor(device->pipes[i]->minor);
		kfree(device->pipes[i]);
	}
	nrf24_gpio_free(device);
gpio_setup_err:
	device_unregister(&device->dev);
	return ret;
}

static int nrf24_remove(struct spi_device *spi)
{
	struct nrf24_device *device = spi_get_drvdata(spi);

	cdev_del(device->cdev);

	nrf24_gpio_free(device);

	kthread_stop(device->tx_task_struct);
	kthread_stop(device->rx_task_struct);

	nrf24_destroy_devices(device);

	device_unregister(&device->dev);

	return 0;
}


static const struct of_device_id nrf24_dt_ids[] = {
	{ .compatible = "nordic,nrf24" },
	{},
};

MODULE_DEVICE_TABLE(of, nrf24_dt_ids);

static  struct spi_driver nrf24_spi_driver = {
	.driver = {
		.name = "nrf24",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nrf24_dt_ids),
	},
	.probe = nrf24_probe,
	.remove = nrf24_remove,
};


static int __init nrf24_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&nrf24_dev, 0, N_NRF24_MINORS,
				  nrf24_spi_driver.driver.name);
	if (ret < 0) {
		pr_err("Unable to alloc chrdev region");
		return ret;
	}

	nrf24_class = class_create(THIS_MODULE, nrf24_spi_driver.driver.name);
	if (IS_ERR(nrf24_class)) {
		pr_err("Unable to create class");
		unregister_chrdev(MAJOR(nrf24_dev),
				  nrf24_spi_driver.driver.name);
		return PTR_ERR(nrf24_class);
	}

	ret = spi_register_driver(&nrf24_spi_driver);
	if (ret < 0) {
		pr_err("spi_register_driver error");
		class_destroy(nrf24_class);
		unregister_chrdev(MAJOR(nrf24_dev),
				  nrf24_spi_driver.driver.name);
		ida_destroy(&nrf24_ida);
	}
	return ret;
}
module_init(nrf24_init);

static void __exit nrf24_exit(void)
{
	spi_unregister_driver(&nrf24_spi_driver);
	class_destroy(nrf24_class);
	unregister_chrdev(MAJOR(nrf24_dev), nrf24_spi_driver.driver.name);
	ida_destroy(&nrf24_ida);
}
module_exit(nrf24_exit);

MODULE_AUTHOR("Marcin Ciupak <marcin.s.ciupak@gmail.com>");
MODULE_DESCRIPTION("Driver for NRF24L01+");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:nrf24");

