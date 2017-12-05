/*
 * nRF24L01 device driver.
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
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
#include <linux/list.h>

#include "nrf24_hal.h"

#define N_NRF24_MINORS                  BIT(MINORBITS)
#define FIFO_SIZE			65536

static dev_t nrf24_dev;
static DEFINE_IDA(nrf24_ida_pipe);
static DEFINE_IDA(nrf24_ida_dev);
static struct class *nrf24_class;

struct nrf24_pipe_cfg {
	u64			address;
	bool			ack;
	ssize_t			plw;
};

struct nrf24_pipe {
	dev_t			devt;
	struct device		*dev;
	struct cdev		cdev;
	int			id;
	struct nrf24_pipe_cfg	cfg;

	STRUCT_KFIFO_REC_1(FIFO_SIZE) rx_fifo;
	wait_queue_head_t	poll_wait_queue;
	ssize_t			rx_size;

	struct list_head list;
};

struct nrf24_device {
	u32			id;
	struct device		dev;
	struct spi_device	*spi;
	struct list_head	pipes;

	struct gpio_desc	*ce;
	spinlock_t		lock;

	struct work_struct	isr_work;

	//tx
	STRUCT_KFIFO_REC_2(FIFO_SIZE) tx_fifo;
	struct mutex		tx_fifo_mutex;
	struct task_struct	*tx_task_struct;
	wait_queue_head_t	tx_wait_queue;
	wait_queue_head_t	tx_done_wait_queue;

	struct task_struct	*rx_task_struct;
	wait_queue_head_t	rx_wait_queue;

	bool			tx_done;
};

static bool nrf24_is_rx_active(struct nrf24_device *device)
{
	struct nrf24_pipe *pipe;

	bool active = false;

	list_for_each_entry(pipe, &device->pipes, list)
		active |= pipe->rx_size > 0;

	return active;
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

static struct nrf24_pipe *nrf24_find_pipe_ptr(struct device *dev)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	struct nrf24_pipe *pipe;

	list_for_each_entry(pipe, &device->pipes, list)
		if (pipe->dev == dev)
			return pipe;

	return ERR_PTR(-ENODEV);
}

static struct nrf24_pipe *nrf24_find_pipe_id(struct nrf24_device *device, int id)
{
	struct nrf24_pipe *pipe;

	list_for_each_entry(pipe, &device->pipes, list)
		if (pipe->id == id)
			return pipe;

	return ERR_PTR(-ENODEV);
}

static ssize_t ack_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	struct nrf24_pipe *pipe;


	pipe = nrf24_find_pipe_ptr(dev);
	if (IS_ERR(pipe))
		return PTR_ERR(pipe);

	ret = nrf24_get_auto_ack(device->spi, pipe->id);
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
	u8 new;
	struct nrf24_pipe *pipe;

	pipe = nrf24_find_pipe_ptr(dev);
	if (IS_ERR(pipe))
		return PTR_ERR(pipe);

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;
	if (new < 0 || new > 1)
		return -EINVAL;

	ret = nrf24_setup_auto_ack(device->spi, pipe->id, new);
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
	struct nrf24_pipe *pipe;

	pipe = nrf24_find_pipe_ptr(dev);
	if (IS_ERR(pipe))
		return PTR_ERR(pipe);

	ret = nrf24_get_rx_pload_width(device->spi, pipe->id);
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
	int ret;
	u8 new;
	u8 old;
	struct nrf24_pipe *pipe;

	pipe = nrf24_find_pipe_ptr(dev);
	if (IS_ERR(pipe))
		return PTR_ERR(pipe);

	ret = kstrtou8(buf, 10, &new);
	if (ret < 0)
		return ret;

	if (new < 0 || new > PLOAD_MAX)
		return -EINVAL;
	old = nrf24_get_rx_pload_width(device->spi, pipe->id);
	if (old < 0)
		return old;

	if (old != new) {
		ret = nrf24_set_rx_pload_width(device->spi, pipe->id, new);
		if (ret < 0)
			return ret;
		pipe->cfg.plw = new;
	}

	return count;
}
static ssize_t address_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	u8 addr[16];
	int ret;
	int count;
	int i;
	struct nrf24_pipe *pipe;

	pipe = nrf24_find_pipe_ptr(dev);
	if (IS_ERR(pipe))
		return PTR_ERR(pipe);

	ret = nrf24_get_address(device->spi, pipe->id, addr);
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
	int ret;
	u64 address;
	int len;
	struct nrf24_pipe *pipe;

	ret = kstrtoull(buf, 16, &address);
	if (ret < 0)
		return ret;

	len = nrf24_get_address_width(device->spi);
	if (len < 0)
		return len;

	if (address >= BIT_ULL(len * BITS_PER_BYTE))
		return -EINVAL;

	pipe = nrf24_find_pipe_ptr(dev);
	if (IS_ERR(pipe))
		return PTR_ERR(pipe);

	ret = nrf24_set_address(device->spi, pipe->id, (u8 *)&address);
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
	u8 pload[PLOAD_MAX];
	int ret;
	ssize_t size;
	ssize_t n;
	ssize_t sent = 0;
	u8 *buf;
	bool spl;
	bool dpl;

	while (true) {
		dev_dbg(&device->dev,
			"%s: waiting for new messages",
			__func__);
		wait_event_interruptible(device->tx_wait_queue,
					 kthread_should_stop() ||
					 (!nrf24_is_rx_active(device) && !kfifo_is_empty(&device->tx_fifo)));

		if (kthread_should_stop())
			return 0;

		//clear flag
		device->tx_done = false;

		//lock fifo
		//this is needed as write to tx fifo may be done by 6 pipes
		mutex_lock(&device->tx_fifo_mutex);

		//take address of pipe which is sending
		ret = kfifo_out(&device->tx_fifo, &p, sizeof(p));
		if (ret != sizeof(p)) {
			dev_dbg(&device->dev, "get pipe from fifo failed");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		//take out size of data
		ret = kfifo_out(&device->tx_fifo, &size, sizeof(size));
		if (ret != sizeof(size)) {
			dev_dbg(&device->dev, "get size from fifo failed");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		//alloc space for data
		buf = kzalloc(size, GFP_KERNEL);
		if (!buf) {
			dev_dbg(&device->dev, "buf alloc failed");
			mutex_unlock(&device->tx_fifo_mutex);
			continue;
		}

		//take out size of data
		ret = kfifo_out(&device->tx_fifo, buf, size);
		if (ret != size) {
			dev_dbg(&device->dev, "get buf from fifo failed");
			mutex_unlock(&device->tx_fifo_mutex);
			goto next;
		}

		//unlock tx fifo
		mutex_unlock(&device->tx_fifo_mutex);

		//enter Standby-I mode
		nrf24_ce_lo(device);

		//set TX MODE
		ret = nrf24_set_mode(device->spi, NRF24_MODE_TX);
		if (ret < 0)
			goto next;

		//set PIPE0 address
		//this is needed to receive ACK
		ret = nrf24_set_address(device->spi,
					NRF24_PIPE0,
					(u8 *)&p->cfg.address);
		if (ret < 0) {
			dev_dbg(&device->dev, "set PIPE0 address failed (%d)", ret);
			goto next;
		}

		//set TX address
		ret = nrf24_set_address(device->spi,
					NRF24_TX,
					(u8 *)&p->cfg.address);
		if (ret < 0) {
			dev_dbg(&device->dev, "set TX address failed (%d)", ret);
			goto next;
		}

		//check if pipe uses static payload length
		spl = p->cfg.plw != 0;

		//check if dynamic payload length is enabled
		dpl = nrf24_get_dynamic_pl(device->spi);

		if (spl && dpl) {
			//disable dynamic payload if pipe
			//does not use dynamic payload
			//and dynamic paload is enabled
			ret = nrf24_disable_dynamic_pl(device->spi);
			if (ret < 0)
				goto next;
		}

		memset(pload, 0, PLOAD_MAX);
		memcpy(pload, &size, sizeof(size));

		//calculate payload length
		n = spl ? p->cfg.plw : sizeof(size);

		//send size
		nrf24_write_tx_pload(device->spi, pload, n);
		if (ret < 0) {
			dev_dbg(&device->dev, "write TX PLOAD failed (%d)", ret);
			goto next;
		}

		//enter TX MODE and start transmission
		nrf24_ce_hi(device);

		//wait for ACK
		wait_event_interruptible(device->tx_done_wait_queue,
					 (device->tx_done ||
					 kthread_should_stop()));

		if (kthread_should_stop())
			goto abort;

		//clear counter
		sent = 0;

		while (size > 0) {

			n = spl ? p->cfg.plw : min_t(ssize_t, size, PLOAD_MAX);

			dev_dbg(&device->dev, "tx %d bytes", n);

			memset(pload, 0, PLOAD_MAX);
			memcpy(pload, buf + sent, n);

			//write PLOAD to nRF FIFO
			ret = nrf24_write_tx_pload(device->spi, pload, n);

			if (ret < 0) {
				dev_dbg(&device->dev,
					 "write TX PLOAD failed (%d)",
					ret);
				goto next;
			}

			sent += n;
			size -= n;

			device->tx_done = false;

			//wait for ACK
			wait_event_interruptible(device->tx_done_wait_queue,
						 (device->tx_done ||
						 kthread_should_stop()));

			if (kthread_should_stop())
				goto abort;
		}
next:
		//free data buffer
		kfree(buf);

		//restore dynamic payload feature
		if (dpl)
			nrf24_enable_dynamic_pl(device->spi);

		//if all sent enter RX MODE
		if (kfifo_is_empty(&device->tx_fifo)) {

			dev_dbg(&device->dev, "%s: NRF24_MODE_RX", __func__);

			//enter Standby-I
			nrf24_ce_lo(device);

			p = nrf24_find_pipe_id(device, NRF24_PIPE0);
			if (!IS_ERR(p)) {
				//restore PIPE0 address
				nrf24_set_address(device->spi,
						  p->id,
						  (u8 *)&p->cfg.address);
			}
			//set RX MODE
			nrf24_set_mode(device->spi, NRF24_MODE_RX);

			//enter RX MODE and start receiving
			nrf24_ce_hi(device);
		}
	}
abort:
	kfree(buf);

	return 0;
}

static ssize_t nrf24_rx_thread(void *data)
{
	struct nrf24_device *device = data;
	ssize_t pipe;
	ssize_t length;
	u8 pload[PLOAD_MAX];
	struct nrf24_pipe *p;

	while (true) {

		wait_event_interruptible(device->rx_wait_queue,
					 (!nrf24_is_rx_fifo_empty(device->spi) ||
					  kthread_should_stop()));
		if (kthread_should_stop())
			return 0;

		pipe = nrf24_get_rx_data_source(device->spi);
		if (pipe < 0) {
			dev_dbg(&device->dev,
				"%s: get pipe failed (err: %d)",
				__func__,
				pipe);
			continue;
		}

		if (pipe > NRF24_PIPE5) {
			dev_dbg(&device->dev, "%s: RX FIFO is empty!", __func__);
			continue;
		}

		p = nrf24_find_pipe_id(device, pipe);
		if (IS_ERR(p))
			continue;

		memset(pload, 0, PLOAD_MAX);
		length = nrf24_read_rx_pload(device->spi, pload);
		if (length < 0) {
			dev_dbg(&device->dev,
				"%s: could not read pload (err = %d)",
				__func__,
				length);
			continue;
		}

		dev_dbg(p->dev, "rx %d bytes", length);
		if (p->rx_size > 0) {
			memcpy(&p->rx_size, pload, sizeof(p->rx_size));
			dev_dbg(p->dev, "RX active");
		} else {
			//get length of remaining
			length = p->rx_size < p->cfg.plw ? p->rx_size : length;

			p->rx_size -= kfifo_in(&p->rx_fifo, &pload, length);

			if (p->rx_size <= 0) {
				dev_dbg(p->dev, "RX done");
				wake_up_interruptible(&p->poll_wait_queue);
			}
		}

		//start tx if all rx done and tx requested during rctive rx
		if (!nrf24_is_rx_active(device) && !kfifo_is_empty(&device->tx_fifo)) {
			dev_dbg(&device->dev, "wake up TX...");
			wake_up_interruptible(&device->tx_wait_queue);
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
		wake_up_interruptible(&device->tx_done_wait_queue);

	}

	if (status & MAX_RT) {
		nrf24_ce_lo(device);
		dev_dbg_ratelimited(&device->dev, "%s: MAX_RT", __func__);
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

	wake_up_interruptible(&device->tx_wait_queue);

	return copied;
error:
	kfifo_reset(&device->tx_fifo);
	mutex_unlock(&device->tx_fifo_mutex);
	return -EAGAIN;
}

static int nrf24_open(struct inode *inode, struct file *filp)
{
	struct nrf24_pipe *pipe;

	pipe = container_of(inode->i_cdev, struct nrf24_pipe, cdev);

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
	poll_wait(filp, &p->poll_wait_queue, wait);
	if (!kfifo_is_empty(&p->rx_fifo)) {
		dev_dbg(p->dev, "%s: got data!", __func__);
		return POLLIN | POLLRDNORM;
	}
	dev_dbg(p->dev, "%s: no data!", __func__);
	return 0;
}

static void nrf24_destroy_devices(struct nrf24_device *device)
{
	struct nrf24_pipe *pipe, *temp;

	list_for_each_entry_safe(pipe, temp, &device->pipes, list) {
		cdev_del(&pipe->cdev);
		device_destroy(nrf24_class, pipe->devt);
		ida_simple_remove(&nrf24_ida_pipe, MINOR(pipe->devt));
		list_del(&pipe->list);
		kfree(pipe);
	}
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

static struct nrf24_pipe *nrf24_create_pipe(struct nrf24_device *device, int id)
{
	int ret;
	struct nrf24_pipe *p;

	//sets flags to false as well
	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	ret = ida_simple_get(&nrf24_ida_pipe, 0, 0, GFP_KERNEL);
	if (ret < 0) {
		dev_err(&device->dev, "%s: get_minor failed", __func__);
		goto error_minor;
	}

	p->devt = MKDEV(MAJOR(nrf24_dev), ret);
	p->id = id;

	INIT_KFIFO(p->rx_fifo);
	init_waitqueue_head(&p->poll_wait_queue);


	p->dev = device_create_with_groups(nrf24_class,
					   &device->dev,
					   p->devt,
					   p,
					   nrf24_pipe_groups,
					   "%s.%d",
					   dev_name(&device->dev),
					   id);

	if (IS_ERR(p->dev)) {
		dev_err(&device->dev,
			"%s: device_create of '%s' failed",
			__func__,
			dev_name(p->dev));
		ret = PTR_ERR(p->dev);
		goto error_device;
	}

	cdev_init(&p->cdev, &nrf24_fops);
	p->cdev.owner = THIS_MODULE;
	ret = cdev_add(&p->cdev, p->devt, 1);
	if (ret < 0) {
		dev_err(&device->dev, "%s: cdev failed", __func__);
		goto cdev_err;
	}

	dev_dbg(&device->dev,
		"%s: device created: major(%d), minor(%d)",
		__func__,
		MAJOR(p->devt),
		MINOR(p->devt));

	return p;

cdev_err:
	device_destroy(nrf24_class, p->devt);
error_device:
	ida_simple_remove(&nrf24_ida_pipe, MINOR(p->devt));
error_minor:
	kfree(p);
error_alloc:
	return ERR_PTR(ret);
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

	ida_simple_remove(&nrf24_ida_dev, device->id);
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

	id = ida_simple_get(&nrf24_ida_dev, 0, 0, GFP_KERNEL);
	if (id < 0)
		return ERR_PTR(id);

	//sets flags to false as well
	device = kzalloc(sizeof(*device), GFP_KERNEL);
	if (!device) {
		ida_simple_remove(&nrf24_ida_dev, id);
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

	init_waitqueue_head(&device->tx_wait_queue);
	init_waitqueue_head(&device->tx_done_wait_queue);
	init_waitqueue_head(&device->rx_wait_queue);

	INIT_WORK(&device->isr_work, nrf24_isr_work_handler);
	INIT_KFIFO(device->tx_fifo);
	spin_lock_init(&device->lock);
	mutex_init(&device->tx_fifo_mutex);

	INIT_LIST_HEAD(&device->pipes);

	return device;
}

static int nrf24_hal_init(struct nrf24_device *device)
{
	int ret;
	struct spi_device *spi = device->spi;
	struct nrf24_pipe *pipe;

	ret = nrf24_soft_reset(spi);
	if (ret < 0)
		return ret;


	list_for_each_entry(pipe, &device->pipes, list) {
		ret = nrf24_get_address(spi,
					pipe->id,
					(u8 *)&pipe->cfg.address);
		if (ret < 0)
			return ret;
		ret = nrf24_get_auto_ack(spi, pipe->id);
		if (ret < 0)
			return ret;
		pipe->cfg.ack = ret;

		//0 -> dynamic pload
		pipe->cfg.plw = 0;
		ret = nrf24_set_rx_pload_width(spi, pipe->id, 0);
		if (ret < 0)
			return ret;
	}

	ret = nrf24_flush_fifo(spi);
	if (ret < 0)
		return ret;
	ret = nrf24_open_pipe(spi, NRF24_PIPE_ALL);
	if (ret < 0)
		return ret;
	ret = nrf24_lock_unlock(spi);
	if (ret < 0)
		return ret;
	ret = nrf24_set_mode(spi, NRF24_MODE_RX);
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

static int nrf24_probe(struct spi_device *spi)
{
	int ret;
	struct nrf24_device *device;
	struct nrf24_pipe *pipe;
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


	for (i = 0; i <= NRF24_PIPE5; i++) {
		pipe = nrf24_create_pipe(device, i);
		if (IS_ERR(pipe)) {
			ret = PTR_ERR(pipe);
			goto device_err;
		}
		list_add(&pipe->list, &device->pipes);
	}

	ret = nrf24_hal_init(device);
	if (ret < 0)
		goto hal_init_err;

	/* start rx thread */
	device->rx_task_struct = kthread_run(nrf24_rx_thread,
					     device,
					     "nrf%d_rx_thread",
					     device->id);
	if (IS_ERR(device->rx_task_struct)) {
		dev_err(&device->dev, "start of tx thread failed");
		goto rx_thread_err;
	}

	/* start tx thread */
	device->tx_task_struct = kthread_run(nrf24_tx_thread,
					     device,
					     "nrf%d_tx_thread",
					     device->id);
	if (IS_ERR(device->tx_task_struct)) {
		dev_err(&device->dev, "start of tx thread failed");
		goto tx_thread_err;
	}


	spi_set_drvdata(spi, device);

	return 0;

tx_thread_err:
	kthread_stop(device->rx_task_struct);
rx_thread_err:
hal_init_err:
device_err:
	nrf24_destroy_devices(device);
	nrf24_gpio_free(device);
gpio_setup_err:
	device_unregister(&device->dev);
	return ret;
}

static int nrf24_remove(struct spi_device *spi)
{
	struct nrf24_device *device = spi_get_drvdata(spi);


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
		goto chrdev_err;
	}

	nrf24_class = class_create(THIS_MODULE, nrf24_spi_driver.driver.name);
	if (IS_ERR(nrf24_class)) {
		pr_err("Unable to create class");
		ret = PTR_ERR(nrf24_class);
		goto class_err;
	}

	ret = spi_register_driver(&nrf24_spi_driver);
	if (ret < 0) {
		pr_err("Unable to register spi driver");
		goto spi_err;
	}

	return 0;

spi_err:
	class_destroy(nrf24_class);
class_err:
	unregister_chrdev(MAJOR(nrf24_dev), nrf24_spi_driver.driver.name);
chrdev_err:
	ida_destroy(&nrf24_ida_dev);
	ida_destroy(&nrf24_ida_pipe);

	return ret;
}
module_init(nrf24_init);

static void __exit nrf24_exit(void)
{
	spi_unregister_driver(&nrf24_spi_driver);
	class_destroy(nrf24_class);
	unregister_chrdev(MAJOR(nrf24_dev), nrf24_spi_driver.driver.name);
	ida_destroy(&nrf24_ida_dev);
	ida_destroy(&nrf24_ida_pipe);
}
module_exit(nrf24_exit);

MODULE_AUTHOR("Marcin Ciupak <marcin.s.ciupak@gmail.com>");
MODULE_DESCRIPTION("Driver for NRF24L01+");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:nrf24");

