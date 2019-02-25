// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2017 Marcin Ciupak <marcin.s.ciupak@gmail.com>
 *
 */

#include <linux/types.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/kfifo.h>
#include <linux/list.h>

#include "nrf24_if.h"
#include "nrf24_hal.h"
#include "nrf24_enums.h"

static struct nrf24_pipe *nrf24_find_pipe_ptr(struct device *dev)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	struct nrf24_pipe *pipe;

	list_for_each_entry(pipe, &device->pipes, list)
		if (pipe->dev == dev)
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

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
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

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t plw_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	struct nrf24_device *device = to_nrf24_device(dev->parent);
	int ret;
	u8 new;
	ssize_t old;
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

	if ((u8)old != new) {
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

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

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

struct attribute *nrf24_pipe_attrs[] = {
	&dev_attr_ack.attr,
	&dev_attr_plw.attr,
	&dev_attr_address.attr,
	NULL,
};

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

	count = scnprintf(buf, PAGE_SIZE, "0x");
	for (i = --ret; i >= 0; i--)
		count += scnprintf(buf + count, PAGE_SIZE - count, "%02X", addr[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

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
	return scnprintf(buf, PAGE_SIZE, "STATUS = 0x%02X\n", ret);
}

static ssize_t available_crc_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0 8 16\n");
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
		ret = scnprintf(buf, PAGE_SIZE, "0\n");
		break;
	case NRF24_CRC_8BIT:
		ret = scnprintf(buf, PAGE_SIZE, "8\n");
		break;
	case NRF24_CRC_16BIT:
		ret = scnprintf(buf, PAGE_SIZE, "16\n");
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
		dev_dbg(dev, "%s: new crc mode = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_address_width_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "3 4 5\n");
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

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
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
		dev_dbg(dev, "%s: new address width = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_output_power_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "0 -6 -12 -18\n");
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
		ret = scnprintf(buf, PAGE_SIZE, "0\n");
		break;
	case NRF24_POWER_6DBM:
		ret = scnprintf(buf, PAGE_SIZE, "-6\n");
		break;
	case NRF24_POWER_12DBM:
		ret = scnprintf(buf, PAGE_SIZE, "-12\n");
		break;
	case NRF24_POWER_18DBM:
		ret = scnprintf(buf, PAGE_SIZE, "-18\n");
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
		dev_dbg(dev, "%s: new rf power level = %d\n", __func__, new);
	}
	return count;
}

static ssize_t available_data_rate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "256 1024 2048\n");
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
		ret = scnprintf(buf, PAGE_SIZE, "256\n");
		break;
	case NRF24_DATARATE_1MBPS:
		ret = scnprintf(buf, PAGE_SIZE, "1024\n");
		break;
	case NRF24_DATARATE_2MBPS:
		ret = scnprintf(buf, PAGE_SIZE, "2048\n");
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
		new = NRF24_DATARATE_1MBPS;
		break;
	case 2048:
		new = NRF24_DATARATE_2MBPS;
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
		dev_dbg(dev, "%s: new datarate = %d\n", __func__, new);
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
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ", i * 250);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

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
		dev_dbg(dev, "%s: new autr retr delay = %d\n", __func__, new);
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
		count += scnprintf(buf + count, PAGE_SIZE - count, "%d ", i);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

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

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
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
		dev_dbg(dev, "%s: new autr retr count = %d\n", __func__, new);
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

struct attribute *nrf24_attrs[] = {
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

