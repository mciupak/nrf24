#!/bin/bash
sudo modprobe -r nrf24
sudo mkdir -p /lib/modules/$(uname -r)/updates/dkms
sudo rm /lib/modules/$(uname -r)/updates/dkms/nrf24.ko
sudo cp nrf24.ko /lib/modules/$(uname -r)/updates/dkms/nrf24.ko
sudo depmod
sudo modprobe nrf24
echo 70 | sudo tee -a /sys/module/nrf24/drivers/spi\:nrf24/spi0.0/nrf24/nrf0/nrf0.0/chan
