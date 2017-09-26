# nrf24

This is a Linux Device Driver for nRF24L01+ radio module.

## Fatures
* Suport for multiple nRF24L01 instances via nrfX
* Separate /dev/nrfX.Y device per pipe per instance
* Dynamic and Static Payload Lengths
* Configuration via sysfs
* poll mechanism
* 64kB RX FIFO per pipe
* 64kB TX FIFO

## TODO
As described in TODO file.

## Environment
Cross compiled 4.13.0-rc7-v7+
Tested on Raspberry Pi 2 model B and Raspberry Pi 3 model B with Raspbian GNU/Linux 9 (stretch) running linux kernel 4.13.0-rc7-v7+


## DeviceTree
In order to use this driver disable spiX overlay and enable nrf24-spiX-overlay.

## Default Configuration
* Pipes enalbed: all
* Dynamic Payload: all pipes
* CRC: 16bit
* Data Rate: 2Mbps
* Channel: default
* RX addresses: default
* TX address: dedault
* Retransmission counter: 15
* Retransmission delay: 4000 us
