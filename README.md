# nrf24

MY FORK OF https://github.com/mciupak/nrf24

* Fixed MAkefile for generic non static options, so can build on more machines unedited
* Made tx and rx address the same on nrf0 nrf0.0 nrf0.0 because my nrf24 network all is equal
* switched spinlock to mutex, gpiod_set_value now gpiod_set_value_cansleep, and threaded irq so it will work over sleepable busses like my ftdi mpsse usb to spi driver/ft2232h
* added check to write_wait_queue, so now we can control c out of it if hardware issue or no ack, before it was frozen
* added the spi device id table so tll enumerate on newer kernel versions
* Added sysfs options for set/get rf channel
* added working device tree example devicetree/rf24.dts
* added install-modules.sh script
* added nrf-kernel-fix this is helpful because the driver tends to (at least random amount of times) start with incorrect address and or channel number, this is fixed with setting the correct channel/address. That said even when it starts with correct settings it most likely wwont work without again setting the address and channel numbers. Unfortunatly it has been observed if another radio sends to it before you can set them it will keep it from functioning correctly. Last but not least usually norf0.0 can send but not receive (hrm maybe remembering backwards rx but no tx? anywho one or the other) but nrf0.1 works fine for both.

HELPFULL HINTS;
* if dmesg shows the msg/address size changing from 5 to 2 or 10 or only shows 2 or 10 and it isnt working the gpio is most likely not setup correctly (almost certainly ce pin in this scenerio) 
* After running the nrf-kernel-fix script (or custom variation) make sure to send before trying to receive, dunno why but it need that order, also make sue you have a radio to send the ack because without that ack on first transmit most likely wont work until reboot.
* If using a usb to spi driver make sure to disable sleeping on the usb ports/hubs/devices

Original README.md Follows;

This is a Linux Device Driver for nRF24L01+ radio module.

## Fatures
* Support  for multiple nRF24L01 instances via nrfX
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
* Pipes enabled: all
* Dynamic Payload: all pipes
* CRC: 16bit
* Data Rate: 2Mbps
* Channel: default
* RX addresses: default
* TX address: default
* Retransmission counter: 15
* Retransmission delay: 4000 us
