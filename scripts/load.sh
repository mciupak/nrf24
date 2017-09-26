#!/bin/bash

rmmod nrf24
dmesg -C
insmod nrf24.ko
dmesg

