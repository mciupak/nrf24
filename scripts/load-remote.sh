#!/bin/bash

HOST=$1

scp nrf24.ko $HOST:/tmp
ssh $HOST rmmod nrf24
ssh $HOST dmesg -C
ssh $HOST insmod /tmp/nrf24.ko
ssh $HOST dmesg
