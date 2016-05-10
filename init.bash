#!/bin/bash

set -e
# enable SPIDEV
echo ADAFRUIT-SPI0 > /sys/devices/bone_capemgr.9/slots
