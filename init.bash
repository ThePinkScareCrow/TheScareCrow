#!/bin/bash

set -e
# enable SPIDEV
echo BB-SPIDEV0 > /sys/devices/bone_capemgr.9/slots
