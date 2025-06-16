#!/bin/bash

arduino-cli upload -v -p /dev/ttyUSB1 --fqbn arduino:avr:uno --input-file ./Sensors.ino.hex
