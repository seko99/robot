#!/bin/bash

arduino-cli upload -v -p /dev/ttyUSB0 --fqbn arduino:avr:uno --input-file ./Motors.ino.hex
