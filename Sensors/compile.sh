#!/bin/bash

arduino-cli compile --fqbn arduino:avr:uno --output-dir /home/seko/Robot/Sensors /home/seko/Robot/Sensors
scp -i /home/seko/.ssh/id_rsa /home/seko/Robot/Sensors/Sensors.ino.hex orangepi@192.168.2.141:~
ssh -i /home/seko/.ssh/id_rsa orangepi@192.168.2.141 "~/upload_sensors.sh"

