#!/bin/bash

arduino-cli compile --fqbn arduino:avr:uno --output-dir /home/seko/Robot /home/seko/Robot
scp -i /home/seko/.ssh/id_rsa /home/seko/Robot/Robot.ino.hex orangepi@192.168.2.141:~
scp -i /home/seko/.ssh/id_rsa /home/seko/Robot/*.py orangepi@192.168.2.141:~
ssh -i /home/seko/.ssh/id_rsa orangepi@192.168.2.141 "~/upload.sh"