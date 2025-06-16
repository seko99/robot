#!/bin/bash

echo "Компилируем скетч Motors..."
arduino-cli compile --fqbn arduino:avr:uno --output-dir /home/seko/VCProjects/robot/Motors /home/seko/VCProjects/robot/Motors

if [ $? -ne 0 ]; then
    echo "Ошибка компиляции!"
    exit 1
fi

echo "Передаем бинарник на робота..."
scp -i /home/seko/.ssh/id_rsa /home/seko/VCProjects/robot/Motors/Motors.ino.hex orangepi@192.168.2.141:~

if [ $? -ne 0 ]; then
    echo "Ошибка передачи файла!"
    exit 1
fi

echo "Загружаем в контроллер через arduino-cli..."
ssh -i /home/seko/.ssh/id_rsa orangepi@192.168.2.141 "/usr/local/bin/arduino-cli upload -v -p /dev/ttyUSB0 --fqbn arduino:avr:uno --input-file ./Motors.ino.hex"

if [ $? -ne 0 ]; then
    echo "Ошибка загрузки в контроллер!"
    exit 1
fi

echo "Успешно загружено!"