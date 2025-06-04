# Robot Odometry Package

ROS2 пакет для публикации одометрии робота на основе данных с Arduino.

## Описание

Этот пакет считывает данные одометрии с Arduino через последовательный порт и публикует информацию о положении робота в ROS2.

## Установка

```bash
cd ~/ros2_ws/src
# Скопируйте пакет robot_odometry в src
cd ~/ros2_ws
colcon build --packages-select robot_odometry
source install/setup.bash
```

## Использование

```bash
# Запуск ноды одометрии
ros2 run robot_odometry odometry_node

# Запуск с launch файлом
ros2 launch robot_odometry odometry.launch.py

# Просмотр топиков одометрии
ros2 topic echo /odom
```

## Параметры

- `serial_port`: Порт Arduino (по умолчанию: /dev/ttyUSB0)
- `baud_rate`: Скорость передачи данных (по умолчанию: 115200)
- `wheel_base`: Расстояние между колесами в метрах (по умолчанию: 0.3)

## Топики

### Публикуемые
- `/odom` (nav_msgs/Odometry): Информация об одометрии робота

### TF Frames
- `odom` -> `base_link`: Трансформация положения робота
