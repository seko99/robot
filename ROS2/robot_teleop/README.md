# Robot Teleop Package

ROS2 пакет для телеуправления роботом через Arduino.

## Описание

Этот пакет принимает команды управления через топик `/cmd_vel` и отправляет соответствующие команды на Arduino для управления моторами робота.

## Установка

```bash
cd ~/ros2_ws/src
# Скопируйте пакет robot_teleop в src
cd ~/ros2_ws
colcon build --packages-select robot_teleop
source install/setup.bash
```

## Использование

```bash
# Запуск ноды телеуправления
ros2 run robot_teleop teleop_node

# Запуск с launch файлом
ros2 launch robot_teleop teleop.launch.py

# Отправка команд управления
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Использование с клавиатурой (требует установки teleop_twist_keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Параметры

- `serial_port`: Порт Arduino (по умолчанию: /dev/ttyUSB0)
- `baud_rate`: Скорость передачи данных (по умолчанию: 115200)
- `max_linear`: Максимальная линейная скорость в м/с (по умолчанию: 0.5)
- `max_angular`: Максимальная угловая скорость в рад/с (по умолчанию: 1.57)
- `wheel_base`: Расстояние между колесами в метрах (по умолчанию: 0.3)
- `max_motor_speed`: Максимальное значение PWM для моторов (по умолчанию: 255)

## Топики

### Подписывается на
- `/cmd_vel` (geometry_msgs/Twist): Команды управления скоростью

## Команды моторов

Пакет отправляет бинарные команды на Arduino:
- `CMD_LEFT_FORWARD` (0): Левый мотор вперед
- `CMD_LEFT_BACKWARD` (1): Левый мотор назад
- `CMD_RIGHT_FORWARD` (2): Правый мотор вперед
- `CMD_RIGHT_BACKWARD` (3): Правый мотор назад
- `CMD_LEFT_STOP` (4): Остановка левого мотора
- `CMD_RIGHT_STOP` (5): Остановка правого мотора
- `CMD_STOP_MOTORS` (6): Остановка всех моторов
