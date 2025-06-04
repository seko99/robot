# Robot Bringup Package

ROS2 пакет для запуска всех компонентов робота одной командой.

## Описание

Этот пакет объединяет запуск всех основных компонентов робота:
- Одометрия (`robot_odometry`)
- Телеуправление (`robot_teleop`) 
- Ультразвуковой датчик (`robot_sonar`)

## Установка

```bash
cd ~/ros2_ws/src
# Скопируйте все пакеты robot_* в src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Использование

### Запуск всех компонентов
```bash
# Запуск с настройками по умолчанию
ros2 launch robot_bringup robot.launch.py

# Запуск с кастомными портами
ros2 launch robot_bringup robot.launch.py \
  odometry_serial_port:=/dev/ttyUSB0 \
  teleop_serial_port:=/dev/ttyUSB0 \
  sonar_serial_port:=/dev/ttyUSB1

# Запуск без сонара
ros2 launch robot_bringup robot.launch.py use_sonar:=false
```

### Проверка работы
```bash
# Просмотр активных топиков
ros2 topic list

# Проверка одометрии
ros2 topic echo /odom

# Проверка сонара
ros2 topic echo /ultrasonic_sensor/range

# Отправка команд управления
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Параметры

- `odometry_serial_port`: Порт для Arduino с одометрией (по умолчанию: /dev/ttyUSB0)
- `teleop_serial_port`: Порт для Arduino с моторами (по умолчанию: /dev/ttyUSB0)
- `sonar_serial_port`: Порт для Arduino с сонаром (по умолчанию: /dev/ttyUSB1)
- `wheel_base`: Расстояние между колесами (по умолчанию: 0.3 м)
- `use_sonar`: Включать ли сонар (по умолчанию: true)

## Зависимости

Этот пакет требует установки следующих пакетов:
- `robot_odometry`
- `robot_teleop`
- `robot_sonar`
