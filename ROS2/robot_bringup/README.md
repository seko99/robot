# Robot Bringup Package

ROS2 пакет для запуска всех компонентов робота одной командой.

## Описание

Этот пакет объединяет запуск всех основных компонентов робота:

- Одометрия с интегрированным телеуправлением (`robot_odometry`)
- Ультразвуковой датчик (`robot_sonar`)
- Лидар LDS01RR (`robot_lidar`)

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
  sonar_serial_port:=/dev/ttyUSB1 \
  lidar_serial_port:=/dev/ttyS5

# Запуск без сонара и лидара
ros2 launch robot_bringup robot.launch.py use_sonar:=false use_lidar:=false

# Запуск с настроенными скоростями
ros2 launch robot_bringup robot.launch.py \
  max_linear:=0.8 \
  max_angular:=2.0

# Запуск в режиме симуляции (без подключения к Arduino)
ros2 launch robot_bringup robot.launch.py simulation_mode:=true
```

### Проверка работы

```bash
# Просмотр активных топиков
ros2 topic list

# Проверка одометрии
ros2 topic echo /odom

# Проверка сонара
ros2 topic echo /ultrasonic_sensor/range

# Проверка лидара
ros2 topic echo /scan

# Отправка команд управления
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Параметры

### Серийные порты

- `odometry_serial_port`: Порт для Arduino с одометрией и моторами (по умолчанию: /dev/ttyUSB0)
- `sonar_serial_port`: Порт для Arduino с сонаром (по умолчанию: /dev/ttyUSB1)
- `lidar_serial_port`: Порт для лидара LDS01RR (по умолчанию: /dev/ttyS5)

### Физические параметры робота

- `wheel_base`: Расстояние между колесами в метрах (по умолчанию: 0.3)

### Параметры управления

- `max_linear`: Максимальная линейная скорость в м/с (по умолчанию: 0.5)
- `max_angular`: Максимальная угловая скорость в рад/с (по умолчанию: 1.57)

### Флаги включения компонентов

- `use_sonar`: Включать ли сонар (по умолчанию: true)
- `use_lidar`: Включать ли лидар (по умолчанию: true)
- `simulation_mode`: Запуск в режиме симуляции без подключения к Arduino (по умолчанию: false)

## Топики

### Публикуемые топики

- `/odom` (nav_msgs/Odometry) - данные одометрии
- `/ultrasonic_sensor/range` (sensor_msgs/Range) - данные ультразвукового датчика
- `/scan` (sensor_msgs/LaserScan) - данные лидара

### Подписываемые топики

- `/cmd_vel` (geometry_msgs/Twist) - команды управления скоростью (обрабатываются нодой odometry)

## Зависимости

Этот пакет требует установки следующих пакетов:

- `robot_odometry` (с интегрированным телеуправлением)
- `robot_sonar`
- `robot_lidar`

## Изменения в архитектуре

В новой версии логика телеуправления интегрирована в пакет `robot_odometry` для избежания конфликтов доступа к одному и тому же серийному порту. Теперь одна нода обрабатывает и команды движения, и данные одометрии.
