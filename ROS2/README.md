# ROS2 Robot Control Nodes

Этот проект содержит как отдельные ROS2 ноды, так и полноценные ROS2 пакеты для управления роботом с дифференциальным приводом.

## 🆕 ROS2 Пакеты (рекомендуется)

Новые готовые к установке ROS2 пакеты:

### Быстрая установка и запуск
```bash
# Установка всех пакетов
./install_packages.sh

# Запуск всех компонентов
./run_robot.sh full

# Управление с клавиатуры
./run_robot.sh keyboard
```

### Пакеты
- **robot_odometry** - Пакет одометрии с launch файлами и конфигурацией
- **robot_teleop** - Пакет телеуправления с launch файлами и конфигурацией  
- **robot_sonar** - Пакет сонара с launch файлами и конфигурацией
- **robot_bringup** - Пакет для запуска всех компонентов одной командой

---

## Отдельные ноды (legacy)

Этот пакет содержит ROS2 ноды для управления роботом с дифференциальным приводом.

### Компоненты

- **teleop.py** - Нода для телеуправления роботом через топик `/cmd_vel`
- **odom.py** - Нода для публикации одометрии в топик `/odom`
- **sonar_reader.py** - Нода для чтения данных с ультразвукового датчика

### Способы запуска

#### 1. Быстрый запуск (рекомендуется)

```bash
# Запуск с портами по умолчанию
./start_robot.sh

# Запуск с указанием портов
./start_robot.sh -m /dev/ttyACM0 -s /dev/ttyACM1

# Справка
./start_robot.sh --help
```

#### 2. Простой bash скрипт

```bash
./nodes_start.sh
```

#### 3. ROS2 Launch файлы

```bash
# Простой launch
ros2 launch simple_launch.py

# Полный launch с параметрами
ros2 launch robot_launch.py serial_port:=/dev/ttyUSB0 sonar_port:=/dev/ttyUSB1
```

#### 4. Ручной запуск отдельных нод

```bash
# В отдельных терминалах
python3 teleop.py
python3 odom.py  
python3 sonar_reader.py
```

### Параметры

#### teleop.py
- `serial_port` - Порт для подключения к Arduino моторов (по умолчанию: `/dev/ttyUSB0`)
- `baud_rate` - Скорость передачи данных (по умолчанию: `115200`)
- `max_linear` - Максимальная линейная скорость м/с (по умолчанию: `0.5`)
- `max_angular` - Максимальная угловая скорость рад/с (по умолчанию: `1.57`)
- `wheel_base` - Расстояние между колесами в метрах (по умолчанию: `0.3`)
- `max_motor_speed` - Максимальная скорость мотора PWM (по умолчанию: `255`)

#### odom.py
- `serial_port` - Порт для подключения к Arduino моторов (по умолчанию: `/dev/ttyUSB0`)
- `baud_rate` - Скорость передачи данных (по умолчанию: `115200`)
- `wheel_base` - Расстояние между колесами в метрах (по умолчанию: `0.3`)

#### sonar_reader.py
- `serial_port` - Порт для подключения к Arduino датчиков (по умолчанию: `/dev/ttyUSB1`)

### Топики ROS2

#### Подписка
- `/cmd_vel` (geometry_msgs/Twist) - Команды управления скоростью

#### Публикация
- `/odom` (nav_msgs/Odometry) - Данные одометрии
- `/ultrasonic_sensor/range` (sensor_msgs/Range) - Данные ультразвукового датчика

#### TF фреймы
- `odom` -> `base_link` - Трансформация одометрии

### Зависимости

#### Python пакеты
```bash
pip3 install pyserial
```

#### ROS2 пакеты
- rclpy
- geometry_msgs
- nav_msgs
- sensor_msgs
- tf2_ros

## Аппаратное обеспечение

- Arduino с загруженной прошивкой для моторов (Motors.ino)
- Arduino с загруженной прошивкой для датчиков (Sensors.ino)
- Энкодеры на моторах
- Ультразвуковой датчик

## Устранение неполадок

### Проблемы с портами
1. Проверьте подключение устройств: `ls /dev/tty*`
2. Проверьте права доступа: `sudo usermod -a -G dialout $USER`
3. Перезагрузитесь после добавления в группу

### Проблемы с ROS2
1. Убедитесь что ROS2 установлен: `ros2 --version`
2. Источник environment: `source /opt/ros/humble/setup.bash`

### Отладка
- Увеличьте уровень логирования: `export RCUTILS_LOGGING_SEVERITY=DEBUG`
- Проверьте топики: `ros2 topic list`
- Смотрите данные: `ros2 topic echo /odom`

## Мониторинг

```bash
# Просмотр топиков
ros2 topic list

# Чтение данных одометрии
ros2 topic echo /odom

# Чтение данных сонара
ros2 topic echo /ultrasonic_sensor/range

# Отправка команд движения
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.1}}'

# Просмотр TF дерева
ros2 run tf2_tools view_frames
```
