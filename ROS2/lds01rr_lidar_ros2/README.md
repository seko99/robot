# LDS01RR Lidar ROS2 Driver

ROS2 драйвер для лидара LDS01RR, переписанный с C++ на Python.

## Установка

### Предварительные требования

- ROS2 (Humble/Iron/Jazzy)
- Python 3.8+
- pyserial

### Установка зависимостей

```bash
# Установка Python зависимостей
pip install -r requirements.txt

# Или через apt (Ubuntu)
sudo apt install python3-serial
```

### Сборка и установка пакета

```bash
# Перейти в workspace
cd ~/ros2_ws/src

# Скопировать пакет
cp -r /path/to/lds01rr_lidar_ros2 .

# Собрать пакет
cd ~/ros2_ws
colcon build --packages-select lds01rr_lidar_ros2

# Источник среды
source install/setup.bash
```

## Использование

### Запуск драйвера

```bash
# Базовый запуск
ros2 run lds01rr_lidar_ros2 lidar_node

# Запуск с кастомными параметрами
ros2 run lds01rr_lidar_ros2 lidar_node --ros-args -p port:=/dev/ttyUSB0 -p baud_rate:=115200

# Запуск через launch файл
ros2 launch lds01rr_lidar_ros2 lidar_launch.py

# Запуск с кастомными параметрами через launch
ros2 launch lds01rr_lidar_ros2 lidar_launch.py port:=/dev/ttyUSB0 baud_rate:=115200
```

### Параметры

| Параметр       | Тип    | По умолчанию   | Описание                    |
| -------------- | ------ | -------------- | --------------------------- |
| `port`         | string | `/dev/ttyACM0` | Серийный порт лидара        |
| `baud_rate`    | int    | `115200`       | Скорость порта              |
| `frame_id`     | string | `lidar`        | Frame ID для данных лидара  |
| `publish_tf`   | bool   | `true`         | Публиковать ли TF трансформ |
| `parent_frame` | string | `base_link`    | Родительский frame для TF   |

### Топики

#### Публикуемые топики

- `/scan` (`sensor_msgs/LaserScan`): Данные лазерного сканирования

#### TF Transforms

- `base_link` → `lidar` (если `publish_tf` == true)

## Настройка прав доступа к порту

```bash
# Добавить пользователя в группу dialout
sudo usermod -a -G dialout $USER

# Или установить права на конкретный порт
sudo chmod 666 /dev/ttyACM0
```

## Устранение неполадок

### Ошибка подключения к порту

1. Проверьте, что устройство подключено:

   ```bash
   ls /dev/ttyACM* /dev/ttyUSB*
   ```

2. Проверьте права доступа:

   ```bash
   ls -l /dev/ttyACM0
   ```

3. Проверьте, что порт не используется другим процессом:
   ```bash
   sudo lsof /dev/ttyACM0
   ```

### Нет данных от лидара

1. Проверьте параметры порта (скорость, порт)
2. Проверьте питание лидара
3. Проверьте логи ноды:
   ```bash
   ros2 run lds01rr_lidar_ros2 lidar_node --ros-args --log-level debug
   ```

## Визуализация

```bash
# Запуск RViz2
rviz2

# Добавить LaserScan display
# Topic: /scan
# Fixed Frame: lidar
```

## Лицензия

BSD-3-Clause

## Авторы

- Оригинальный C++ код: Eric Perko, Chad Rockey, iliasam
- Python порт для ROS2: Your Name
