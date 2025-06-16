# Robot Control System

Проект системы управления мобильным роботом с дифференциальным приводом на базе Arduino и Orange Pi Zero 3.

## 🤖 Описание

Это комплексная система управления роботом, состоящая из:

- **Arduino микроконтроллеры** для низкоуровневого управления моторами и сенсорами
- **Orange Pi Zero 3** как основной компьютер для высокоуровневой логики
- **ROS2 пакеты** для навигации и управления
- **Ультразвуковые датчики** для определения расстояний
- **Энкодеры** для одометрии
- **Лидар LDS01RR** для 2D сканирования окружения

## 📁 Структура проекта

```
robot/
├── Motors/           # Arduino код для управления моторами
│   ├── Motors.ino   # Основной код управления моторами
│   └── compile.sh   # Скрипт компиляции для Arduino
├── Sensors/          # Arduino код для работы с датчиками
│   ├── Sensors.ino  # Код для ультразвукового датчика
│   └── compile.sh   # Скрипт компиляции для Arduino
├── ROS2/            # ROS2 пакеты для управления роботом
│   ├── robot_bringup/      # Основной пакет запуска
│   ├── robot_odometry/     # Расчет одометрии
│   ├── robot_sonar/        # Обработка ультразвука
│   ├── robot_teleop/       # Телеуправление
│   ├── robot_lidar/ # Драйвер лидара LDS01RR
│   ├── install_packages.sh # Установка пакетов
│   ├── run_robot.sh        # Запуск системы
│   ├── upload_packages.sh  # Загрузка на робота
│   ├── build_on_robot.sh   # Сборка на роботе
│   ├── diagnose_packages.sh # Диагностика пакетов
│   ├── test_package.sh     # Тестирование пакетов
│   ├── PACKAGES_SUMMARY.md # Описание пакетов
│   └── requirements.txt    # Python зависимости
├── 3dprint/         # 3D модели креплений и корпуса (FreeCAD .FCStd)
├── docs/            # Документация и схемы
│   ├── LDS01RR/     # Документация лидара
│   └── *.pdf        # Схемы Orange Pi Zero 3
├── motor_test.py    # Скрипт тестирования моторов
├── serial_read.py   # Утилита чтения Serial данных
├── compile.sh       # Основной скрипт компиляции и загрузки
└── upload_py.sh     # Загрузка Python скриптов
```

## 🐳 Docker контейнеризация

Каждый ROS2 пакет упакован в отдельный Docker контейнер для обеспечения:

- **Изоляции зависимостей** - каждый пакет работает в своем окружении
- **Простоты развертывания** - один команда для запуска
- **Портабильности** - работает на любой системе с Docker
- **Масштабируемости** - легко добавить новые контейнеры

### Особенности реализации

- **Entrypoint скрипты** - корректная инициализация ROS2 окружения в контейнерах
- **Управление устройствами** - проброс USB портов для Serial связи
- **Сетевое взаимодействие** - режим host для DDS коммуникации между контейнерами
- **Автоматическая сборка** - Dockerfile для каждого пакета

## 🔧 Компоненты системы

### Arduino модули

#### Motors (Управление моторами)

- **Файл**: `Motors/Motors.ino`
- **Функции**:
  - Управление двумя DC моторами через драйвер TB6612FNG
  - Чтение энкодеров для одометрии
  - Обработка команд по Serial
- **Пины**:
  - Левый мотор: IN1=5, IN2=6, ENA=9
  - Правый мотор: IN3=7, IN4=8, ENB=10
  - Энкодеры: LEFT_H1=4, LEFT_H2=12, RIGHT_H1=2, RIGHT_H2=3

#### Sensors (Датчики)

- **Файл**: `Sensors/Sensors.ino`
- **Функции**:
  - Ультразвуковой датчик HC-SR04
  - Передача данных о расстояниях
- **Пины**:
  - TRIG=9, ECHO=10

### ROS2 пакеты

Полноценные ROS2 пакеты для автономного управления с поддержкой Docker:

#### 📦 robot_bringup

Основной пакет для запуска всех компонентов робота

#### 📦 robot_odometry

Расчет одометрии на основе данных энкодеров

- **Топики**: `/odom` (nav_msgs/Odometry)
- **Конфигурация**: `config/odometry_params.yaml`
- **Docker**: Собственный контейнер с entrypoint скриптом

#### 📦 robot_sonar

Обработка данных ультразвукового датчика

- **Топики**: `/ultrasonic_sensor/range` (sensor_msgs/Range)
- **Конфигурация**: `config/sonar_params.yaml`
- **Docker**: Собственный контейнер с entrypoint скриптом

#### 📦 robot_teleop

Телеуправление роботом с клавиатуры

- **Подписка**: `/cmd_vel` (geometry_msgs/Twist)
- **Конфигурация**: `config/teleop_params.yaml`
- **Docker**: Собственный контейнер с entrypoint скриптом

#### 📦 robot_lidar

Драйвер для лидара LDS01RR

- **Топики**: `/scan` (sensor_msgs/LaserScan)
- **Функции**: 2D лазерное сканирование для навигации
- **Docker**: Собственный контейнер с entrypoint скриптом

## 🚀 Быстрый старт

### Нативная установка ROS2 пакетов

```bash
cd ROS2/
./install_packages.sh
```

### Запуск через Docker

Каждый ROS2 пакет имеет собственный Docker контейнер для изолированного запуска:

```bash
# Сборка Docker образов для всех пакетов
cd ROS2/robot_teleop/
docker build -t robot_teleop .

cd ../robot_sonar/
docker build -t robot_sonar .

cd ../robot_odometry/
docker build -t robot_odometry .

cd ../robot_lidar/
docker build -t robot_lidar .

# Запуск отдельных пакетов в контейнерах
docker run --rm --device=/dev/ttyUSB0 robot_teleop
docker run --rm --device=/dev/ttyUSB1 robot_sonar
docker run --rm --device=/dev/ttyUSB0 robot_odometry
docker run --rm --device=/dev/ttyUSB2 robot_lidar

# Запуск с сетевым режимом host для ROS2 DDS
docker run --rm --network host --device=/dev/ttyUSB0 robot_teleop
```

### pyhton3 зависимости

```bash
sudo apt install python3-pip
sudo apt install python3-serial
```

### Запуск системы (нативно)

```bash
# Полный запуск всех компонентов
./run_robot.sh full

# Управление с клавиатуры
./run_robot.sh keyboard

# Только одометрия
./run_robot.sh odometry

# С лидаром
./run_robot.sh lidar
```

### Сборка и загрузка на робота

```bash
# Установка пакетов на удаленного робота
cd ROS2/
./upload_packages.sh

# Сборка на роботе
./build_on_robot.sh

# Удаленный запуск
ssh -i ~/.ssh/id_rsa orangepi@192.168.2.141 'cd ~/ros2_ws && ./run_robot.sh full'
```

### Компиляция Arduino кода

#### arduino-cli

```bash
wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARM64.tar.gz
tar -xf arduino-cli_latest_Linux_ARM64.tar.gz
sudo mv arduino-cli /usr/local/bin/

/usr/local/bin/arduino-cli version

arduino-cli core update-index
arduino-cli core install arduino:avr

```

```bash
# Основной скрипт (компиляция и загрузка)
./compile.sh

# Для моторов отдельно
cd Motors/
./compile.sh

# Для датчиков отдельно
cd Sensors/
./compile.sh
```

## 🛠 Настройка оборудования

### Подключение Arduino

- **Motors Arduino**: подключить к Orange Pi через USB (/dev/ttyUSB0)
- **Sensors Arduino**: подключить к Orange Pi через USB (/dev/ttyUSB1)
- Настроить правильные порты в ROS2 конфигурации

### Подключение лидара LDS01RR

- **Питание**: 5V
- **Интерфейс**: Serial (115200 baud)
- **Подключение**: к Orange Pi через USB-Serial конвертер

### Схема подключения моторов

```
TB6612FNG драйвер:
- STBY -> Pin 11
- Левый мотор: AIN1->5, AIN2->6, PWMA->9
- Правый мотор: BIN1->7, BIN2->8, PWMB->10
- Энкодеры: подключить к цифровым пинам с прерываниями
```

### Датчики

```
HC-SR04:
- VCC -> 5V
- GND -> GND
- Trig -> Pin 9
- Echo -> Pin 10
```

## 📡 Протокол связи

### Команды для моторов

- `CMD_LEFT_FORWARD (0)` - левый мотор вперед
- `CMD_LEFT_BACKWARD (1)` - левый мотор назад
- `CMD_RIGHT_FORWARD (2)` - правый мотор вперед
- `CMD_RIGHT_BACKWARD (3)` - правый мотор назад
- `CMD_STOP_MOTORS (6)` - остановка всех моторов

### Структуры данных

```cpp
struct Command {
  uint8_t cmd;
  uint8_t value;  // скорость 0-255
};

struct SensorData {
  uint32_t timestamp;
  float left_odometry;
  float right_odometry;
  // ... дополнительные поля
};
```

## 🧪 Тестирование

### Тест моторов

```bash
python3 motor_test.py
```

### Чтение Serial данных

```bash
python3 serial_read.py
```

### Диагностика ROS2

```bash
cd ROS2/
./diagnose_packages.sh
./test_package.sh
```

### Телеуправление

```bash
# Через ROS2 пакет
cd ROS2/
./run_robot.sh keyboard

# Или стандартный teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Мониторинг данных

```bash
# Одометрия
ros2 topic echo /odom

# Ультразвук
ros2 topic echo /ultrasonic_sensor/range

# Лидар
ros2 topic echo /scan

# Команды управления
ros2 topic echo /cmd_vel
```

## 📋 Требования

### Программное обеспечение

- **Arduino IDE** или **arduino-cli**
- **ROS2 Jazzy** (на Orange Pi) или **Docker**
- **Python 3.8+**
- **pyserial** для Python скриптов
- **colcon** для сборки ROS2 пакетов (при нативной установке)
- **Docker** (для контейнерного развертывания)

### Аппаратное обеспечение

- **Orange Pi Zero 3** (основной компьютер)
- **Arduino Uno/Nano** x2 (моторы и датчики)
- **TB6612FNG** (драйвер моторов)
- **HC-SR04** (ультразвуковой датчик)
- **LDS01RR** (лидар для 2D сканирования)
- **DC моторы с энкодерами** x2
- **Аккумулятор** и **DC-DC конвертер**

## 🔍 Отладка

### Проверка подключения

```bash
# Список доступных портов
ls /dev/ttyUSB* /dev/ttyACM*

# Тест связи
python3 serial_read.py

# Проверка ROS2 пакетов
ros2 pkg list | grep robot
```

### Логи ROS2

```bash
# Проверка топиков
ros2 topic list

# Мониторинг данных
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /ultrasonic_sensor/range

# Отправка команд управления
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" --once
```

### Диагностика компонентов

```bash
# Проверка лидара
cd ROS2/robot_lidar/
python3 test_driver.py

# Диагностика всех пакетов
cd ROS2/
./diagnose_packages.sh
```

## 📄 Документация

### 3D модели (`3dprint/`)

В папке содержатся 3D модели компонентов робота в формате **FreeCAD (.FCStd)**:

- Крепления для зарядного устройства
- Крепления для DC-DC конвертера
- Крепления для повербанка
- Крепления для драйвера моторов TB6612FNG
- Крепления для аккумулятора
- Крепления для колес
- Крепление заднего колеса
- Пластина управления питанием
- Стойки 65мм

### Техническая документация (`docs/`)

Дополнительная документация и схемы:

- **Лидар LDS01RR**: datasheet, SDK, примеры ROS интеграции
- **Orange Pi Zero 3**: руководство пользователя и схемы
- Схемы подключения и фотографии сборки

## 🌐 Сетевая настройка

### Cyclone DDS

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
```

#### Проверка

##### host1

```bash
sudo apt install ros-jazzy-demo-nodes-cpp
ros2 run demo_nodes_cpp listener
```

##### host2

```bash
sudo apt install ros-jazzy-demo-nodes-cpp
ros2 run demo_nodes_cpp talker
```

### SSH подключение

```bash
# Подключение к роботу
ssh -i ~/.ssh/id_rsa orangepi@192.168.2.141

# Загрузка файлов
scp -i ~/.ssh/id_rsa file.py orangepi@192.168.2.141:~
```

## Камера

### Установка

```bash
sudo apt install ros-jazzy-usb-cam
```

### На роботе

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video1" -p pixel_format:="yuyv"
```

### На GUI

```bash
ros2 run rqt_image_view rqt_image_view --ros-args -r image:=/image_raw
```

### Удаленное управление

```bash
# Запуск робота
ssh -i ~/.ssh/id_rsa orangepi@192.168.2.141 'cd ~/ros2_ws && ./run_robot.sh full'

# Загрузка пакетов на робота
cd ROS2/
./upload_packages.sh
```

## 🤝 Разработка

### Структура команд

Все команды используют бинарный протокол через Serial (115200 baud).

### Добавление новых сенсоров

1. Добавить код в `Sensors/Sensors.ino`
2. Создать соответствующий ROS2 пакет в `ROS2/`
3. Обновить launch файлы
4. Добавить в `run_robot.sh`

### Создание новых ROS2 пакетов

```bash
cd ROS2/
# Создание нового пакета
ros2 pkg create --build-type ament_python robot_new_package

# Установка
./install_packages.sh
```

## 🚀 Возможности системы

### Автономная навигация

- Одометрия на основе энкодеров
- 2D SLAM с использованием лидара LDS01RR
- Избегание препятствий через ультразвук
- Планирование пути

### Дистанционное управление

- Телеуправление через ROS2 топики
- SSH доступ для удаленной отладки
- Мониторинг состояния в реальном времени

### Модульная архитектура

- Независимые ROS2 пакеты
- Гибкая конфигурация через YAML файлы
- Простое добавление новых сенсоров

## 📞 Поддержка

При возникновении проблем:

1. Проверьте подключение всех компонентов
2. Убедитесь в правильности настройки портов
3. Используйте diagnostic скрипты для проверки ROS2
4. Проверьте логи: `ros2 topic echo /diagnostics`

### Частые проблемы

- **Нет данных с Arduino**: Проверить порты в конфигурации
- **Лидар не работает**: Проверить питание и скорость передачи
- **ROS2 пакеты не найдены**: Выполнить `source install/setup.bash`

---

**Автор**: seko  
**Дата**: 2025  
**Лицензия**: MIT
