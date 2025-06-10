# Docker контейнеры для ROS2 нод робота

Этот проект содержит Docker контейнеры для всех ROS2 нод робота с поддержкой ROS2 Jazzy.

## Структура

- `robot_sonar/Dockerfile` - контейнер для ультразвукового сенсора
- `robot_odometry/Dockerfile` - контейнер для одометрии
- `robot_teleop/Dockerfile` - контейнер для телеуправления
- `lds01rr_lidar_ros2/Dockerfile` - контейнер для лидара
- `docker-compose.yml` - полная конфигурация со всеми сервисами
- `docker-compose.robot.yml` - упрощенная конфигурация только для основных нод
- `docker-compose.discovery.yml` - конфигурация с ROS Discovery Server для более надежной сетевой связи

## Настройка сети

Для обеспечения видимости ROS2 нод между разными компьютерами в сети:

### 1. Настройка переменных окружения

```bash
# Создайте файл .env в директории ROS2
cp env.example .env

# Отредактируйте .env файл:
# - Установите одинаковый ROS_DOMAIN_ID на всех машинах
# - Укажите IP адрес робота в ROBOT_IP
```

### 2. Настройка на локальной машине (для RViz)

На вашей локальной машине установите те же переменные окружения:

**Вариант A: С CycloneDX (по умолчанию)**

```bash
export ROS_DOMAIN_ID=0  # Тот же, что и на роботе
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI='<Discovery><Peers><Peer address="localhost"/><Peer address="192.168.1.100"/></Peers></Discovery>'
export ROS_LOCALHOST_ONLY=0
```

**Вариант B: С Discovery Server (более надежно)**

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.1.100:11811  # IP робота
export ROS_LOCALHOST_ONLY=0
```

## Использование

### Запуск всех нод робота (основной режим)

```bash
cd ROS2
# Убедитесь, что .env файл настроен правильно
docker-compose -f docker-compose.robot.yml up -d
```

### Запуск с Discovery Server (рекомендуется для сетевого использования)

```bash
cd ROS2
# Для более надежной сетевой связи
docker-compose -f docker-compose.discovery.yml up -d
```

### Запуск всех нод включая GUI (для разработки)

```bash
cd ROS2
docker-compose up -d
```

### Запуск отдельных нод

```bash
# Только сонар
docker-compose -f docker-compose.robot.yml up robot-sonar

# Только лидар
docker-compose -f docker-compose.robot.yml up lidar

# Только одометрия
docker-compose -f docker-compose.robot.yml up robot-odometry
```

### Остановка всех контейнеров

```bash
docker-compose -f docker-compose.robot.yml down
```

### Просмотр логов

```bash
# Все логи
docker-compose -f docker-compose.robot.yml logs -f

# Логи конкретного сервиса
docker-compose -f docker-compose.robot.yml logs -f robot-sonar
```

## Особенности

### Доступ к устройствам

Все контейнеры имеют доступ к последовательным портам:

- `/dev/ttyUSB0`, `/dev/ttyUSB1`, `/dev/ttyUSB2`, `/dev/ttyUSB3`
- `/dev/ttyACM0`, `/dev/ttyACM1`

### Привилегированный режим

Контейнеры запускаются в привилегированном режиме (`privileged: true`) для обеспечения доступа к устройствам.

### Сеть

Используется `network_mode: host` для обеспечения ROS2 DDS коммуникации между контейнерами.

### Переменные окружения

- `ROS_DOMAIN_ID=0` - домен ROS2 по умолчанию

## Построение образов

Образы строятся автоматически при первом запуске. Для принудительной пересборки:

```bash
docker-compose -f docker-compose.robot.yml build
```

## Отладка

### Вход в контейнер

```bash
docker exec -it robot-sonar bash
```

### Проверка ROS2 нод

```bash
# Список активных нод
docker exec -it robot-sonar ros2 node list

# Список топиков
docker exec -it robot-sonar ros2 topic list

# Просмотр сообщений
docker exec -it robot-sonar ros2 topic echo /ultrasonic_sensor/range
```

### Проверка сетевого соединения

```bash
# На роботе - проверить активные ноды
docker exec -it lidar ros2 node list

# На локальной машине - проверить видимость удаленных нод
ros2 node list

# Проверить топики лидара с локальной машины
ros2 topic list | grep scan
ros2 topic echo /scan

# Запуск RViz на локальной машине
rviz2
```

### Устранение проблем с сетью

Если ноды не видны между машинами:

1. **Проверьте сетевое соединение:**

   ```bash
   ping <ROBOT_IP>
   ```

2. **Проверьте firewall:**

   ```bash
   # На Ubuntu отключите firewall временно для тестирования
   sudo ufw disable
   ```

3. **Проверьте переменные окружения:**

   ```bash
   # На роботе
   docker exec -it lidar printenv | grep ROS

   # На локальной машине
   printenv | grep ROS
   ```

4. **Проверьте мультикаст:**

   ```bash
   # Установите netcat если его нет
   sudo apt install netcat

   # Тест мультикаст (выполните на обеих машинах)
   # На одной машине:
   nc -u 239.255.0.1 7400
   # На другой:
   nc -l -u -p 7400
   ```

## Требования

- Docker >= 20.10
- Docker Compose >= 2.0
- Права доступа к `/dev/tty*` устройствам

## Настройка прав доступа к устройствам

```bash
# Добавить пользователя в группу dialout
sudo usermod -a -G dialout $USER

# Перезагрузить сессию или выйти/войти в систему
```
