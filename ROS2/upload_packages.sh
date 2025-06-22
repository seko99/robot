#!/bin/bash

# Скрипт для загрузки ROS2 пакетов на Orange Pi робота

# Конфигурация подключения
REMOTE_USER="orangepi"
REMOTE_HOST="192.168.2.141"
REMOTE_PATH="~/ros2_ws/src"
SSH_KEY="/home/seko/.ssh/id_rsa"

echo "=== Загрузка ROS2 пакетов на робота ==="

# Проверка SSH ключа
if [ ! -f "$SSH_KEY" ]; then
    echo "Ошибка: SSH ключ не найден: $SSH_KEY"
    exit 1
fi

# Проверка подключения
echo "Проверка подключения к роботу..."
if ! ssh -i "$SSH_KEY" -o ConnectTimeout=5 "$REMOTE_USER@$REMOTE_HOST" "echo 'Подключение успешно'"; then
    echo "Ошибка: Не удается подключиться к роботу"
    echo "Проверьте:"
    echo "  - IP адрес робота: $REMOTE_HOST"
    echo "  - SSH ключ: $SSH_KEY"
    echo "  - Пользователь: $REMOTE_USER"
    exit 1
fi

# Загрузка пакетов
echo "Загрузка пакетов..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Загрузка каждого пакета
for package in robot_lidar robot_odometry robot_teleop robot_sonar robot_bringup; do
    if [ -d "$SCRIPT_DIR/$package" ]; then
        echo "Загрузка пакета $package..."
        scp -i "$SSH_KEY" -r "$SCRIPT_DIR/$package" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/"
    else
        echo "Предупреждение: Пакет $package не найден"
    fi
done

# Загрузка вспомогательных файлов
echo "Загрузка скриптов..."
scp -i "$SSH_KEY" "$SCRIPT_DIR/install_packages.sh" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/run_robot.sh" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/build_on_robot.sh" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/diagnose_packages.sh" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/requirements.txt" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/docker-compose.discovery.yml" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/docker-compose.robot.yml" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/docker-compose.yml" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"

echo "Загрузка тестовых скриптов..."
scp -i "$SSH_KEY" "$SCRIPT_DIR/../Motors/test_motors.py" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/../Motors/test_odometry.py" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
scp -i "$SSH_KEY" "$SCRIPT_DIR/../Sensors/test_sensors.py" "$REMOTE_USER@$REMOTE_HOST:~/ros2_ws/"
