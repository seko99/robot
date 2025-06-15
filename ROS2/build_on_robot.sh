#!/bin/bash

# Скрипт для правильной сборки ROS2 пакетов на роботе

echo "=== Сборка ROS2 пакетов на роботе ==="

# Переход в workspace
cd ~/ros2_ws

# Источник ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "Используется ROS2 Humble"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo "Используется ROS2 Galactic"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "Используется ROS2 Foxy"
else
    echo "Ошибка: ROS2 не найден!"
    exit 1
fi

# Установка Python зависимостей
echo "Установка Python зависимостей..."
pip3 install pyserial setuptools

# Проверка структуры пакетов
echo "Проверка структуры пакетов..."
for pkg in robot_lidar robot_odometry robot_teleop robot_sonar robot_bringup; do
    echo "Пакет $pkg:"
    if [ -d "src/$pkg" ]; then
        echo "  ✅ Директория пакета существует"
        if [ -f "src/$pkg/setup.py" ]; then
            echo "  ✅ setup.py найден"
        else
            echo "  ❌ setup.py не найден"
        fi
        if [ -f "src/$pkg/package.xml" ]; then
            echo "  ✅ package.xml найден"
        else
            echo "  ❌ package.xml не найден"
        fi
        if [ -f "src/$pkg/$pkg/__init__.py" ]; then
            echo "  ✅ __init__.py найден"
        else
            echo "  ❌ __init__.py не найден"
        fi
    else
        echo "  ❌ Директория пакета не найдена"
    fi
    echo ""
done

# Очистка предыдущей сборки
echo "Очистка предыдущей сборки..."
rm -rf build/ install/ log/

# Сборка всех пакетов с подробным выводом
echo "Сборка только наших пакетов..."
colcon build --packages-select robot_lidar robot_odometry robot_teleop robot_sonar robot_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release

# Проверка результатов сборки
# Проверка результатов сборки
echo "Проверка результатов сборки..."

for pkg in robot_lidar robot_odometry robot_teleop robot_sonar; do
    echo "Проверка $pkg:"
    if [ -d "install/$pkg/lib/$pkg" ]; then
        echo "  ✅ Директория lib создана"
        echo "  Содержимое:"
        ls -la install/$pkg/lib/$pkg/
    else
        echo "  ❌ Директория lib не найдена"
        echo "  Структура install/$pkg:"
        if [ -d "install/$pkg" ]; then
            find install/$pkg -type f
        else
            echo "  Пакет не установлен"
        fi
    fi
    echo ""
done

# Источник собранных пакетов
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Источник install/setup.bash загружен"
else
    echo "❌ install/setup.bash не найден"
fi

# Добавление в bashrc если еще не добавлено
if ! grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    echo "Добавлено в ~/.bashrc"
fi

echo "=== Сборка завершена ==="

# Проверка доступных исполняемых файлов
echo "Проверка доступных исполняемых файлов:"
for pkg in robot_lidar robot_odometry robot_teleop robot_sonar; do
    echo "$pkg:"
    ros2 pkg executables $pkg || echo "  Исполняемые файлы не найдены"
done

echo ""
echo "Для запуска используйте:"
echo "  ./run_robot.sh full"
