#!/bin/bash

# Простой скрипт для тестирования отдельного пакета ROS2

if [ $# -eq 0 ]; then
    echo "Использование: $0 <package_name>"
    echo "Доступные пакеты: robot_odometry robot_teleop robot_sonar"
    exit 1
fi

PACKAGE_NAME=$1

echo "=== Тестирование пакета $PACKAGE_NAME ==="

# Проверка существования пакета
if [ ! -d "/home/seko/Robot/ROS2/$PACKAGE_NAME" ]; then
    echo "Ошибка: Пакет $PACKAGE_NAME не найден"
    exit 1
fi

cd "/home/seko/Robot/ROS2/$PACKAGE_NAME"

# Проверка setup.py
echo "Проверка setup.py..."
python3 setup.py check

# Попытка создать временный workspace и собрать пакет
TEMP_WS="/tmp/test_ros2_ws"
rm -rf "$TEMP_WS"
mkdir -p "$TEMP_WS/src"
cp -r "/home/seko/Robot/ROS2/$PACKAGE_NAME" "$TEMP_WS/src/"

cd "$TEMP_WS"

echo "Попытка сборки в изолированном окружении..."

# Проверка ROS2
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
    echo "ROS2 не найден, продолжаем без него"
fi

# Проверка colcon
if command -v colcon &> /dev/null; then
    echo "Сборка с colcon..."
    colcon build --packages-select "$PACKAGE_NAME" --cmake-args -DCMAKE_BUILD_TYPE=Release --verbose
    
    if [ $? -eq 0 ]; then
        echo "✅ Сборка успешна!"
        
        # Проверка исполняемых файлов
        if [ -d "install/$PACKAGE_NAME/lib/$PACKAGE_NAME" ]; then
            echo "Найденные исполняемые файлы:"
            ls -la "install/$PACKAGE_NAME/lib/$PACKAGE_NAME/"
        fi
        
        # Попытка запуска
        source install/setup.bash
        echo "Доступные команды для $PACKAGE_NAME:"
        ros2 pkg executables "$PACKAGE_NAME" || echo "Команды не найдены"
    else
        echo "❌ Ошибка сборки"
    fi
else
    echo "colcon не найден, используем pip..."
    pip3 install --user -e "src/$PACKAGE_NAME"
fi

# Очистка
cd /
rm -rf "$TEMP_WS"

echo "Тестирование завершено"
