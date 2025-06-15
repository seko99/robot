#!/bin/bash

# Скрипт для сборки всех ROS2 пакетов робота

echo "=== Сборка ROS2 пакетов робота ==="

# Проверка наличия ROS2
if ! command -v ros2 &> /dev/null; then
    echo "Ошибка: ROS2 не установлен или не настроен"
    echo "Убедитесь, что ROS2 установлен и выполните: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Проверка workspace
if [ ! -d "$HOME/ros2_ws" ]; then
    echo "Создание ROS2 workspace..."
    mkdir -p $HOME/ros2_ws/src
    cd $HOME/ros2_ws
    colcon build
    echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

cd $HOME/ros2_ws

# Установка зависимостей Python
echo "Установка зависимостей Python..."
pip3 install pyserial

# Копирование пакетов
echo "Копирование пакетов в workspace..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Удаление старых пакетов если есть
rm -rf src/robot_*

# Копирование новых пакетов
cp -r "$SCRIPT_DIR/robot_lidar" src/
cp -r "$SCRIPT_DIR/robot_odometry" src/
cp -r "$SCRIPT_DIR/robot_teleop" src/
cp -r "$SCRIPT_DIR/robot_sonar" src/
cp -r "$SCRIPT_DIR/robot_bringup" src/

# Сборка пакетов
echo "Сборка пакетов..."
colcon build --packages-select robot_lidar robot_odometry robot_teleop robot_sonar robot_bringup

if [ $? -eq 0 ]; then
    echo "=== Сборка завершена успешно! ==="
    echo ""
    echo "Для использования выполните:"
    echo "  source $HOME/ros2_ws/install/setup.bash"
    echo ""
    echo "Запуск робота:"
    echo "  ros2 launch robot_bringup robot.launch.py"
    echo ""
    echo "Управление с клавиатуры (требует teleop_twist_keyboard):"
    echo "  sudo apt install ros-humble-teleop-twist-keyboard"
    echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
else
    echo "=== Ошибка при сборке! ==="
    exit 1
fi
