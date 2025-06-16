#!/bin/bash

# Скрипт для быстрого запуска всех компонентов робота через ROS2 пакеты

echo "=== Запуск ROS2 робота ==="

# Проверка установки пакетов
if [ ! -f "$HOME/ros2_ws/install/setup.bash" ]; then
    echo "Ошибка: ROS2 пакеты не установлены"
    echo "Сначала выполните: ./install_packages.sh"
    exit 1
fi

# Настройка среды
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash
source $HOME/ros2_ws/install/setup.bash

echo "Доступные команды запуска:"
echo ""
echo "1. Полный запуск (все компоненты):"
echo "   ros2 launch robot_bringup robot.launch.py"
echo ""
echo "2. Только одометрия:"
echo "   ros2 run robot_odometry odometry_node"
echo ""
echo "3. Только телеуправление:"
echo "   ros2 run robot_teleop teleop_node"
echo ""
echo "4. Только сонар:"
echo "   ros2 run robot_sonar sonar_node"
echo ""
echo "5. Только камера:"
echo "   ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=\"/dev/video1\" -p pixel_format:=\"yuyv\""
echo ""
echo "6. Управление с клавиатуры:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Дополнительно можно добавить 'cam' для запуска камеры:"
echo "   $0 full cam      # Все компоненты + камера"
echo "   $0 lidar cam     # Лидар + камера"
echo "   $0 cam           # Только камера"
echo ""

# Функция запуска камеры
start_camera() {
    echo "Запуск камеры в фоне..."
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video1" -p pixel_format:="yuyv" &
    CAMERA_PID=$!
    echo "Камера запущена (PID: $CAMERA_PID)"
}

# Проверка на камеру в параметрах
CAMERA_ENABLED=false
for arg in "$@"; do
    if [ "$arg" = "cam" ]; then
        CAMERA_ENABLED=true
        break
    fi
done

# Запуск с параметрами
if [ "$1" = "full" ]; then
    echo "Запуск всех компонентов..."
    if [ "$CAMERA_ENABLED" = true ]; then
        start_camera
        sleep 2
    fi
    ros2 launch robot_bringup robot.launch.py
elif [ "$1" = "odom" ]; then
    echo "Запуск одометрии..."
    if [ "$CAMERA_ENABLED" = true ]; then
        start_camera
        sleep 1
    fi
    ros2 run robot_odometry odometry_node
elif [ "$1" = "teleop" ]; then
    echo "Запуск телеуправления..."
    if [ "$CAMERA_ENABLED" = true ]; then
        start_camera
        sleep 1
    fi
    ros2 run robot_teleop teleop_node
elif [ "$1" = "sonar" ]; then
    echo "Запуск сонара..."
    if [ "$CAMERA_ENABLED" = true ]; then
        start_camera
        sleep 1
    fi
    ros2 run robot_sonar sonar_node
elif [ "$1" = "lidar" ]; then
    echo "Запуск лидара..."
    if [ "$CAMERA_ENABLED" = true ]; then
        start_camera
        sleep 1
    fi
    ros2 run robot_lidar lidar_node
elif [ "$1" = "cam" ]; then
    echo "Запуск только камеры..."
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video1" -p pixel_format:="yuyv"
elif [ "$1" = "keyboard" ]; then
    echo "Запуск управления с клавиатуры..."
    if [ "$CAMERA_ENABLED" = true ]; then
        start_camera
        sleep 1
    fi
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
    echo "Использование:"
    echo "  $0 [full|odom|teleop|sonar|lidar|cam|keyboard] [cam]"
    echo ""
    echo "Примеры:"
    echo "  $0 full      # Запуск всех компонентов"
    echo "  $0 full cam  # Запуск всех компонентов + камера"
    echo "  $0 odom      # Только одометрия"
    echo "  $0 teleop    # Только телеуправление"
    echo "  $0 sonar     # Только сонар"
    echo "  $0 lidar     # Только лидар"
    echo "  $0 lidar cam # Лидар + камера"
    echo "  $0 cam       # Только камера"
    echo "  $0 keyboard  # Управление с клавиатуры"
fi
