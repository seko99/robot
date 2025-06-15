#!/bin/bash

# Диагностический скрипт для проверки Python пакетов ROS2

echo "=== Диагностика Python пакетов ROS2 ==="

# Проверка Python окружения
echo "Python версия:"
python3 --version

echo "Python путь:"
which python3

echo "Доступные модули setuptools:"
python3 -c "import setuptools; print(f'setuptools версия: {setuptools.__version__}')"

# Проверка каждого пакета
for pkg in robot_lidar robot_odometry robot_teleop robot_sonar robot_bringup; do
    echo ""
    echo "=== Проверка пакета $pkg ==="
    
    cd "/home/seko/Robot/ROS2/$pkg"
    
    # Проверка структуры
    echo "Структура пакета:"
    find . -name "*.py" -o -name "setup.py" -o -name "package.xml" | sort
    
    # Проверка setup.py
    echo "Проверка setup.py:"
    if python3 setup.py check; then
        echo "✅ setup.py корректен"
    else
        echo "❌ Проблемы с setup.py"
    fi
    
    # Проверка entry_points
    echo "Entry points в setup.py:"
    grep -A 5 "console_scripts" setup.py || echo "Entry points не найдены"
    
    cd -
done

echo ""
echo "=== Попытка установки локально ==="
cd /home/seko/Robot/ROS2/robot_odometry
pip3 install -e . --user
echo "Установлена robot_odometry"

# Проверка установки
echo "Проверка установленного пакета:"
python3 -c "import robot_odometry; print('✅ robot_odometry импортирован успешно')" || echo "❌ Не удалось импортировать robot_odometry"

# Проверка entry point
echo "Проверка entry point:"
which odometry_node || echo "odometry_node не найден в PATH"

echo "Поиск исполняемого файла:"
find ~/.local/bin -name "*odometry*" 2>/dev/null || echo "Исполняемый файл не найден"
