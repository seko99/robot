FROM ros:humble

# Установка необходимых системных пакетов
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Установка Python зависимостей
RUN pip3 install pyserial>=3.4 setuptools>=45.2.0

# Создание рабочей директории
WORKDIR /ros2_ws

# Копирование пакета robot_lidar
COPY . /ros2_ws/src/robot_lidar/
COPY fastdds_config.xml /root/

# Инициализация rosdep
RUN rosdep update

# Установка зависимостей пакета
RUN cd /ros2_ws && rosdep install --from-paths src --ignore-src -r -y

# Сборка пакета
RUN cd /ros2_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --packages-select robot_lidar

# Копирование и настройка скрипта запуска
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Точка входа
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "robot_lidar", "lidar_node"] 