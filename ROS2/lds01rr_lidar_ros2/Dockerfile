FROM osrf/ros:jazzy-desktop

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

# Копирование пакета lds01rr_lidar_ros2
COPY . /ros2_ws/src/lds01rr_lidar_ros2/

# Инициализация rosdep
RUN rosdep init && rosdep update

# Установка зависимостей пакета
RUN cd /ros2_ws && rosdep install --from-paths src --ignore-src -r -y

# Сборка пакета
RUN cd /ros2_ws && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select lds01rr_lidar_ros2

# Настройка окружения
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Точка входа
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "lds01rr_lidar_ros2", "lidar_node"] 