#!/bin/bash

echo "Копирование конфигурации FastDDS во все пакеты..."

cp robot_teleop/fastdds_config.xml robot_sonar/fastdds_config.xml
cp robot_teleop/fastdds_config.xml robot_odometry/fastdds_config.xml
cp robot_teleop/fastdds_config.xml robot_lidar/fastdds_config.xml

echo "Добавление конфигурации в Dockerfiles..."

for package in robot_sonar robot_odometry robot_lidar; do
    if ! grep -q "fastdds_config.xml" $package/Dockerfile; then
        sed -i '/COPY . \/ros2_ws\/src\//a COPY fastdds_config.xml /root/' $package/Dockerfile
    fi
done

echo "Обновление entrypoint.sh файлов..."

for package in robot_sonar robot_odometry robot_lidar; do
    if [ -f $package/entrypoint.sh ]; then
        if ! grep -q "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" $package/entrypoint.sh; then
            sed -i '/source \/ros2_ws\/install\/setup.bash/a\\nexport RMW_IMPLEMENTATION=rmw_fastrtps_cpp\nexport FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_config.xml\nexport ROS_DISABLE_LOANED_MESSAGES=1' $package/entrypoint.sh
        fi
    fi
done

echo "Готово! Теперь пересоберите контейнеры: docker-compose build" 