# Robot Sonar Package

ROS2 пакет для чтения данных с ультразвукового датчика через Arduino.

## Описание

Этот пакет считывает данные с ультразвукового датчика через Arduino и публикует их в стандартном формате ROS2 `sensor_msgs/Range`.

## Установка

```bash
cd ~/ros2_ws/src
# Скопируйте пакет robot_sonar в src
cd ~/ros2_ws
colcon build --packages-select robot_sonar
source install/setup.bash
```

## Использование

```bash
# Запуск ноды сонара
ros2 run robot_sonar sonar_node

# Запуск с launch файлом
ros2 launch robot_sonar sonar.launch.py

# Просмотр данных сонара
ros2 topic echo /ultrasonic_sensor/range

# Просмотр частоты публикации
ros2 topic hz /ultrasonic_sensor/range
```

## Параметры

- `serial_port`: Порт Arduino (по умолчанию: /dev/ttyUSB1)
- `baud_rate`: Скорость передачи данных (по умолчанию: 115200)
- `frame_id`: ID фрейма для датчика (по умолчанию: sonar_link)
- `min_range`: Минимальная дальность в метрах (по умолчанию: 0.02)
- `max_range`: Максимальная дальность в метрах (по умолчанию: 4.0)
- `field_of_view`: Угол обзора в радианах (по умолчанию: 0.1)
- `publish_rate`: Частота публикации в Гц (по умолчанию: 10.0)

## Топики

### Публикуемые
- `/ultrasonic_sensor/range` (sensor_msgs/Range): Данные ультразвукового датчика

## Формат данных

Пакет ожидает бинарные данные от Arduino в формате:
- `uint8_t cmd`: Команда (обычно не используется)
- `float distance`: Расстояние в сантиметрах

## Обработка данных

- Данные автоматически преобразуются из сантиметров в метры
- Измерения вне диапазона обрабатываются специальным образом:
  - Слишком близкие: устанавливается значение меньше min_range
  - Слишком далекие: устанавливается infinity
