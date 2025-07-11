# Сравнение драйверов LDS01RR v1 и v2

## Основные различия

### Драйвер v1 (lds01rr_driver.py)

- **Протокол**: Использует заголовки пакетов `0xE7 0x7E`
- **Базовый код**: Основан на более старой реализации
- **Структура пакетов**: 34-байтовые пакеты измерений
- **Синхронизация**: Простая последовательность синхронизации
- **Проблема**: Возможно использует устаревший протокол

### Драйвер v2 (lds01rr_driver_v2.py)

- **Протокол**: Использует заголовки `0xAA` (информация) и `0xFA` (измерения)
- **Базовый код**: Основан на LDS.c (оригинальная C реализация)
- **Структура пакетов**:
  - Информационные пакеты: 85 байт
  - Измерительные пакеты: 22 байта
- **Синхронизация**: Полная реализация с escape-последовательностями
- **Контрольные суммы**: Правильная проверка целостности данных

## Протокольные отличия

### v1 Protocol

```
Sync: 0xE7 0x7E
Packet size: 34 bytes
Structure: [sync][size][data...]
```

### v2 Protocol

```
Info packets: 0xAA [data...] (85 bytes total)
Measurement packets: 0xFA [angle][speed][4x measurements] (22 bytes total)
Escape sequences: 0xA9 0x00 -> 0xA9, 0xA9 0x01 -> 0xAA
Checksums: Different algorithms for info vs measurement packets
```

## Особенности v2

1. **Правильная обработка escape-последовательностей**

   - 0xA9 0x00 представляет байт 0xA9
   - 0xA9 0x01 представляет байт 0xAA

2. **Корректные контрольные суммы**

   - Информационные пакеты: простая сумма 16-битных слов
   - Измерительные пакеты: циклический сдвиг с накоплением

3. **Правильная структура данных измерений**

   - Каждый пакет содержит индекс угла и скорость мотора
   - 4 точки измерения на пакет
   - Правильное вычисление углов

4. **Контроль скорости мотора**
   - Извлечение данных о скорости из пакетов
   - Публикация скорости в отдельном топике

## Использование v2

### Тестирование драйвера

```bash
cd ~/Robot/ROS2/lds01rr_lidar_ros2
python3 -m lds01rr_lidar_ros2.test_driver_v2
```

### Запуск ROS2 ноды

```bash
# Сборка пакета
colcon build --packages-select lds01rr_lidar_ros2

# Запуск ноды v2
ros2 run lds01rr_lidar_ros2 lidar_node_v2

# Или с launch файлом
ros2 launch lds01rr_lidar_ros2 lds01rr_v2.launch.py serial_port:=/dev/ttyUSB0
```

### Параметры

- `serial_port`: Последовательный порт (по умолчанию /dev/ttyUSB0)
- `baud_rate`: Скорость передачи (по умолчанию 115200)
- `frame_id`: ID фрейма для данных лидара
- `publish_rate`: Частота публикации в Гц

### Топики

- `/scan`: sensor_msgs/LaserScan - данные сканирования
- `/lidar_speed`: std_msgs/Float32 - скорость мотора в RPM

## Рекомендации

1. **Используйте v2** если текущий драйвер не работает корректно
2. **Проверьте последовательный порт** - может быть /dev/ttyUSB0 или /dev/ttyS5
3. **Мониторьте топик скорости** для контроля работы мотора лидара
4. **Проверяйте логи** на наличие ошибок контрольных сумм

## Отладка

Если v2 тоже не работает:

1. Проверьте подключение лидара
2. Убедитесь в правильности последовательного порта
3. Проверьте скорость передачи данных
4. Возможно, ваш лидар использует другой протокол

Для получения подробной информации используйте:

```bash
ros2 run lds01rr_lidar_ros2 test_driver_v2
```
