# Task Offloading System for Mobile Robots

Система адаптивного розподілу обчислювальних завдань між мобільним роботом та крайовою станцією.

## Структура проекту
```
task_offloading_ws/
├── src/
│   ├── task_offloading_interfaces/  # ROS 2 messages and services
│   ├── task_offloading_core/        # Core models and algorithms
│   ├── system_monitor/              # System resource monitoring
│   ├── network_monitor/             # Network quality monitoring
│   ├── decision_maker/              # Decision making logic
│   ├── task_executor/               # Task execution coordinator
│   └── edge_server/                 # Edge computing station
```

## Вимоги

- Ubuntu 22.04 або macOS 11.0+ (Big Sur)
- ROS 2 Humble
- Python 3.10+
- CMake 3.8+
- OpenCV 4.x

## Встановлення

### 1. Встановіть ROS 2 Humble
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop

sudo apt install ros-dev-tools
```

### 2. Встановіть залежності
```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs \
  libopencv-dev \
  python3-colcon-common-extensions
```

### 3. Клонуйте репозиторій
```bash
mkdir -p ~/task_offloading_ws/src
cd ~/task_offloading_ws/src
git clone <repository_url>
```

### 4. Зберіть проект
```bash
cd ~/task_offloading_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Використання

### Запуск системи на роботі
```bash
# Terminal 1: Source workspace
source ~/task_offloading_ws/install/setup.bash

# Launch robot nodes
ros2 launch task_offloading_core robot_nodes.launch.py
```

### Запуск крайової станції
```bash
# Terminal 2: Source workspace
source ~/task_offloading_ws/install/setup.bash

# Launch edge station
ros2 launch edge_server edge_station.launch.py
```

### Запуск тестового генератора завдань
```bash
# Terminal 3: Source workspace
source ~/task_offloading_ws/install/setup.bash

# Launch task generator (0.2 Hz = one task every 5 seconds)
ros2 launch task_executor test_generator.launch.py rate:=0.2
```

### Запуск повної системи
```bash
source ~/task_offloading_ws/install/setup.bash
ros2 launch task_offloading_core full_system.launch.py
```

## Моніторинг системи

### Перегляд топіків
```bash
# System state
ros2 topic echo /robot/system_state

# Network state
ros2 topic echo /robot/network_state

# Edge station status
ros2 topic echo /edge/status
```

### Виклик сервісів вручну
```bash
# Get decision for a task
ros2 service call /offloading/decide task_offloading_interfaces/srv/DecideOffloading \
  "{task_id: 1, task_type: 'object_detection', input_data_size: 921600, \
    output_data_size: 1024, max_latency_sec: 1.0, priority: 100}"

# Execute a task
ros2 service call /task/execute task_offloading_interfaces/srv/ExecuteTask \
  "{task_id: 1, task_type: 'object_detection', priority: 100, max_latency_sec: 2.0}"
```

## Налаштування

Параметри можна змінювати через launch файли або командний рядок:
```bash
# Змінити адресу крайової станції
ros2 launch task_offloading_core robot_nodes.launch.py \
  edge_address:=192.168.1.100 edge_port:=50051

# Змінити кількість worker threads на крайовій станції
ros2 launch edge_server edge_station.launch.py num_workers:=4

# Змінити частоту генерації завдань
ros2 launch task_executor test_generator.launch.py rate:=0.5
```

## Архітектура

### Компоненти на роботі

- **system_monitor**: Моніторинг батареї, CPU, пам'яті, температури
- **network_monitor**: Моніторинг якості мережі (пропускна здатність, латентність)
- **decision_maker**: Прийняття рішень про розподіл завдань
- **task_executor**: Координація виконання завдань

### Компоненти на крайовій станції

- **edge_server**: Прийом та виконання завдань від роботів
- **station_monitor**: Моніторинг ресурсів станції

### Потік даних
```
Robot Application
      ↓
  Task Executor
      ↓
  Decision Maker ← System Monitor
      ↓            ↓ Network Monitor
   [Local] or [Edge Server]
      ↓
   Result
```

## Тестування
```bash
# Run tests
colcon test

# View test results
colcon test-result --verbose
```

## Troubleshooting

### Проблема: "Decision service not available"

Переконайтеся, що decision_maker_node запущено:
```bash
ros2 node list | grep decision_maker
```

### Проблема: "Edge service not available"

Перевірте, що edge_server запущено та доступний:
```bash
ros2 service list | grep edge
```

### Проблема: Повільне виконання

Перевірте завантаженість системи:
```bash
ros2 topic echo /robot/system_state
```