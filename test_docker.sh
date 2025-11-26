#!/bin/bash

# Скрипт автоматизованого тестування системи Task Offloading
# Автор: Система аналізу для Бондарчук Артем

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# Кольори для виводу
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Функції для виводу
info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

error() {
    echo -e "${RED}[✗]${NC} $1"
}

# Перевірка Docker
check_docker() {
    info "Перевірка Docker..."
    if ! command -v docker &> /dev/null; then
        error "Docker не встановлено!"
        error "Встановіть Docker Desktop: https://www.docker.com/products/docker-desktop/"
        exit 1
    fi
    
    if ! docker ps &> /dev/null; then
        error "Docker не запущено!"
        error "Запустіть Docker Desktop і спробуйте знову."
        exit 1
    fi
    
    success "Docker встановлено і запущено"
}

# Збірка проекту
build_project() {
    info "Збірка Docker образу..."
    if docker compose build; then
        success "Docker образ успішно зібрано"
    else
        error "Помилка збірки Docker образу"
        exit 1
    fi
}

# Перевірка збірки
verify_build() {
    info "Перевірка збірки..."
    
    # Визначаємо правильну назву образу
    IMAGE_NAME=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "robot.*-robot_nodes" | head -n 1)
    if [ -z "$IMAGE_NAME" ]; then
        IMAGE_NAME="robot-task-distributor"
    fi
    
    docker run --rm $IMAGE_NAME bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 pkg list | grep -E 'task_offloading|system_monitor|decision_maker|network_monitor|task_executor|edge'
    " > /tmp/ros2_packages.txt
    
    EXPECTED_PACKAGES=(
        "task_offloading_interfaces"
        "task_offloading_core"
        "system_monitor"
        "network_monitor"
        "decision_maker"
        "task_executor"
        "edge_server"
        "edge_client"
    )
    
    MISSING=0
    for pkg in "${EXPECTED_PACKAGES[@]}"; do
        if grep -q "$pkg" /tmp/ros2_packages.txt; then
            success "Пакет $pkg знайдено"
        else
            error "Пакет $pkg НЕ знайдено!"
            MISSING=$((MISSING + 1))
        fi
    done
    
    if [ $MISSING -eq 0 ]; then
        success "Всі пакети присутні"
    else
        error "$MISSING пакет(ів) відсутні"
        exit 1
    fi
}

# Запуск системи
start_system() {
    info "Запуск системи..."
    
    # Зупинити попередні контейнери
    docker compose down 2>/dev/null || true
    
    # Запустити в фоновому режимі
    if docker compose up -d robot_nodes edge_station; then
        success "Вузли запущено в фоновому режимі"
        info "Очікування ініціалізації (10 секунд)..."
        sleep 10
    else
        error "Помилка запуску вузлів"
        exit 1
    fi
}

# Перевірка запущених вузлів
check_nodes() {
    info "Перевірка запущених ROS 2 вузлів..."
    
    NODES=$(docker exec robot_nodes bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 node list
    ")
    
    EXPECTED_NODES=(
        "/system_monitor"
        "/network_monitor"
        "/decision_maker"
        "/task_executor"
        "/edge_server"
        "/station_monitor"
    )
    
    for node in "${EXPECTED_NODES[@]}"; do
        if echo "$NODES" | grep -q "$node"; then
            success "Вузол $node запущено"
        else
            warning "Вузол $node НЕ знайдено"
        fi
    done
}

# Перевірка топіків
check_topics() {
    info "Перевірка топіків ROS 2..."
    
    TOPICS=$(docker exec robot_nodes bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 topic list
    ")
    
    EXPECTED_TOPICS=(
        "/robot/system_state"
        "/robot/network_state"
        "/edge/status"
    )
    
    for topic in "${EXPECTED_TOPICS[@]}"; do
        if echo "$TOPICS" | grep -q "$topic"; then
            success "Топік $topic активний"
        else
            warning "Топік $topic НЕ знайдено"
        fi
    done
}

# Перевірка сервісів
check_services() {
    info "Перевірка сервісів ROS 2..."
    
    SERVICES=$(docker exec robot_nodes bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        ros2 service list
    ")
    
    EXPECTED_SERVICES=(
        "/offloading/decide"
        "/task/execute"
        "/edge/execute_task"
    )
    
    for service in "${EXPECTED_SERVICES[@]}"; do
        if echo "$SERVICES" | grep -q "$service"; then
            success "Сервіс $service доступний"
        else
            warning "Сервіс $service НЕ знайдено"
        fi
    done
}

# Тестовий виклик сервісу
test_service_call() {
    info "Тестовий виклик сервісу прийняття рішення..."
    
    RESULT=$(docker exec robot_nodes bash -c "
        source /opt/ros/humble/setup.bash &&
        source /workspace/install/setup.bash &&
        timeout 10 ros2 service call /offloading/decide task_offloading_interfaces/srv/DecideOffloading \
            '{task_id: 999, task_type: \"object_detection\", parameters: {task_type: \"object_detection\", 
             image_width: 640, image_height: 480, image_encoding: \"rgb8\", confidence_threshold: 0.5, 
             target_classes: [\"person\", \"car\"], num_points: 0, voxel_size: 0.0, complexity_estimate: 10.0}, 
             input_data_size: 921600, output_data_size: 1024, max_latency_sec: 1.0, 
             priority: 100, is_critical: false}'
    " 2>&1 || true)
    
    if echo "$RESULT" | grep -q "response:"; then
        success "Сервіс відповів успішно"
        info "Результат:\n$RESULT"
    else
        warning "Сервіс не відповів або повернув помилку"
        info "Вивід: $RESULT"
    fi
}

# Запуск тестового генератора
start_test_generator() {
    info "Запуск генератора тестових завдань..."
    
    if docker compose up -d test_generator; then
        success "Генератор тестів запущено"
        info "Генератор буде створювати 1 завдання кожні 5 секунд"
        info "Переглянути логи: docker compose logs -f test_generator"
    else
        error "Помилка запуску генератора тестів"
    fi
}

# Показати логи
show_logs() {
    info "Останні 20 рядків логів системи:"
    echo ""
    echo "=== Robot Nodes ==="
    docker compose logs --tail=10 robot_nodes
    echo ""
    echo "=== Edge Station ==="
    docker compose logs --tail=10 edge_station
    echo ""
}

# Зупинка системи
stop_system() {
    info "Зупинка системи..."
    docker compose down
    success "Систему зупинено"
}

# Головне меню
show_menu() {
    echo ""
    echo "========================================="
    echo "   Тестування Robot Task Distributor"
    echo "========================================="
    echo ""
    echo "1) Повне тестування (автоматичний запуск)"
    echo "2) Збірка проекту"
    echo "3) Запуск системи"
    echo "4) Перевірка вузлів/топіків/сервісів"
    echo "5) Запуск генератора тестів"
    echo "6) Показати логи"
    echo "7) Зупинити систему"
    echo "8) Інтерактивний режим (bash)"
    echo "0) Вихід"
    echo ""
    read -p "Виберіть опцію: " choice
    
    case $choice in
        1)
            check_docker
            build_project
            verify_build
            start_system
            check_nodes
            check_topics
            check_services
            test_service_call
            start_test_generator
            show_logs
            success "Повне тестування завершено!"
            info "Систему запущено. Використайте опцію 6 для перегляду логів."
            ;;
        2)
            check_docker
            build_project
            verify_build
            ;;
        3)
            start_system
            check_nodes
            ;;
        4)
            check_nodes
            check_topics
            check_services
            ;;
        5)
            start_test_generator
            ;;
        6)
            show_logs
            ;;
        7)
            stop_system
            ;;
        8)
            info "Запуск інтерактивного режиму..."
            IMAGE_NAME=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep "robot.*-robot_nodes" | head -n 1)
            if [ -z "$IMAGE_NAME" ]; then
                IMAGE_NAME="robot-task-distributor"
            fi
            docker run -it --rm --network host \
                -v $(pwd):/workspace \
                $IMAGE_NAME \
                bash -c "source /opt/ros/humble/setup.bash && \
                         source /workspace/install/setup.bash && \
                         bash"
            ;;
        0)
            info "Вихід"
            exit 0
            ;;
        *)
            error "Невірна опція"
            ;;
    esac
}

# Якщо скрипт запущено без аргументів - показати меню
if [ $# -eq 0 ]; then
    while true; do
        show_menu
    done
else
    # Інакше виконати команду з аргументу
    case $1 in
        "full")
            check_docker
            build_project
            verify_build
            start_system
            check_nodes
            check_topics
            check_services
            test_service_call
            start_test_generator
            show_logs
            ;;
        "build")
            check_docker
            build_project
            verify_build
            ;;
        "start")
            start_system
            ;;
        "check")
            check_nodes
            check_topics
            check_services
            ;;
        "test")
            start_test_generator
            ;;
        "logs")
            show_logs
            ;;
        "stop")
            stop_system
            ;;
        *)
            error "Невірна команда: $1"
            info "Доступні команди: full, build, start, check, test, logs, stop"
            exit 1
            ;;
    esac
fi
