FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    libopencv-dev \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

# Зберіть проект
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

CMD ["/bin/bash"]