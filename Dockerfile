FROM ros:foxy

RUN apt-get update && apt-get install -y \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . ./src
RUN . /opt/ros/foxy/setup.sh && colcon build
CMD ["bash"]