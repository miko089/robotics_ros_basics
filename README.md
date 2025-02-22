# Robotics ROS Basics

## Overview
This project implements a simple ROS2-based robot control system. It consists of two main nodes:
- **position_publisher** – publishes simulated robot position data.
- **velocity_calculator** – subscribes to the position data, calculates the velocity, and publishes the results.
- **velocity_monitor** – subscribes to the velocity data and logs the information.

The project uses custom message types for transmitting position and velocity data and is deployed using Docker (Docker Compose is encouraged).

## Functionality
- **Position Publisher Node**
  - Publishes simulated 2D robot position (x, y) at 10 Hz.
  - Uses a custom message type `RobotPosition` with the following fields:
    - `position_x` (float)
    - `position_y` (float)
    - `timestamp` (float)
  - Simulates robot movement in a circular pattern.

- **Velocity Calculator Node**
  - Subscribes to the data published by the `position_publisher` node.
  - Calculates the robot's velocity based on the differences between consecutive position values.
  - Publishes velocity data using a custom message type `RobotVelocity`, which includes:
    - `velocity_x` (float)
    - `velocity_y` (float)
    - `speed` (float)
    - `timestamp` (float)

- **Velocity Monitor Node**
  - Subscribes to the velocity data published by the `velocity_calculator` node.
  - Logs the velocity information to the console.

- **Docker Deployment**
  - The project is containerized using Docker.
  - Docker Compose is recommended for simplified deployment.

## Project Architecture
- **Nodes:**
  - `position_publisher`: Publishes simulated robot position data.
  - `velocity_calculator`: Calculates velocity based on the received position data.
- **Custom Messages:**
  - `RobotPosition.msg`
  - `RobotVelocity.msg`
- **Deployment:**
  - Uses Docker and Docker Compose for containerization and easy deployment.

## Custom Message Types

### RobotPosition.msg
```plaintext
float32 position_x
float32 position_y
float32 timestamp
```
### RobotVelocity.msg
```plaintext
float32 velocity_x
float32 velocity_y
float32 speed
float32 timestamp
```
## Getting Started

Prerequisites
- ROS2 (e.g., Foxy, Humble, or Galactic – depending on compatibility)
- Docker and Docker Compose (for containerized deployment)

### Installation and Build
1. Clone the repository:
```bash
git clone https://github.com/miko089/robotics_ros_basics
cd robotics-ros-basics
```
2. Build the ROS2 workspace:
```bash
colcon build
source install/setup.bash
```

### Running the Project

### Running Without Docker
1.	Launch the position_publisher node:
```bash
ros2 run nodes position_publisher
```
2.	Launch the velocity_calculator node (in a new terminal):
```bash
ros2 run nodes velocity_calculator
```
3. Launch the velocity_monitor node (in a new terminal):
```bash
ros2 run nodes velocity_monitor
```


Running with Docker
1.	Build the Docker image:
```bash
docker build -t robotics-ros-basics .
```
2.	Run the containers using Docker Compose:
```bash
docker-compose up
```