# Sancho HRI System

![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-Apache--2.0-blue)

Sancho is a Human–Robot Interaction platform built with ROS&nbsp;2, Python, and a modern web interface. It aims to deliver natural interactions through speech, vision and expressive animation. The project brings together multiple ROS&nbsp;2 packages, a React+Vite frontend, and an Arduino-based facial display.

## Features

- **Speech and Vision**: integrated audio and video processing nodes.
- **Web Interface**: responsive React app for control and monitoring.
- **Extensible Architecture**: modular ROS&nbsp;2 packages ready for new capabilities.

A simplified view of the system:

```text
+-------------+        +-----------------+        +--------------+
|   Frontend  | <----> |  ROS 2 Nodes    | <----> |   Arduino    |
|  (React)    |        |  (HRI packages) |        |   firmware   |
+-------------+        +-----------------+        +--------------+
```

## Installation

These steps prepare a fresh Ubuntu installation with ROS&nbsp;2 Humble and all project dependencies.

### 1. Install ROS&nbsp;2 Humble

Follow the [official installation guide](https://docs.ros.org/en/humble/Installation.html) or run:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
```

### 2. Install Python Dependencies

```bash
python3 -m pip install -r requirements.txt
```

### 3. Build and Launch the ROS&nbsp;2 Workspace

```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch hri_audio the_launch.py
```

### 4. Set Up the Frontend

Install Node.js (16+ recommended) and run:

```bash
cd web_interface
npm install
npm run dev   # for development
npm run build # required for the_launch.py
```

### 5. Flash the Arduino Firmware

Install the Arduino IDE or [Arduino CLI](https://arduino.github.io/arduino-cli/installation/). Open `face_code/user.cpp`, select your board, compile and upload the sketch to enable mouth and LED control.

### 6. Install K6 for Load Testing

```bash
sudo apt install k6
```

### 7. Install Pytest for Unit Testing

```bash
python3 -m pip install pytest
```

You're all set! The system is now fully installed and ready to run.
