# Installation Guide

This repository contains multiple ROS2 packages targeting ROS 2 Humble and Python 3.10.12.

## Prerequisites

- ROS 2 Humble installed and sourced
- Python 3.10.12

## Install Python dependencies

Use the provided `requirements.txt` file to install all Python packages:

```bash
python3 -m pip install -r requirements.txt
```

## Build the ROS2 workspace

```bash
cd ros2_ws
colcon build
```

Remember to source the workspace after building:

```bash
source install/setup.bash
```

## Running

After sourcing the workspace, you can run any of the provided nodes using `ros2 run <package> <executable>`.

