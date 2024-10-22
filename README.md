# Robot Fleet Management System

**Description**  
This project implements a Robot Fleet Management system using **ROS 2 Humble**. It allows multiple robots to communicate, 
share information, and work collaboratively to complete tasks within an environment. The system is designed for automation in warehouses, 
factories, or any scenario requiring the coordination of multiple robots.

---
## Table of Contents

1. [Installation](#installation)
2. [Getting Started](#getting-started)
3. [Features](#features)
4. [Usage](#usage)
5. [Contributing](#contributing)
6. [License](#license)

---

## Installation

### Prerequisites

1. **Operating System**: Ubuntu 22.04 (Jammy)
2. **ROS 2 Humble**: Follow the installation steps [here](#ros-2-humble-installation-for-ubuntu).
3. **Colcon build system**: ROS 2 development tools like `colcon` are required.

### Step-by-Step Instructions

**Install ros package as below**
   ```bash
    sudo apt install ros-humble-cyclonedds
    sudo apt install ros-humble-rmf-fleet-msgs
    sudo apt install ros-humble-cyclonedds
    sudo apt install ros-humble-rmf-fleet-msgs
    sudo apt install ros-humble-rmf*
    sudo apt install ros-humble--ament-python
    sudo apt install ros-humble-ament-python
    sudo apt install ros-humble-ament-package 
    sudo apt install ros-humble-ament-pycodestyle 
   ```
**Install python package as below**
   ```bash
      pip3 install flask==2.0.1
      pip3 install Flask-CORS
      pip3 install Flask-SocketIO==4.3.2
      pip3 install websockets==13.1
      pip3 install werkzeug==2.0.2
   ```
**Clone this repository to your workspace**
   ```bash
      cd <your_ws>/src
      git clone https://github.com/JirawatChayayan/robot_rmf_simmulation.git .
   ```
**Build your workspace**
   ```bash
      cd <your_ws>
      colcon build
   ```



