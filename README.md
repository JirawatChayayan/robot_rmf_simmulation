# Robot Fleet Management System

**Description**  
This project implements a Robot Fleet Management system using **ROS 2 Humble**. It allows multiple robots to communicate, 
share information, and work collaboratively to complete tasks within an environment. The system is designed for automation in warehouses, 
factories, or any scenario requiring the coordination of multiple robots.

---

## Special Thanks and Credit to Original Sources

I would like to extend my gratitude to the following projects and repositories, which have been invaluable resources:

1. [TurtleBot3 by ROBOTIS-GIT](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel)  
   A fantastic resource for ROS-based robot development.

2. [Open RMF by open_rmf](https://github.com/open-rmf)  
   Providing crucial libraries for the development of multi-fleet robotic systems.

3. [RMF Demos by open_rmf](https://github.com/open-rmf/rmf_demos)  
   Offering insightful demonstrations and examples on the use of RMF (Robotics Middleware Framework).

4. [RMF Traffic Editor by open_rmf](https://github.com/open-rmf/rmf_traffic_editor)  
   A vital tool for editing and managing traffic schedules in robotic fleet management.

5. [ROSCON_Workshop](https://github.com/open-rmf/roscon_workshop)
   Providing code for Open-RMF workshop at ROSCon 2022 Kyoto.

## Table of Contents

1. [Installation](#installation)
2. [Getting Started](#getting-started)
3. [Usage](#usage)
4. [Document](#Document)
5. [Conclusion](Conclusion)
---

## Installation

### Prerequisites

1. **Operating System**: Ubuntu 22.04 (Jammy)
2. **ROS 2 Humble**: Follow the installation steps [here](https://docs.ros.org/en/humble/Installation.html).
3. **Colcon build system**: ROS 2 development tools like `colcon` are required.

### Step-by-Step Instructions

**Install ros package as below**
   ```bash
   sudo apt install ros-humble-cyclonedds
   sudo apt install ros-humble-rmf-fleet-msgs
   sudo apt install ros-humble-cyclonedds
   sudo apt install ros-humble-rmf-fleet-msgs
   sudo apt install ros-humble-rmf*
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
**Add the following environment variable to your terminal (you can also add this to your ~/.bashrc)***
   ```bash
   export SPAWN_ROBOT=3
   export TURTLEBOT3_MODEL=burger
   ```

## Getting Started

**Starting gazebo and navigation stack for simulate**
   ```bash
   ros2 launch turtlebot3_bringup simulator_launch.launch.py 
   ```
result after run this launch file 
![Screenshot from 2024-10-23 09-20-24](https://github.com/user-attachments/assets/fc5708fd-d95f-4f56-8fac-0d11ddfabebd)

**Starting freefleet server and robot management fleet**
   ```bash
   ros2 launch turtlebot3_bringup freefleet_and_rmf_server.launch.xml
   ```
result after run this launch file 
![image](https://github.com/user-attachments/assets/01fa95ef-4712-49b6-ba5d-2335a8f440bb)

**Starting control robot by RMF panel [here](https://open-rmf.github.io/rmf-panel-js/)**
![image](https://github.com/user-attachments/assets/80ca5381-2109-4fb9-bd51-f22ccc72ca12)


# Usage

Please try to play with __Loop mode__ for delivery tasks (delivery mode is not yet implemented). Follow the image below for testing:
![image](https://github.com/user-attachments/assets/8cc357b8-4088-498e-b877-0abfb6b55b79)

Some robots will move to the goal like this:
![image](https://github.com/user-attachments/assets/7f59f51b-dda3-4bb1-8502-6bde016594a9)

# Document
**[ROSCon Open-RMF Workshop 2022](https://docs.google.com/presentation/d/1Lt79xlM_XkITmURSbI5hkAAgnjSX8dHKBkgvz3x3Uzw/edit#slide=id.g117b0289c78_0_0)**

# Conclusion

The Robot Fleet Management System project serves as a foundational framework for multi-robot coordination using ROS 2 Humble. 
With this system, various environments like warehouses and factories can benefit from automated robotic fleets. 
It provides scalability, flexibility, and adaptability through the use of open-source tools, and offers room for further improvements, including dynamic task allocation and real-world deployment.
By leveraging the contributions from TurtleBot3 and Open RMF, the system is designed for both research and practical application. 


