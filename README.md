# ROS2-Projects-
Collection of ROS 2 robotics tasks and package simulations
Welcome to our team repository, which contains all completed tasks and packages for our ROS 2 system design projects.

## 👥 The Team
* **Rowan tamer** 
* **Nada Mahmoud**  
* **Abd El-Rhman Saad**
* **Marina said**
* **Razan ramzy**
* **Tasneem alaa**
* **Mariam Mamdouh**
* **Asmaa galal**

## 🚀 Project Overview & Key Tasks
This repository contains the source code, custom nodes, and launch configurations for our automated workflows. 

### 1. Robot Control & Trajectory Planning
* Implemented nodes handling coordinate transforms and motor trajectory configurations.
* Programmed safety thresholds to prevent actuator saturation during high-acceleration motion profiles.

### 2. Multi-Node Communication & Custom Interfaces
* Designed customized publishers and subscribers using custom `.msg` and `.srv` interfaces to communicate robot joints states.
* Configured launch files to orchestrate multiple life-cycle nodes simultaneously.

## 🛠️ Prerequisites & Setup
To run these packages locally, ensure you have a working installation of **Ubuntu Linux** and **ROS 2**.

### Workspace Installation
Follow these sequential steps to clone, build, and source the workspace:

1. Create a clean colcon workspace directory structure on your system.
2. Clone this repository directly into your workspace's `src` folder.
3. Run `rosdep install` from the root of your workspace to automatically fetch missing dependencies.
4. Build the packages using `colcon build --symlink-install` to allow quick updates to script modifications.
5. Source the overlay by running `source install/setup.bash` in your current terminal session.
