# 🤖 Mobile Manipulator

A ROS 2 based **Mobile Manipulator** that combines autonomous navigation and robotic manipulation into a single platform. The project integrates a differential-drive mobile base with a **5-DOF Moveo BCN robotic arm**, enabling full-body planning, navigation, and pick-and-place operations in simulation.

Designed using **ROS 2**, **MoveIt 2**, **Nav2**, **Gazebo**, and **RViz**, the repository provides a complete simulation environment for developing and testing mobile manipulation algorithms.

---

## 🎥 Demo

> **[Insert Hero GIF here]**

A short demonstration showing:

- Autonomous navigation
- Full-body robot visualization
- Arm motion planning
- Pick-and-place task

---

## ✨ Features

- Differential drive mobile robot
- 5-DOF Moveo BCN robotic arm
- ROS 2 Control integration
- Gazebo simulation
- RViz visualization
- Nav2 autonomous navigation
- SLAM Toolbox mapping and localization
- MoveIt 2 motion planning
- Full-body planning support
- Modular ROS 2 package architecture

---

## 🏗 System Architecture

> **[Insert Architecture Diagram]**

Main software stack:

```
                 ROS 2

        +-------------------+
        |      Nav2         |
        +-------------------+
                  |
        +-------------------+
        |  Mobile Base      |
        +-------------------+

                  +

        +-------------------+
        |    MoveIt 2       |
        +-------------------+
                  |
        +-------------------+
        |  Moveo BCN Arm    |
        +-------------------+

                  |

        Gazebo • RViz • ros2_control
```

---

## 📂 Repository Structure

```
ebot_description/
├── Launch files
├── URDF/Xacro
├── Gazebo world
├── Navigation
├── ros2_control

moveo_description/
├── Arm description
├── URDF

moveo_moveit_config/
├── MoveIt configuration
├── Controllers
├── Planning pipelines

ebot_serial/
├── Base communication

ebot_slam/
├── IMU
├── Magnetometer

moveo_hardware/
├── Arm hardware interface
```

---

# 🚀 Installation

Clone the workspace and build using Colcon.

```bash
colcon build --symlink-install
source install/setup.bash
```

---

# 🚀 Running the Simulation

## 1. Launch Gazebo

Starts the simulation world.

```bash
ros2 launch ebot_description ebot_gazebo.launch.py
```

---

## 2. Spawn the Robot

Spawns the mobile manipulator into Gazebo along with all required controllers.

```bash
ros2 launch ebot_description spawn_ebot.launch.py
```

---

## 3. Start Navigation Stack

Launches SLAM Toolbox and Nav2.

```bash
ros2 launch ebot_description navigation_stack.launch.py
```

By default the system starts in **mapping mode**.

To use localization with an existing map:

- Modify `lidar_slam.yaml`
- or provide a saved map

---

## 4. Launch MoveIt 2

```bash
ros2 launch moveo_moveit_config move_group.launch.py use_sim_time:=true use_base:=true
```

Arguments:

| Argument | Description |
|----------|-------------|
| use_sim_time | Use Gazebo simulation clock |
| use_base | Enables full-body planning including the mobile base |

---

# 🗺 Navigation

> **[Insert Navigation GIF]**

Navigation is powered by:

- Nav2
- SLAM Toolbox
- AMCL / Mapping
- ROS 2 Control

The robot can autonomously navigate to user-defined goal poses while avoiding obstacles.

---

# 🦾 Manipulation

> **[Insert MoveIt GIF]**

Manipulator features:

- 5-DOF Moveo BCN arm
- MoveIt 2 planning
- Collision checking
- Inverse Kinematics
- OMPL planning
- Full-body planning support

---

# 📦 Packages

| Package | Description |
|---------|-------------|
| ebot_description | Robot description, launch files, Gazebo simulation |
| moveo_description | Moveo BCN arm URDF |
| moveo_moveit_config | MoveIt configuration |
| moveo_hardware | Hardware interface |
| ebot_serial | Base communication |
| ebot_slam | IMU and localization |
| moveo_moveit_demos | Example MoveIt demos |

---

# 🔮 Future Work

- RGB-D object detection
- Autonomous pick-and-place pipeline
- Behavior Tree task execution
- Multi-goal navigation
- MoveIt Task Constructor integration
- Dynamic obstacle avoidance
- Real robot deployment improvements

---

# 📄 License

This project is intended for educational and research purposes.

---

## 👨‍💻 Author

**Atharva Deshmukh**

**Sumit Shelwane**

If you found this project useful, consider giving the repository a ⭐.
