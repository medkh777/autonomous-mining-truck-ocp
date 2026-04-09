# autonomous-mining-truck-ocp
Autonomous navigation system for Komatsu 730E mining truck — ROS2 · EKF · MATLAB/Simulink · OCP Group
# 🚛 Autonomous Navigation System — Komatsu 730E Mining Truck

> Developed at **OCP Group Benguérir** | ENSA Tanger — Academic Year 2024–2025
> Author: **Mohamed Khlifate**
> Supervisor: M. AJEMAHRI Abdelilah — OCP Benguérir

---

## 📌 Project Overview

Full-stack autonomous and remote-controlled driving system for the **Komatsu 730E**
phosphate haul truck (186 metric tons) operating in the OCP Benguérir mining site —
one of the world's largest phosphate operations.

The system enables the truck to:
- Navigate autonomously in a defined mining zone
- Detect and avoid obstacles in real-time
- Localize itself precisely using sensor fusion
- Fall back to remote teleoperation when needed

---

## 🏗️ System Architecture
┌─────────────────────────────────────────────────┐
│           AUTONOMOUS TRUCK SYSTEM               │
│                                                 │
│  ┌──────────┐    ┌──────────┐   ┌────────────┐ │
│  │  LIDAR   │    │  CAMERA  │   │  IMU + GPS │ │
│  │ HOKOYO   │    │ 640×480  │   │  Fusion    │ │
│  └────┬─────┘    └────┬─────┘   └─────┬──────┘ │
│       │               │               │        │
│       └───────────────┼───────────────┘        │
│                       ▼                        │
│              ┌─────────────────┐               │
│              │  ROS2 Nav Stack │               │
│              │  Gazebo + RVIZ  │               │
│              └────────┬────────┘               │
│                       ▼                        │
│         ┌─────────────────────────┐            │
│         │   EKF Sensor Fusion     │            │
│         │  Odometry + IMU + GPS   │            │
│         └────────────┬────────────┘            │
│                      ▼                         │
│         ┌─────────────────────────┐            │
│         │  StateX III Controller  │            │
│         │  Autonomous / Remote    │            │
│         └────────────┬────────────┘            │
│                      ▼                         │
│         ┌─────────────────────────┐            │
│         │   PI Speed Controller   │            │
│         │   MATLAB / Simulink     │            │
│         └─────────────────────────┘            │
└─────────────────────────────────────────────────┘

---

## 🛠️ Tech Stack

| Layer | Technology |
|-------|-----------|
| **Robot OS** | ROS2 (Humble) |
| **Simulation** | Gazebo · RVIZ |
| **Robot Model** | URDF / SDF |
| **Perception** | LiDAR · Camera · IMU |
| **Localization** | EKF · Odometry · GPS |
| **Control** | MATLAB/Simulink · Stateflow |
| **Programming** | C++ · Python |
| **Platform** | Ubuntu 22.04 |

---

## 🤖 The Vehicle — Komatsu 730E

| Spec | Value |
|------|-------|
| **Payload** | 186 metric tons |
| **Power** | 2,000 HP (1,492 kW) |
| **Drive** | Electric (AC/DC) |
| **Brain** | StateX III Controller |
| **Max Speed** | 55–56 km/h |
| **Mass (model)** | 230,000 kg |

---

## 🧠 StateX III — The Truck's Brain

The **StateX III** is an IGBT AC/DC controller that:
- Regulates traction and dynamic braking
- Manages autonomous motor control
- Detects anomalies and limits overcurrents
- Optimizes fuel consumption and reduces tire/brake wear

---

## 📡 Sensor Suite

### LiDAR (HOKOYO)
- Range: 0.02m to 300m
- Used for: obstacle detection, 2D mapping, EKF input
- 720 samples per scan filtered into 5 sectors of 60 degrees each

### Camera
- Resolution: 640x480 at 30Hz
- Topic: /camera/image_raw
- Used for: environment vision, object detection

### IMU
- Provides: orientation (quaternion), angular velocity, linear acceleration
- Topic: /imu
- Update rate: 100Hz

### GPS
- Precision: 2 to 10m average, 1m optimal
- Used for: absolute position via trilateration
- Combined with odometry for improved accuracy

---

## 🗺️ Localization — Extended Kalman Filter (EKF)

The EKF fuses three complementary sources:
Input Sources:
├── Wheel Odometry    → relative position estimate
├── IMU               → orientation + acceleration
└── GPS               → absolute position reference
Output:
└── Fused Robot Pose  → precise position + orientation

### Quaternion to Euler Conversion (Python)
```python
quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w
)
euler = transformations.euler_from_quaternion(quaternion)
yaw = euler[2]
```

---

## ⚙️ PI Speed Controller — MATLAB/Simulink

### Transfer Function
Vehicle model:  V(s) = 1 / (m·s + b)
PI Controller:  C(s) = Kp × (1 + 1/(Ti·s))

### Parameters

| Parameter | Value |
|-----------|-------|
| Vehicle mass (m) | 230,000 kg |
| Damping coefficient (b) | m x 9.81 x 0.008 N·s/m |
| Integration time (Ti) | m/b = 12.74 s |
| Proportional gain (Kp) | m/τ = 2,300,000 |
| Sampling period | 0.1s via c2d conversion |

### Controller Performance
- Fast response time
- No oscillations observed
- Near-zero steady-state error
- Stable across all test scenarios

---

## 🤖 Robot Model — URDF Structure
```xml
<robot name="robot_mobile">

  <!-- Chassis: 0.5 x 0.3 x 0.1 m -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
    </visual>
  </link>

  <!-- LiDAR: -90 to +90 degrees, 720 samples, range 0.1 to 10m -->
  <sensor name="lidar_sensor" type="ray">
    <ray>
      <scan><horizontal>
        <samples>720</samples>
        <min_angle>-1.57</min_angle>
        <max_angle>1.57</max_angle>
      </horizontal></scan>
      <range><min>0.1</min><max>10.0</max></range>
    </ray>
    <update_rate>30</update_rate>
  </sensor>

  <!-- Camera: 640x480, 30Hz -->
  <sensor name="camera_sensor" type="camera">
    <camera>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <update_rate>30</update_rate>
  </sensor>

  <!-- IMU: 100Hz, angular velocity + linear acceleration -->
  <sensor name="imu_sensor" type="imu">
    <imu>
      <angular_velocity><x>true</x><y>true</y><z>true</z></angular_velocity>
      <linear_acceleration><x>true</x><y>true</y><z>true</z></linear_acceleration>
    </imu>
    <update_rate>100</update_rate>
  </sensor>

</robot>
```

---

## 🔄 ROS2 Node Architecture
/odom ──────────────► /path_odom_plotter ──► /odompath
► /rosout
/tf_static ─────────► /robot_pose_ekf ──────► /tf
└──► /path_ekf_plotter ──► /ekfpath
/imu ───────────────►

---

## 🚀 ROS2 Navigation Node (C++)
```cpp
#include "rclcpp/rclcpp.hpp"

class NavNode : public rclcpp::Node
{
public:
  NavNode() : Node("nav_node")
  {
    RCLCPP_INFO(this->get_logger(), "Navigation node started!");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}
```

---

## 🧪 Simulation Launch
```python
# simulation_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn robot model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', 'models/model.sdf', '-entity', 'my_robot',
                       '-x', '0', '-y', '0', '-z', '0.5'],
            output='screen'
        ),
        # Launch navigation node
        Node(
            package='my_robot_nav',
            executable='my_robot_nav_node',
            name='my_robot_nav_node',
            output='screen'
        ),
    ])
```

---

## 📋 System Requirements Diagram
Transport du phosphate — OCP Benguérir
│
├── Conduite Autonome
│   ├── Perception environnementale
│   │   └── Détection d'obstacles
│   ├── Localisation
│   │   └── Cartographie (SLAM)
│   └── Planification
│       ├── Planification chemin
│       ├── Contrôle freinage
│       └── Contrôle accélération
│
└── Conduite Déportée
├── Pilotage à distance
└── Supervision temps réel

---

## 📊 Signal Builder — Decision Logic

| Signal | Type | Description |
|--------|------|-------------|
| ON | 0/1 | Activates the system |
| OFF | 0/1 | Deactivates the system |
| SET | 0/1 | Launches regulator at chosen speed |
| SET_SPEED | float | Reference speed value |
| LEAD_SPEED | float | Speed of vehicle ahead |
| DETECTION | 0/1 | Object detected within 20m via LiDAR |
| IDENTIFICATION | 0/1/2 | 0=nothing · 1=vehicle · 2=person |
| DISTANCE | float | Distance to detected object |

---

## 🧮 Odometry Model

Robot position updated at each time step k:
N(k+1) = N(k) + d × cos(ψk + β)
E(k+1) = E(k) + d × sin(ψk + β)
Where:
d    = distance traveled between k and k+1
ψk   = heading angle (North to vehicle axis)
β    = correction angle

---

## 🔧 How to Run
```bash
# 1. Clone the repository
git clone https://github.com/medkh777/autonomous-mining-truck-ocp****
cd autonomous-mining-truck-ocp

# 2. Build ROS2 workspace
colcon build
source install/setup.bash

# 3. Launch full simulation
ros2 launch my_robot_nav simulation_launch.py

# 4. Open RVIZ visualization
rviz2 -d config/robot_viz.rviz

# 5. Read raw LiDAR data
ros2 topic echo /scan

# 6. Run EKF localization
ros2 launch robot_pose_ekf ekf.launch.py

# 7. Launch MATLAB/Simulink controller
# Open speed_controller.slx in MATLAB and run simulation
```

---

## 📁 Repository Structure
autonomous-mining-truck-ocp/
│
├── README.md
├── docs/
│   ├── architecture_diagram.png
│   ├── requirements_diagram.png
│   └── simulation_results.png
│
├── ros2_ws/
│   ├── src/
│   │   └── my_robot_nav/
│   │       ├── CMakeLists.txt
│   │       ├── package.xml
│   │       ├── src/
│   │       │   └── main.cpp
│   │       └── launch/
│   │           └── simulation_launch.py
│   └── models/
│       ├── robot_mobile.urdf
│       └── model.sdf
│
└── matlab_simulink/
├── speed_controller.slx
├── signal_builder_scenarios.slx
└── README_matlab.md

---

## 📚 References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Classic](https://classic.gazebosim.org/)
- [OCP Group](https://www.ocpgroup.ma)
- [Komatsu 730E Technical Manual](https://www.komatsu.com)
- [robot_pose_ekf ROS Package](http://wiki.ros.org/robot_pose_ekf)
- MATLAB/Simulink — MathWorks Documentation

---

## 🎓 Academic Context

| Field | Detail |
|-------|--------|
| **School** | ENSA Tanger |
| **Degree** | Génie des Systèmes Électroniques et Automatiques |
| **Type** | Projet de Fin d'Année (PFA) |
| **Company** | OCP Group — Benguérir |
| **Period** | July – September 2025 |
| **Grade** | — |

---

## 👤 Author

**Mohamed Khlifate**
Power Electronics & Embedded Engineer @ KAFEB MAROC
Embedded Developer (AI & Robotics) @ OCP Group
ENSA Tanger — Electronic & Automatic Systems Engineering

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Mohamed_Khlifate-0077B5?style=flat&logo=linkedin)](https://linkedin.com/in/mohamedkhlifate)
[![GitHub](https://img.shields.io/badge/GitHub-medkh777-181717?style=flat&logo=github)](https://github.com/medkh777)

---

> ⚠️ **Important Notice**
> Source code and technical data are proprietary to OCP Group Benguérir.
> This repository contains only architecture documentation,
> simulation configurations, methodology, and academic content.
> All industrial data has been anonymized or removed.

---

*Built with ROS2 · Gazebo · MATLAB/Simulink · Python · C++*
*OCP Group Benguérir — Mining Automation & Industry 4.0*
