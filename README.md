# Coordinated Multi-Agent Warehouse Automation (Holo Battalion)

### **National 3rd Rank (e-Yantra Robotics Competition 2025-26, IIT Bombay)**

This repository contains the complete software implementation and algorithms developed for the **Holo Battalion** theme in the **e-Yantra Robotics Competition (eYRC)**, a national-level robotics challenge organized by the **Indian Institute of Technology (IIT) Bombay**.

---

## Competition Images

<img width="7008" height="4672" alt="AF205621" src="https://github.com/user-attachments/assets/1037603f-12a5-4d59-8468-8f6244a00882" />
<img width="7008" height="4672" alt="AF205617" src="https://github.com/user-attachments/assets/711338cf-0e0d-43b2-aa54-ca5695557eb9" />
<img width="4898" height="3265" alt="AF205640" src="https://github.com/user-attachments/assets/4b698456-e596-458b-9d4e-a3c9b55218f1" />
<img width="4898" height="3265" alt="AF205637" src="https://github.com/user-attachments/assets/41784bb3-d693-424d-b07c-6b3719b01ff1" />

---

## 🚀 Project Overview
The project simulates and deploys an automated warehouse logistics fleet. A swarm of three custom-built 3-wheel holonomic mobile robots (**Glacio**, **Crystal**, and **Frostbite**) must coordinate to identify, transport, and stack color-coded crates (Red, Green, Blue) into designated cooling bays. 

The system bridges a high-level **ROS 2** central coordinator running on a master computer with low-level microcontrollers (**ESP32**) on the physical robots via **MQTT** communication over Wi-Fi. Real-world feedback is provided by an overhead USB webcam tracking **ArUco markers**.

---

## 🛠️ Key Features & Architecture

### 1. Multi-Agent Path Finding (MAPF) & Task Allocation
*   **Cooperative Space-Time A\* Planner**: Implemented a dynamic A* search algorithm operating in a 3D coordinate space $(x, y, t)$ where time is treated as a discrete dimension. It maintains a global space-time reservation table to resolve path conflicts, preventing inter-robot collisions and deadlocks.
*   **Greedy Task Allocation**: Distributes sorting tasks by executing a distance-cost matrix matching algorithm, pairing the closest idle robot with the nearest pending crate.
*   **Trajectory Smoothing**: A collinear point-reduction filter simplifies dense path coordinates, eliminating redundant waypoints to prevent jerky motor deceleration cycles.

### 2. Mobile Base Navigation & Control
*   **3-Wheel Holonomic Kinematics**: Derived the mathematical equations translating global body velocities $(v_x, v_y, \omega)$ into local robot frames and individual wheel angular speeds.
*   **Closed-Loop PID Control**: Integrated separate proportional-integral-derivative controllers for $x, y$, and $\theta$ (yaw) to achieve precise docking.
*   **Dead-band Compensation**: Calibrated velocity thresholds to offset static friction in the continuous-rotation drive servos, ensuring linear wheel responsiveness.

### 3. Perception & Localization Pipeline
*   **Optical Undistortion**: Calibrated the camera intrinsic matrix and distortion coefficients to correct lens distortion.
*   **Homography Mapping**: Computed a 2D homography matrix ($3\times3$) using corner markers (IDs 1, 3, 5, 7) to project distorted camera pixel coordinates into real-world metric coordinates.
*   **Parallax Parity Calibration**: Designed height-scale correction factors to compensate for perspective projection errors caused by the physical height of the robots and crates relative to the floor grid.
*   **Time-Series Telemetry Filtering**: Developed a median threshold filter to eliminate transient yaw angle estimation jumps caused by optical reflections.

### 4. Hardware Actuation & IoT Bridge
*   **MQTT-to-ROS 2 Bridge**: Developed a central bridge node that subscribes to ROS 2 motion topics, serializes data into CSV strings, and broadcasts them to specific ESP32 MQTT command topics at 25Hz.
*   **Embedded Control (ESP32)**: Configured ESP32 microcontrollers to parse MQTT strings, actuate servos via PWM, read IR proximity sensors, and drive electromagnets using MOSFET switches.
*   **Multi-Level Stacking**: Implemented a Finite State Machine (FSM) that adjusts jointed robotic arm angles dynamically for Level-0, Level-1, and Level-2 stack placements.

---

## 📂 Repository Structure
All custom software implementation is stored in the `src/` directory:

```bash
src/
├── PID_controller.py        # ROS 2 PID controller and 3-wheel inverse kinematics node
├── acc.py                   # Space-Time A* planner, reservation table, and path smoothing
├── hardware_controller.py   # Cyber-physical bridge (translates ROS 2 topics to MQTT topics)
├── holonomic_perception.py  # Computer vision (ArUco detection, homography warping, scale correction)
├── multi_robot_controller.py# Central fleet coordinator & FSM state manager
├── cam_feed.py              # Optical publisher node for real-time USB webcam streaming
├── pnp_test.py              # Test script evaluating solvePnP translation vector tracking
├── test_perception.py       # Direct camera-capture vision thread bypassing ROS latency
├── vel.py                   # Telemetry node computing numerical derivatives for velocity monitoring
├── plot.py                  # Graphing utility to visualize tracking errors and tune PID gains
├── plottings.py             # Telemetry script to filter and plot yaw angle time-series
└── PID_test_*.py            # Configuration and tuning scripts used during testing phases
```

---

## ⚙️ Setup & Execution

### Prerequisites
*   Ubuntu 22.04 LTS
*   ROS 2 Humble Hawksbill
*   Gazebo Fortress
*   Python 3.10+ (with `numpy`, `opencv-python`, `paho-mqtt`, `matplotlib`, `pyyaml`)

### Installation & Build
1.  Initialize your ROS 2 workspace:
    ```bash
    mkdir -p ~/hb_ws/src
    cd ~/hb_ws/src
    ```
2.  Clone this repository:
    ```bash
    git clone <your-repository-url>
    cd ~/hb_ws
    ```
3.  Compile and build the workspace:
    ```bash
    colcon build
    source install/setup.bash
    ```

### Running the System (Simulation)
1.  Launch the Gazebo environment:
    ```bash
    ros2 launch hb_description task_6a.launch.py
    ```
2.  Launch the camera localization node:
    ```bash
    ros2 run hb_control holonomic_perception.py
    ```
3.  Launch the multi-robot fleet coordinator:
    ```bash
    ros2 run hb_control multi_robot_controller.py
    ```

---

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Acknowledgments
*   Secured **3rd Rank Nationally** in eYRC 2025-26.
*   We express our gratitude to the **Theme Developers at IIT Bombay (e-Yantra)** for designing this learning experience and providing simulated assets.
