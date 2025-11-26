# RL_HW3
This is the repository for RoboticsLab HW3 - PX4 Ciccio Drone Autonomous Flight Control System.

Link to the playlist containing our demo videos on Youtube: 
https://youtube.com/playlist?list=[YOUR_PLAYLIST_ID]

## 🔨 Installation

### Prerequisites
- Docker installed and configured
- PX4-Autopilot repository cloned locally
- ROS 2 Humble installed

### Clone Required Repositories
```bash
cd ~/ros2_ws/src
# Clone your project repositories here
```

### Install Required Packages
```bash
# Core ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-px4-msgs

# Micro XRCE-DDS Agent for PX4-ROS2 communication
sudo snap install micro-xrce-dds-agent --edge
```

### Setup Ciccio Drone Model
Copy the `ciccio_drone` model folder to `PX4-Autopilot/Tools/simulation/gz/models/` containing:
- `model.config`
- `model.sdf`
- `/meshes/propeller.dae`

Copy the airframe file `4022_gz_ciccio_drone` to `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`

Register the airframe in CMakeLists.txt by adding `4022_gz_ciccio_drone` to the `px4_add_romfs_files()` function in `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

*Alternatively, use the updated `CMakeLists.txt` file provided in this repository.*

### Configure DDS Topics
Add ActuatorOutputs message to `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`:
```yaml
publications:
  - topic: /fmu/out/actuator_outputs
    type: px4_msgs::msg::ActuatorOutputs
```
*Alternatively, use the updated `dds_topics.yaml` file provided in this repository.*

### Build the Workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Initial PX4 Compilation
```bash
cd PX4-Autopilot
make clean
make px4_sitl gz_ciccio_drone
```

## ✅ Usage

---

## **1. Basic Drone Simulation (1.a)**

### Launch Ciccio Drone Simulation
**Terminal 1** - Start PX4 SITL with Gazebo:
```bash
cd PX4-Autopilot
make px4_sitl gz_ciccio_drone
```

This command compiles PX4 in Software-In-The-Loop (SITL) mode, starts Gazebo Harmonic and loads the ciccio_drone model with all configured plugins and sensors. Once loading is complete, the drone will be ready to receive commands via the PX4 console or external ground stations such as QGroundControl.

To demonstrate the correct implementation of collisions and the correct dynamic behaviour of the drone, we have recorded a demo video: https://www.youtube.com/watch?v=DMF_y9ble9E

---

## **2. Actuator Outputs Visualization (1.b)**

### Launch Sequence
**Terminal 1** - Start drone simulation:
```bash
cd PX4-Autopilot
make px4_sitl gz_ciccio_drone
```

**Terminal 2** - Start Micro XRCE-DDS bridge for PX4-ROS2 communication:
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3** - Visualize actuator outputs in real-time:
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /fmu/out/actuator_outputs
```

---

## **3. Force Land Safety System (2.a)**

**ForceLand** is a ROS 2 node that automatically triggers a PX4 UAV landing when the altitude exceeds **20 m**, while **preventing the landing procedure from re-triggering** if the pilot manually intervenes.

The node monitors:
- `VehicleLocalPosition` → altitude
- `VehicleLandDetected` → ground contact
- `ManualControlSetpoint` → throttle input

It publishes landing commands via:
- `VehicleCommand` (NAV_LAND)

### Purpose of the Modification
Originally, climbing above 20 m always triggered a landing. If the pilot interrupted the maneuver and climbed again, the system **wrongly reactivated** automatic landing.

**Goal**: Once the pilot intervenes **before landing is complete**, the auto-landing **must not re-trigger**, even if altitude exceeds 20 m again. Reset occurs *only* after PX4 detects `ground_contact == true`.

### Implemented Logic

**✔️ Triggering Condition**

Auto-landing activates only if:
- altitude_enu > 20 m
- ready_to_trigger == true

**✔️ Pilot Override**

If the pilot moves the throttle during landing:
- auto-landing stops
- cannot restart until real touchdown

**✔️ Ground Contact Reset**

Once `VehicleLandDetected.ground_contact == true`:
- system resets
- ready for next cycle

### Launch Sequence
**Terminal 1** - Start drone simulation:
```bash
cd PX4-Autopilot
make px4_sitl gz_ciccio_drone
```

**Terminal 2** - Start Micro XRCE-DDS bridge:
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3** - Run ForceLand node:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run force_land force_land_node
```

### Data Recording for Analysis
Record necessary signals for plotting:
```bash
ros2 bag record /fmu/out/vehicle_local_position
```

### Required Plots

**1. UAV Altitude (ENU)**

Compute: `altitude_enu = -vehicle_local_position.z`

**2. Manual Throttle Setpoint**

From: `manual_control_setpoint.throttle`

These plots must demonstrate:
- auto-landing triggers at > 20 m
- pilot override prevents re-triggering
- reset occurs only at ground contact

---

## **4. Multi-Waypoint Trajectory Planner (3.a)**

### Execution and Validation Procedure
To verify the planner's functionality, after launching the PX4 simulation environment, execute the following from the `/home/user/ros2_ws/src` folder:

**Terminal 1** - Start drone simulation:
```bash
cd PX4-Autopilot
make px4_sitl gz_ciccio_drone
```

**Terminal 2** - Start Micro XRCE-DDS bridge:
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 3** - Run multi-waypoint planner:
```bash
cd ~/ros2_ws/src
colcon build --packages-select offboard_rl
source install/setup.bash
ros2 run offboard_rl multi_waypoint_planner
```

Once the node is started, the terminal displays a text menu requesting the input of waypoints in the discussed format. For testing purposes, a sequence of seven waypoints was used (although more could be used), for example:

```bash
5 0 3 0
5 5 3 1.57
0 5 3 3.14
-5 5 3 -1.57
-5 0 3 0
0 0 3 0
0 0 0 0
done
5
```

where the last line indicates a uniform time of 5 seconds for each segment.

---

## 📄 License
Apache-2.0

---

## 🔗 References
- [PX4 Autopilot Documentation](https://docs.px4.io/)
- [PX4 User Guide - SITL Simulation](https://docs.px4.io/main/en/simulation/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [PX4-ROS 2 Bridge](https://docs.px4.io/main/en/ros/ros2_comm.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/)
