# RL_HW3

This is the repository for RoboticsLab HW3 - PX4 Ciccio Drone Autonomous Flight Control System.

Link to the playlist containing our demo videos on Youtube:
https://www.youtube.com/playlist?list=PLIDEFmFRzNxxQFPCKc3mNf2lnarBoGnwy

## 🔨 Installation

### Prerequisites

- Docker installed and configured
- PX4-Autopilot repository cloned locally
- ROS 2 Humble installed
- Repository https://github.com/PX4/px4_msgs.git cloned

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

## ✅ Usage

---

## **1. Basic Drone Simulation (1.a)**

### Launch Ciccio Drone Simulation

**Terminal 1** - Start PX4 SITL with Gazebo:

```bash
cd PX4-Autopilot
make clean
make px4_sitl gz_ciccio_drone

```

This command compiles PX4 in Software-In-The-Loop (SITL) mode, starts Gazebo Harmonic and loads the ciccio_drone model with all configured plugins and sensors. Once loading is complete, the drone will be ready to receive commands via the PX4 console or external ground stations such as QGroundControl.

(To demonstrate the correct implementation of collisions and the correct dynamic behaviour of the drone, we have recorded a demo video: https://www.youtube.com/watch?v=DMF_y9ble9E).

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
cd home/user/ros2_ws/src
colcon build
source install/setup.bash
ros2 topic echo /fmu/out/actuator_outputs

```

Then launch the executable file of QGroundControl. Now, to fly the drone in position flight mode, you have two choices: you can set the "commander takeoff" in the first terminal (PX4 terminal) or you can set the takeoff directly in QGroundControl.
While you drive your drone, you can visualize the actuator outputs in the fourth terminal.

Here is the link to the demo video: https://youtu.be/4mg3Z719B8A

---

## **3. Force Land Safety System (2.a-2.b)**

### Launch Sequence

**Terminal 1** - Start drone simulation:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500

```

**Terminal 2** - Start Micro XRCE-DDS bridge:

```bash
MicroXRCEAgent udp4 -p 8888

```

After updating the force_land package with the provided one:

**Terminal 3** - Run ForceLand node:

```bash
cd home/user/ros2_ws/src
colcon build
source install/setup.bash
ros2 run force_land force_land

```

### Data Recording for Analysis

Even if you can record necessary signals for plotting executing:

```bash
ros2 bag record -o fight_data /fmu/out/vehicle_local_position /fmu/out/manual_control_setpoint

```

we provide an already saved bag data file (metadata.yaml).

Here are the links to the demo videos regarding respectively 2a and 2b:  https://youtu.be/sBBLaQh54Js ; https://youtu.be/Ju3Iz9ULEwI and https://youtu.be/B0iTyZc5vtc .

---

## **4. Multi-Waypoint Trajectory Planner (3.a)**

### Execution and Validation Procedure

To verify the planner's functionality, after launching the PX4 simulation environment, execute the following from the `/home/user/ros2_ws/src` folder:

**Terminal 1** - Start drone simulation:

```bash
cd PX4-Autopilot
make px4_sitl gz_x500

```

**Terminal 2** - Start Micro XRCE-DDS bridge:

```bash
MicroXRCEAgent udp4 -p 8888

```

**Terminal 3** - Run multi-waypoint planner:

```bash
cd home/user/ros2_ws/src
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

Here is the link to the demo video regarding 3b:

- xy: https://youtu.be/B5FcKZlTeLk
- z: https://youtu.be/eVtOTnoNAOg
- yaw: https://youtu.be/wGpUq9WT8xA
- velocity: https://youtu.be/RArwbE9gq5U
- acceleration: https://youtu.be/_elXurXmuKg

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
