PX4 Ciccio Drone - Autonomous Flight Control System
This is the repository for Robotics Lab homework on PX4 UAV simulation, ROS 2 integration, and autonomous flight control systems.
Demo Videos Playlist: https://youtube.com/playlist?list=[YOUR_PLAYLIST_ID]

🔨 Installation
Prerequisites

Docker installed and configured
PX4-Autopilot repository cloned locally
ROS 2 Humble installed

Clone Required Repositories
bashcd ~/ros2_ws/src
# Clone your project repositories here
Install Required Packages
bash# Core ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-px4-msgs

# Micro XRCE-DDS Agent for PX4-ROS2 communication
sudo snap install micro-xrce-dds-agent --edge
Setup Ciccio Drone Model

Copy the ciccio_drone model folder to PX4-Autopilot/Tools/simulation/gz/models/

The folder must contain:

model.config
model.sdf
/meshes/propeller.dae




Copy the airframe file 4022_gz_ciccio_drone to PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
Register the airframe in CMakeLists.txt:

Add 4022_gz_ciccio_drone to the px4_add_romfs_files() function in PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
Alternative: Use the updated CMakeLists.txt file provided in this repository



Configure DDS Topics for Actuator Outputs

Add ActuatorOutputs message to PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml:

yamlpublications:
  - topic: /fmu/out/actuator_outputs
    type: px4_msgs::msg::ActuatorOutputs
Alternative: Use the updated dds_topics.yaml file provided in this repository.
Build the Workspace
bashcd ~/ros2_ws
colcon build
source install/setup.bash
Initial PX4 Compilation
bashcd PX4-Autopilot
make clean
make px4_sitl gz_ciccio_drone

✅ Usage
1. Basic Drone Simulation (1.a)
Launch Ciccio Drone Simulation
Terminal 1 - Start PX4 SITL with Gazebo:
bashcd PX4-Autopilot
make px4_sitl gz_ciccio_drone
This command:

Compiles PX4 in Software-In-The-Loop (SITL) mode
Starts Gazebo Harmonic
Loads the ciccio_drone model with all configured plugins and sensors

Once loading is complete, the drone will be ready to receive commands via:

PX4 console (in the same terminal)
QGroundControl (external ground station)

Demo Video: Watch the collision implementation and dynamic behavior demonstration at: https://www.youtube.com/watch?v=DMF_y9ble9E
Basic Flight Commands (PX4 Console)
bash# Arm the drone
commander arm

# Take off to 2.5m altitude
commander takeoff

# Land
commander land

2. Actuator Outputs Visualization (1.b)
This section demonstrates real-time monitoring of motor outputs through ROS 2.
Launch Sequence
Terminal 1 - Start drone simulation:
bashcd PX4-Autopilot
make px4_sitl gz_ciccio_drone
Terminal 2 - Start Micro XRCE-DDS bridge:
bashMicroXRCEAgent udp4 -p 8888
This bridge enables PX4-ROS2 communication by translating PX4 uORB messages to ROS 2 topics.
Terminal 3 - Visualize actuator outputs in real-time:
bashsource ~/ros2_ws/install/setup.bash
ros2 topic echo /fmu/out/actuator_outputs
Expected Output
You should see real-time motor PWM values updating as the drone operates:
yamltimestamp: 1234567890
noutputs: 4
output: [1500, 1500, 1500, 1500]  # PWM values for each motor

3. Force Land Safety System (2.a)
ForceLand is a ROS 2 safety node that automatically triggers landing when altitude exceeds 20 m, with intelligent pilot override detection.
Key Features

Automatic Landing: Triggers when altitude > 20 m
Pilot Override Protection: Prevents re-triggering if pilot manually intervenes
Ground Contact Reset: Resets only after confirmed touchdown

Monitored Topics

/fmu/out/vehicle_local_position → altitude monitoring
/fmu/out/vehicle_land_detected → ground contact detection
/fmu/out/manual_control_setpoint → throttle input detection

Published Commands

/fmu/in/vehicle_command → NAV_LAND command

Control Logic
Triggering Condition
Auto-landing activates only if:

altitude_enu > 20 m
ready_to_trigger == true

Pilot Override
If pilot moves throttle during descent:

Auto-landing stops immediately
System cannot restart until real touchdown

Ground Contact Reset
Once VehicleLandDetected.ground_contact == true:

System resets to initial state
Ready for next automatic landing cycle

Launch Sequence
Terminal 1 - Start drone simulation:
bashcd PX4-Autopilot
make px4_sitl gz_ciccio_drone
Terminal 2 - Start Micro XRCE-DDS bridge:
bashMicroXRCEAgent udp4 -p 8888
Terminal 3 - Build and run ForceLand node:
bashcd ~/ros2_ws
colcon build --packages-select force_land
source install/setup.bash
ros2 run force_land force_land_node
Terminal 4 - Arm and fly the drone (PX4 console in Terminal 1):
bashcommander arm
commander takeoff
# Wait for auto-landing at 20m, or manually control with RC/QGC
Data Recording for Analysis
Record flight data for plotting:
bashros2 bag record /fmu/out/vehicle_local_position /fmu/out/manual_control_setpoint /fmu/out/vehicle_land_detected
Required Analysis Plots
1. UAV Altitude (ENU Frame)
Compute from: altitude_enu = -vehicle_local_position.z
2. Manual Throttle Setpoint
Plot: manual_control_setpoint.throttle
3. Ground Contact Status
Plot: vehicle_land_detected.ground_contact
Plots must demonstrate:

✅ Auto-landing triggers correctly at > 20 m
✅ Pilot override prevents re-triggering
✅ Reset occurs only after ground contact


4. Multi-Waypoint Trajectory Planner (3.a)
Autonomous trajectory execution system using offboard control mode.
Launch Sequence
Terminal 1 - Start drone simulation:
bashcd PX4-Autopilot
make px4_sitl gz_ciccio_drone
Terminal 2 - Start Micro XRCE-DDS bridge:
bashMicroXRCEAgent udp4 -p 8888
Terminal 3 - Build and run planner:
bashcd ~/ros2_ws
colcon build --packages-select offboard_rl
source install/setup.bash
ros2 run offboard_rl multi_waypoint_planner
Waypoint Input Format
The terminal displays an interactive menu. Input waypoints in the format:
x y z yaw
Where:

x, y, z: Position coordinates in meters (NED frame)
yaw: Orientation in radians

Example Test Trajectory
Square pattern with 7 waypoints:
bash5 0 3 0
5 5 3 1.57
0 5 3 3.14
-5 5 3 -1.57
-5 0 3 0
0 0 3 0
0 0 0 0
done
5
The last line (5) specifies 5 seconds as uniform segment duration.
How It Works

Node enters offboard mode and arms the drone automatically
Executes waypoints sequentially with smooth transitions
Returns to home position (0, 0, 0) at the end
Lands and disarms


📁 Project Structure
.
├── PX4-Autopilot/
│   ├── Tools/simulation/gz/models/
│   │   └── ciccio_drone/
│   │       ├── model.config
│   │       ├── model.sdf
│   │       └── meshes/propeller.dae
│   ├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   │   ├── 4022_gz_ciccio_drone
│   │   └── CMakeLists.txt
│   └── src/modules/uxrce_dds_client/
│       └── dds_topics.yaml
├── ros2_ws/
│   └── src/
│       ├── force_land/
│       │   └── src/force_land_node.cpp
│       └── offboard_rl/
│           └── src/multi_waypoint_planner.cpp
└── README.md

🐛 Troubleshooting
Drone doesn't spawn in Gazebo
bashcd PX4-Autopilot
make clean
make px4_sitl gz_ciccio_drone
ROS 2 topics not visible
Check if Micro XRCE-DDS Agent is running:
bashps aux | grep MicroXRCEAgent
Actuator outputs not publishing
Verify dds_topics.yaml configuration and recompile PX4:
bashcd PX4-Autopilot
make clean
make px4_sitl gz_ciccio_drone
ForceLand node doesn't trigger landing
Check altitude topic:
bashros2 topic echo /fmu/out/vehicle_local_position

📄 License
This project is part of a university assignment. Please refer to your institution's policies regarding code sharing and reuse.

👥 Contributors
[Add your names and student IDs here]

🔗 References

PX4 Autopilot Documentation
PX4 User Guide - SITL Simulation
ROS 2 Humble Documentation
PX4-ROS 2 Bridge
Gazebo Harmonic Documentation
QGroundControl User Guide
