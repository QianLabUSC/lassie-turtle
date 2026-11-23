# Gantry world-model data collection

This branch targets the duplicated single-flipper system: **can0**, **adduction node 2**,
**sweeping node 3**, classic CAN at **1,000,000 bit/s**. Latest observed Wi-Fi IP:
`192.168.10.28` (DHCP can change it). No ODrive serial numbers are required.

## Deploy from your development computer

Run from your local `lassie-turtle` checkout:

```bash
./deploy_dependencies.sh
```

This creates `/home/ubuntu/lassie-turtle/dependencies` if needed and copies the
dependency sources there. It excludes local `build/`, `install/`, and `log/`
directories so the Pi can compile its own binaries. No `roboland` directory is
required. Override a changed IP with `PI_HOST=192.168.10.28 ./deploy_dependencies.sh`.
The script uses normal SSH authentication; it does not require `~/.ssh/id_rsa`.

Only the Pi needs to build the motor controller. Build on a Linux laptop too
only if you need the custom ROS messages or nodes locally on that laptop.

## Build on the Pi

Clone this branch on the Pi, then run from the repository root:

```bash
sudo apt update
sudo apt install build-essential python3-colcon-common-extensions ros-humble-control-msgs can-utils
source /opt/ros/humble/setup.bash
cd dependencies
colcon build --executor sequential
source install/setup.bash
```

For every new SSH terminal, source ROS and the workspace setup again. For a clone in
`~/lassie-turtle`, run:

```bash
source /opt/ros/humble/setup.bash
source ~/lassie-turtle/dependencies/install/setup.bash
```

## Run a trajectory

On the Pi, enable CAN after each reboot (ODrives must also use 1 Mbps):

```bash
sudo ip link set can0 up type can bitrate 1000000
```

If the original robot is running on the same network, use a distinct ROS domain.
For example, run `export ROS_DOMAIN_ID=23` in **every terminal** controlling this
system, including the controller terminal. Use that same domain on a laptop
when publishing directly from the laptop instead of through SSH.

Connect with `ssh ubuntu@192.168.10.28`. In the first sourced terminal:

```bash
ros2 run turtle_high_controller turtle_high_controller
```

In a second sourced terminal, publish a trajectory to `/trajectory_points` using
`std_msgs/msg/Float64MultiArray`. Each triple is **[adduction radians, sweep radians,
positive interpolation speed in rad/s]**. The existing random, circle, and fixed
trajectory arrays use this format. These are joint-angle paths, not Cartesian
positions in metres.

**Loading waypoints waits for Start.** Then run:

```bash
# Start the loaded trajectory
ros2 topic pub --once /Gui_information std_msgs/msg/Float64MultiArray '{data: [1.0, 0.0]}'
# Stop: request ODrive idle (releases motor torque)
ros2 topic pub --once /Gui_information std_msgs/msg/Float64MultiArray '{data: [0.0, 0.0]}'
```

The path starts at the first supplied waypoint; it does not ramp there from the
measured position. Confirm the new hardware has the same mechanical zero,
rotation directions and effective gearing before using the old full trajectories.
The conversion remains `motor_turns = -joint_radians / (2*pi)` for both joints.
Completion clears Start and releases control; `/trajectory_complete` reports it.
This software stop is not a hardware emergency stop.

Encoder feedback should be enabled on both ODrives. Iq feedback is optional for
motion timing because CAN reads are nonblocking; current values remain at their
last/default values if Iq messages are disabled. This controller does not yet
stop automatically on stale feedback or check ODrive errors.

The controller is `turtle_high_controller`; the separate
`odrivepro_can_interface` package mentioned in older notes is not needed for this
execution path. Installing desktop environments is not required.

---

# LASSIE Turtle Robot

Software distribution for the LASSIE (Legged Autonomous Surface Science In Analogue Environments) turtle robot system.

## Project Structure of Trajectory Selection

### 1. `dependencies/` - Low-level ROS2 packages
   - **`traveler_controller/`** - C++ ROS2 package (`turtle_high_controller`) that handles:
     - CAN bus communication with ODrive motor controllers
     - Inverse kinematics and trajectory generation
        - only using Inverse_kinematics.cpp, trajectories_parser.cpp, controller_monitor.cpp, pid_speed_controller.cpp
     - PID speed control
     - Controller monitoring
   - **`traveler_msgs/`** - ROS2 message and service definitions for:
     - Traveler status, configuration, and mode messages
     - ODrive status messages
     - ODrive service definitions (control, configuration, diagnostics)

### 2. `highlevel/` - High-level Python control and GUI
   - **`main.py`** - Launcher for Raspberry Pi (launches controller and Optitrack)
   - **`rasp_main.py`** - Alternative launcher for Raspberry Pi (launches controller and video sync)
   - **`LASSIE_GUI/`** - Kivy-based custom graphical user interface (runs on laptop, not Raspberry Pi)
     - `lassie_gui.py` - Main custom GUI application (runs on laptop)
     - `ros2_interface_turtle.py` - ROS2 interface for turtle robot
   - **`traveler_optitrack/`** - Optitrack motion capture system integration
   - **`config/`** - Configuration files
   - **`experiment_data/`** - Experimental data storage

### 3. `dataanalysis/` - Data analysis tools
   - **`GUI_plotter.py`** - GUI tool for analyzing force/position data from experiments
   - Analysis scripts and visualization tools

## Prerequisites

- ROS2 (Humble or compatible version)
- Python 3.8+
- Required Python packages (see `highlevel/depend.txt`):
  - pyserial==3.5
  - kivy==2.1.0
  - kivymd==0.104.2

## Setup Instructions

### 1. Deploy Dependencies to Raspberry Pi

1. **Configure deployment script**: Edit `deploy_dependencies.sh` to set:
   - `user` - SSH username (default: `ubuntu`)
   - `host` - Raspberry Pi IP address (default: `192.168.10.28`)
   - `remote_dir` - Remote directory path (default: `/home/ubuntu/lassie-turtle`)

2. **Deploy dependencies folder to Raspberry Pi**:
   ```bash
   ./deploy_dependencies.sh
   ```
   This copies the `dependencies/` folder to the Raspberry Pi.

3. **Connect to Raspberry Pi**:
   ```bash
   ./enter_turtle.sh
   ```
   (Edit `enter_turtle.sh` to match your SSH credentials if needed)

### 2. Build ROS2 Dependencies on Raspberry Pi

Once connected to the Raspberry Pi, navigate to the dependencies folder and build:

```bash
cd ~/lassie-turtle/dependencies
./build.sh
```

This will:
- Build the `traveler_msgs` and `traveler_controller` packages using colcon
- Source the setup script to make messages available
- Add the source command to your `~/.bashrc`

### 3. Configure ROS2 Domain ID

**IMPORTANT**: For ROS2 communication to work between the laptop and Raspberry Pi, they must:
1. Be on the same network
2. Have the same ROS domain ID set

Set the ROS domain ID on both systems:

**On Raspberry Pi:**
```bash
export ROS_DOMAIN_ID=0  # or your chosen domain ID
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

**On Laptop:**
```bash
export ROS_DOMAIN_ID=0  # must match the Raspberry Pi domain ID
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

The default domain ID is 0, but you can use any number (0-232) as long as both systems use the same value.

## Running the System

### On Raspberry Pi

The Raspberry Pi runs the low-level controller and other ROS2 nodes.

**1. Using ros2 run (Recommended)**
```bash
ros2 run turtle_high_controller turtle_high_controller
```

This launches the `turtle_high_controller` node directly, which handles:
- Low-level CAN communication and trajectory control
- Inverse kinematics and PID speed control

**2. Send Waypoints** 
```bash
ros2 topic pub --once /trajectory_points std_msgs/msg/Float64MultiArray \
		"{data: [
		  -0.4,  0.0, 0.5,
		   0.4,  0.0, 0.5,
		   0.0, -0.4, 0.5,
		   0.2,  0.3, 0.5,
		  -0.2,  0.3, 0.5
		]}"
```

**3. start/stop run** 
```bash
// Start motion
ros2 topic pub --once /Gui_information std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0]}"

// stop motion
ros2 topic pub --once /Gui_information std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

### Running Individual Components

**Data analysis (on laptop):**
```bash
cd dataanalysis
python3 GUI_plotter.py
```

Note: For running the low-level controller on Raspberry Pi, use Option 3 above.


## Hardware Requirements

- **Turtle Robot**: Dynamixel servo motors (for turtle robot variant)
- **CAN Communication**: ODrive motor controllers (for CAN-based control)
- **Network**: Same network connection for remote monitoring/control

## Configuration

- Robot configuration files are stored in `highlevel/config/`
- Last used configuration is saved in `last_config.csv`
- Modify configuration through the GUI or directly edit CSV files

## System Architecture

- **Raspberry Pi**: Runs `main.py` or `rasp_main.py` which launch the low-level controller and other ROS2 nodes
- **Laptop**: Runs `lassie_gui.py` (custom GUI) to control the robot via ROS2 topics over network
- **Communication**: Both systems communicate via ROS2 topics when on the same network and using the same ROS domain ID

## Notes

- The `traveler_controller` package is named `turtle_high_controller` in the ROS2 workspace to differentiate with traverler robot
- Ensure proper permissions for serial/USB devices on Raspberry Pi: `sudo chown $USER /dev/ttyUSB0` (or appropriate device) (if use serial communication)
- The system uses ROS2 topics for inter-node communication between Raspberry Pi and laptop
- Experimental data is automatically saved to `highlevel/experiment_data/turtle/`
- **Critical for communication**: Both Raspberry Pi and laptop must be:
  - On the same network
  - Using the same ROS domain ID (set via `ROS_DOMAIN_ID` environment variable) 