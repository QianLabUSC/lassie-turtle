# LASSIE Turtle Robot

Software distribution for the LASSIE (Legged Autonomous Surface Science In Analogue Environments) turtle robot system.
This branch features adaptive gait capabilities. 

## Project Structure

### 1. `dependencies/` - Low-level ROS2 packages
   - **`traveler_controller/`** - C++ ROS2 package (`turtle_high_controller`) that handles:
     - CAN bus communication with ODrive motor controllers
     - Inverse kinematics and trajectory generation
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
   - `host` - Raspberry Pi IP address (default: `192.168.8.203`)
   - `remote_dir` - Remote directory path (default: `/home/ubuntu/roboland`)

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
cd ~/roboland/dependencies  # or wherever you deployed it
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

**Option 0: Using ros2 run (Recommended)**
```bash
ros2 run turtle_high_controller turtle_high_controller
```

This launches the `turtle_high_controller` node directly, which handles:
- Low-level CAN communication and trajectory control
- Inverse kinematics and PID speed control

**Option 1: Using main.py** (deprecated)
```bash
cd highlevel
python3 main.py
```

**Option 2: Using rasp_main.py** (deprecated)
```bash
cd highlevel
python3 rasp_main.py
```

Both deprecated scripts launch:
- `turtle_high_controller` - Low-level CAN communication and trajectory control
- Additional nodes (Optitrack for main.py, video sync for rasp_main.py)

### On Laptop (Custom GUI)

Run the custom LASSIE GUI on your laptop to control the robot:

```bash
cd highlevel
python3 LASSIE_GUI/lassie_gui.py
```

This launches the Kivy-based custom GUI that communicates with the robot via ROS2 topics over the network.

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
