# Robot Client API - ROSMASTER-A1

This folder contains the robot-side API server for ROSMASTER-A1 that receives commands from mission control and executes ROS2 commands locally on the robot.

## Key Features

- **Automatic rosbridge Installation**: On first run, the API automatically installs `ros-humble-rosbridge-server`
- **Repository Troubleshooting**: If installation fails due to outdated repositories, it automatically:
  1. Installs nano
  2. Updates apt repositories
  3. Installs curl, gnupg, lsb-release
  4. Adds ROS GPG key
  5. Adds ROS repository
  6. Updates apt again
  7. Installs ros-humble-rosbridge-server

## Setup

1. Install dependencies:
```bash
cd robot_client/Yahboom/ROSMASTER-A1/api
pip3 install -r requirements.txt
```

2. Run the robot API server:
```bash
python3 robot_api.py
```

The API server will:
- First check and install rosbridge-server if needed
- Start on port 8004 by default

## API Endpoints

### Domain ID Management
- `POST /api/set_domain_id` - Set ROS_DOMAIN_ID (called by mission control)
- `GET /api/get_domain_id` - Get current ROS_DOMAIN_ID

### ROSBridge Server (Expose ROS2)
- `POST /api/rosbridge/start` - Start rosbridge_server to expose ROS2 via WebSocket
- `POST /api/rosbridge/stop` - Stop rosbridge_server
- `GET /api/rosbridge/status` - Get rosbridge_server status

### Navigation
- `POST /api/navigation/start` - Start navigation driver (runs `ros2 run yahboomcar_bringup Ackman_driver_A1`)
- `POST /api/navigation/stop` - Stop navigation driver
- `GET /api/navigation/status` - Get navigation driver status

### SLAM
- `POST /api/slam/start` - Start SLAM
- `POST /api/slam/stop` - Stop SLAM

### Health
- `GET /health` - Health check (includes rosbridge installation and running status)

## How It Works

1. **On Startup**: The API automatically checks if `ros-humble-rosbridge-server` is installed
2. **Installation**: If not installed, it attempts to install it:
   - First tries direct installation: `sudo apt install ros-humble-rosbridge-server`
   - If that fails (due to old repos), it applies troubleshooting steps:
     - Installs nano
     - Updates apt
     - Installs curl, gnupg, lsb-release
     - Adds ROS GPG key
     - Adds ROS repository
     - Updates apt again
     - Installs ros-humble-rosbridge-server
3. **Mission Control Connection**: Mission control assigns a ROS_DOMAIN_ID to each robot (1, 2, 3, etc.)
4. **Domain ID Assignment**: Mission control sends the domain ID via `POST /api/set_domain_id`
5. **ROS Environment**: Robot API sets `ROS_DOMAIN_ID` environment variable
6. **ROS2 Commands**: All subsequent ROS2 commands use this domain ID
7. **Isolation**: This isolates robots on the same network, preventing topic conflicts
8. **Expose ROS2**: Use `POST /api/rosbridge/start` to launch rosbridge_server, which exposes ROS2 topics via WebSocket on port 9090
   - The rosbridge_server will use the current ROS_DOMAIN_ID
   - If domain ID changes while rosbridge is running, it will automatically restart with the new domain ID

## Configuration

The robot API runs on port 8004 by default. To change this, modify the `port` parameter in `robot_api.py`.

## Notes

- The robot API must be running on the robot before mission control can send commands
- ROS_DOMAIN_ID is set as an environment variable and written to `~/.ros_domain_id` for persistence
- **Navigation driver is implemented** - Uses `ros2 run yahboomcar_bringup Ackman_driver_A1` to start the navigation stack
- **SLAM commands are not yet implemented** - these will be added later based on robot brand
- The API currently provides skeleton endpoints that return success responses but don't execute robot-specific commands
- The rosbridge installation process requires sudo privileges and may prompt for password
- **rosbridge_server** exposes ROS2 topics via WebSocket on port 9090 (default)
- When ROS_DOMAIN_ID is changed via `/api/set_domain_id`, rosbridge will automatically restart with the new domain ID if it's currently running
- Use `POST /api/rosbridge/start` to launch rosbridge after installation, or start it manually: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- **Navigation Driver**: Use `POST /api/navigation/start` to start the navigation driver (`ros2 run yahboomcar_bringup Ackman_driver_A1`). This enables robot control from the BiaLogic dashboard.
- The navigation driver uses the current ROS_DOMAIN_ID and will run in the background

## Troubleshooting

### Installation Fails
If rosbridge installation fails:
1. Check that you have sudo privileges
2. Verify internet connectivity
3. Manually run the installation commands:
   ```bash
   sudo apt install nano
   sudo apt update
   sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
   sudo apt update
   sudo apt install ros-humble-rosbridge-server
   ```

### Port Already in Use
If port 8004 is already in use:
- Change the port in `robot_api.py` (last line)
- Or stop the conflicting service
