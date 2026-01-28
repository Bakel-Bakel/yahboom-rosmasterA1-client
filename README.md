# Robot Client API

This folder contains the robot-side API server that receives commands from mission control and executes ROS2 commands locally on the robot.

## Setup

1. Install dependencies:
```bash
cd robot_client/api
pip3 install -r requirements.txt
```

2. Run the robot API server:
```bash
python3 robot_api.py
```

The API server will start on port 8004 by default.

## API Endpoints

### Domain ID Management
- `POST /api/set_domain_id` - Set ROS_DOMAIN_ID (called by mission control)
- `GET /api/get_domain_id` - Get current ROS_DOMAIN_ID

### Navigation
- `POST /api/navigation/start` - Start navigation stack
- `POST /api/navigation/stop` - Stop navigation stack

### SLAM
- `POST /api/slam/start` - Start SLAM
- `POST /api/slam/stop` - Stop SLAM

### Health
- `GET /health` - Health check

## How It Works

1. Mission control assigns a ROS_DOMAIN_ID to each robot (1, 2, 3, etc.)
2. Mission control sends the domain ID to the robot via `POST /api/set_domain_id`
3. Robot API sets `ROS_DOMAIN_ID` environment variable
4. All subsequent ROS2 commands use this domain ID
5. This isolates robots on the same network, preventing topic conflicts

## Configuration

The robot API runs on port 8004 by default. To change this, modify the `port` parameter in `robot_api.py`.

## Notes

- The robot API must be running on the robot before mission control can send commands
- ROS_DOMAIN_ID is set as an environment variable and written to `~/.ros_domain_id` for persistence
- **Robot-specific ROS2 commands (navigation, SLAM, etc.) are not yet implemented** - these will be added later based on robot brand
- The API currently provides skeleton endpoints that return success responses but don't execute robot-specific commands
