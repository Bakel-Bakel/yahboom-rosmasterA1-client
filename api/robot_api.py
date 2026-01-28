#!/usr/bin/env python3
"""
Robot-side API server.
This API runs on the robot and receives commands from mission control.
It executes ROS2 commands locally on the robot.
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import os
import signal
from pathlib import Path
from datetime import datetime

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Store current domain ID
current_domain_id = None

# Store active processes
active_processes = {
    'navigation': None,
    'slam': None,
    'explorer': None,
    'rosbridge': None
}


def detect_ros2_setup():
    """Detect ROS 2 installation and return path to setup.bash."""
    ros2_distros = ['humble', 'iron', 'jazzy', 'rolling', 'galactic', 'foxy']
    
    for distro in ros2_distros:
        setup_path = Path(f'/opt/ros/{distro}/setup.bash')
        if setup_path.exists():
            return str(setup_path)
    
    # Check ROS_DISTRO environment variable if set
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        setup_path = Path(f'/opt/ros/{ros_distro}/setup.bash')
        if setup_path.exists():
            return str(setup_path)
    
    # Fallback: try to find any ROS 2 installation
    ros_dir = Path('/opt/ros')
    if ros_dir.exists():
        distros = sorted([d for d in ros_dir.iterdir() if d.is_dir()])
        if distros:
            setup_path = distros[0] / 'setup.bash'
            if setup_path.exists():
                return str(setup_path)
    
    return None


def set_ros_domain_id(domain_id):
    """Set ROS_DOMAIN_ID environment variable for this process and export it."""
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    # Also write to a file that can be sourced by other processes
    domain_file = Path.home() / '.ros_domain_id'
    try:
        with open(domain_file, 'w') as f:
            f.write(f'export ROS_DOMAIN_ID={domain_id}\n')
        print(f'ROS_DOMAIN_ID={domain_id} written to {domain_file}')
    except Exception as e:
        print(f'Warning: Could not write domain ID to file: {e}')
    
    return True


@app.route('/api/set_domain_id', methods=['POST'])
def set_domain_id():
    """Receive and set ROS_DOMAIN_ID from mission control.
    
    This sets the ROS_DOMAIN_ID environment variable for the robot.
    All ROS2 commands run after this will use this domain ID.
    """
    try:
        data = request.get_json() or {}
        domain_id = data.get('domain_id')
        
        if domain_id is None:
            return jsonify({
                'success': False,
                'error': 'domain_id is required'
            }), 400
        
        # Validate domain ID (0-232)
        try:
            domain_id_int = int(domain_id)
            if domain_id_int < 0 or domain_id_int > 232:
                return jsonify({
                    'success': False,
                    'error': 'domain_id must be between 0 and 232'
                }), 400
        except (ValueError, TypeError):
            return jsonify({
                'success': False,
                'error': 'domain_id must be a valid integer'
            }), 400
        
        # Set ROS_DOMAIN_ID
        global current_domain_id
        current_domain_id = domain_id_int
        set_ros_domain_id(domain_id_int)
        
        print(f'ROS_DOMAIN_ID set to {domain_id_int}')
        
        return jsonify({
            'success': True,
            'message': f'ROS_DOMAIN_ID set to {domain_id_int}',
            'domain_id': domain_id_int
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/get_domain_id', methods=['GET'])
def get_domain_id():
    """Get current ROS_DOMAIN_ID."""
    global current_domain_id
    env_domain_id = os.environ.get('ROS_DOMAIN_ID')
    
    return jsonify({
        'success': True,
        'domain_id': current_domain_id if current_domain_id is not None else env_domain_id,
        'from_env': env_domain_id
    }), 200


@app.route('/api/navigation/start', methods=['POST'])
def start_navigation():
    """Start navigation stack on robot.
    
    Note: Robot-specific implementation will be added later based on robot brand.
    """
    try:
        data = request.get_json() or {}
        robot_ip = request.remote_addr
        
        # TODO: Implement robot-specific navigation start command
        # This will vary by robot brand (e.g., TurtleBot, Limo, etc.)
        
        return jsonify({
            'success': True,
            'message': 'Navigation start command received',
            'robot_ip': robot_ip,
            'domain_id': current_domain_id,
            'note': 'Robot-specific navigation implementation pending'
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/navigation/stop', methods=['POST'])
def stop_navigation():
    """Stop navigation stack."""
    try:
        if active_processes['navigation'] is None:
            return jsonify({
                'success': False,
                'error': 'Navigation is not running'
            }), 400
        
        process = active_processes['navigation']
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except (ProcessLookupError, OSError):
            pass
        
        active_processes['navigation'] = None
        
        return jsonify({
            'success': True,
            'message': 'Navigation stop command received',
            'robot_ip': robot_ip,
            'note': 'Robot-specific navigation implementation pending'
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/slam/start', methods=['POST'])
def start_slam():
    """Start SLAM on robot.
    
    Note: Robot-specific implementation will be added later based on robot brand.
    """
    try:
        robot_ip = request.remote_addr
        
        # TODO: Implement robot-specific SLAM start command
        
        return jsonify({
            'success': True,
            'message': 'SLAM start command received',
            'robot_ip': robot_ip,
            'domain_id': current_domain_id,
            'note': 'Robot-specific SLAM implementation pending'
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/slam/stop', methods=['POST'])
def stop_slam():
    """Stop SLAM.
    
    Note: Robot-specific implementation will be added later based on robot brand.
    """
    try:
        robot_ip = request.remote_addr
        
        # TODO: Implement robot-specific SLAM stop command
        
        return jsonify({
            'success': True,
            'message': 'SLAM stop command received',
            'robot_ip': robot_ip,
            'note': 'Robot-specific SLAM implementation pending'
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint."""
    global current_domain_id
    return jsonify({
        'status': 'healthy',
        'service': 'robot_api',
        'current_domain_id': current_domain_id,
        'ros_domain_id_env': os.environ.get('ROS_DOMAIN_ID')
    }), 200


if __name__ == '__main__':
    print('Starting Robot API Server...')
    print('API endpoints:')
    print('  POST /api/set_domain_id - Set ROS_DOMAIN_ID')
    print('  GET  /api/get_domain_id - Get current ROS_DOMAIN_ID')
    print('  POST /api/navigation/start - Start navigation (skeleton)')
    print('  POST /api/navigation/stop - Stop navigation (skeleton)')
    print('  POST /api/slam/start - Start SLAM (skeleton)')
    print('  POST /api/slam/stop - Stop SLAM (skeleton)')
    print('  GET  /health - Health check')
    print('\nServer starting on http://0.0.0.0:8004')
    print('NOTE: Robot-specific ROS2 commands will be implemented later based on robot brand.')
    
    app.run(host='0.0.0.0', port=8004, debug=False, use_reloader=False)
