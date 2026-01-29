#!/usr/bin/env python3
"""
Robot-side API server for ROSMASTER-A1.
This API runs on the robot and receives commands from mission control.
It executes ROS2 commands locally on the robot.
On first run, it automatically installs rosbridge-server.
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import os
import signal
from pathlib import Path
from datetime import datetime
import sys
import requests
import tempfile
import shutil
import pwd
import socket

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# Store current domain ID
current_domain_id = None

# Store active processes
active_processes = {
    'navigation': None,
    'slam': None,
    'explorer': None,
    'rosbridge': None,
    'digital_twin_camera': None,
    'digital_twin_mapping': None,
    'digital_twin_display': None
}


def run_command(cmd, shell=True, check=True, capture_output=False):
    """Run a shell command and return the result."""
    try:
        if capture_output:
            result = subprocess.run(
                cmd,
                shell=shell,
                check=check,
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout
            )
            return result.returncode == 0, result.stdout, result.stderr
        else:
            result = subprocess.run(
                cmd,
                shell=shell,
                check=check,
                timeout=300
            )
            return result.returncode == 0, "", ""
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out after 5 minutes"
    except subprocess.CalledProcessError as e:
        return False, "", str(e)
    except Exception as e:
        return False, "", str(e)


def check_rosbridge_installed():
    """Check if ros-humble-rosbridge-server is installed."""
    success, _, _ = run_command(
        "dpkg -l | grep -q ros-humble-rosbridge-server",
        check=False,
        capture_output=True
    )
    return success


def install_rosbridge():
    """Install ros-humble-rosbridge-server with fallback for old repos."""
    print("=" * 60)
    print("Installing ros-humble-rosbridge-server...")
    print("=" * 60)
    
    # First, check if already installed
    if check_rosbridge_installed():
        print("✓ ros-humble-rosbridge-server is already installed")
        return True
    
    # Try direct installation first
    print("\n[1/2] Attempting direct installation...")
    success, stdout, stderr = run_command(
        "sudo apt install -y ros-humble-rosbridge-server",
        check=False,
        capture_output=True
    )
    
    if success:
        print("✓ Successfully installed ros-humble-rosbridge-server")
        return True
    
    # If direct installation failed, try troubleshooting steps
    print("\n[2/2] Direct installation failed. Applying troubleshooting steps...")
    print("This may be due to outdated repositories.")
    
    # Step 1: Install nano first
    print("\n  → Installing nano...")
    success, _, _ = run_command(
        "sudo apt install -y nano",
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to install nano")
        return False
    print("  ✓ nano installed")
    
    # Step 2: Update apt
    print("\n  → Updating apt repositories...")
    success, _, _ = run_command(
        "sudo apt update",
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to update apt")
        return False
    print("  ✓ apt updated")
    
    # Step 3: Install curl, gnupg, lsb-release
    print("\n  → Installing curl, gnupg, lsb-release...")
    success, _, _ = run_command(
        "sudo apt install -y curl gnupg lsb-release",
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to install dependencies")
        return False
    print("  ✓ Dependencies installed")
    
    # Step 4: Add ROS GPG key
    print("\n  → Adding ROS GPG key...")
    success, _, _ = run_command(
        "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg",
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to add ROS GPG key")
        return False
    print("  ✓ ROS GPG key added")
    
    # Step 5: Add ROS repository
    print("\n  → Adding ROS repository...")
    # Get the distribution codename
    success, codename, _ = run_command(
        "lsb_release -cs",
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to get distribution codename")
        return False
    
    codename = codename.strip()
    repo_cmd = f'echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu {codename} main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null'
    
    success, _, _ = run_command(
        repo_cmd,
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to add ROS repository")
        return False
    print("  ✓ ROS repository added")
    
    # Step 6: Update apt again
    print("\n  → Updating apt repositories again...")
    success, _, _ = run_command(
        "sudo apt update",
        check=False,
        capture_output=True
    )
    if not success:
        print("  ✗ Failed to update apt")
        return False
    print("  ✓ apt updated")
    
    # Step 7: Install ros-humble-rosbridge-server
    print("\n  → Installing ros-humble-rosbridge-server...")
    success, stdout, stderr = run_command(
        "sudo apt install -y ros-humble-rosbridge-server",
        check=False,
        capture_output=True
    )
    
    if success:
        print("  ✓ Successfully installed ros-humble-rosbridge-server")
        return True
    else:
        print("  ✗ Failed to install ros-humble-rosbridge-server")
        print(f"  Error output: {stderr}")
        return False


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


def start_rosbridge():
    """Start rosbridge_server using ros2 launch."""
    global active_processes
    
    # Check if already running
    if active_processes['rosbridge'] is not None:
        if active_processes['rosbridge'].poll() is None:
            print("rosbridge_server is already running")
            return True, active_processes['rosbridge'].pid, None
        else:
            # Process died, reset it
            active_processes['rosbridge'] = None
    
    # Check if rosbridge is installed
    if not check_rosbridge_installed():
        return False, None, "rosbridge-server is not installed. Please install it first."
    
    # Detect ROS2 setup
    ros2_setup = detect_ros2_setup()
    if not ros2_setup:
        return False, None, "ROS2 setup.bash not found. Please ensure ROS2 is installed."
    
    # Get current domain ID
    domain_id = current_domain_id if current_domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0')
    
    # Build the command to source ROS2 and launch rosbridge
    # We need to source ROS2 setup, set ROS_DOMAIN_ID, and then launch rosbridge
    launch_cmd = f'source {ros2_setup} && export ROS_DOMAIN_ID={domain_id} && ros2 launch rosbridge_server rosbridge_websocket_launch.xml'
    
    try:
        # Start rosbridge in background
        # Use bash -c to properly handle sourcing and environment variables
        process = subprocess.Popen(
            ['bash', '-c', launch_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create new process group
        )
        
        active_processes['rosbridge'] = process
        
        # Wait a moment to check if it started successfully
        import time
        time.sleep(2)
        
        if process.poll() is None:
            print(f"✓ rosbridge_server started successfully (PID: {process.pid}, ROS_DOMAIN_ID: {domain_id})")
            return True, process.pid, None
        else:
            # Process died immediately, get error
            stdout, stderr = process.communicate()
            error_msg = stderr.decode('utf-8') if stderr else stdout.decode('utf-8')
            active_processes['rosbridge'] = None
            return False, None, f"rosbridge_server failed to start: {error_msg}"
            
    except Exception as e:
        return False, None, f"Error starting rosbridge_server: {str(e)}"


def stop_rosbridge():
    """Stop rosbridge_server process."""
    global active_processes
    
    if active_processes['rosbridge'] is None:
        return False, "rosbridge_server is not running"
    
    process = active_processes['rosbridge']
    
    try:
        # Kill the process group
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
        active_processes['rosbridge'] = None
        print(f"✓ rosbridge_server stopped (PID: {process.pid})")
        return True, None
    except subprocess.TimeoutExpired:
        # Force kill if it doesn't stop
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        except (ProcessLookupError, OSError):
            pass
        active_processes['rosbridge'] = None
        print(f"✓ rosbridge_server force stopped (PID: {process.pid})")
        return True, None
    except (ProcessLookupError, OSError) as e:
        active_processes['rosbridge'] = None
        return False, f"Error stopping rosbridge_server: {str(e)}"


@app.route('/api/set_domain_id', methods=['POST'])
def set_domain_id():
    """Receive and set ROS_DOMAIN_ID from mission control.
    
    This sets the ROS_DOMAIN_ID environment variable for the robot.
    All ROS2 commands run after this will use this domain ID.
    If rosbridge is running, it will be restarted with the new domain ID.
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
        
        # If rosbridge is running, stop it first (will restart with new domain ID)
        rosbridge_was_running = False
        if active_processes['rosbridge'] is not None and active_processes['rosbridge'].poll() is None:
            rosbridge_was_running = True
            stop_rosbridge()
        
        # Set ROS_DOMAIN_ID
        global current_domain_id
        current_domain_id = domain_id_int
        set_ros_domain_id(domain_id_int)
        
        print(f'ROS_DOMAIN_ID set to {domain_id_int}')
        
        # Restart rosbridge if it was running
        if rosbridge_was_running:
            success, pid, error = start_rosbridge()
            if success:
                return jsonify({
                    'success': True,
                    'message': f'ROS_DOMAIN_ID set to {domain_id_int} and rosbridge restarted',
                    'domain_id': domain_id_int,
                    'rosbridge_restarted': True,
                    'rosbridge_pid': pid
                }), 200
            else:
                return jsonify({
                    'success': True,
                    'message': f'ROS_DOMAIN_ID set to {domain_id_int}, but rosbridge restart failed',
                    'domain_id': domain_id_int,
                    'rosbridge_restart_error': error
                }), 200
        
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
    
    For ROSMASTER-A1, this runs: ros2 run yahboomcar_bringup Ackman_driver_A1
    This starts up the navigation driver so the robot can be controlled from BiaLogic dashboard.
    """
    try:
        data = request.get_json() or {}
        robot_ip = request.remote_addr
        
        # Check if already running
        if active_processes['navigation'] is not None:
            if active_processes['navigation'].poll() is None:
                return jsonify({
                    'success': True,
                    'message': 'Navigation is already running',
                    'robot_ip': robot_ip,
                    'domain_id': current_domain_id,
                    'pid': active_processes['navigation'].pid
                }), 200
            else:
                # Process died, reset it
                active_processes['navigation'] = None
        
        # Detect ROS2 setup
        ros2_setup = detect_ros2_setup()
        if not ros2_setup:
            return jsonify({
                'success': False,
                'error': 'ROS2 setup.bash not found. Please ensure ROS2 is installed.'
            }), 500
        
        # Get current domain ID
        domain_id = current_domain_id if current_domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0')
        
        # Build the command to source ROS2 and run navigation driver
        nav_cmd = f'source {ros2_setup} && export ROS_DOMAIN_ID={domain_id} && ros2 run yahboomcar_bringup Ackman_driver_A1'
        
        try:
            # Start navigation driver in background
            process = subprocess.Popen(
                ['bash', '-c', nav_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            active_processes['navigation'] = process
            
            # Wait a moment to check if it started successfully
            import time
            time.sleep(2)
            
            if process.poll() is None:
                print(f"✓ Navigation driver started successfully (PID: {process.pid}, ROS_DOMAIN_ID: {domain_id})")
                return jsonify({
                    'success': True,
                    'message': 'Navigation driver started successfully',
                    'robot_ip': robot_ip,
                    'domain_id': domain_id,
                    'pid': process.pid,
                    'command': 'ros2 run yahboomcar_bringup Ackman_driver_A1'
                }), 200
            else:
                # Process died immediately, get error
                stdout, stderr = process.communicate()
                error_msg = stderr.decode('utf-8') if stderr else stdout.decode('utf-8')
                active_processes['navigation'] = None
                return jsonify({
                    'success': False,
                    'error': f'Navigation driver failed to start: {error_msg}',
                    'robot_ip': robot_ip
                }), 500
                
        except Exception as e:
            return jsonify({
                'success': False,
                'error': f'Error starting navigation driver: {str(e)}',
                'robot_ip': robot_ip
            }), 500
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/navigation/stop', methods=['POST'])
def stop_navigation():
    """Stop navigation stack (Ackman_driver_A1)."""
    try:
        robot_ip = request.remote_addr
        
        if active_processes['navigation'] is None:
            return jsonify({
                'success': False,
                'error': 'Navigation is not running'
            }), 400
        
        # Check if process is still alive
        if active_processes['navigation'].poll() is not None:
            # Process already dead
            active_processes['navigation'] = None
            return jsonify({
                'success': False,
                'error': 'Navigation process is not running (may have crashed)'
            }), 400
        
        process = active_processes['navigation']
        
        try:
            # Kill the process group
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            active_processes['navigation'] = None
            print(f"✓ Navigation driver stopped (PID: {process.pid})")
            return jsonify({
                'success': True,
                'message': 'Navigation driver stopped successfully',
                'robot_ip': robot_ip
            }), 200
        except subprocess.TimeoutExpired:
            # Force kill if it doesn't stop
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                process.wait()
            except (ProcessLookupError, OSError):
                pass
            active_processes['navigation'] = None
            print(f"✓ Navigation driver force stopped (PID: {process.pid})")
            return jsonify({
                'success': True,
                'message': 'Navigation driver force stopped',
                'robot_ip': robot_ip
            }), 200
        except (ProcessLookupError, OSError) as e:
            active_processes['navigation'] = None
            return jsonify({
                'success': False,
                'error': f'Error stopping navigation driver: {str(e)}',
                'robot_ip': robot_ip
            }), 500
        
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


@app.route('/api/rosbridge/start', methods=['POST'])
def start_rosbridge_endpoint():
    """Start rosbridge_server to expose ROS2 on the robot."""
    try:
        success, pid, error = start_rosbridge()
        
        if success:
            return jsonify({
                'success': True,
                'message': 'rosbridge_server started successfully',
                'pid': pid,
                'ros_domain_id': current_domain_id if current_domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0'),
                'websocket_url': 'ws://localhost:9090'
            }), 200
        else:
            return jsonify({
                'success': False,
                'error': error or 'Failed to start rosbridge_server'
            }), 500
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/rosbridge/stop', methods=['POST'])
def stop_rosbridge_endpoint():
    """Stop rosbridge_server."""
    try:
        success, error = stop_rosbridge()
        
        if success:
            return jsonify({
                'success': True,
                'message': 'rosbridge_server stopped successfully'
            }), 200
        else:
            return jsonify({
                'success': False,
                'error': error or 'Failed to stop rosbridge_server'
            }), 500
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/rosbridge/status', methods=['GET'])
def get_rosbridge_status():
    """Get rosbridge_server status."""
    try:
        is_running = False
        pid = None
        
        if active_processes['rosbridge'] is not None:
            if active_processes['rosbridge'].poll() is None:
                is_running = True
                pid = active_processes['rosbridge'].pid
            else:
                # Process died, reset it
                active_processes['rosbridge'] = None
        
        return jsonify({
            'success': True,
            'running': is_running,
            'pid': pid,
            'ros_domain_id': current_domain_id if current_domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0'),
            'websocket_url': 'ws://localhost:9090' if is_running else None
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/navigation/status', methods=['GET'])
def get_navigation_status():
    """Get navigation driver status."""
    try:
        is_running = False
        pid = None
        
        if active_processes['navigation'] is not None:
            if active_processes['navigation'].poll() is None:
                is_running = True
                pid = active_processes['navigation'].pid
            else:
                # Process died, reset it
                active_processes['navigation'] = None
        
        return jsonify({
            'success': True,
            'running': is_running,
            'pid': pid,
            'ros_domain_id': current_domain_id if current_domain_id is not None else os.environ.get('ROS_DOMAIN_ID', '0'),
            'command': 'ros2 run yahboomcar_bringup Ackman_driver_A1'
        }), 200
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


def check_docker_container(container_name):
    """Check if docker container exists and is running."""
    try:
        # Use docker ps with filter to check if container is running
        result = subprocess.run(
            ['docker', 'ps', '--filter', f'name={container_name}', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode == 0:
            names = result.stdout.strip().split('\n')
            return container_name in names
        return False
    except Exception as e:
        print(f"Error checking docker container: {e}")
        return False

def ensure_docker_container_running(container_name):
    """Ensure docker container is running, start it if it exists but is stopped."""
    try:
        # Check if container is running
        if check_docker_container(container_name):
            print(f"✓ Docker container '{container_name}' is already running")
            return True
        
        # Check if container exists but is stopped
        result = subprocess.run(
            ['docker', 'ps', '-a', '--filter', f'name={container_name}', '--format', '{{.Names}}'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            names = result.stdout.strip().split('\n')
            if container_name in names:
                # Container exists but is stopped, start it
                print(f"Starting stopped docker container '{container_name}'...")
                start_result = subprocess.run(
                    ['docker', 'start', container_name],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if start_result.returncode == 0:
                    print(f"✓ Docker container '{container_name}' started successfully")
                    # Wait a moment for container to be ready
                    import time
                    time.sleep(2)
                    return True
                else:
                    print(f"✗ Failed to start container: {start_result.stderr}")
                    return False
        
        # Container doesn't exist
        print(f"✗ Docker container '{container_name}' does not exist")
        return False
        
    except Exception as e:
        print(f"Error ensuring docker container is running: {e}")
        return False

def is_running_in_docker():
    """Check if we're running inside a Docker container."""
    # Check for .dockerenv file (exists in Docker containers)
    if os.path.exists('/.dockerenv'):
        return True
    
    # Check cgroup (Docker containers have 'docker' in cgroup)
    try:
        with open('/proc/self/cgroup', 'r') as f:
            cgroup_content = f.read()
            if 'docker' in cgroup_content:
                return True
    except:
        pass
    
    return False

def get_container_name():
    """Best-effort detection of the current Docker container name/ID."""
    # Primary: kernel hostname (Docker usually sets this to the container ID/name)
    try:
        return os.uname().nodename
    except Exception:
        pass

    # Fallback: environment variables that might hold the container name
    for key in ('HOSTNAME', 'CONTAINER_NAME'):
        val = os.environ.get(key)
        if val:
            return val

    return None

def get_host_docker_command():
    """
    Get docker command that works from inside a container.
    Requires Docker socket to be mounted: -v /var/run/docker.sock:/var/run/docker.sock
    And optionally docker binary: -v /usr/bin/docker:/usr/bin/docker
    """
    # Check if docker socket is available (required for docker commands from inside container)
    docker_socket = '/var/run/docker.sock'
    if not os.path.exists(docker_socket):
        print(f"⚠ ERROR: Docker socket not found at {docker_socket}")
        print("  To fix: Mount the Docker socket when starting the container:")
        print("  docker run -v /var/run/docker.sock:/var/run/docker.sock ...")
        return None
    
    # Try to find docker binary (might be mounted from host or installed in container)
    docker_paths = [
        '/usr/bin/docker',           # Host docker binary (if mounted)
        '/usr/local/bin/docker',    # Alternative location
        'docker'                     # From PATH (if docker client installed in container)
    ]
    
    for docker_path in docker_paths:
        try:
            # Test if docker works by checking version
            test_result = subprocess.run(
                [docker_path, '--version'],
                capture_output=True,
                text=True,
                timeout=2,
                env=os.environ.copy()
            )
            if test_result.returncode == 0:
                print(f"✓ Found working docker at: {docker_path}")
                print(f"  Docker version: {test_result.stdout.strip()}")
                return docker_path
        except FileNotFoundError:
            continue
        except Exception as e:
            print(f"  Failed to test {docker_path}: {e}")
            continue
    
    print("⚠ WARNING: Could not find working docker command")
    print("  Options:")
    print("  1. Mount host docker binary: docker run -v /usr/bin/docker:/usr/bin/docker ...")
    print("  2. Install docker client in container: apt-get install docker.io")
    print("  3. Use Docker API directly via socket (more complex)")
    return None

def find_docker_command():
    """Find the docker command. Returns /usr/bin/docker (confirmed location)."""
    # Debug: Check runtime environment
    try:
        runtime_path = os.environ.get('PATH', 'NOT SET')
        print(f"DEBUG: Runtime PATH: {runtime_path}")
    except:
        print("DEBUG: Could not get PATH")
    
    try:
        running_user = os.getlogin()
        print(f"DEBUG: Running as user: {running_user}")
    except:
        try:
            running_user = os.environ.get('USER', os.environ.get('USERNAME', 'unknown'))
            print(f"DEBUG: Running as user (from env): {running_user}")
        except:
            print("DEBUG: Could not determine user")
    
    # Check if we're running in Docker
    in_docker = is_running_in_docker()
    container_name = get_container_name()
    print(f"DEBUG: Running in Docker container: {in_docker}")
    if container_name:
        print(f"DEBUG: Container name: {container_name}")
    
    if in_docker:
        print("⚠ Running inside Docker container - need access to host docker")
        if container_name:
            print(f"  Container name: {container_name}")
        docker_path = get_host_docker_command()
        if docker_path:
            return docker_path
        else:
            print("⚠ ERROR: Cannot access docker from inside container!")
            print("  Solution: When starting the container, mount Docker socket and binary:")
            print("  docker run -v /var/run/docker.sock:/var/run/docker.sock \\")
            print("             -v /usr/bin/docker:/usr/bin/docker \\")
            print("             ...")
            if container_name:
                print(f"  Or execute from host: docker exec {container_name} <command>")
            print("  Falling back to /usr/bin/docker (will likely fail)")
    
    # Check if docker is a symlink and what it points to
    docker_path = '/usr/bin/docker'
    try:
        if os.path.islink(docker_path):
            real_path = os.readlink(docker_path)
            if not os.path.isabs(real_path):
                real_path = os.path.join(os.path.dirname(docker_path), real_path)
            print(f"DEBUG: /usr/bin/docker is a symlink pointing to: {real_path}")
            # Check if the target exists
            if os.path.exists(real_path):
                print(f"DEBUG: Symlink target exists: {real_path}")
            else:
                print(f"DEBUG: WARNING - Symlink target does NOT exist: {real_path}")
        else:
            print(f"DEBUG: /usr/bin/docker is not a symlink")
    except Exception as e:
        print(f"DEBUG: Could not check symlink: {e}")
    
    # Try to verify docker actually works by running it
    try:
        test_result = subprocess.run(
            [docker_path, '--version'],
            capture_output=True,
            text=True,
            timeout=2,
            env=os.environ.copy()
        )
        if test_result.returncode == 0:
            print(f"✓ Verified docker works: {docker_path}")
            print(f"  Docker version: {test_result.stdout.strip()}")
        else:
            print(f"⚠ Docker test failed: {test_result.stderr}")
    except Exception as e:
        print(f"⚠ Could not test docker: {e}")
    
    # Hardcoded: Use /usr/bin/docker (confirmed location from terminal: which docker)
    # Even if os.path.exists() fails in Python, we know docker is at /usr/bin/docker
    print(f"Using docker at: {docker_path} (hardcoded - confirmed from terminal)")
    return docker_path

def open_terminal_window(title, command):
    """Open a NEW visible terminal window with the given command (separate from robot_api terminal)."""
    import time
    
    # Use bash explicitly, not sh
    bash_cmd = '/bin/bash'
    
    # Don't replace docker in command - it's already been set with full path in start_3d_digital_twin()
    # The command passed in already has the correct docker path
    print(f"Command to execute: {command}")
    
    # Detect which terminal is actually being used
    def detect_terminal():
        """Detect the terminal program being used."""
        try:
            # Check parent process to see what terminal we're running in
            result = subprocess.run(
                ['ps', '-p', str(os.getppid()), '-o', 'comm=',],
                capture_output=True,
                text=True,
                timeout=2
            )
            parent_process = result.stdout.strip()
            print(f"Detected parent process: {parent_process}")
            
            # Map parent process to terminal command
            if 'gnome-terminal' in parent_process.lower():
                return 'gnome-terminal'
            elif 'xterm' in parent_process.lower():
                return 'xterm'
            elif 'konsole' in parent_process.lower():
                return 'konsole'
            elif 'terminator' in parent_process.lower():
                return 'terminator'
        except Exception as e:
            print(f"Could not detect terminal: {e}")
        
        return None
    
    # Check if we have DISPLAY (GUI available)
    display = os.environ.get('DISPLAY')
    detected_terminal = detect_terminal()
    
    # Ensure PATH includes common locations for docker and other tools
    enhanced_env = os.environ.copy()
    enhanced_env['PATH'] = '/usr/bin:/usr/local/bin:/snap/bin:' + enhanced_env.get('PATH', '')
    
    # Try detected terminal first, then fallback to common ones
    terminals_to_try = []
    if detected_terminal:
        terminals_to_try.append(detected_terminal)
    
    # Add common terminals
    terminals_to_try.extend(['gnome-terminal', 'xterm', 'konsole', 'terminator', 'tilix', 'mate-terminal'])
    
    if display or detected_terminal:
        for term_cmd in terminals_to_try:
            try:
                if term_cmd == 'gnome-terminal':
                    # Escape the command properly for bash -c
                    # Use single quotes around the entire command to avoid quote issues
                    escaped_command = command.replace("'", "'\"'\"'")  # Escape single quotes
                    process = subprocess.Popen(
                        [
                            'gnome-terminal',
                            '--new-window',
                            '--title', title,
                            '--', bash_cmd, '-c', f"{escaped_command}; exec {bash_cmd}"
                        ],
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=enhanced_env  # Use enhanced PATH
                    )
                elif term_cmd == 'xterm':
                    process = subprocess.Popen(
                        [
                            'xterm',
                            '-title', title,
                            '-e', bash_cmd, '-c', f"{command}; exec {bash_cmd}"
                        ],
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=enhanced_env  # Use enhanced PATH
                    )
                elif term_cmd == 'konsole':
                    process = subprocess.Popen(
                        ['konsole', '--new-tab', '-e', bash_cmd, '-c', f"{command}; exec {bash_cmd}"],
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=enhanced_env  # Use enhanced PATH
                    )
                elif term_cmd == 'terminator':
                    process = subprocess.Popen(
                        ['terminator', '-e', f"{bash_cmd} -c '{command}; exec {bash_cmd}'"],
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=enhanced_env  # Use enhanced PATH
                    )
                elif term_cmd == 'tilix':
                    process = subprocess.Popen(
                        ['tilix', '-e', bash_cmd, '-c', f"{command}; exec {bash_cmd}"],
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=enhanced_env  # Use enhanced PATH
                    )
                elif term_cmd == 'mate-terminal':
                    process = subprocess.Popen(
                        ['mate-terminal', '--title', title, '-e', f"{bash_cmd} -c '{command}; exec {bash_cmd}'"],
                        preexec_fn=os.setsid,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        env=enhanced_env  # Use enhanced PATH
                    )
                else:
                    continue
                
                time.sleep(1.5)
                print(f"✓ Opened new {term_cmd} window: {title}")
                return process
            except FileNotFoundError:
                if term_cmd == detected_terminal:
                    print(f"⚠ Detected terminal '{detected_terminal}' not found in PATH, trying others...")
                continue
    
    # If no GUI terminal available, run the command using bash explicitly
    print(f"⚠ No GUI terminal available. Running command directly with {bash_cmd}...")
    print(f"  Command: {command}")
    print(f"  Output will be visible in the process")
    
    # Ensure PATH includes common locations for docker and other tools
    env = os.environ.copy()
    env['PATH'] = '/usr/bin:/usr/local/bin:/snap/bin:/bin:' + env.get('PATH', '')
    
    # Check if docker is accessible before running
    try:
        test_result = subprocess.run(
            ['/usr/bin/docker', '--version'],
            capture_output=True,
            text=True,
            timeout=2,
            env=env
        )
        if test_result.returncode == 0:
            print(f"✓ Docker is accessible: {test_result.stdout.strip()}")
        else:
            print(f"⚠ Docker test failed: {test_result.stderr}")
    except Exception as e:
        print(f"⚠ Could not test docker before execution: {e}")
    
    process = subprocess.Popen(
        command,
        shell=True,
        executable=bash_cmd,  # Use bash explicitly, not sh
        preexec_fn=os.setsid,
        env=env  # Use enhanced PATH
    )
    return process

@app.route('/api/3d-digital-twin/start', methods=['POST'])
def start_3d_digital_twin():
    """Start 3D Digital Twin mission (camera, mapping, and display)."""
    global active_processes
    
    try:
        # Check if already running
        if (active_processes['digital_twin_camera'] is not None and 
            active_processes['digital_twin_camera'].poll() is None):
            return jsonify({
                'success': True,
                'message': '3D Digital Twin is already running',
                'camera_pid': active_processes['digital_twin_camera'].pid,
                'mapping_pid': active_processes['digital_twin_mapping'].pid if active_processes['digital_twin_mapping'] else None,
                'display_pid': active_processes['digital_twin_display'].pid if active_processes['digital_twin_display'] else None
            }), 200
        
        import time
        
        # Find docker command path
        docker_cmd = find_docker_command()
        print(f"Using docker command: {docker_cmd}")
        
        # ------------------------------------------------------------------
        # Extra debug: check ROS2 availability inside the docker container
        # BEFORE opening the visible terminals. This helps narrow down
        # issues like "ros2: command not found" inside the container.
        # ------------------------------------------------------------------
        try:
            debug_cmd = (
                f"{docker_cmd} exec elated_bhaskara "
                "/bin/bash -c '"
                "echo \"[DEBUG inside container] Waiting for /opt/ros/humble/setup.bash\"; "
                "until [ -f /opt/ros/humble/setup.bash ]; do sleep 1; done; "
                "echo \"[DEBUG inside container] Sourcing /opt/ros/humble/setup.bash\"; "
                "source /opt/ros/humble/setup.bash; "
                "if [ -f /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then "
                "echo \"[DEBUG inside container] Sourcing library_ws overlay\"; "
                "source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash; "
                "fi; "
                "if [ -f /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then "
                "echo \"[DEBUG inside container] Sourcing yahboomcar_ws overlay\"; "
                "source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash; "
                "fi; "
                "echo \"[DEBUG inside container] PATH=$PATH\"; "
                "which ros2 2>/dev/null || echo \"[DEBUG inside container] ros2 NOT found in PATH\"; "
                "env | grep -E \"^(ROS_|AMENT_)\" || echo \"[DEBUG inside container] No ROS_* or AMENT_* env vars\""
                "'"
            )
            print(f"Running docker ROS2 debug command:\n  {debug_cmd}")
            debug_result = subprocess.run(
                debug_cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=15
            )
            print("[DEBUG docker ROS2] return code:", debug_result.returncode)
            if debug_result.stdout:
                print("[DEBUG docker ROS2] STDOUT:\n" + debug_result.stdout)
            if debug_result.stderr:
                print("[DEBUG docker ROS2] STDERR:\n" + debug_result.stderr)
        except Exception as e:
            print(f"[DEBUG docker ROS2] Failed to run debug command: {e}")
        
        # Terminal 1: Open new terminal, enter docker container, then run camera launch
        print("Opening Terminal 1: Entering docker container and starting camera launch...")
        camera_command = (
            f"{docker_cmd} exec -it elated_bhaskara /bin/bash -c "
            "'echo \"[CAMERA] Attached to container elated_bhaskara\"; "
            "echo \"[CAMERA] Waiting for /opt/ros/humble/setup.bash\"; "
            "until [ -f /opt/ros/humble/setup.bash ]; do sleep 1; done; "
            "echo \"[CAMERA] Sourcing /opt/ros/humble/setup.bash\"; "
            "source /opt/ros/humble/setup.bash; "
            "if [ -f /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then "
            "echo \"[CAMERA] Sourcing library_ws overlay\"; "
            "source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash; "
            "fi; "
            "if [ -f /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then "
            "echo \"[CAMERA] Sourcing yahboomcar_ws overlay\"; "
            "source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash; "
            "fi; "
            "echo \"[CAMERA] PATH=$PATH\"; "
            "which ros2 2>/dev/null || echo \"[CAMERA] ros2 NOT found in PATH\"; "
            "env | grep -E \"^(ROS_|AMENT_)\" || echo \"[CAMERA] No ROS_* or AMENT_* env vars\"; "
            "ros2 launch ascamera hp60c.launch.py'"
        )
        camera_process = open_terminal_window("3D Digital Twin - Camera", camera_command)
        active_processes['digital_twin_camera'] = camera_process
        
        # Wait 5 seconds before starting mapping
        print("Waiting 5 seconds before starting mapping...")
        time.sleep(5)
        
        # Terminal 2: Open new terminal, enter docker container, then run mapping launch
        print("Opening Terminal 2: Entering docker container and starting mapping launch...")
        mapping_command = (
            f"{docker_cmd} exec -it elated_bhaskara /bin/bash -c "
            "'echo \"[MAPPING] Attached to container elated_bhaskara\"; "
            "echo \"[MAPPING] Waiting for /opt/ros/humble/setup.bash\"; "
            "until [ -f /opt/ros/humble/setup.bash ]; do sleep 1; done; "
            "echo \"[MAPPING] Sourcing /opt/ros/humble/setup.bash\"; "
            "source /opt/ros/humble/setup.bash; "
            "if [ -f /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then "
            "echo \"[MAPPING] Sourcing library_ws overlay\"; "
            "source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash; "
            "fi; "
            "if [ -f /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then "
            "echo \"[MAPPING] Sourcing yahboomcar_ws overlay\"; "
            "source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash; "
            "fi; "
            "echo \"[MAPPING] PATH=$PATH\"; "
            "which ros2 2>/dev/null || echo \"[MAPPING] ros2 NOT found in PATH\"; "
            "env | grep -E \"^(ROS_|AMENT_)\" || echo \"[MAPPING] No ROS_* or AMENT_* env vars\"; "
            "ros2 launch yahboomcar_nav map_rtabmap_launch.py'"
        )
        mapping_process = open_terminal_window("3D Digital Twin - Mapping", mapping_command)
        active_processes['digital_twin_mapping'] = mapping_process
        
        # Terminal 3: Open new terminal, enter docker container, then run display launch
        print("Opening Terminal 3: Entering docker container and starting display launch...")
        display_command = (
            f"{docker_cmd} exec -it elated_bhaskara /bin/bash -c "
            "'echo \"[DISPLAY] Attached to container elated_bhaskara\"; "
            "echo \"[DISPLAY] Waiting for /opt/ros/humble/setup.bash\"; "
            "until [ -f /opt/ros/humble/setup.bash ]; do sleep 1; done; "
            "echo \"[DISPLAY] Sourcing /opt/ros/humble/setup.bash\"; "
            "source /opt/ros/humble/setup.bash; "
            "if [ -f /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash ]; then "
            "echo \"[DISPLAY] Sourcing library_ws overlay\"; "
            "source /root/yahboomcar_ros2_ws/software/library_ws/install/setup.bash; "
            "fi; "
            "if [ -f /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash ]; then "
            "echo \"[DISPLAY] Sourcing yahboomcar_ws overlay\"; "
            "source /root/yahboomcar_ros2_ws/yahboomcar_ws/install/setup.bash; "
            "fi; "
            "echo \"[DISPLAY] PATH=$PATH\"; "
            "which ros2 2>/dev/null || echo \"[DISPLAY] ros2 NOT found in PATH\"; "
            "env | grep -E \"^(ROS_|AMENT_)\" || echo \"[DISPLAY] No ROS_* or AMENT_* env vars\"; "
            "ros2 launch yahboomcar_nav display_rtabmap_map_launch.py'"
        )
        display_process = open_terminal_window("3D Digital Twin - Display", display_command)
        active_processes['digital_twin_display'] = display_process
        
        # Give processes a moment to start
        time.sleep(3)
        
        # Check if processes are still running
        camera_running = camera_process.poll() is None
        mapping_running = mapping_process.poll() is None if mapping_process else False
        display_running = display_process.poll() is None if display_process else False
        
        # Get PIDs if available
        camera_pid = getattr(camera_process, 'pid', None)
        mapping_pid = getattr(mapping_process, 'pid', None)
        display_pid = getattr(display_process, 'pid', None)
        
        if camera_running and mapping_running and display_running:
            print(f"✓ 3D Digital Twin started successfully")
            print(f"  Camera PID: {camera_pid}")
            print(f"  Mapping PID: {mapping_pid}")
            print(f"  Display PID: {display_pid}")
            return jsonify({
                'success': True,
                'message': '3D Digital Twin started successfully. Check terminal windows for output.',
                'camera_pid': camera_pid,
                'mapping_pid': mapping_pid,
                'display_pid': display_pid
            }), 200
        else:
            # Some processes failed, get error details
            error_msg = []
            error_details = []
            
            if not camera_running:
                error_msg.append("Camera launch failed")
                # Try to get stderr if available
                if hasattr(camera_process, 'stderr') and camera_process.stderr:
                    try:
                        camera_process.stderr.seek(0)
                        err = camera_process.stderr.read().decode('utf-8', errors='ignore')
                        if err:
                            error_details.append(f"Camera error: {err[:200]}")
                    except:
                        pass
            
            if not mapping_running:
                error_msg.append("Mapping launch failed")
                if hasattr(mapping_process, 'stderr') and mapping_process.stderr:
                    try:
                        mapping_process.stderr.seek(0)
                        err = mapping_process.stderr.read().decode('utf-8', errors='ignore')
                        if err:
                            error_details.append(f"Mapping error: {err[:200]}")
                    except:
                        pass
            
            if not display_running:
                error_msg.append("Display launch failed")
                if hasattr(display_process, 'stderr') and display_process.stderr:
                    try:
                        display_process.stderr.seek(0)
                        err = display_process.stderr.read().decode('utf-8', errors='ignore')
                        if err:
                            error_details.append(f"Display error: {err[:200]}")
                    except:
                        pass
            
            full_error = f"Some processes failed to start: {', '.join(error_msg)}"
            if error_details:
                full_error += f"\n\nDetails:\n" + "\n".join(error_details)
            
            print(f"ERROR: {full_error}")
            return jsonify({
                'success': False,
                'error': full_error
            }), 500
            
    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"Error starting 3D Digital Twin: {str(e)}")
        print(f"Traceback: {error_trace}")
        return jsonify({
            'success': False,
            'error': f"{str(e)}\n\nTraceback:\n{error_trace}"
        }), 500


@app.route('/api/3d-digital-twin/stop', methods=['POST'])
def stop_3d_digital_twin():
    """Stop 3D Digital Twin mission and transfer RTABMap database."""
    global active_processes
    
    try:
        # Get mission_id and mission_control_url from request
        data = request.get_json() or {}
        mission_id = data.get('mission_id')
        mission_control_url = data.get('mission_control_url', 'http://localhost:8002')
        robot_id = data.get('robot_id', 'unknown')
        
        stopped_processes = []
        database_transferred = False
        
        # Before stopping display process, copy RTABMap database from docker
        if active_processes['digital_twin_display'] is not None:
            process = active_processes['digital_twin_display']
            if process.poll() is None:
                print("Copying RTABMap database from docker container...")
                try:
                    # Copy file from docker container to temporary location
                    temp_dir = tempfile.gettempdir()
                    temp_file = os.path.join(temp_dir, 'rtabmap.db')
                    
                    # Copy file from docker container
                    copy_cmd = f"docker cp elated_bhaskara:/root/.ros/rtabmap.db {temp_file}"
                    result = subprocess.run(
                        copy_cmd,
                        shell=True,
                        capture_output=True,
                        text=True,
                        timeout=30
                    )
                    
                    if result.returncode == 0 and os.path.exists(temp_file):
                        print(f"✓ Copied RTABMap database to {temp_file}")
                        
                        # Send file to mission control API
                        try:
                            with open(temp_file, 'rb') as f:
                                files = {'file': ('rtabmap.db', f, 'application/octet-stream')}
                                data_form = {
                                    'mission_id': mission_id or '',
                                    'robot_id': robot_id
                                }
                                
                                upload_url = f"{mission_control_url}/api/digital-twin/upload-database"
                                print(f"Sending RTABMap database to {upload_url}...")
                                
                                response = requests.post(
                                    upload_url,
                                    files=files,
                                    data=data_form,
                                    timeout=60
                                )
                                
                                if response.status_code == 200:
                                    result_data = response.json()
                                    print(f"✓ Successfully transferred RTABMap database: {result_data.get('filename')}")
                                    database_transferred = True
                                else:
                                    print(f"✗ Failed to transfer database: {response.status_code} - {response.text}")
                        except Exception as e:
                            print(f"Error sending database to mission control: {e}")
                        
                        # Clean up temporary file
                        try:
                            os.remove(temp_file)
                        except:
                            pass
                    else:
                        print(f"✗ Failed to copy database from docker: {result.stderr}")
                        if os.path.exists(temp_file):
                            try:
                                os.remove(temp_file)
                            except:
                                pass
                except Exception as e:
                    print(f"Error copying RTABMap database: {e}")
                
                # Now stop the display process (but don't exit docker)
                try:
                    # Send SIGTERM to gracefully stop the ros2 launch process
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    stopped_processes.append('display')
                    print("✓ Stopped display process (docker container remains running)")
                except Exception as e:
                    print(f"Error stopping display process: {e}")
            active_processes['digital_twin_display'] = None
        
        # Stop camera process
        if active_processes['digital_twin_camera'] is not None:
            process = active_processes['digital_twin_camera']
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    stopped_processes.append('camera')
                except Exception as e:
                    print(f"Error stopping camera process: {e}")
            active_processes['digital_twin_camera'] = None
        
        # Stop mapping process
        if active_processes['digital_twin_mapping'] is not None:
            process = active_processes['digital_twin_mapping']
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    stopped_processes.append('mapping')
                except Exception as e:
                    print(f"Error stopping mapping process: {e}")
            active_processes['digital_twin_mapping'] = None
        
        message_parts = []
        if stopped_processes:
            message_parts.append(f'Stopped 3D Digital Twin processes: {", ".join(stopped_processes)}')
        if database_transferred:
            message_parts.append('RTABMap database transferred successfully')
        
        if message_parts:
            print(f"✓ {' | '.join(message_parts)}")
            return jsonify({
                'success': True,
                'message': ' | '.join(message_parts),
                'database_transferred': database_transferred
            }), 200
        else:
            return jsonify({
                'success': True,
                'message': 'No 3D Digital Twin processes were running',
                'database_transferred': False
            }), 200
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/3d-digital-twin/status', methods=['GET'])
def get_3d_digital_twin_status():
    """Get 3D Digital Twin status."""
    try:
        camera_running = False
        camera_pid = None
        if active_processes['digital_twin_camera'] is not None:
            if active_processes['digital_twin_camera'].poll() is None:
                camera_running = True
                camera_pid = active_processes['digital_twin_camera'].pid
            else:
                active_processes['digital_twin_camera'] = None
        
        mapping_running = False
        mapping_pid = None
        if active_processes['digital_twin_mapping'] is not None:
            if active_processes['digital_twin_mapping'].poll() is None:
                mapping_running = True
                mapping_pid = active_processes['digital_twin_mapping'].pid
            else:
                active_processes['digital_twin_mapping'] = None
        
        display_running = False
        display_pid = None
        if active_processes['digital_twin_display'] is not None:
            if active_processes['digital_twin_display'].poll() is None:
                display_running = True
                display_pid = active_processes['digital_twin_display'].pid
            else:
                active_processes['digital_twin_display'] = None
        
        return jsonify({
            'success': True,
            'camera_running': camera_running,
            'camera_pid': camera_pid,
            'mapping_running': mapping_running,
            'mapping_pid': mapping_pid,
            'display_running': display_running,
            'display_pid': display_pid,
            'all_running': camera_running and mapping_running and display_running
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
    rosbridge_installed = check_rosbridge_installed()
    
    # Check rosbridge status
    rosbridge_running = False
    rosbridge_pid = None
    if active_processes['rosbridge'] is not None:
        if active_processes['rosbridge'].poll() is None:
            rosbridge_running = True
            rosbridge_pid = active_processes['rosbridge'].pid
        else:
            active_processes['rosbridge'] = None
    
    # Check navigation status
    navigation_running = False
    navigation_pid = None
    if active_processes['navigation'] is not None:
        if active_processes['navigation'].poll() is None:
            navigation_running = True
            navigation_pid = active_processes['navigation'].pid
        else:
            active_processes['navigation'] = None
    
    return jsonify({
        'status': 'healthy',
        'service': 'robot_api',
        'robot_type': 'ROSMASTER-A1',
        'current_domain_id': current_domain_id,
        'ros_domain_id_env': os.environ.get('ROS_DOMAIN_ID'),
        'rosbridge_installed': rosbridge_installed,
        'rosbridge_running': rosbridge_running,
        'rosbridge_pid': rosbridge_pid,
        'navigation_running': navigation_running,
        'navigation_pid': navigation_pid
    }), 200


if __name__ == '__main__':
    print('=' * 60)
    print('ROSMASTER-A1 Robot API Server')
    print('=' * 60)
    
    # Debug: Print runtime environment at startup
    print('\nDEBUG: Runtime Environment:')
    try:
        runtime_path = os.environ.get('PATH', 'NOT SET')
        print(f"  Runtime PATH: {runtime_path}")
    except Exception as e:
        print(f"  Could not get PATH: {e}")
    
    try:
        running_user = os.getlogin()
        print(f"  Running as user: {running_user}")
    except Exception as e:
        try:
            running_user = os.environ.get('USER', os.environ.get('USERNAME', 'unknown'))
            print(f"  Running as user (from env): {running_user}")
        except Exception as e2:
            print(f"  Could not determine user: {e2}")
    
    # Check if docker is accessible
    docker_cmd = find_docker_command()
    print(f"  Docker command found: {docker_cmd}")
    print('=' * 60)
    
    # Install rosbridge-server on startup
    print('\nChecking rosbridge-server installation...')
    if not install_rosbridge():
        print('\n✗ Failed to install rosbridge-server')
        print('The API server will continue, but rosbridge functionality may not work.')
        print('Please install manually: sudo apt install ros-humble-rosbridge-server')
    else:
        print('\n✓ rosbridge-server installation check complete')
    
    print('\n' + '=' * 60)
    print('Starting Robot API Server...')
    print('=' * 60)
    print('API endpoints:')
    print('  POST /api/set_domain_id - Set ROS_DOMAIN_ID')
    print('  GET  /api/get_domain_id - Get current ROS_DOMAIN_ID')
    print('  POST /api/rosbridge/start - Start rosbridge_server')
    print('  POST /api/rosbridge/stop - Stop rosbridge_server')
    print('  GET  /api/rosbridge/status - Get rosbridge status')
    print('  POST /api/navigation/start - Start navigation driver (Ackman_driver_A1)')
    print('  POST /api/navigation/stop - Stop navigation driver')
    print('  GET  /api/navigation/status - Get navigation status')
    print('  POST /api/3d-digital-twin/start - Start 3D Digital Twin mission')
    print('  POST /api/3d-digital-twin/stop - Stop 3D Digital Twin mission')
    print('  GET  /api/3d-digital-twin/status - Get 3D Digital Twin status')
    print('  POST /api/slam/start - Start SLAM (skeleton)')
    print('  POST /api/slam/stop - Stop SLAM (skeleton)')
    print('  GET  /health - Health check')
    print('\nServer starting on http://0.0.0.0:8004')
    print('NOTE: Robot-specific ROS2 commands will be implemented later based on robot brand.')
    print('NOTE: Use /api/rosbridge/start to expose ROS2 via WebSocket (port 9090)')
    print('=' * 60 + '\n')
    
    app.run(host='0.0.0.0', port=8004, debug=False, use_reloader=False)
