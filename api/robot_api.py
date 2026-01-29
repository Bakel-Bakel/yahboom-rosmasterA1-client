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
        result = subprocess.run(
            f"docker ps --format '{{{{.Names}}}}' | grep -q '^{container_name}$'",
            shell=True,
            capture_output=True,
            text=True
        )
        return result.returncode == 0
    except Exception as e:
        print(f"Error checking docker container: {e}")
        return False

def open_terminal_window(title, command):
    """Open a visible terminal window with the given command."""
    import time
    
    # Check if we have DISPLAY (GUI available)
    display = os.environ.get('DISPLAY')
    
    if display:
        # Try gnome-terminal first (most common on Ubuntu/Debian)
        try:
            # Use gnome-terminal with -- bash -c to run command and keep terminal open
            process = subprocess.Popen(
                [
                    'gnome-terminal',
                    '--title', title,
                    '--', 'bash', '-c', f"{command}; exec bash"  # Keep terminal open after command
                ],
                preexec_fn=os.setsid
            )
            time.sleep(1)  # Give terminal a moment to open
            return process
        except FileNotFoundError:
            pass
        
        # Try xterm as fallback
        try:
            process = subprocess.Popen(
                [
                    'xterm',
                    '-title', title,
                    '-e', 'bash', '-c', f"{command}; exec bash"  # Keep terminal open
                ],
                preexec_fn=os.setsid
            )
            time.sleep(1)
            return process
        except FileNotFoundError:
            pass
    
    # Fallback: Use screen session (works without GUI)
    # Create a screen session with a name
    screen_name = title.lower().replace(' ', '_')
    try:
        # Start screen session in detached mode with the command
        subprocess.run(
            ['screen', '-dmS', screen_name, 'bash', '-c', command],
            check=True
        )
        print(f"Started screen session '{screen_name}' for {title}")
        print(f"  To view: screen -r {screen_name}")
        # Return a dummy process object
        class ScreenProcess:
            def __init__(self, name):
                self.name = name
                self.pid = None
            
            def poll(self):
                # Check if screen session exists
                result = subprocess.run(
                    ['screen', '-list', self.name],
                    capture_output=True,
                    text=True
                )
                return None if result.returncode == 0 else 1
        
        return ScreenProcess(screen_name)
    except Exception as e:
        print(f"Failed to start screen session: {e}")
        # Last resort: run in background but log output
        return subprocess.Popen(
            command,
            shell=True,
            preexec_fn=os.setsid,
            stdout=open(f'/tmp/{screen_name}.log', 'w'),
            stderr=subprocess.STDOUT
        )

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
        
        # Check if docker container exists
        if not check_docker_container('cool_solomon'):
            error_msg = "Docker container 'cool_solomon' not found or not running"
            print(f"ERROR: {error_msg}")
            return jsonify({
                'success': False,
                'error': error_msg
            }), 500
        
        import time
        
        # Terminal 1: Start camera launch in visible terminal
        print("Starting camera launch in docker container (Terminal 1)...")
        camera_command = "docker exec -it cool_solomon bash -c 'ros2 launch ascamera hp60c.launch.py'"
        camera_process = open_terminal_window("3D Digital Twin - Camera", camera_command)
        active_processes['digital_twin_camera'] = camera_process
        
        # Wait 5 seconds before starting mapping
        print("Waiting 5 seconds before starting mapping...")
        time.sleep(5)
        
        # Terminal 1 (continued): Start mapping launch in same terminal (chained)
        # Actually, let's use a separate terminal for mapping
        print("Starting RTABMap mapping launch in docker container (Terminal 2)...")
        mapping_command = "docker exec -it cool_solomon bash -c 'ros2 launch yahboomcar_nav map_rtabmap_launch.py'"
        mapping_process = open_terminal_window("3D Digital Twin - Mapping", mapping_command)
        active_processes['digital_twin_mapping'] = mapping_process
        
        # Terminal 3: Start display launch in separate terminal
        print("Starting RTABMap display launch in docker container (Terminal 3)...")
        display_command = "docker exec -it cool_solomon bash -c 'ros2 launch yahboomcar_nav display_rtabmap_map_launch.py'"
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
                    copy_cmd = f"docker cp cool_solomon:/root/.ros/rtabmap.db {temp_file}"
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
