#!/bin/bash
# Start script for ROSMASTER-A1 robot API server
# This script should be run on the robot
# The API will automatically install rosbridge-server on first run

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_API_DIR="$SCRIPT_DIR/../api"

cd "$ROBOT_API_DIR"

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 not found. Please install Python 3."
    exit 1
fi

# Check if virtual environment exists, create if not
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
source venv/bin/activate

# Install dependencies if needed
if [ ! -f "venv/.deps_installed" ]; then
    echo "Installing Python dependencies..."
    pip install -r requirements.txt
    touch venv/.deps_installed
fi

# Start the robot API server
# Note: The API will automatically install rosbridge-server on startup
echo "Starting ROSMASTER-A1 Robot API Server on port 8004..."
echo "The API will automatically check and install rosbridge-server if needed."
python3 robot_api.py
