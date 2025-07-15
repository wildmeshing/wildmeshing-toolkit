#!/bin/bash

# Bijective Map Web GUI Launcher Script
# This script launches the Python web GUI for bijective_map_app

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "🌐 Starting Bijective Map Web GUI..."
echo "📂 Project root: $PROJECT_ROOT"
echo "🔧 Script location: $SCRIPT_DIR"

# Change to project root directory
cd "$PROJECT_ROOT"

# Check if Python environment is available
PYTHON_PATH="/Users/leyi/anaconda3/envs/pyvista_env/bin/python"
if [ ! -f "$PYTHON_PATH" ]; then
    echo "❌ Error: Python environment not found at $PYTHON_PATH"
    echo "Please check your conda environment"
    exit 1
fi

# Check if build directory exists
if [ ! -d "build" ]; then
    echo "❌ Error: Build directory not found"
    echo "Please build the project first:"
    echo "  mkdir -p build && cd build"
    echo "  cmake -DCMAKE_BUILD_TYPE=Release .."
    echo "  make bijective_map_app"
    exit 1
fi

# Check if bijective_map_app exists
if [ ! -f "build/bijective_map/bijective_map_app" ]; then
    echo "❌ Error: bijective_map_app not found"
    echo "Please build the application first:"
    echo "  cd build && make bijective_map_app"
    exit 1
fi

# Launch the web GUI
echo "🚀 Launching web interface..."
"$PYTHON_PATH" "$SCRIPT_DIR/bijective_map_web_gui.py"