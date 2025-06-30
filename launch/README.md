# Puffin Brain Launch Files

This directory contains several launch files for the puffin_brain package:

## Launch Files

### 1. `puffin_brain_complete.launch.py` (ROS 2)
**Primary launch file** - Starts all main puffin_brain nodes
- `whisper_listener_node` - Speech recognition using Whisper
- `ollama_publisher_node` - AI language model integration  
- `command_publisher_node` - Command processing and publishing

**Usage:**
```bash
ros2 launch puffin_brain puffin_brain_complete.launch.py
```

**With parameters:**
```bash
ros2 launch puffin_brain puffin_brain_complete.launch.py use_sim_time:=true log_level:=debug
```

### 2. `puffin_brain_modular.launch.py` (ROS 2)
**Selective launch file** - Allows you to choose which nodes to start

**Usage:**
```bash
# Launch only whisper and command nodes
ros2 launch puffin_brain puffin_brain_modular.launch.py launch_ollama:=false

# Launch only ollama node
ros2 launch puffin_brain puffin_brain_modular.launch.py launch_whisper:=false launch_command:=false

# Launch with mock ollama for testing
ros2 launch puffin_brain puffin_brain_modular.launch.py launch_ollama:=false launch_mock_ollama:=true
```

### 3. `puffin_brain_all_nodes.launch` (ROS 1 style)
**Legacy/comprehensive launch file** - Includes all nodes from scripts directory
- All main executable nodes
- Additional script-based nodes
- Service nodes
- Mock/testing nodes

**Usage:**
```bash
roslaunch puffin_brain puffin_brain_all_nodes.launch
```

### 4. `whisper_ollama_command.launch` (ROS 1 style)
**Original launch file** - Basic three-node setup (your existing file)

## Parameters

### Available Launch Arguments:
- `use_sim_time` (default: false) - Use simulation time
- `log_level` (default: info) - Set logging level (debug, info, warn, error)
- `launch_whisper` (default: true) - Enable whisper listener node
- `launch_ollama` (default: true) - Enable ollama publisher node  
- `launch_command` (default: true) - Enable command publisher node
- `launch_mock_ollama` (default: false) - Enable mock ollama for testing

## Node Features:
- **Respawn enabled** - Nodes will restart if they crash
- **Screen output** - All node logs visible in terminal
- **Configurable logging** - Adjustable log levels
- **Modular launching** - Choose which components to run

## Building and Running:

1. Build the workspace:
```bash
cd /home/jetson/puffin_ws
colcon build --packages-select puffin_brain
```

2. Source the workspace:
```bash
source install/setup.bash
```

3. Launch the system:
```bash
ros2 launch puffin_brain puffin_brain_complete.launch.py
```

## Troubleshooting:
- If nodes fail to start, check that all dependencies are installed
- Ensure the workspace is properly built and sourced
- Use `ros2 node list` to verify which nodes are running
- Check node logs with `ros2 log view <node_name>` for debugging
