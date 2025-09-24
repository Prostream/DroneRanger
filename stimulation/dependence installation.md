# System Requirements

## Minimum Hardware
- CPU: Intel i5 or AMD Ryzen 5 (8th gen or newer)
- RAM: 16GB (32GB recommended)
- GPU: NVIDIA GTX 1060 or better
- Storage: 50GB free space (SSD recommended)
- OS: Windows 10 (version 2004+) or Windows 11 / Ubuntu 22.4 LTS (for WSL)

## Network 

- Stable internet connection for downloads
- Firewall permissions for localhost connections
- Required ports:
    - 14550 (MAVLink/QGroundControl)
    - 8888 (MicroXRCE-DDS)
    - 10000-10005 (ROS-TCP-Connector)

## Dependency Installation List

- WSL2
- ROS2 Humble
- PX4 Autopilot
- MicroXRCE-DDS Agent
- Gazebo
- QGroundControl
- Unity

### WLS2 

- Enable WSL2

Open PowerShell as Administrator and run:
    
```
# Enable WSL
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux/all /norestart

# Enable Virtual Machine Platform
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform/all /norestart

# Restart computer
Restart-Computer
```

After restart:

```
# Set WSL2 as default
wsl --set-default-version 2

# Install Ubuntu 22.04
wsl --install -d Ubuntu-22.04
```

- Config WSL2 Memory
  

Create `.wslconfig` in your Windows user directory:

```
notepad $env:USERPROFILE\.wslconfig
```

Add configuration (adjust based on your RAM):

```
[wsl2]
memory=12GB      # For 32GB system
processors=6     # Half of your CPU cores
swap=4GB
nestedVirtualization=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

- Update Ubuntu and Install Base Tools

```
# In WSL2 Ubuntu
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
curl wget git build-essential cmake \
python3-pip python3-venv \
software-properties-common \
lsb-release gnupg2 x11-apps

# Test GUI support
xclock  # Should display a clock window
```

### ROS2 Humble Installation

- Add ROS2 Repository
   
```
# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

- Setup Environment

```
# Add to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 -h  # Should show ROS2 commands
```

### PX4 Autopilot Installation

- Clone and Build PX4

```
# Create apps directory
mkdir -p ~/app
cd ~/app

# Clone PX4 repository with submodules
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dependencies
bash ./Tools/setup/ubuntu.sh

# Build PX4 SITL (Software In The Loop)
make px4_sitl_default

# Gazebo Installation
cd ~

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic
```

- Test PX4 with Gazebo

```
# Test with quadcopter model
cd ~/app/PX4-Autopilot
make px4_sitl gz_x500
```

Should see:
- Gazebo window with drone
- PX4 console showing "Ready for takeoff!"
- Press Ctrl+C to stop


### MicroXRCE-DDS Agent Installation
This component bridges PX4 (MAVLink/uORB) with ROS2 (DDS).

- Build px4_msgs and px4_ros_com

```
# Create ROS2 workspace
mkdir -p ~/drone_ws/src
cd ~/drone_ws/src

# Clone required packages
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

# Build packages
cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Add to .bashrc
echo "source ~/drone_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- Install MicroXRCE-DDS Agent

```
# Clone and build the agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

# Verify installation
which MicroXRCEAgent  # Should show: /usr/local/bin/MicroXRCEAgent
```

### Gazebo Installation

PX4 comes with its own Gazebo, but we'll also install system-wide Gazebo Harmonic.

-  Add Gazebo Repository

```
# Add GPG key
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

- Install Gazebo Harmonic

```
sudo apt update
sudo apt install -y gz-harmonic

# Verify installation
gz sim --version  # Should show version 8.x.x
```

### QGroundControl Installation

- Download and Install ( in Windows not WSL)

```
# In Windows PowerShell
# Download QGroundControl
curl -o QGroundControl-installer.exe `
    https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-installer.exe

# Run installer (will show security warning - click "Run anyway")
.\QGroundControl-installer.exe
```

- Configure Communication
1. Launch QGroundControl
2. Click the "Q" icon (top-left)
3. Go to "Application Settings" → "Comm Links"
4. Verify UDP is opened and configured on port 14550


### Unity Installation and Setup

- Install Unity Hub (Windows)

```
# Download Unity Hub
curl -o UnityHubSetup.exe https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.exe

# Install Unity Hub
.\UnityHubSetup.exe
```

- Install Unity 2022.3 LTS
1. Open Unity Hub
2. Go to "Installs" → "Install Editor"
3. Select Unity 2022.3.XX LTS
4. Add following modules:
    - Windows Build Support (IL2CPP)
    - Windows Build Support (Mono)
    - Microsoft Visual Studio Community 2022
    - Documentation
      
- Create Project and Add ROS-TCP-Connector
1. Create new 3D project: "DroneSimulationViz"
2. Open Package Manager (Window → Package Manager)
3. Click "+" → "Add package from git URL"
4. Add: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`
