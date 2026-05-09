# Pick and Place Robotic System with Pose Estimation

A vision-based robotic pick and place system using **Franka Emika Panda Robotic Arm** in Gazebo simulation. The robot detects colored boxes using OpenCV pose estimation and uses MoveIt2 to autonomously pick and place them with precision.

---

</div>


##  Overview

This project simulates an autonomous robotic pick and place pipeline where:

1. A **camera** observes the Gazebo world
2. **OpenCV** detects colored rectangular boxes and estimates their 3D pose
3. **MoveIt2** plans a collision-free path to the target box
4. The **Franka Panda arm** picks the box and places it at the target location
---

##  Requirements

| Requirement | Version |
|-------------|---------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| Python | 3.10+ |
| OpenCV | 4.x |
| Gazebo | Ignition / Classic |
| MoveIt2 | Humble |

---

# Installation and setup

## Step 1: Install ROS 2 Humble

### 1.1 Set Locale

```bash
sudo apt update && sudo apt upgrade -y
locale  # check for UTF-8

sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2 Setup Sources

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.3 Install ROS 2 Packages

```bash
sudo apt update
sudo apt install ros-humble-desktop-full -y
```

### 1.4 Environment Setup

```bash

source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5 Install Development Tools

```bash
sudo apt install python3-rosdep python3-colcon-common-extensions python3-pip -y

sudo rosdep init
rosdep update
```

---

## Step 2: Install MoveIt 2 and Dependencies

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-common \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-visual-tools \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-setup-assistant \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros-gz \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rqt-image-view
```

---

## Step 3: Install Python Dependencies

```bash
pip3 install --no-cache-dir \
  opencv-python==4.10.0.84 \
  numpy==1.24.4 \
  transforms3d
```

---

## Step 4: Create and Build Workspace

### 4.1 Create Workspace

```bash
mkdir -p ~/panda_ws/src
cd ~/panda_ws
```

### 4.2 Clone the Repository

```bash
cd ~/panda_ws/src
git clone https://github.com/RishikaIITJ/pose-guided-pick-place-manipulator.git
```

### 4.3 Install Package Dependencies

```bash
cd ~/panda_ws
rosdep install --from-paths src --ignore-src --skip-keys=opencv_python -r -y
```

### 4.4 Build the Workspace

```bash
colcon build
```


### 4.5 Source the Workspace

```bash
source install/setup.bash

echo "source ~/panda_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 5: Verify Installation

Test that all packages are properly installed:

```bash

ros2 pkg list | grep panda

# Expected output:
# panda_bringup
# panda_controller
# panda_description
# panda_moveit
# panda_vision
```

---

## Running the Project

Open **two terminal windows**:

#### Terminal 1: Launch System

```bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```

#### Terminal 2: Start Pick-and-Place

```bash
source ~/panda_ws/install/setup.bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
```

**Change colors by stopping** (`Ctrl+C`) **and rerunning with different parameter:**

```bash
# For Green objects
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G

# For Blue objects
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B
```

---




## System Architecture

```
Camera Feed → Color and Pose Detection → Object Localization
                                        ↓
                                 Motion Planning (MoveIt 2)
                                        ↓
                               Trajectory Execution
                                        ↓
                         Pick Object → Move to Bin → Place
```
---
## Credits

This project builds upon
[Franka_Panda_Color_Sorting_Robot](https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot)
by MechaMind-Labs.
---



## 📄 License

This project is licensed under the MIT License.


</div>
