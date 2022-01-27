# HSR motion planning

# Description
https://github.com/ori-drs/hsr_driveby_full

## Requirements:
System requirements:
- ROS Noetic
- Follow the HSR simulator installation instructions on the [Installing HSR Packages and Simulator Locally](https://github.com/ori-orion/orion-documentation/wiki/Installing-HSR-Packages-and-Simulator-Locally)

Set up workspace and install required dependencies:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:zhlzhl123/hsr.git
git clone https://github.com/ToyotaResearchInstitute/hsr_meshes.git
sudo apt install ros-noetic-exotica-examples
```

## Instructions
#### Description
- Pick up a soda can.
#### Run a trajectory
- Start the gazebo simulation:
`roslaunch hsr123 basic.launch`
- Wait for gazebo to finish loading and press play to start the simulation
- In an new terminal window, run `hsr123/src/traj_follow.py`
#### Generate a trajectory using AICO
- Start the rviz:
`roslaunch hsr123 rviz_test.launch`
- In an new terminal window, source `devel/setup.bash` and run `hsr123/src/traj_aico.py`