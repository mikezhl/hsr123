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

## Instructions - Basic
#### Description
- Pick up a soda can in a fixed place without stopping.
#### Run a trajectory
- Start the gazebo simulation: `roslaunch hsr123 basic.launch`
- Wait for gazebo to finish loading and press play to start the simulation
- In an new terminal window, run `hsr123/src/basic_traj_follow.py`
#### Generate a new trajectory using AICO
- Start the rviz if gazebo isn't running: `roslaunch hsr123 basic_rviztest.launch`
- In an new terminal window, source `devel/setup.bash` and run `hsr123/src/basic_traj_aico.py`

## Instructions - Detection
#### Description
- Pick up a soda can in a fixed place with apriltag. A modified version of the basic one above, remove the hardcoded part.
#### Run!
- Srat the gazebo simulation, `roslaunch hsr123 detect.launch`
- Choice 1: Run function `detect_traj_follow(4.5, 0.15, 2.5, new_traj=1)` in `hsr123/src/detect_traj_follow.py` to generate and run the new trajectory.
- Choice 2: Run `hsr123/src/detect_traj_aico.py` first to generate the trajectory, and then run function `detect_traj_follow(4.5, 0.15, new_traj=0)` in `hsr123/src/detect_traj_follow.py`to run the trajectory

## Instructions - Pick Up
### Description
- Pick up a soda can in a random place with apriltag from a random start place. Using IK, AICO and RRT solver.
#### Run!
- Set the starting and ending position of the robot. Set the position of the can since the detection is not working well in gazebo.
- Start the rviz if gazebo isn't running: `roslaunch hsr123 basic_rviztest.launch`, run `hsr123/src/pickup_plan.py` to generate all the trajectories needed.
- Run the trajectory in gazebo, firstly start the gazebo with `roslaunch hsr123 pickup.launch`, then run `hsr123/src/pickup_follow.py`.