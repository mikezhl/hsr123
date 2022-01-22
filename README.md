# HSR motion planning

# Description
https://github.com/ori-drs/hsr_driveby_full

## Requirements:
- ROS Noetic
- Follow the HSR simulator installation instructions on the [Installing HSR Packages and Simulator Locally](https://github.com/ori-orion/orion-documentation/wiki/Installing-HSR-Packages-and-Simulator-Locally)

Set up workspace and install required dependencies:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:zhlzhl123/hsr.git
git clone https://github.com/wxmerkt/hsr_description.git
git clone https://github.com/ToyotaResearchInstitute/hsr_meshes.git
git clone https://github.com/hsr-project/tmc_wrs_gazebo.git
sudo apt install ros-noetic-exotica-examples
```
## Instructions
#### Description
- Pick up a soda can.
#### Run
- Start the gazebo simulation:
`roslaunch hsr123 basic.launch`
- Wait for gazebo to finish loading and press play to start the simulation
- In an new terminal window, `python3 src/hsr123/src/follow.py`

<!-- - 3 choices:
  - 1. Run a saved trajectory output of AICO solver `./hsrb_pickup.py`
  - 2. Rerun AICO solver `./hsr_meeting_table_aico`
  - 3. comment and uncomment line in main function. -->