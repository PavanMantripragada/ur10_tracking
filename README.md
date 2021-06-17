### ur10_tracking
#### (ROS package for moving UR10 arm to track a live position marker)
---

#### Dependencies
- ROS melodic is required!
- MoveIt ROS Package, it can be installed by following command
  - `sudo apt install ros-melodic-moveit`
  - more details can be found [here](https://moveit.ros.org/install/).
- ROS-Industrial Universal Robot meta-package, installation instructions can be found [here](https://github.com/ros-industrial/universal_robot).

#### Setup
Add this package to the src folder of your catkin workspace and execute following commands
  ```bash
  source /opt/ros/melodic/setup.bash
  cd ~/workspace/
  catkin_make
  source ~/workspace/devel/setup.bash
  ```
#### Run the code
- Start the UR10 simulation in Gazebo
  - `roslaunch ur_gazebo ur10.launch limited:=true`
- Start movegroup for UR10 
  - `roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true`
- Start rviz 
  - `rosrun rviz rviz`
- To start the interactive marker server node
  - `rosrun ur10_tracking ur10_im_tracker`
  - Now add an interactive marker in Rviz and set it to /simple_marker
  - Move the marker and hold it in a position to move the arm.

  ![](https://github.com/DrKraig/ur10_tracking/blob/master/media/tracker.gif)