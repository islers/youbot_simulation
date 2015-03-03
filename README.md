# Youbot Simulation

Catkinized and updated version of KUKA's outdated youbot-ros-pkg/youbot_common package. Updated to work under ROS indigo and Gazebo 4.

##Instructions and Information
ROS features two basic ways to visualize robots on the computer: RViz and Gazebo. While RViz is a visualization tool for ros topics, Gazebo is a simulation environment that features a physics engine, modeling the forces exerted on robots and other objects in your simulation environment.

### Gazebo
Gazebo has been integrated with ROS from the beginning. Since version 1.9 it is a stand-alone tool and has ROS-independent development. Still, Gazebo is included in the ROS repositories, though only in the version released at the time of the latest ROS release (which is Gazebo 2.x for Indigo). To get the latest release, you'll have to install from OSR repositories.
#### Installation
Whether or not to install the latest Gazebo or the one featured with your ROS version depends on your needs: There is detailed instruction and information about installing Gazebo for usage with ROS given in the Gazebo tutorials. Refer to [Which combination of ROS/Gazebo versions to use](http://gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros) and the [ROS integration overview](http://gazebosim.org/tutorials?tut=ros_overview&cat=connect_ros). Then adjust the instructions given for [Gazebo installation](http://gazebosim.org/tutorials?cat=install) and the [installation of the gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros), which is needed for integration of Gazebo with ROS, accordingly. The [Gazebo tutorials](http://gazebosim.org/tutorials) are a good reference in general.  
Gazebo can be started with GUI, showing the environment and robot, or without if you don't need any visualization or want to use RViz for that purpose.  
Note: Gazebo occasionally crashes at startup without any apparent reason. If the GUI appears frozen, watch out for a notification in the command line and restart the program.

### Youbot Simulation
#### Setup
KUKA provides 3D models of the Youbot along with controllers and launch files for simulation in Gazebo. The package has however not been updated since ROS Fuerte and is no longer working. What you find in this repository is a catkinized and updated version of the original, which should work out of the box under ROS Indigo and also with Gazebo 4. Just as the Youbot driver it still depends on PR2 and brics_actuator packages which you'll need to add if you haven't done so yet.
#### What you get
The simulation emulates the physics and all topics and messages you get with the real Youbot. This means that you can use any Youbot controller for the real Youbot or its simulation and any visualization for the real Youbot or its simulation (see the RViz chapter). Its behaviour should basically mirror the behaviour of the real Youbot. If you want to use the JointTrajectory action server, e.g. to receive commands from MoveIt, you will most likely have to revise the JointTrajectoryAction class. If you need your youbot to only follow the positions in trajectories without considering velocities, accelerations and timing, use the develop branch.
#### Launch the simulation
A set of launch files are provided to start up the Youbot simulation in Gazebo. You can launch the whole robot or arm and base separately.  
  
**Complete Youbot:**  
`roslaunch youbot_description youbot_publisher.launch`  
**Complete Youbot without GUI:**  
`roslaunch youbot_description youbot_publisher_no_x.launch`
  
  
For arm or base only(fill in 'arm' or 'base' for $):  
`roslaunch youbot_description youbot_$_publisher.launch`
#### Use your own environment
You can create your own 3D environment in Gazebo with whatever objects you like and then spawn the Youbot inside of it. To do that, uncomment the line  
`<!--<arg name="world_name" value="$(find youbot_gazebo)/worlds/sim_1.world"/>	--> `  
in the launch file and fill in the path to your own gazebo world file.

### Visualization with RViz
If you want RViz to visualize the Youbot (either the one in simulation or the real one), RViz needs the robot's URDF description file pushed on the ROS parameter server in the "robot_description" variable. The Gazebo simulation launch files do that since Gazebo uses the same variable. If you'd like to visualize the real Youbot however, you can use the following launch file provided in the youbot_description package. Simply do:  
`roslaunch youbot_description youbot_description.launch`  
  
Then start RViz.    
`rosrun rviz rviz`  
  
With the description on the parameter server, you can load the Youbot by choosing **Add>>RobotModel**. The robot movement is visualized by applying the TF-tree (published on the /tf - topic) to the robot links. The TF-tree represents the relative positions of all links to their respective parent link. It is published by the "robot_state_publisher"-node for the simulation, which should have been launched already by the launch file. In order to make the model visible in RViz you need to tell the program the name of the base link. If you want to see the movement of the base, choose "odom" (for odometry) under **Global Options>>Fixed Frame** in the "Displays" section. To fix the base and only make arm movements visible, you could for instance choose the "base_footprint" link.
