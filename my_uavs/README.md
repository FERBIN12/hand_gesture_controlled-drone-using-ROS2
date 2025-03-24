# Package documentation

This ROS 2 package is used to simulate unmanned aerial vehicles in Ignition Gazebo.

Author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado".<br>
Contact email: cmauricioae8@gmail.com
Udemy profile: https://www.udemy.com/user/cruz-mauricio-arteaga-escamilla/
LinkedIn: linkedin.com/in/cruz-mauricio-arteaga-escamilla/

For more information, please refer to the following YouTube channel: https://www.youtube.com/channel/UCNmZp0rCuWxqaKVljny2zyg


## Information sources

- ROS Documentation: https://docs.ros.org/en/humble/index.html
- Ignition vs Gazebo: https://www.allisonthackston.com/articles/ignition-vs-gazebo.html#:~:text=Both%20Ignition%20and%20Gazebo%20calculate,like%20they%20are%20in%20Ignition
- Some information about Ignition: https://gazebosim.org/docs/fortress/getstarted
- ROS and Ignition message type comparison: 
https://medium.com/@geetkal67/how-to-subscribe-to-ignition-gazebo-topics-using-ros2-8bcff7a0242e
- IgNition with ROS tutorials: https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
- Migration from Gazebo classic to SDF: https://gazebosim.org/api/gazebo/3.7/migrationsdf.html
- Mesh to fuel tutorial: https://gazebosim.org/api/gazebo/4.0/meshtofuel.html


## Installing and cloning ROS 2 packages

It is assumed that ROS 2 is already installed, if not, please refer to the following tutorial: <br>

https://www.youtube.com/watch?v=NhnX3aeruHs


After installing ROS 2, to avoid possible errors, please update your system and install the following ROS 2 dependencies.

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf2-* ros-$ROS_DISTRO-rviz-default-plugins
```

If the following error appears:<br>
_LookupError: Could not find the resource '<package_name>' of type 'packages'_

Try to install the correponding ROS dependency with

`sudo apt-get install ros-$ROS_DISTRO-<package-name>`

For example:

`sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui`

<br>

To install Ignition to work with ROS 2, run the following command:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

**Important:** If Gazebo classic has been also installed, and if the following error appears when launching the robot

`gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion px != 0' failed`

then run (at any directory):

```
. /usr/share/gazebo/setup.sh
```

This is needed to set some necessary environment variables in case they're going to be overridden, which is a common case when also using Ignition Gazebo simulator. 



## Setting user configuration and an empty world

To open Ignition using the default empty world, run:

```
ign gazebo empty.sdf
```

**Note:** the first time may take a while.

Arguments '-v' and '-r' can be added to use verbose and to run the simulation from start, respectively.

Once the world was loaded, user can add some useful plugins into the scene by clicking on the three vertical points icon at the top-right button.
Some additional common plugins are: 'View Angle', 'Topic Viewer', 'Topic Echo', 'Visualize Lidar', 'Teleop', 'Image Display' and 'Key Publisher'.

At `/usr/share/ignition/ignition-gazebo6/worlds/`, several worlds examples sdf files can be found. Some important files are listed below.

`ign gazebo quadcopter.sdf`

`ign gazebo multicopter_velocity_control.sdf`



## Ignition topics only (not ROS topics)

To list all ign topics:

```
ign topic -l
```

Sending velocity commands:

```
ign topic -t "/r1/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.1} angular {z: 0}"
```

To hover:

```
ign topic -t "/r1/gazebo/command/twist" -m ignition.msgs.Twist -p " "
```

Listen to odometry:

```
ign topic -e -t "/model/r1/odometry"
```

To communicate with ROS, it is required a node that converts from ign type messages to ROS type messages.
More details will be given later.



## Launching a custom world with a custom model

For a custom model, first some environment variables must be set in the '.bashrc' file, according to the path for the models folder, for example:

```
export IGN_GAZEBO_RESOURCE_PATH=/home/username/.ignition/models
```

You can also set several lookup paths separating them with :, for example:

`export IGN_GAZEBO_RESOURCE_PATH=/home/username/.ignition/models:~/colcon_ws/src/my_uavs/models`

<span style="color:green">
For simplicity, the model folder must be copied at /home/user/.ignition/models.
</span>

<br>
With this, the model can be added in a world.sdf file by using the following snippet of code inside the world tag:

```xml
<include>
  <static>false</static>
  <name>model_name</name>
  <pose>X Y Z Roll Pitch Yaw</pose>
  <uri>model://model_name</uri>
</include>
```

Therefore, the `model://` prefix will be substituted by GAZEBO_MODEL_PATH env variable.

To launch the custom world:

```
ign gazebo ~/colcon_ws/src/my_uavs/worlds/my_custom_world.sdf
```


The 'my_custom_world.sdf' file spawns a custom quadcopter. The model is defined at models folder.
The ' ~/colcon_ws/src/my_uavs/models/r1/model.config' set the configuration file only, and the 'my_drone.sdf' file describes the custom drone using namespaces for all links.<br>
Commonly, the 'my_drone.sdf' file is called model.sdf for any model.



# Ignition and ROS 2

To be able to communicate our simulation with ROS 2, you need to use a package called 'ros_gz_bridge'. This package provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo transport. You can install this package by typing:

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-ign-bridge
```

To use ROS topics, the following node must be run

`ros2 run ros_gz_bridge parameter_bridge \<arguments> --ros-args -r /topic_name:=/new_topic_name`

where arguments are the conversions between messages type. Topics remaps can be done as well. For example:

```bash
ros2 run ros_gz_bridge parameter_bridge \
/r1/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist \
/model/r1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry \
--ros-args -r /r1/gazebo/command/twist:=/r1/cmd_vel -r /model/r1/odometry:=/r1/odom
```

**Note:** the conversion from ign type to ROS type is set according every topic. Also, at once topics remap are used.

The ROS message type is followed by an @, [, or ] symbol where:

@ is a bidirectional bridge.<br>
[ is a bridge from Ignition to ROS.<br>
] is a bridge from ROS to Ignition.


Now, after remapping, to publish a velocity from terminal, using ROS commands:

`ros2 topic pub --once /r1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1, y: 0.0, z: 0.1}, angular: {z: -0.3}}"`

To see the robot posture, in case you are not interested in covariance matrices, you can use the --no-arr parameter to hide arrays:

`ros2 topic echo /r1/odom --no-arr`

In adittion, a launch.py file can be configured to run the 'ros_gz_bridge' node with this configuration.


