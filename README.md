**Hand gesture contolled:drone**


Hand Gesture-Controlled Drone Simulation in Gazebo Ignition | ROS2-Humble & OpenCV
🔹 Overview: Demonstration of a simulated drone controlled using hand gestures in Gazebo Ignition with ROS2.
 🔹 Tools Used:
MediaPipe - real-time hand tracking
OpenCV - image processing
ROS2 - drone control and communication
Gazebo Ignition - simulation

 🔹 Control Gestures:
✊ Closed Fist → Hover 
☝ 1 Finger → Move Up
✌ 2 Fingers → Move Forward
🤟 3 Fingers → Move Backward
✋ 5 Fingers → Move Down


OS: UBUNTU 22.04

ROS DISTRO: HUMBLE

Humble installation link::https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html



Clone this repo in your ros2 work directory 
Then build the package in your workspace ,

            colcon build --symlink-install

ROBOT with 2 depth camera LAUNCHING COMMAND:

In terminal 1:

    ros2 launch my_uavs ign_world_launch.py
Output:

![image](https://github.com/user-attachments/assets/0ebd579e-2bdf-4ca8-b4bb-1419e1a5f8ce)


In termihal 2:

    ros2 run my_uavs hand_gesture_drone_controller.py 
    
The result:


![image](https://github.com/user-attachments/assets/83ed8063-5923-4a81-be22-f2aec052487e)

            
