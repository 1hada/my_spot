
I took what I learned from the Gazebo ROS demo ( specifically http://gazebosim.org/tutorials/?tut=ros_urdf )
to make a frame for my own spot. 
My main goals for the project were to become familiar with macros.
I in turn also created a base c++ robot class which I believe will
be useful in the future. The robot moves through a transmission/ actuator.

Enjoy!

![My spot demo](https://raw.githubusercontent.com/1hada/my_spot/master/my_spot_first_posture.gif)


# Prerequisites

	Ubuntu 18.04 
	ROS melodic
	gazebo9

These are the steps I take. They may be different for you depending on how you set your machine up.

#STEP1
```
catkin_create_pkg my_spot
```
#STEP2
```
cd ~/catkin_ws
catkin build
. ~/catkin_ws/devel/setup.bash
```
#STEP3
```
roslaunch my_spot my_spot_world.launch
```
#STEP4
```
cd ~/catkin_ws/src/my_spot/src

g++ move_robot.cpp -o execute_move_robot -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization

./execute_move_robot 
```

