# komodo_coffee_taker
ROS add-on package for the komodo (with elevator) - the package is under construction

Instructions
============
1. clone your workspace to https://github.com/robotican/ric.git

2. clone your workspace to https://github.com/orsalmon/komodo_coffee_taker.git

3. catkin_make your workspace

4. roslaunch komodo_coffee_taker run_komodo_gazebo.launch

5. rosrun komodo_coffee_taker komodo_coffee_taker

6. rosservice call /coffee_taker TAB and fill the data: position [m] & orientation [deg]
