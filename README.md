ARENA and CoopExp (Version 1.0 - September 2016)
---------------------------------------

# ARENA cooperAtive multi-Robot frontiEr exploratioN simulAtor

# CoopExp Cooperative Multi-Robot Exploration

---------------------------------------

Wiki: http://wiki.ros.org/arena and http://wiki.ros.org/coopexp
Source: https://github.com/ruipiresc/arena

Notes:
--------------
* Before the first launch you should change the "path_to_arena" param to your path to arena, should be something like this

	arg name="path_to_arena" default="/home/your_username/your_ws/src/arena"/

* Is required a full instalation of ROS Indigo on Ubuntu 14.04 or higher

	http://wiki.ros.org/indigo/Installation/Ubuntu

*In order to launch ARENA there a very intuitive launch file named "multirobot.launch" so just edit as you want and run (CoopExp algorithm included, but you can develop your own exploration package and use it in ARENA):

	roslaunch simulator multirobot.launch
  
