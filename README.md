# Pioneer #



Simplified version of the Pioneer Packages from [Jen Jen Chung](https://github.com/JenJenChung) for navigation and mapping with an Adept MobileRobots Pioneer 3 Robot and an RPLidar Laser Scanner.

Dependencies: 
* [ROSARIA](http://wiki.ros.org/ROSARIA)
* [RPLIDAR ROS](http://wiki.ros.org/rplidar)



## Setup ##

Go to your workspace src directory:

```roscd ; cd ../src/```

Clone the dependencies and the main package:

```git clone https://github.com/amor-ros-pkg/rosaria.git```

```git clone https://github.com/robopeak/rplidar_ros.git```

```git clone git@github.com:gilbertoamarcon/pioneer.git```

```rosdep update```

```rosdep install ROSARIA```

Go to the root workspace directory:

```cd ../ ```

Run catkin_make:

```catkin_make ```



## Usage ##


### On the pioneer computer ###

For localization:

```rosrun pioneer pioneer.sh localization $(rospack find pioneer)/maps/printer_000.yaml```

For mapping:

```rosrun pioneer pioneer.sh mapping```


### On a workstation computer ###

[Optional] To visualize with Rviz and teleoperate:

```roslaunch pioneer teleop_ps3.launch```




