# Pioneer

Simplified version of the Pioneer Packages from [Jen Jen Chung](https://github.com/JenJenChung).

I kept the minimal necessary for navigation using a Adept MobileRobots Pioneer Robot. 

All of the code is based on packages from [Jen Jen Chung](https://github.com/JenJenChung).

Dependencies: 

[ROSARIA](http://wiki.ros.org/ROSARIA).

[Jen Jen Chung's pioneer_test](https://github.com/JenJenChung/pioneer_test).

On a pioneer, run:

```rosrun pioneer pioneer.sh localization $(rospack find pioneer)/maps/printer_000.yaml```

[Optional] On a desktop computer, run:

```roslaunch pioneer teleop_ps3.launch```



For mapping, launch on the pioneer:

```rosrun pioneer pioneer.sh mapping```

