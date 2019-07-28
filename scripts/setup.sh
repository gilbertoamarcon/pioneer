#!/usr/bin/env bash


# Check that computer is up to date
sudo apt-get update
sudo apt-get upgrade --yes
# NOTE: Check that "restricted", "universe", and "multiverse" are allowed repository sources (they are by default)

# Install pleasentries, like vim and git
sudo apt-get -y install git
sudo apt-get -y install openssh-server

# ROS Kinetic installation commands
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
printf "\n# ROS:\nsource /opt/ros/kinetic/setup.bash\n" >> $HOME/.bashrc
source $HOME/.bashrc
sudo apt-get -y install python-rosinstall
sudo apt-get -y install python-rosinstall-generator
sudo apt-get -y install python-wstool
sudo apt-get -y install build-essential

# Install ROS packages needed to run pioneer code
sudo apt-get -y install ros-kinetic-navigation
sudo apt-get -y install ros-kinetic-gmapping
sudo apt-get -y install ros-kinetic-ros-control
sudo apt-get -y install ros-kinetic-ros-controllers
sudo apt-get -y install ros-kinetic-rviz 

# Install ARIA (dependency of rosaria)
sudo dpkg -i setup/libaria_2.9.1+ubuntu16_amd64.deb

# Set up catkin
source $HOME/.bashrc
mkdir -p $HOME/catkin_ws/src
cd $HOME/catkin_ws/src
catkin_init_workspace
cd $HOME/catkin_ws
catkin_make
echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

# Setup Pioneer code
cd $HOME/catkin_ws/src
git clone https://github.com/amor-ros-pkg/rosaria.git
git clone https://github.com/robopeak/rplidar_ros.git
git clone https://github.com/gilbertoamarcon/pioneer.git

source $HOME/.bashrc
cd $HOME/catkin_ws/
catkin_make

$HOME/catkin_ws/src/pioneer/scripts/create_udev_rules.sh

# Color Terminal
sed -i '/force_color_prompt=yes/s/^#//g' ${HOME}/.bashrc

# Removing Examples folder
rm -Rf $HOME/examples.desktop

# Disable Ubuntu Crash Reports
sudo sed -i 's/enabled=1/enabled=0/g' /etc/default/apport

# Clean up apt, since it will probably have unnecessary packages now
sudo apt-get -y autoremove
