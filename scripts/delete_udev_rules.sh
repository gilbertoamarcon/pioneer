#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to rplidar and rosaria"
sudo rm  /etc/udev/rules.d/rplidar.rules
sudo rm  /etc/udev/rules.d/rosaria.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish delete"
