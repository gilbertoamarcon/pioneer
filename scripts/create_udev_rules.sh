#!/bin/bash

echo "Remapping the device serial port (ttyUSBX) to rplidar and rosaria"
echo "Check it using the command: ls -l /dev|grep ttyUSB"
echo "Copying *.rules to /etc/udev/rules.d/"
sudo cp `rospack find pioneer`/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp `rospack find pioneer`/scripts/rosaria.rules /etc/udev/rules.d/
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
