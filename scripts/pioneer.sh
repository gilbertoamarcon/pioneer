#!/bin/bash

## Usage: rosrun pioneer pioneer.sh localization $(rospack find pioneer)/maps/printer_000.yaml
## Usage: rosrun pioneer pioneer.sh mapping

# Command-line argument renaming 
mode=$1
map_name=$2

echo "Launching roscore..."
roscore &
pid=$!
sleep 5s

echo "Connecting to RosAria..."
roslaunch pioneer rosaria.launch &
pid="$! $pid"
sleep 3s

echo "Launching sensors..."
roslaunch pioneer sensors.launch &
pid="$! $pid"

sleep 3s

echo "Launching mapping/localization stack..."
roslaunch pioneer $mode.launch map_name:=$map_name &
pid="$! $pid"

sleep 3s

echo "Launching pioneer controller..."
roslaunch pioneer controller.launch &
pid="$! $pid"

sleep 3s

echo "Launching robot description..."
roslaunch pioneer description.launch &
pid="$! $pid"

sleep 2s

trap "echo Killing all processes.; kill -2 $pid; exit" SIGINT SIGTERM

sleep 24h
