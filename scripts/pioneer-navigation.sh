#!/bin/bash

echo "Launching roscore..."
roscore &
pid=$!
sleep 5s

echo "Connecting to RosAria..."
rosrun rosaria RosAria &
pid="$! $pid"
sleep 3s

echo "Launching sensors..."
roslaunch pioneer_test pioneer_sensors.launch &
pid="$! $pid"

sleep 3s

echo "Launching navigation stack..."
roslaunch pioneer navigation.launch map_name:=$(rospack find pioneer)/maps/printer_000.yaml &
pid="$! $pid"

sleep 3s

echo "Launching pioneer controller..."
roslaunch pioneer_test pioneer_controller.launch &
pid="$! $pid"

sleep 3s

echo "Launching robot description..."
roslaunch pioneer_test pioneer_description.launch &
pid="$! $pid"

sleep 2s

trap "echo Killing all processes.; kill -2 $pid; exit" SIGINT SIGTERM

sleep 24h
