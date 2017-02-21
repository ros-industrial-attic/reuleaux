#!/bin/bash

if [ "$1" != "" ]; then
  echo "hello"
  echo "$PWD/$1"

  
  roslaunch "$1" planning_context.launch load_robot_description:=true
  roslaunch "$1" move_group.launch allow_trajectory_execution:=true fake_execution:=true info:=true
else 
  echo "$0: Please provide a moveit package for your robot:"
fi
