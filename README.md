# Introduction

This project is a part of my Master Thesis at University of Stuttgart. 

This source code works as a environment playground for franka emika panda robot only, following steps depict quick start guide to setup the environment and run fault injection experiments. 


## Folder Structure

- build: contains all the ROS build files
- devel: contains all the ROS development files
- src
  - [failure_monitor](src/failure_monitor/): this package contains the failure monitor module to monitor environment
    - [scripts](src/failure_monitor/scripts/): source code
  - [joint_state_publisher](src/joint_state_publisher/joint_state_publisher/): this package containes the fault injection module and recorder
    - [msg](src/joint_state_publisher/joint_state_publisher/msg/): custom message
    - [scripts](src/joint_state_publisher/joint_state_publisher/scripts/): source code
  - [moveit](src/moveit/): moveit controller code and configurations for panda robot
    - custom pick place code (editable version) please go through the ROS C++ design schematic before editing and building this file : [own_pick_place_VJZL.cpp](src/moveit/moveit_tutorials/doc/pick_place/src/own_pick_place_VJZL.cpp)
  - [panda gazebo](src/panda_gazebo/): panda robot package  to support gazebo simulation this package also houses robot and its environment
    - [resources](src/panda_gazebo/resources/): has all the required models and cube sdf for simulation
      - [models](src/panda-gazebo/panda_gazebo/resources/models/): add or remove models if wanted
      - [world](src/panda-gazebo/panda_gazebo/resources/worlds/): defines gazebo world the main world file used is [pick_and_place4.world](src/panda_gazebo/resources/worlds/pick_and_place4.world)
  - [random_position_generator](src/random_position_generator/): this package is used to generate random positions for the cube and for the robot to move to
  - [risk_assessment](src/risk_assessment/): this package contains the risk assessment module to assess the risks involved during the operation/simulation
    - [scripts](src/random_position_generator/scripts/): source code
- [fault_config](fault_config.yaml): configuration file for fault injection
- [pipeline](pipeline.sh): bash script to run the pipeline

# Quick Start

## Requirements
- `pip3 install -r requirements.txt`

## build

`catkin build -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build`

## FI Experiment

- `bash pipiline.sh`

this starts random fault injection experiment and saves the bag files into [rosbags](rosbags/) folder
