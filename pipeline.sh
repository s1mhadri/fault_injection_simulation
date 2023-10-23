#!/bin/bash

# for (( i=0;$i<50;i=`expr $i+1`))
for (( i=0; i<1; i=i+1 ))
do
    gnome-terminal -t "start_gazebo" --geometry=80x20+2000+0 -- bash -c "source devel/setup.bash;roslaunch panda_gazebo start_simulation.launch; exec bash"
    sleep 2s

    gnome-terminal -t "start_monitor" --geometry=80x20+2000+540 -- bash -c "source devel/setup.bash;rosrun failure_monitor failure_monitor; exec bash"
    sleep 8s

    gnome-terminal -t "start_injecting" --geometry=80x20+3000+0 -- bash -c "source devel/setup.bash;rosrun joint_state_publisher fault_injector; exec bash"
    sleep 2s
   
    gnome-terminal -t "start_playing" --geometry=80x20+3000+540 -- bash -c "source devel/setup.bash;rosrun moveit_tutorials own_pick_place_VJZL"
    sleep 2s
    
    gnome-terminal -t "record" --geometry=80x20+3000+540 -- bash -c "source devel/setup.bash;rosbag record -o ./rosbags/data.bag /record __name:=ros_record"
    sleep 56s
    
    gnome-terminal -t "kill_node_1" -- bash -c "source devel/setup.bash;rosnode kill /ros_record; sleep 1s; exit"
    sleep 1s

    gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}' | xargs kill -9;ps aux | grep rviz | awk '{print \$2}' | xargs kill -9;exec bash"
    sleep 2s

done



