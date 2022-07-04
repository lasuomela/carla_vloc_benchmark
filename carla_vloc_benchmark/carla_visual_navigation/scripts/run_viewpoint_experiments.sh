#!/bin/bash

roll_dir=/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/roll
yaw_dir=/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/yaw
zpitch_dir=/opt/carla_vloc_benchmark/src/carla_visual_navigation/config/viewpoint_experiment_objects/z_pitch

tmux new -s "$session_uuid"
tmux send -t "$session_uuid:" "clear" Enter
tmux splitw -t "$session_uuid:" -dh


tmux send -t "$session_uuid:.1" "clear ; printf 'Executing viewpoint-experiments... Starting with roll changes.\n' ; sleep 2 " Enter

counter=1
for entry in "$yaw_dir"/*
do
  tmux send -t "$session_uuid:.1" "sleep 20 ; \
  tmux send -t '$session_uuid:' 'ros2 launch carla_visual_navigation rviz_scenario_runner.launch.py town:='Town01' objects_config:='$entry'' Enter ; \
  sleep 45 ; ros2 launch carla_visual_navigation scenario_executor.launch.py scenario_dir:='/scenarios/viewpoint_experiment_yaw${counter}_town01' repetitions:=5 ; \
  tmux send -t '$session_uuid:' C-c ; counter=$((counter++))" Enter
done


tmux attach -t "$session_uuid"
tmux send -t"$session_uuid:.1" C-d
