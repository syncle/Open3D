#!/usr/bin/env bash

reconstruct_scene()
{
    CONFIG="config/"$1".json"
    COMMAND="--make --register --refine --integrate"
    python run_system.py $CONFIG $COMMAND
}

declare -a dataset=("redwood_objects/car"
    "redwood_objects/chair"
    "redwood_objects/motorcycle"
    "redwood_objects/plant"
    "redwood_objects/truck"
    "redwood_simulated/livingroom1-clean"
    "redwood_simulated/livingroom1-simulated"
    "redwood_simulated/livingroom2-clean"
    "redwood_simulated/livingroom2-simulated"
    "redwood_simulated/office1-clean"
    "redwood_simulated/office1-simulated"
    "redwood_simulated/office2-clean"
    "redwood_simulated/office2-simulated"
    "stanford/burghers"
    "stanford/cactusgarden"
    "stanford/copyroom"
    "stanford/lounge"
    "stanford/stonewall"
    "stanford/totempole"
    "indoor_lidar_rgbd/apartment"
    "indoor_lidar_rgbd/bedroom"
    "indoor_lidar_rgbd/boardroom"
    "indoor_lidar_rgbd/lobby"
    "indoor_lidar_rgbd/loft"
    )

for i in "${dataset[@]}"
do
    cd ..
    reconstruct_scene "$i"
    cd scripts
done
