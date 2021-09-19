#! /bin/bash

ROS_FOLDER="$HOME/.ros/"

MAP_FILES=($(ls -alvr $ROS_FOLDER | grep 'backup_map_' | awk -F':[0-9]* ' '/:/{print $2}'))
NB_MAP_FILES=${#MAP_FILES[@]}
if [ $NB_MAP_FILES -eq 0 ]; then
  echo "No backup map to load..."
  MAP_TO_LOAD=""
elif [ $NB_MAP_FILES -eq 1 ]; then
  echo "Loading backup map from disk..."
  MAP_TO_LOAD=$ROS_FOLDER${MAP_FILES[0]}
elif [ $NB_MAP_FILES -eq 2 ]; then
  echo "Loading backup map from disk..."
  MAP_TO_LOAD=$ROS_FOLDER${MAP_FILES[0]}
else
  echo "Loading backup map from disk..."
  MAP_TO_LOAD=$ROS_FOLDER${MAP_FILES[1]}
fi

LOCALIZATION_FILES=($(ls -alvr $ROS_FOLDER | grep 'backup_localization_' | awk -F':[0-9]* ' '/:/{print $2}'))
NB_LOCALIZATION_FILES=${#LOCALIZATION_FILES[@]}
if [ $NB_LOCALIZATION_FILES -eq 0 ]; then
  echo "No backup localization to load..."
  LOCALIZATION_TO_LOAD="[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]"
elif [ $NB_LOCALIZATION_FILES -eq 1 ]; then
  echo "Loading backup localization from disk..."
  LOCALIZATION_TO_LOAD=`cat $ROS_FOLDER${LOCALIZATION_FILES[0]}`
elif [ $NB_LOCALIZATION_FILES -eq 2 ]; then
  echo "Loading backup localization from disk..."
  LOCALIZATION_TO_LOAD=`cat $ROS_FOLDER${LOCALIZATION_FILES[0]}`
else
  echo "Loading backup localization from disk..."
  LOCALIZATION_TO_LOAD=`cat $ROS_FOLDER${LOCALIZATION_FILES[1]}`
fi

rosparam set /mapper_node/initial_map_file_name "$MAP_TO_LOAD"
rosparam set /mapper_node/initial_robot_pose "`echo $LOCALIZATION_TO_LOAD`"
$HOME/getIcp.sh
rosparam delete /mapper_node/initial_map_file_name
rosparam delete /mapper_node/initial_robot_pose
