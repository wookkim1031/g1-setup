#!/bin/bash
# Run DLIO offline on a ROS 2 bag (a PC with ROS 2 Humble). Usage: dlio_run.sh <bag_dir> <out_dir> [rate]
# Isolated DDS domain + localhost only: never touches the G1's DDS domain 0.
set +u
BAG=$1; OUT=$2; RATE=${3:-1.0}; DOMAIN=${4:-42}
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/dlio_ws/install/setup.bash
if pgrep -f "dlio_odom_node|dlio_map_node|ros2 bag play" > /dev/null; then echo "ERROR: another DLIO / bag play is still running:"; pgrep -af "dlio_odom_node|dlio_map_node|ros2 bag play"; exit 1; fi
mkdir -p "$OUT"; cd "$OUT"
rm -rf odom_bag
ros2 launch direct_lidar_inertial_odometry dlio.launch.py rviz:=false \
    pointcloud_topic:=/livox/lidar imu_topic:=/livox/imu > dlio.log 2>&1 &
DLIO=$!
sleep 4
ros2 bag record -o odom_bag /dlio/odom_node/odom /dlio/odom_node/pose /dlio/odom_node/keyframes /dlio/odom_node/pointcloud/deskewed > record.log 2>&1 &
REC=$!
sleep 3
echo "[$(date +%T)] playing $BAG at rate $RATE"
ros2 bag play "$BAG" --clock --rate "$RATE" --topics /livox/lidar /livox/imu --disable-keyboard-controls < /dev/null > play.log 2>&1
echo "[$(date +%T)] play finished, waiting for DLIO to drain"
sleep 5
echo "[$(date +%T)] saving map (leaf 0.05 -> map_0.05/, leaf 0.02 -> map_0.02/)"
mkdir -p map_0.05 map_0.02
timeout 900 ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD "{leaf_size: 0.05, save_path: '$OUT/map_0.05'}"
timeout 900 ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD "{leaf_size: 0.02, save_path: '$OUT/map_0.02'}"
sleep 2
kill -INT $REC; sleep 3
kill -INT $DLIO; sleep 5
pkill -INT -f dlio_odom_node; pkill -INT -f dlio_map_node; sleep 3; pkill -9 -f dlio_odom_node; pkill -9 -f dlio_map_node
pgrep -af "dlio_odom_node|dlio_map_node" && echo "WARNING: DLIO nodes survived" || echo "nodes stopped"
echo "[$(date +%T)] done"; ls -la map_0.05 map_0.02 odom_bag
