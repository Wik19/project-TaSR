#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

ROBOT_HOST=${ROBOT_IP:-ursim}
PROGRAM_NAME="WORKS.urp"

# Function to send command to Dashboard Server
send_dashboard_command() {
    echo "$1" | nc -w 1 $ROBOT_HOST 29999
}

echo "--- ROBOT SETUP START ---"

# 1. Wait for Simulator Network
echo "[Setup] Waiting for Simulator connectivity..."
while ! nc -z $ROBOT_HOST 29999; do
  sleep 1
done
echo "[Setup] Simulator is Online!"

# 2. Boot Sequence
echo "[Setup] Powering ON robot..."
send_dashboard_command "power on"
sleep 5

echo "[Setup] Releasing Brakes..."
send_dashboard_command "brake release"
sleep 5

echo "[Setup] Loading Program: $PROGRAM_NAME..."
send_dashboard_command "load $PROGRAM_NAME"
sleep 2


echo "--- ROBOT READY. LAUNCHING ROS DRIVER ---"

# 3. Launch ROS Driver
(
    sleep 15
    echo "[Auto-Start] Loading controllers..."
    ros2 control load_controller joint_state_broadcaster || true
    ros2 control load_controller scaled_joint_trajectory_controller || true

    echo "[Auto-Start] Configuring controllers..."
    ros2 control set_controller_state joint_state_broadcaster inactive || true
    ros2 control set_controller_state scaled_joint_trajectory_controller inactive || true
    
    echo "[Auto-Start] Activating controllers..."
    for i in {1..10}; do
        ros2 control set_controller_state joint_state_broadcaster active || true
        if ros2 control set_controller_state scaled_joint_trajectory_controller active; then
            echo "[Auto-Start] SUCCESS: System is Ready!"
            break
        fi
        sleep 3
    done
) &

exec "$@"