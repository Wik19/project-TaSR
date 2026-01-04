#!/bin/bash
xhost +local:root
echo "Starting Simulation..."
docker compose up -d --build
echo "Waiting for robot auto-boot..."
timeout 60s grep -q "SUCCESS: System is Ready!" <(docker compose logs -f ros_driver)

if [ $? -eq 0 ]; then
	echo "Robot is Ready!"
else
	echo "Timed out waiting for robot. Check logs."
fi

echo "Launching Camera Control..."
docker compose exec ros_driver bash -c "source /opt/ros/humble/setup.bash && python3 src/fake_camera_move.py"
