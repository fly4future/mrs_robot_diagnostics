#!/bin/bash

# Check if UAV_NAME is empty
if [ -z "$UAV_NAME" ]; then
    echo "Error: UAV_NAME environment variable is not set."
    echo "Usage example: export UAV_NAME=uav1 && ./takeoff.sh"
    exit 1
fi

echo "Arming $UAV_NAME..."
ros2 service call /$UAV_NAME/hw_api/arming std_srvs/srv/SetBool "{data: true}"

# Wait a moment to ensure the arming service has time to process
sleep 1.0

echo "Toggling offboard for $UAV_NAME..."
ros2 service call /$UAV_NAME/hw_api/offboard std_srvs/srv/Trigger "{}"

echo "Sequence complete."
