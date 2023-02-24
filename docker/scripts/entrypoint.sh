#!/bin/bash
set -ex

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# add source to bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

exec /usr/bin/tini -- /usr/bin/supervisord -n -c /app/supervisord.conf