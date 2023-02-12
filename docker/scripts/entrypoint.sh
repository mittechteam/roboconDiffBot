#!/bin/bash
set -ex

RUN_XTERM=${RUN_XTERM:-yes}

case $RUN_XTERM in
  false|no|n|0)
    rm -f /app/conf.d/xterm.conf
    ;;
esac

# setup ros2 environment
# source "/opt/ros/$ROS_DISTRO/setup.bash"

exec /bin/tini -- /usr/bin/supervisord -n -c /app/supervisord.conf