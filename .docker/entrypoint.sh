#!/bin/bash
set -e

# Setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Setup app workspace
source "/root/workspaces/devel/setup.bash"

# Exec entrypoint commands. Forwards any command arguments here
# i.e. `docker run -it image_name <args>`
exec "$@"
