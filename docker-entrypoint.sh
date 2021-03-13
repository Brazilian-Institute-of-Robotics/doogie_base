#!/usr/bin/env bash

set -e

find . \! -user doogie -exec chown doogie '{}' +

source "/opt/ros/$ROS_DISTRO/setup.bash"

exec gosu doogie "$@"