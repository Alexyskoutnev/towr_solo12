#!/bin/bash
#* Hide popd and pushd stdout by defining new commands.
popdq () {
	command popd "$@" > /dev/null
}
pushdq () {
	command pushd "$@" > /dev/null
}
#* Change the cwd to the script dir temporarily until the script exits for any reason.
#* (If it exists use BASH_SOURCE, otherwise fall back to $0.)
trap popdq EXIT
pushdq "$(dirname ${BASH_SOURCE[0]:-$0})"

#* catkin project's relative path with respect to this script
CATKIN_PATH=../../..

pushdq $CATKIN_PATH
source ./devel_isolated/setup.bash
roslaunch towr_ros towr_ros.launch 
echo "$0 returned $?."
