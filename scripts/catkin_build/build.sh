#!/bin/bash
# Hide popd and pushd stdout by defining new commands.
popdq () {
	command popd "$@" > /dev/null
}
pushdq () {
	command pushd "$@" > /dev/null
}
# Change the cwd to the script dir temporarily until the script exits for any reason.
# (If it exists use BASH_SOURCE, otherwise fall back to $0.)
trap popdq EXIT
pushdq "$(dirname ${BASH_SOURCE[0]:-$0})"

# catkin project's relative path with respect to this script
CATKIN_PATH=../../../..

# copy data folder to .ros because relative paths are difficult with ros
rsync -a $CATKIN_PATH/src/towr_solo12/towr/data/ ~/.ros/data/ --delete

pushdq $CATKIN_PATH
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release # or `catkin build`
echo "$0 returned $?."
