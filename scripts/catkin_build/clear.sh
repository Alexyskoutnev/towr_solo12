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

#* towr's relative path with respect to this script
CATKIN_PATH=../../../..
BUILD_DIR=$CATKIN_PATH/build_isolated
DEVEL_DIR=$CATKIN_PATH/devel_isolated

if [[ -d "$BUILD_DIR" ]]; then
rm -r -f $BUILD_DIR
fi

if [[ -d "$DEVEL_DIR" ]]; then
rm -r -f $DEVEL_DIR
fi
echo "$0 returned $?."
