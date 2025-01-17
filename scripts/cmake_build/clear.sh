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
TOWR_PATH=../../towr
CLEAR_DIR=$TOWR_PATH/build

if [[ -d "$CLEAR_DIR" ]]; then
rm -r -f $CLEAR_DIR
if [[ $? -eq 0 ]]; then
   echo "Removed $CLEAR_DIR"
fi
fi
echo "$0 returned $?."
