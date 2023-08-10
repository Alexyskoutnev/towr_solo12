#!/bin/bash

#* project's relative path with respect to this script
IFOPT_PATH=../../ifopt

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

mkdir $IFOPT_PATH/build
pushdq $IFOPT_PATH/build
sudo xargs rm < install_manifest.txt

echo "$0 done."
