#!/bin/bash
WORKSPACE="/home/user/repos"

cd $WORKSPACE/ \
	&& git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
	&& cd Pangolin/ \
	&& git checkout v0.6 \
	&& mkdir build && cd build \
	&& cmake -DCPP11_NO_BOOST=1 .. \
	&& make -j4

cd $WORKSPACE
source $WORKSPACE/build.sh

cd $WORKSPACE

# Execute the command passed into this entrypoint
exec "$@"
