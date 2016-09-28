#!/usr/bin/env sh
set -ev

sudo apt-get update
sudo apt-get install -y rake

rake dependencies:trusty

# cmake \
# 		libopencv-dev libboost-all-dev libeigen3-dev \
# 		libtclap-dev libgomp1 libsuitesparse-dev git \
# 		libglew-dev libglm-dev autoconf libtool freeglut3-dev
