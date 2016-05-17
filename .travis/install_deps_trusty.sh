#!/usr/bin/env sh
set -ev

sudo apt-get update
sudo apt-get install -y cmake \
		libopencv-dev libboost-all-dev libeigen3-dev \
		libtclap-dev libgomp1 libsuitesparse-dev git \
		libglm-dev autoconf libtool freeglut3-dev
