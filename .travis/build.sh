#! /usr/bin/env sh

if [ ! -d build_ci ]; then
	mkdir build_ci
fi

cmake -Bbuild_ci -H.
cd build_ci
make
make test
