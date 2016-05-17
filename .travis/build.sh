#! /usr/bin/env sh

set -ev

BUILD_TYPE=${BUILD_TYPE:=Release}

if [ ! -d build_ci ]; then mkdir build_ci; fi

cmake -Bbuild_ci -H. -DCMAKE_BUILD_TYPE:string=$BUILD_TYPE
cd build_ci && make && make unit_test
