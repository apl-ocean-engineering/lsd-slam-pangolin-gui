#!/usr/bin/env sh

set -ev

brew update
for pkg in homebrew/science/opencv homebrew/science/suite-sparse \
					cmake tclap eigen glew glm homebrew/x11/freeglut
do
	brew outdated $pkg || brew upgrade $pkg
done

## Eigen 3.2.8 has a very minor bug
# (http://eigen.tuxfamily.org/bz/show_bug.cgi?id=537)
# that causes compilation problems on OSX.   A patch
# has been accepted and is present in 3.2 HEAD as well
# as devel
EIGEN_VER=`brew info eigen | head -1 | awk '{print $3}'`
if [[ $EIGEN_VER = "3.2.8" ]]; then
	patch --forward -p1 -d /usr/local/include/eigen3 < .travis/eigen-3.2.8-patch.diff
fi
