#!/usr/bin/env sh

set -ev

brew update
brew install homebrew/science/opencv homebrew/science/suite-sparse
brew install tclap eigen glew glm homebrew/x11/freeglut
brew outdated cmake || brew upgrade cmake
