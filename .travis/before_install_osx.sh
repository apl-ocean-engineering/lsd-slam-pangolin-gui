#!/usr/bin/env sh

brew update
brew install homebrew/science/opencv homebrew/science/suite-sparse
brew install tclap eigen
brew outdated cmake || brew upgrade cmake
