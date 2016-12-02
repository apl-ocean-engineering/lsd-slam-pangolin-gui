# LSD-SLAM: Large-Scale Direct Monocular SLAM

CI Status: [![Build Status](https://travis-ci.org/amarburg/lsd_slam_conan.svg)](https://travis-ci.org/amarburg/lsd_slam_conan)

See my [Development Blog](https://faculty.washington.edu/amarburg/press/category/lsdslam/) for current status.

This is my development wrapper/GUI for [LSD-SLAM](https://github.com/amarburg/lsd_slam) which uses the [Conan](https://conan.io/)
packaging tool to add new dependencies and features.   At present, this version is pretty much redundant with the [Pangolin](https://github.com/stevenlovegrove/Pangolin) GUI version included in the `LSD-SLAM` distribution, but
I expect greater divergence.

I've automated the building with a Rakefile (yes, I now, Ruby to coordinate Python, thanx).   

Assuming all the (non-Conan, apt-gettable) dependencies are in place,

> rake debug:test

Should work.

# License

LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.
