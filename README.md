# LSD-SLAM: Large-Scale Direct Monocular SLAM

This fork started from [Thomas Whelan's fork](https://github.com/mp3guy/lsd_slam) which "relieves the user of the horrors of a ROS dependency and uses the much nicer lightweight [Pangolin](https://github.com/stevenlovegrove/Pangolin) framework instead."

Here is Jakob's original description:

> LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct (i.e. does not use keypoints / features) and creates large-scale,
> semi-dense maps in real-time on a laptop. For more information see
> [http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
> where you can also find the corresponding publications and Youtube videos, as well as some
> example-input datasets, and the generated output as rosbag or .ply point cloud.

This repo contains my experiments with LSD-SLAM, for performance, functionality
and structure.   As of March 2016, it diverges significantly from either Jakob or Thomas's branches in structure (I refactored as a way of learning the code), but not significantly in terms of functionality.   

**master**  is my working / stable-ish branch.   **aaron_dev** is my
**really** unstable branch.   Other branches are for hardware-specific ports
although in the long run I try to merge those functionalities into master
and use CMake to turn hardware-specific elements on and off.

# 1. Quickstart / Minimal Setup

Requires OpenCV 2.4 (with nonfree if you want FABMAP), Boost, Eigen, Pangolin and g2o.

I'm developing on Ubuntu 14.04.2 and OS X 10.11 with Homebrew.

# 2. Installation

Install everything from apt repos if you can, otherwise there are githubs for Pangolin and g2o.

I have not tested this on a clean install, but:

    apt-get install libeigen3-dev libboost1.55-dev libopencv-dev ....

or

    brew install eigen boost ...

Then usual cmake building process.

# 3. Running

Supports raw PNG images. For example, you can down any dataset from [here](http://vision.in.tum.de/lsdslam) in PNG format, and run like;

./LSD -c ~/Mono_Logs/LSD_machine/cameraCalibration.cfg -f ~/Mono_Logs/LSD_machine/images/

# 4. Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

# 5. License

LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.
