/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <zed/Camera.hpp>

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "IOWrapper/Pangolin/PangolinOutput3DWrapper.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistorter.h"

#include "opencv2/opencv.hpp"

#include "GUI.h"

// 1080 is not divisible by 16
const cv::Size cropSize( 1920, 1072 );
// Maintains same aspect ratio as 1920x1072, also both as divisible by 16
const cv::Size scaledSize( 640, 352 );
const cv::Size slamSize( scaledSize );


ThreadMutexObject<bool> lsdDone(false);
GUI gui;

int numFrames = 0;

sl::zed::Camera *camera = NULL;

using namespace lsd_slam;

void run(SlamSystem * system, Output3DWrapper* outputWrapper, Sophus::Matrix3f K)
{
    // get HZ
    double hz = camera->getCurrentFPS();
    if( hz < 0 ) {
      printf("Unable to get FPS from input, using 30\n");
      hz = 30;
    }

    int wait = (1.0/hz) * 1000;

    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; (numFrames < 0) || (i < numFrames); i++)
    {
        if(lsdDone.getValue())
            break;

        printf("Loop %d\n", runningIDX );

        // At present, just grab images
        if( camera->grab( sl::zed::SENSING_MODE::RAW, false, false ) ) {
          printf("Error reading data from camera\n" );
          break;
        }

        sl::zed::Mat left = camera->retrieveImage(sl::zed::SIDE::LEFT);
        cv::Mat imageRGB( left.height, left.width, CV_8UC4, left.data );

        // Convert to greyscale, crop to multiples of 16 (hardcoded at present)
        cv::Mat imageROI( imageRGB, cv::Rect( cv::Point(0,0), cropSize) );
        cv::Mat imageGray( cropSize, CV_8UC1 );
        cv::cvtColor( imageROI, imageGray, cv::COLOR_BGRA2GRAY );

        // Downscale (for now)
        cv::Mat imageScaled( scaledSize, CV_8UC1 );
        cv::resize( imageGray, imageScaled, scaledSize );

        runningIDX++;

        cv::imshow( "imageScaled", imageScaled );
        cv::waitKey( wait );
    }

    lsdDone.assignValue(true);
}

int main( int argc, char** argv )
{

  // open image files: first try to open as file.
	std::string source;
	if( Parse::arg(argc, argv, "-f", source) > 0)
	{
		printf("Loading SVO file %s\n", source.c_str() );
    camera = new sl::zed::Camera( source );
	} else {
    printf("Using live Zed data\n");

    const sl::zed::ZEDResolution_mode resolution = sl::zed::HD1080;
    camera = new sl::zed::Camera( resolution );
  }

  // get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	// std::string calibFile;
	//Undistorter* undistorter = 0;

  sl::zed::ERRCODE err = camera->init( sl::zed::MODE::NONE, -1, true );
  if (err != sl::zed::SUCCESS) {
    printf( "Unable to init the zed: %s\n", errcode2str(err).c_str() );
    delete camera;
    exit(-1);
  }

  sl::zed::StereoParameters *params = camera->getParameters();

  float fx = params->LeftCam.fx;
	float fy = params->LeftCam.fy;
	float cx = params->LeftCam.cx;
	float cy = params->LeftCam.cy;

	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
	Intrinsics::getInstance(fx, fy, cx, cy);
  //delete params;


  {
    sl::zed::resolution resolution = camera->getImageSize();

    // Intermediate sizes are set with const Size global (at top of file)
    // Just a quick sanity check that  they're appropriate for this
    // camera resolution
    assert( resolution.width >= cropSize.width );
    assert( resolution.height >= cropSize.height );

    // Set up these two singletons
  	Resolution::getInstance( slamSize.width, slamSize.height );
  }


	gui.initImages();

	Output3DWrapper* outputWrapper = new PangolinOutput3DWrapper( slamSize.width, slamSize.height, gui);


	// make slam system
	SlamSystem * system = new SlamSystem( slamSize.width, slamSize.height, K, doSlam);
	system->setVisualization(outputWrapper);

  numFrames = camera->getSVONumberOfFrames();


  // printf("Launching LSD thread\n");
	boost::thread lsdThread(run, system, outputWrapper, K);

	while(!pangolin::ShouldQuit())
	{
	    if(lsdDone.getValue() && !system->finalized)
	    {
	        system->finalize();
	    }

	    gui.preCall();

	    gui.drawKeyframes();

	    gui.drawFrustum();

	    gui.drawImages();

	    gui.postCall();
	}

	lsdDone.assignValue(true);

	lsdThread.join();

  if( camera ) delete camera;

	delete system;
	delete outputWrapper;
	return 0;
}
