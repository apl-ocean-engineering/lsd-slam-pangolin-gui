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

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>


#include <tclap/CmdLine.h>

#include "util/Undistorter.h"

#include "opencv2/opencv.hpp"

#include "GUI.h"

const sl::zed::ZEDResolution_mode zedResolution = sl::zed::HD1080;

// 1080 is not divisible by 16

ThreadMutexObject<bool> lsdDone(false);

int numFrames = 0;

sl::zed::Camera *camera = NULL;
enum { NO_STEREO, STEREO_ZED } doStereo = NO_STEREO;

using namespace lsd_slam;

void run(SlamSystem * system, Undistorter* undistorter, GUI *gui )
{
    CHECK( system );
    CHECK( undistorter );

    // get HZ
    double hz = camera->getCurrentFPS();
    if( hz < 0 ) {
      printf("Unable to get FPS from input, using 30\n");
      hz = 30;
    }

    useconds_t dt_msec = (1.0/hz) * 1e3;

    int runningIdx = 0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; (numFrames < 0) || (i < numFrames); i++, runningIdx++)
    {
        if(lsdDone.getValue())
            break;

        if( gui ) gui->updateFrameNumber( i );

        if(fullResetRequested)
        {
            SlamSystem *newSystem = new SlamSystem( system->conf(), system->SLAMEnabled );
            newSystem->set3DOutputWrapper( system->get3DOutputWrapper() );

            LOG(WARNING) << "FULL RESET!";
            delete system;

            system = newSystem;

            fullResetRequested = false;
            runningIdx = 0;
        }

        if( camera->grab( sl::zed::SENSING_MODE::RAW, false, false ) ) {
          LOG(WARNING) << "Error reading data from camera";
          continue;
        }

        sl::zed::Mat left = camera->retrieveImage(sl::zed::SIDE::LEFT);

        cv::Mat imageScaled( system->conf().slamImage.cvSize(), CV_8UC1 );
        undistorter->undistort( sl::zed::slMat2cvMat(left), imageScaled );

        if( gui ) gui->updateLiveImage( imageScaled.data );

        if( doStereo == STEREO_ZED ) {

          // Fetch depth map as well
          //sl::zed::Mat depth = camera->retrieveMeasure( sl::zed::DEPTH );
          //cv::Mat depthCropped( sl::zed::slMat2cvMat(depth), cv::Rect( cv::Point(0,0), cropSize) );
          //cv::Mat depthScaled( scaledSize, CV_32FC1 );
          //cv::resize( depthCropped, depthScaled, scaledSize );

          //CHECK( depthScaled.type() == CV_32FC1 );
          //CHECK( (imageScaled.rows == depthScaled.rows) && (imageScaled.cols == depthScaled.cols) );

          //if(runningIdx == 0) {
          //  system->gtDepthInit(imageScaled.data, (float *)depthScaled.data, fakeTimeStamp, runningIdx);
          //} else {
          //  system->trackStereoFrame(imageScaled.data, (float *)depthScaled.data, runningIdx, hz == 0, fakeTimeStamp);
          //}

        } else {
          if(runningIdx == 0)
          {
            system->randomInit(imageScaled.data, fakeTimeStamp, runningIdx);
          }
          else
          {
            system->trackFrame(imageScaled.data, runningIdx, hz == 0, fakeTimeStamp);
          }
        }

        if( gui ) gui->pose.assignValue(system->getCurrentPoseEstimateScale());

        fakeTimeStamp+=0.03;


        boost::this_thread::sleep_for(boost::chrono::milliseconds(dt_msec));
    }

    lsdDone.assignValue(true);
}


void runGui(SlamSystem * system, GUI *gui )
{
	while(!pangolin::ShouldQuit())
	{
	    if(lsdDone.getValue() && !system->finalized)
	    {
	        system->finalize();
	    }

	    gui->preCall();

	    gui->drawKeyframes();

	    gui->drawFrustum();

	    gui->drawImages();

	    gui->postCall();
	}
}

int main( int argc, char** argv )
{

  auto worker = g3::LogWorker::createLogWorker();
  auto handle = worker->addDefaultLogger(argv[0], ".");
  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n\n" << std::endl;

  LOG(INFO) << "Starting log.";

bool doGui = true;

  try {
    TCLAP::CmdLine cmd("LSD_Zed", ' ', "0.1");

    TCLAP::ValueArg<std::string> svoFileArg("i","input","Name of SVO file to read",false,"","SVO filename", cmd);
    TCLAP::SwitchArg stereoSwitch("","stereo","Use stereo data", cmd, false);
    TCLAP::SwitchArg noGuiSwitch("","no-gui","Use stereo data", cmd, false);

    cmd.parse(argc, argv );

  	if( svoFileArg.isSet() > 0)
  	{
      LOG(INFO) << "Loading SVO file " << svoFileArg.getValue();
      camera = new sl::zed::Camera( svoFileArg.getValue() );
      numFrames = camera->getSVONumberOfFrames();
  	} else {
      LOG(INFO) << "Using live Zed data";
      camera = new sl::zed::Camera( zedResolution );
      numFrames = -1;
    }

    if( stereoSwitch.getValue() ) {
      LOG(INFO) << "Using stereo data from Stereolabs libraries";
      doStereo = STEREO_ZED;
    }

    doGui = !noGuiSwitch.getValue();

  } catch (TCLAP::ArgException &e)  // catch any exceptions
	{
    LOG(FATAL) << "error: " << e.error() << " for arg " << e.argId();
  }



  // get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	// std::string calibFile;
	//Undistorter* undistorter = 0;

  sl::zed::MODE zedMode = sl::zed::MODE::NONE;
  if( doStereo == STEREO_ZED ) zedMode = sl::zed::MODE::QUALITY;

  const int whichGpu = -1;
  const bool verboseInit = true;
  sl::zed::ERRCODE err = camera->init( zedMode, whichGpu, verboseInit );
  if (err != sl::zed::SUCCESS) {
    LOG(FATAL) << "Unable to init the zed: " << errcode2str(err);
    delete camera;
    exit(-1);
  }

  const ImageSize originalSize( 1920, 1080 );
  const ImageSize cropSize( 1920, 1056 );
  const SlamImageSize slamSize( cropSize.width / 2, cropSize.height / 2 );

  UndistorterZED *undistorter = new UndistorterZED( camera, cropSize, slamSize );

  Configuration conf;
  conf.inputImage = undistorter->inputSize();
  conf.slamImage  = undistorter->outputSize();
  conf.camera = undistorter->getCamera();

  // make slam system
  SlamSystem * system = new SlamSystem( conf, doSlam );
  
  GUI *gui = NULL;
  Output3DWrapper *outputWrapper = NULL;
  if( doGui ) {
	  gui = new GUI( conf );
	  gui->initImages();
	  outputWrapper = new PangolinOutput3DWrapper( conf, *gui);
	  system->set3DOutputWrapper(outputWrapper);

  boost::thread guiThread(runGui, system, gui );
 
    guiThread.join();
  } else {

	while(true)
	{
	    if(lsdDone.getValue() && !system->finalized)
	    {
	        system->finalize();
	    }

	  sleep(1);
	}

  }



  printf("Launching LSD thread\n");
  boost::thread lsdThread(run, system, undistorter, gui );

  

	lsdDone.assignValue(true);

	lsdThread.join();

  if( camera ) delete camera;


	delete system;

	if( outputWrapper ) delete outputWrapper;
	if( gui ) delete gui;
	return 0;
}
