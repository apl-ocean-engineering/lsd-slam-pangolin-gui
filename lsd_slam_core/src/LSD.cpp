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

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include <tclap/CmdLine.h>

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/DataSource.h"
#include "util/Configuration.h"
#include "util/Undistorter.h"
#include "util/RawLogReader.h"

#include "util/G3LogSinks.h"

#include "SlamSystem.h"

#include "IOWrapper/Pangolin/PangolinOutput3DWrapper.h"

// #include "util/FileUtils.h"

#include "GUI.h"

// std::vector<std::string> files;

ThreadMutexObject<bool> lsdDone(false);

// RawLogReader * logReader = 0;

using namespace lsd_slam;

void run(SlamSystem * system, DataSource *dataSource, Undistorter* undistorter, GUI *gui )
{
    // get HZ
    double hz = 30;

    int numFrames = dataSource->numFrames();

    cv::Mat image = cv::Mat(system->conf().slamImage.cvSize(), CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; (numFrames < 0) || (i < numFrames); ++i)
    {
        if(lsdDone.getValue()) break;

        cv::Mat imageDist = cv::Mat( system->conf().inputImage.cvSize(), CV_8U);

        if( dataSource->grab() < 0 )
          if( system->conf().stopOnFailedRead )
            break;
          else
            continue;

        dataSource->getImage( imageDist );
        undistorter->undistort(imageDist, image);

        CHECK(image.type() == CV_8U);

        if(runningIDX == 0)
        {
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }

        gui->pose.assignValue(system->getCurrentPoseEstimateScale());
        gui->updateFrameNumber( runningIDX );
        gui->updateLiveImage( image.data );

        runningIDX++;
        fakeTimeStamp+=0.03;

        if(fullResetRequested)
        {
            SlamSystem *newSystem = new SlamSystem( system->conf(), system->SLAMEnabled );
            newSystem->set3DOutputWrapper( system->get3DOutputWrapper() );

            LOG(WARNING) << "FULL RESET!";
            delete system;

            system = newSystem;

            fullResetRequested = false;
            runningIDX = 0;
        }
    }

    lsdDone.assignValue(true);
}

int main( int argc, char** argv )
{
  auto worker = g3::LogWorker::createLogWorker();
  auto handle = worker->addDefaultLogger(argv[0], ".");
  auto stderrHangle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
                                       &ColorStderrSink::ReceiveLogMessage);


  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n\n" << std::endl;

  LOG(INFO) << "Starting log.";

  DataSource *dataSource = NULL;
  Undistorter* undistorter = NULL;

  Configuration conf;

  bool doGui = true;

    try {
      TCLAP::CmdLine cmd("LSD", ' ', "0.1");

      TCLAP::ValueArg<std::string> calibFileArg("c", "calib", "Calibration file", false, "", "Calibration filename", cmd );
#ifdef USE_ZED
      TCLAP::SwitchArg zedSwitch("","zed","Use ZED", cmd, false);
      TCLAP::ValueArg<std::string> svoFileArg("","svo","Name of SVO file to read",false,"","SVO filename", cmd);
      TCLAP::SwitchArg stereoSwitch("","stereo","Use stereo data", cmd, false);
#endif
      TCLAP::SwitchArg noGuiSwitch("","no-gui","Use stereo data", cmd, false);

      TCLAP::UnlabeledMultiArg<std::string> imageFilesArg("input-files","Name of image files / directories to read", false, "Files or directories", cmd );

      cmd.parse(argc, argv );

    	// if( svoFileArg.isSet() > 0)
    	// {
      //   LOG(INFO) << "Loading SVO file " << svoFileArg.getValue();
      //   camera = new sl::zed::Camera( svoFileArg.getValue() );
      //   numFrames = camera->getSVONumberOfFrames();
    	// } else {
      //   LOG(INFO) << "Using live Zed data";
      //   camera = new sl::zed::Camera( zedResolution );
      //   numFrames = -1;
      // }
      //

#ifdef USE_ZED
      if( zedSwitch.getValue() || svoFileArg.isSet() ) {
        if( stereoSwitch.getValue() ) {
          LOG(INFO) << "Using stereo data from Stereolabs libraries";
          conf.doStereo = Configuration::STEREO_ZED;
        }

        const sl::zed::ZEDResolution_mode zedResolution = sl::zed::HD1080;
        const sl::zed::MODE zedMode = ( conf.doStereo == Configuration::STEREO_ZED ) ? sl::zed::MODE::QUALITY : sl::zed::MODE::NONE;
        const int whichGpu = -1;
        const bool verboseInit = true;

        sl::zed::Camera *camera = NULL;

        if( svoFileArg.isSet() )
      	{
          LOG(INFO) << "Loading SVO file " << svoFileArg.getValue();
          camera = new sl::zed::Camera( svoFileArg.getValue() );
      	} else {
          LOG(INFO) << "Using live Zed data";
          camera = new sl::zed::Camera( zedResolution );
          conf.stopOnFailedRead = false;
        }



        sl::zed::ERRCODE err = camera->init( zedMode, whichGpu, verboseInit );
        if (err != sl::zed::SUCCESS) {
          LOG(WARNING) << "Unable to init the zed: " << errcode2str(err);
          delete camera;
          exit(-1);
        }

        const ImageSize cropSize( 1920, 1056 );
        const SlamImageSize slamSize( cropSize.width / 2, cropSize.height / 2 );

        dataSource = new ZedSource( camera );
        undistorter = new UndistorterZED( camera, cropSize, slamSize );
      } else
#endif
      {
        std::vector< std::string > imageFiles = imageFilesArg.getValue();
        dataSource = new ImagesSource( imageFiles );

        if( !calibFileArg.isSet() ) {
          LOG(WARNING) << "Must specify camera calibration!";
          exit(-1);
        }

        undistorter = Undistorter::getUndistorterForFile(calibFileArg.getValue());

      }

      doGui = !noGuiSwitch.getValue();

    } catch (TCLAP::ArgException &e)  // catch any exceptions
  	{
      LOG(WARNING) << "error: " << e.error() << " for arg " << e.argId();
      exit(-1);
    }

  CHECK(undistorter != NULL) << "Undistorter doesn't exist.";
  CHECK( dataSource != NULL ) << "Data source doesn't exist.";

  conf.inputImage = ImageSize( undistorter->getInputWidth(), undistorter->getInputHeight() );
  conf.slamImage  = SlamImageSize( undistorter->getOutputWidth(), undistorter->getOutputHeight() );
  conf.camera     = Camera( undistorter->getK() );

  Output3DWrapper* outputWrapper = NULL;
  GUI *gui = NULL;
  if( doGui ) {
    gui = new GUI( conf );
    gui->initImages();
    outputWrapper = new PangolinOutput3DWrapper( conf, *gui );
  }

	// make slam system
	SlamSystem * system = new SlamSystem(conf, doSlam);
	system->set3DOutputWrapper( outputWrapper );


	boost::thread lsdThread(run, system, dataSource, undistorter, gui );

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

	lsdDone.assignValue(true);

	lsdThread.join();

  if( system ) delete system;
  if( undistorter ) delete undistorter;
  if( outputWrapper ) delete outputWrapper;
  if( gui ) delete gui;
  return 0;
}
