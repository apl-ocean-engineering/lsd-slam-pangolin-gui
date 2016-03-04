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
#include "util/G3LogSinks.h"

#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Configuration.h"

#include <LSD.h>

using namespace lsd_slam;

ThreadMutexObject<bool> lsdDone(false), guiDone(false);

ThreadSynchronizer lsdReady, guiReady, startAll;

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

  CHECK( undistorter != NULL ) << "Undistorter doesn't exist.";
  CHECK( dataSource != NULL ) << "Data source doesn't exist.";

  conf.inputImage = ImageSize( undistorter->getInputWidth(), undistorter->getInputHeight() );
  conf.slamImage  = SlamImageSize( undistorter->getOutputWidth(), undistorter->getOutputHeight() );
  conf.camera     = Camera( undistorter->getK() );

	SlamSystem * system = new SlamSystem(conf, doSlam);

  if( doGui ) {
    boost::thread guiThread(runGui, system );
    guiReady.wait();
  }

  boost::thread lsdThread(run, system, dataSource, undistorter );
  lsdReady.wait();

  // Wait for threads to be ready.
  startAll.notify();


  while(true)
  {
      if( (lsdDone.getValue() || guiDone.getValue()) && !system->finalized)
      {
          LOG(INFO) << "Finalizing system.";
          system->finalize();
      }

    sleep(1);
  }

	// make slam system

  if( system ) delete system;
  if( undistorter ) delete undistorter;
  // if( outputWrapper ) delete outputWrapper;
  // if( gui ) delete gui;
  return 0;
}
