/**
*
* Based on original LSD-SLAM code from:
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

#include <boost/thread.hpp>

#include "App/g3logger.h"

#include "SlamSystem.h"

#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/Configuration.h"


#include "GUI.h"
#include "Pangolin_IOWrapper/PangolinOutput3DWrapper.h"

#include "LSD.h"
#include "ParseArgs.h"
#include "InputThread.h"


using namespace lsd_slam;

int main( int argc, char** argv )
{
  auto logWorker = initializeG3Log( argv[0] );
  logBanner();

  Configuration conf;
  ParseArgs args( argc, argv );

  // Load configuration for LSD-SLAM
  conf.inputImage = args.undistorter->inputImageSize();
  conf.slamImage  = args.undistorter->outputImageSize();
  conf.camera     = args.undistorter->getCamera();

  LOG(INFO) << "Slam image: " << conf.slamImage.width << " x " << conf.slamImage.height;
  CHECK( (conf.camera.fx) > 0 && (conf.camera.fy > 0) ) << "Camera focal length is zero";

	std::shared_ptr<SlamSystem> system( new SlamSystem(conf) );

  // GUI elements need to be initialized in main thread on OSX, so run GUI elements
  // in this thread by default.
  std::shared_ptr<GUI> gui( nullptr );

  if( args.doGui ) {
    gui.reset( new GUI( system->conf() ) );
	lsd_slam::PangolinOutput3DWrapper *outputWrapper = new PangolinOutput3DWrapper( system->conf(), *gui );
	system->set3DOutputWrapper( outputWrapper );
}

  LOG(INFO) << "Starting input thread.";
  InputThread input( system, args.dataSource, args.undistorter );
  boost::thread inputThread( boost::ref(input) );
  input.inputReady.wait();

  // Wait for all threads to be ready.
  LOG(INFO) << "Starting all threads.";
  startAll.notify();

    while(!pangolin::ShouldQuit() && !input.inputDone.getValue() )
  	{
      if( gui ) gui->update();
    
    }

    LOG(INFO) << "Finalizing system.";
    system->finalize();

    while( ! system->finalized() ) { sleep(1); }

  return 0;
}
