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
#include "Pangolin_IOWrapper/PangolinOutputIOWrapper.h"


#include "LSDArgs.h"

#include <App/InputThread.h>
#include <App/App.h>


using namespace lsd_slam;

int main( int argc, char** argv )
{
  // Initialize the logging system
  G3Logger logWorker( argv[0] );
  logWorker.logBanner();

  // Parse command line args
  LSDArgs args( argc, argv );

  logWorker.verbose( args.verbose() );

  // Load configuration for LSD-SLAM
  lsd_slam::Configuration conf;
  conf.inputImage = args.undistorter->inputImageSize();
  conf.slamImage  = args.undistorter->outputImageSize();
  conf.camera     = args.undistorter->getCamera();

  LOG(INFO) << "Slam image: " << conf.slamImage.width << " x " << conf.slamImage.height;
  CHECK( (conf.camera.fx) > 0 && (conf.camera.fy > 0) ) << "Camera focal length is zero";

  std::shared_ptr<SlamSystem> system( new SlamSystem(conf) );

  // GUI need to be initialized in main thread on OSX,
  // so run GUI elements in the main thread.
  std::shared_ptr<GUI> gui( nullptr );
std::shared_ptr<PangolinOutputIOWrapper> ioWrapper(nullptr);

  if( args.doGui ) {
    gui.reset( new GUI( system->conf() ) );
    lsd_slam::PangolinOutput3DWrapper *outputWrapper = new PangolinOutput3DWrapper( system->conf(), *gui );
    system->set3DOutputWrapper( outputWrapper );

    ioWrapper.reset( new PangolinOutputIOWrapper( system->conf(), *gui ));

  }

  LOG(INFO) << "Starting input thread.";
  InputThread input( system, args.dataSource, args.undistorter, ioWrapper );
  boost::thread inputThread( boost::ref(input) );

  // Wait for all threads to indicate they are ready to go
  input.inputReady.wait();

  LOG(INFO) << "Starting all threads.";
  startAll.notify();

  if( gui ) {
    while(!pangolin::ShouldQuit() && !input.inputDone.getValue() )
    {
      if( gui ) gui->update();
    }
  } else {
    input.inputDone.wait();
  }

  LOG(INFO) << "Finalizing system.";
  system->finalize();

  return 0;
}
