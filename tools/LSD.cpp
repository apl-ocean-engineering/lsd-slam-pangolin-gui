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
#include <memory>

#include "libg3logger/g3logger.h"

#include "SlamSystem.h"

#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/Configuration.h"

#include "libvideoio/ImageSource.h"
#include "libvideoio/Undistorter.h"

#include "CLI11.hpp"

#include "lsd-slam-pangolin-gui/App/InputThread.h"
#include "lsd-slam-pangolin-gui/App/MakeInput.h"
#include "lsd-slam-pangolin-gui/GUI.h"
#include "lsd-slam-pangolin-gui/TextOutputIOWrapper.h"



using namespace lsd_slam;
using namespace libvideoio;

using std::string;

int main( int argc, char** argv )
{
  // Initialize the logging system
  libg3logger::G3Logger logWorker( argv[0] );
  logWorker.logBanner();

  CLI::App app;

  // Add new options/flags here
  std::string calibFile;
  app.add_option("-c,--calib", calibFile, "Calibration file" )->required()->check(CLI::ExistingFile);

  bool verbose;
  app.add_flag("-v,--verbose", verbose, "Print DEBUG output to console");

  bool noGui;
  app.add_flag("--no-gui", noGui, "Don't display GUI");

  bool noRealtime = false;
  app.add_flag("--no-realtime", noRealtime, "Don't display GUI");

  std::vector<std::string> inFiles;
  app.add_option("--input,input", inFiles, "Input files or directories");

  // Defines the configuration file which can roll up all of the above options
  //    https://cliutils.gitlab.io/CLI11Tutorial/chapters/config.html
  app.set_config("--config");

  CLI11_PARSE(app, argc, argv);

  std::shared_ptr<ImageSource> dataSource( input::MakeInput( inFiles ));
  CHECK((bool)dataSource) << "Data source shouldn't be null";

  dataSource->setFPS( 30 ); //fpsArg.getValue() );
  dataSource->setOutputType( CV_8UC1 );

  std::shared_ptr<Undistorter> undistorter(libvideoio::UndistorterFactory::getUndistorterFromFile( calibFile ));
  if(!(bool)undistorter) {
    LOG(WARNING) << "Undistorter shouldn't be NULL";
    return -1;
  }

  logWorker.verbose( verbose );

  // Load configuration for LSD-SLAM
  Conf().setSlamImageSize( undistorter->outputImageSize() );
  LOG(INFO) << "Slam image: " << Conf().slamImageSize.width << " x " << Conf().slamImageSize.height;

  Conf().runRealTime = !noRealtime;
  Conf().useVarianceFiltering = false;
  Conf().useVoxelFilter = false;

  std::shared_ptr<SlamSystem> system( new SlamSystem() );

  // GUI need to be initialized in main thread on OSX,
  // so run GUI elements in the main thread.
  std::shared_ptr<GUI> gui( nullptr );

  LOG(INFO) << "Starting input thread.";
  InputThread input( system, dataSource, undistorter );

  if( !noGui ) {
    gui.reset( new GUI( Conf().slamImageSize, undistorter->getCamera() ) );
    system->addOutputWrapper( gui );
    input.setIOOutputWrapper( gui );
  }

  system->addOutputWrapper( std::make_shared<TextOutputIOWrapper>() );

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
