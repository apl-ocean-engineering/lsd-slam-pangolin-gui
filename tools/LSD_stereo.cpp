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

#include <App/StereoInputThread.h>

#include "Input.h"

#include "GUI.h"
#include "Pangolin_IOWrapper/PangolinOutputIOWrapper.h"
#include "Pangolin_IOWrapper/TextOutputIOWrapper.h"

#include <yaml-cpp/yaml.h>

using namespace lsd_slam;
using namespace libvideoio;

using std::string;


/// Quick and dirty

static Sophus::SE3d loadExtrinsics( const std::string &yamlFile )
{
  YAML::Node config = YAML::LoadFile(yamlFile);

  if( !config["extrinsics"]) {
    LOG(FATAL) << "Unable to load extrinsics from " << yamlFile;
  }

  auto ext = config["extrinsics"]["combined"];
  const int nrows = ext["rows"].as<int>();
  const int ncols = ext["cols"].as<int>();

  CHECK(ext["data"].size() == 12 ) << "Expected 12 row-major elements in the extrinsics, got " << ext.size();
  CHECK(nrows == 3 && ncols == 4) << "Expected 3x4 extrinsics, got " << nrows << " x " << ncols;

  std::vector<double> extVec;
  // for( size_t i = 0; i < nrows*ncols; ++i ) {
  //   extVec.push_back(ext["data"][i].as<double>());
  // }

  for( auto d : ext["data"] ) {
    extVec.push_back(d.as<double>());
  }

  if( extVec.size() == 12 ) {
    extVec.push_back(0.0);
    extVec.push_back(0.0);
    extVec.push_back(0.0);
    extVec.push_back(1.0);
  }

  CHECK( extVec.size()==16) << "Loaded extrinsics, but it's the wrong length (" << extVec.size();

  // Eigne loads columns-order by default, need to transpose
  Eigen::Matrix4d mat(extVec.data());
  auto matt = mat.transpose();

  LOG(WARNING) << "Loaded extrinsics: " << matt;

  return Sophus::SE3d( matt );
}


int main( int argc, char** argv )
{
  // Initialize the logging system
  libg3logger::G3Logger logWorker( argv[0] );
  logWorker.logBanner();

  CLI::App app;

  // Add new options/flags here
  std::string calibLeft;
  app.add_option("--calib-left", calibLeft, "Left calibration file" )->required()->check(CLI::ExistingFile);

  std::string calibRight;
  app.add_option("--calib-right", calibRight, "Right calibration file" )->required()->check(CLI::ExistingFile);

  // std::string extrinsicsFile;
  // app.add_option("--extrinsics", extrinsicsFile, "Extrinsics file")->required()->check(CLI::ExistingFile);

  bool verbose;
  app.add_flag("-v,--verbose", verbose, "Print DEBUG output to console");

  bool doRotate;
  app.add_flag("--rotate", doRotate, "Rotate incoming images 180degrees");

  bool noStereo;
  app.add_flag("--no-stereo", noStereo, "Don't so ImageSetStereo");

  bool noGui;
  app.add_flag("--no-gui", noGui, "Don't display GUI");

  bool noRealtime = false;
  app.add_flag("--no-realtime", noRealtime, "Don't run in realtime, run only as fast as frames can be processed");

  std::vector<std::string> inFiles;
  app.add_option("--input,input", inFiles, "Input files or directories");

  // Defines the configuration file which can roll up all of the above options
  //    https://cliutils.gitlab.io/CLI11Tutorial/chapters/config.html
  app.set_config("--config");

  CLI11_PARSE(app, argc, argv);

  std::shared_ptr<ImageSource> dataSource( Input::makeImageSource( inFiles ));
  CHECK((bool)dataSource) << "Data source shouldn't be null";

  dataSource->setFPS( 30 ); //fpsArg.getValue() );
  dataSource->setOutputType( CV_8UC1 );

  const std::shared_ptr<OpenCVUndistorter> leftUndistorter(libvideoio::ROSUndistorterFactory::loadFromFile( calibLeft ));
  if(!(bool)leftUndistorter) {
    LOG(WARNING) << "Left undistorter shouldn't be NULL";
    return -1;
  }

  const std::shared_ptr<OpenCVUndistorter> rightUndistorter(libvideoio::ROSUndistorterFactory::loadFromFile( calibRight ));
  if(!(bool)rightUndistorter) {
    LOG(WARNING) << "Right undistorter shouldn't be NULL";
    return -1;
  }

  //Sophus::SE3d extrinsics( loadExtrinsics( extrinsicsFile ) );
  cv::Vec3d bl = rightUndistorter->baseline();
  Eigen::Vector3d baseline( bl[0], bl[1], bl[2] );
  Sophus::SE3d extrinsics( Eigen::Matrix3d::Identity(), baseline );

  logWorker.verbose( verbose );

  // Load configuration for LSD-SLAM
  Conf().setSlamImageSize( leftUndistorter->outputImageSize() );
  LOG(INFO) << "Slam image: " << Conf().slamImageSize.width << " x " << Conf().slamImageSize.height;



  Conf().runRealTime = !noRealtime;
  Conf().doLeftRightStereo = !noStereo;

  Conf().plot.doWaitKey = 0;
  Conf().plot.debugStereo = true;

  std::shared_ptr<SlamSystem> system( new SlamSystem() );

  // GUI need to be initialized in main thread on OSX,
  // so run GUI elements in the main thread.
  std::shared_ptr<GUI> gui( nullptr );

  LOG(INFO) << "Starting input thread.";
  StereoInputThread input( system, dataSource,
                            std::shared_ptr<libvideoio::Undistorter>(leftUndistorter.get()),
                            std::shared_ptr<libvideoio::Undistorter>(rightUndistorter.get()), extrinsics );
  input.setDoRotate( doRotate );

  if( !noGui ) {
    gui.reset( new GUI( Conf().slamImageSize, leftUndistorter->getCamera() ) );
    auto outputWrapper( std::make_shared<PangolinOutputIOWrapper>( *gui ) );
    system->addOutputWrapper( outputWrapper );
    input.setIOOutputWrapper( outputWrapper );
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
