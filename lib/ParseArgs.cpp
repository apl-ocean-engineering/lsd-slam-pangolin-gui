

#include <tclap/CmdLine.h>

#include "util/Parse.h"

#include "ParseArgs.h"



namespace lsd_slam {

  ParseArgs::ParseArgs( int argc, char **argv )
    : dataSource( nullptr ),
      undistorter( nullptr ),
      doGui( true )
  {

    bool doGui = true;

      try {
        TCLAP::CmdLine cmd("LSD", ' ', "0.1");

        TCLAP::ValueArg<std::string> calibFileArg("c", "calib", "Calibration file", false, "", "Calibration filename", cmd );
        TCLAP::ValueArg<std::string> resolutionArg("r", "resolution", "", false, "hd1080", "{hd2k, hd1080, hd720, vga}", cmd );

        TCLAP::SwitchArg debugOutputSwitch("","debug-to-console","Print DEBUG output to console", cmd, false);
        TCLAP::SwitchArg noGuiSwitch("","no-gui","Do not run GUI", cmd, false);
        TCLAP::ValueArg<int> fpsArg("", "fps","FPS", false, 0, "", cmd );

        TCLAP::UnlabeledMultiArg<std::string> imageFilesArg("input-files","Name of image files / directories to read", false, "Files or directories", cmd );

        cmd.parse(argc, argv );

        //if( debugOutputSwitch.getValue() ) stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );

        std::vector< std::string > imageFiles = imageFilesArg.getValue();

        dataSource.reset( new ImagesSource( imageFiles ) );

        if( fpsArg.isSet() ) dataSource->setFPS( fpsArg.getValue() );

        if( !calibFileArg.isSet() ) {
          LOG(WARNING) << "Must specify camera calibration!";
          exit(-1);
        }

        undistorter.reset( Undistorter::getUndistorterForFile(calibFileArg.getValue()) );
        CHECK(undistorter) << "Undistorter shouldn't be null";

        doGui = !noGuiSwitch.getValue();

      } catch (TCLAP::ArgException &e)  // catch any exceptions
    	{
        LOG(WARNING) << "error: " << e.error() << " for arg " << e.argId();
        exit(-1);
      }


  }


  }
