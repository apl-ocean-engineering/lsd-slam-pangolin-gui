
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <zed/Camera.hpp>

#include <tclap/CmdLine.h>

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include "util/G3LogSinks.h"


#ifndef USE_ZED
#error "This shouldn't be built unless USE_ZED is defined."
#endif

bool keepGoing = true;

void signal_handler( int sig )
{
	switch( sig ) {
	case SIGINT:
		keepGoing = false;
		break;
	}
}

static sl::zed::ZEDResolution_mode parseResolution( const string &arg )
{
	if( arg == "hd2k" )
		return sl::zed::HD2K;
	else if( arg == "hd1080" )
	 return sl::zed::HD1080;
	else if( arg == "hd720" )
		return sl::zed::HD720;
	else if( arg == "vga" )
		return sl::zed::VGA;
	else
		LOG(FATAL) << "Couldn't parse resolution \"" << arg << "\"";
}

static string resolutionToString( sl::zed::ZEDResolution_mode arg )
{
	switch( arg ) {
		case sl::zed::HD2K:
				return "HD2K";
		case sl::zed::HD1080:
				return "HD1080";
		case sl::zed::HD720:
				return "HD720";
		case sl::zed::VGA:
				return "VGA";
		default:
				return "(unknown)";
	}
}

int main( int argc, char** argv )
{
	auto worker = g3::LogWorker::createLogWorker();
	auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
																			&ColorStderrSink::ReceiveLogMessage);
	g3::initializeLogging(worker.get());

	signal( SIGINT, signal_handler );

	try {
		TCLAP::CmdLine cmd("LSDRecorder", ' ', "0.1");

		TCLAP::ValueArg<std::string> resolutionArg("r","resolution","",false,"hd1080","", cmd);

		TCLAP::ValueArg<std::string> svoInputArg("i","svo-input","Name of SVO file to read",false,"","SVO filename", cmd);
		TCLAP::ValueArg<std::string> svoOutputArg("o","svo-output","Name of SVO file to read",false,"","SVO filename", cmd);
		TCLAP::SwitchArg noGuiSwitch("","no-gui","Use stereo data", cmd, false);

		TCLAP::ValueArg<int> durationArg("","duration","Duration",false,0,"seconds", cmd);

		cmd.parse(argc, argv );

		const sl::zed::ZEDResolution_mode zedResolution = parseResolution( resolutionArg.getValue() );
		const sl::zed::MODE zedMode = sl::zed::MODE::NONE; //( conf.doStereo == Configuration::STEREO_ZED ) ? sl::zed::MODE::QUALITY : sl::zed::MODE::NONE;
		const int whichGpu = -1;
		const bool verboseInit = true;

		sl::zed::Camera *camera = NULL;

		if( svoInputArg.isSet() )
		{
			LOG(INFO) << "Loading SVO file " << svoInputArg.getValue();
			camera = new sl::zed::Camera( svoInputArg.getValue() );
		} else {
			LOG(INFO) << "Using live Zed data";
			camera = new sl::zed::Camera( zedResolution );
		}

		sl::zed::ERRCODE err =camera->initRecording( svoOutputArg.getValue() );
		if (err != sl::zed::SUCCESS) {
			LOG(WARNING) << "Unable to init the zed: " << errcode2str(err);
			delete camera;
			exit(-1);
		}

		float fps = camera->getCurrentFPS();
		if( fps < 0 ) {
			LOG(WARNING) << "Problem getting FPS.";
			delete camera;
			exit(-1);
		}
		int dt_us = (fps > 0) ? (1e6/fps) : 0;
		LOG(INFO) << "Input is at  " << resolutionToString( zedResolution ) << " at nominal " << fps << "FPS";

		if( svoOutputArg.isSet() ) {


			std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
			int duration = durationArg.getValue();
			std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );

			if( duration > 0 )
				LOG(INFO) << "Will log for " << duration << " seconds or press CTRL-C to stop.";
			else
				LOG(INFO) << "Logging now, press CTRL-C to stop.";

			int count = 0;
			while( keepGoing ) {
				if( (count % 100)==0 ) LOG(INFO) << count << " frames";
				count++;

				std::chrono::steady_clock::time_point present( std::chrono::steady_clock::now() );

				if( (duration > 0) && (present > end) ) { keepGoing = false;  break; }

				camera->record();

				if( dt_us > 0 )
					std::this_thread::sleep_until( present + std::chrono::microseconds( dt_us ) );
			};

			std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

			LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
			LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";

			unsigned int fileSize = fs::file_size( fs::path(svoOutputArg.getValue() ));
			unsigned int fileSizeMB = fileSize / (1024*1024);
			LOG(INFO) << "Resulting file is " << fileSizeMB << " MB ()" << fileSizeMB/dur.count() << " MB/sec)";

		} else {
			LOG(WARNING) << "No output format specified.";
		}

		if( camera ) delete camera;

		//
		// 	if( svoFileArg.isSet() )
		// 	{
		// 		LOG(INFO) << "Loading SVO file " << svoFileArg.getValue();
		// 		camera = new sl::zed::Camera( svoFileArg.getValue() );
		// 	} else {
		// 		LOG(INFO) << "Using live Zed data";
		// 		camera = new sl::zed::Camera( zedResolution );
		// 		conf.stopOnFailedRead = false;
		// 	}
		//
		// 	sl::zed::ERRCODE err = camera->init( zedMode, whichGpu, verboseInit );
		// 	if (err != sl::zed::SUCCESS) {
		// 		LOG(WARNING) << "Unable to init the zed: " << errcode2str(err);
		// 		delete camera;
		// 		exit(-1);
		// 	}
		//
		// 	const ImageSize cropSize( 1920, 1056 );
		// 	const SlamImageSize slamSize( cropSize.width / 2, cropSize.height / 2 );
		//
		// 	dataSource = new ZedSource( camera );
		// 	undistorter = new UndistorterZED( camera, cropSize, slamSize );
		// } else
		// #endif
		// {
		// 	std::vector< std::string > imageFiles = imageFilesArg.getValue();
		// 	dataSource = new ImagesSource( imageFiles );
		//
		// 	if( !calibFileArg.isSet() ) {
		// 		LOG(WARNING) << "Must specify camera calibration!";
		// 		exit(-1);
		// 	}
		// 	undistorter = Undistorter::getUndistorterForFile(calibFileArg.getValue());
		//
		// }
		//
		// if( fpsArg.isSet() ) dataSource->setFPS( fpsArg.getValue() );
		//
		// doGui = !noGuiSwitch.getValue();

	} catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		LOG(WARNING) << "error: " << e.error() << " for arg " << e.argId();
		exit(-1);
	}



	return 0;
}
