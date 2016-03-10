
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
#include "util/ZedUtils.h"

#include "logger/LogWriter.h"


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

int main( int argc, char** argv )
{
	auto worker = g3::LogWorker::createLogWorker();
	auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
	&ColorStderrSink::ReceiveLogMessage);
	g3::initializeLogging(worker.get());

	signal( SIGINT, signal_handler );

	bool doDepth = false, doRight = false;

	try {
		TCLAP::CmdLine cmd("LSDRecorder", ' ', "0.1");

		TCLAP::ValueArg<std::string> resolutionArg("r","resolution","",false,"hd1080","", cmd);
		TCLAP::ValueArg<float> fpsArg("f","fps","",false,0,"", cmd);

		TCLAP::ValueArg<std::string> svoInputArg("i","svo-input","Name of SVO file to read",false,"","SVO filename", cmd);
		TCLAP::ValueArg<std::string> svoOutputArg("s","svo-output","Name of SVO file to read",false,"","SVO filename", cmd);
		TCLAP::ValueArg<std::string> loggerOutputArg("l","logger-output","Name of SVO file to read",false,"","SVO filename", cmd);

		TCLAP::ValueArg<std::string> compressionArg("","compression","",false,"snappy","SVO filename", cmd);

		TCLAP::ValueArg<std::string> imageOutputArg("","image-output","",false,"","SVO filename", cmd);

		// TCLAP::SwitchArg noGuiSwitch("","no-gui","Don't show a GUI", cmd, false);

		TCLAP::SwitchArg depthSwitch("","depth","", cmd, false);
		TCLAP::SwitchArg rightSwitch("","right","", cmd, false);


		TCLAP::ValueArg<int> durationArg("","duration","Duration",false,0,"seconds", cmd);

		cmd.parse(argc, argv );

		doDepth = depthSwitch.getValue();
		doRight = rightSwitch.getValue();

		int compressLevel = logger::LogWriter::DefaultCompressLevel;
		if( compressionArg.isSet() ) {
			if( compressionArg.getValue() == "snappy" )
				compressLevel = logger::LogWriter::SnappyCompressLevel;
			else {
				try {
					compressLevel = std::stoi(compressionArg.getValue() );
				} catch ( std::invalid_argument &e ) {
					throw TCLAP::ArgException("Don't understand compression level.");
				}

			}
		}

		// Output validation
		if( !svoOutputArg.isSet() && !imageOutputArg.isSet() && !loggerOutputArg.isSet() ) {
			LOG(WARNING) << "No output options set.";
			exit(-1);
		}

		fs::path imageOutputDir( imageOutputArg.getValue() );
		if( imageOutputArg.isSet() ) {
			LOG(INFO) << "Recording to directory " << imageOutputDir.string();

			if( !is_directory( imageOutputDir ) ) {
				LOG(WARNING) << "Making directory " << imageOutputDir.string();
				create_directory( imageOutputDir );
			}
		}

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
			camera = new sl::zed::Camera( zedResolution, fpsArg.getValue() );
		}

		int numFrames = camera->getSVONumberOfFrames();

		sl::zed::ERRCODE err;
		if( svoOutputArg.isSet() ) {
			err = camera->initRecording( svoOutputArg.getValue() );
		} else {
			err = camera->init( sl::zed::PERFORMANCE, -1, true );
		}

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

		logger::LogWriter logWriter( compressLevel );
		logger::FieldHandle_t leftHandle, rightHandle = -1, depthHandle = -1;
		if( loggerOutputArg.isSet() ) {
			sl::zed::resolution res( camera->getImageSize() );
			cv::Size sz( res.width, res.height);

			leftHandle = logWriter.registerField( "left", sz, logger::FIELD_BGRA_8C );
			if( doDepth ) depthHandle = logWriter.registerField( "depth", sz, logger::FIELD_DEPTH_32F );
			if( doRight ) rightHandle = logWriter.registerField( "right", sz, logger::FIELD_BGRA_8C );

			if( !logWriter.open( loggerOutputArg.getValue() ) ) {
				LOG(FATAL) << "Unable to open file " << loggerOutputArg.getValue() << " for logging.";
			}
		}


		int dt_us = (fps > 0) ? (1e6/fps) : 0;
		LOG(INFO) << "Input is at " << resolutionToString( zedResolution ) << " at nominal " << fps << "FPS";

		std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
		int duration = durationArg.getValue();
		std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );

		if( duration > 0 )
		LOG(INFO) << "Will log for " << duration << " seconds or press CTRL-C to stop.";
		else
		LOG(INFO) << "Logging now, press CTRL-C to stop.";

		// Wait for the auto exposure and white balance
		std::this_thread::sleep_for(std::chrono::seconds(1));

		int count = 0;
		while( keepGoing ) {
			if( (count % 100)==0 ) LOG(INFO) << count << " frames";

			std::chrono::steady_clock::time_point present( std::chrono::steady_clock::now() );

			if( (duration > 0) && (present > end) ) { keepGoing = false;  break; }

			if( svoOutputArg.isSet() ) {
				camera->record();
			} else {
				if( !camera->grab() ) {

					cv::Mat left( sl::zed::slMat2cvMat( camera->retrieveImage( sl::zed::LEFT ) ) );

					if( imageOutputArg.isSet() ) {
						char filename[80];
						snprintf(filename, 79, "left_%06d.png", count );
						cv::imwrite( (imageOutputDir / filename).string(), left );
					} else if( loggerOutputArg.isSet() ) {
						logWriter.newFrame();
						// This makes a copy of the data to send it to the compressor
						logWriter.addField( leftHandle, left );
					}

					if( doRight ) {
						cv::Mat right(sl::zed::slMat2cvMat( camera->retrieveImage( sl::zed::RIGHT ) ) );
						if( imageOutputArg.isSet() ) {
							char filename[80];
							snprintf(filename, 79, "right_%06d.png", count );
							cv::imwrite( (imageOutputDir / filename).string(), left );
						} else if( loggerOutputArg.isSet() ) {
							logWriter.addField( rightHandle, right.data );
						}
					}

					if( doDepth ) {
						cv::Mat depth(sl::zed::slMat2cvMat( camera->retrieveMeasure( sl::zed::DEPTH ) ) );
						if( imageOutputArg.isSet() ) {
							char filename[80];
							snprintf(filename, 79, "depth_%06d.png", count );
							cv::imwrite( (imageOutputDir / filename).string(), left );
						} else if( loggerOutputArg.isSet() ) {
							logWriter.addField( depthHandle, depth.data );
						}
					}

					if( loggerOutputArg.isSet() ) {
						const bool doBlock = false; //( dt_us == 0 );
						if( !logWriter.writeFrame( doBlock ) ) {
							LOG(WARNING) << "Error while writing frame...";
						}
					}

				} else {
					LOG(WARNING) << "Problem grabbing from camera.";
				}
			}

			if( dt_us > 0 )
				std::this_thread::sleep_until( present + std::chrono::microseconds( dt_us ) );

				count++;

				if( numFrames > 0 && count >= numFrames ) {
					keepGoing = false;
				}
		}

		std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

		LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
		LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";

		if( svoOutputArg.isSet() ) {
			unsigned int fileSize = fs::file_size( fs::path(svoOutputArg.getValue() ));
			unsigned int fileSizeMB = fileSize / (1024*1024);
			LOG(INFO) << "Resulting file is " << fileSizeMB << " MB (" << fileSizeMB/dur.count() << " MB/sec)";
		}

		if( loggerOutputArg.isSet() ) {
			unsigned int fileSize = fs::file_size( fs::path(loggerOutputArg.getValue() ));
			unsigned int fileSizeMB = fileSize / (1024*1024);
			LOG(INFO) << "Resulting file is " << fileSizeMB << " MB (" << fileSizeMB/dur.count() << " MB/sec)";
		}

		if( loggerOutputArg.isSet() ) { logWriter.close(); }

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
