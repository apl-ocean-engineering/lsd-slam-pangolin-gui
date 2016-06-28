
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
#include "util/DataSource.h"
#include "util/Undistorter.h"

#include "logger/LogWriter.h"

#include "ZedRecorder/Display.h"
#include "ZedRecorder/ImageOutput.h"
#include "ZedRecorder/VideoOutput.h"

using namespace lsd_slam;

#ifndef USE_ZED
	#error "ZedRecorder shouldn't be built unless USE_ZED is defined."
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

using cv::Mat;

int main( int argc, char** argv )
{
	auto worker = g3::LogWorker::createLogWorker();
	auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
	&ColorStderrSink::ReceiveLogMessage);
	g3::initializeLogging(worker.get());

	signal( SIGINT, signal_handler );

	try {
		TCLAP::CmdLine cmd("LSDRecorder", ' ', "0.1");

		TCLAP::ValueArg<std::string> resolutionArg("r","resolution","Input resolution: hd2k,hd1080,hd720,vga",false,"hd1080","", cmd);
		TCLAP::ValueArg<float> fpsArg("f","fps","Input FPS, otherwise defaults to max FPS from input source",false,0.0,"", cmd);

		TCLAP::ValueArg<std::string> logInputArg("","log-input","Input Logger file",false,"","", cmd);
		TCLAP::ValueArg<std::string> svoInputArg("i","svo-input","Input SVO file",false,"","", cmd);

		TCLAP::ValueArg<std::string> svoOutputArg("s","svo-output","Output SVO file",false,"","", cmd);
		TCLAP::ValueArg<std::string> loggerOutputArg("l","log-output","Output Logger filename",false,"","", cmd);
		TCLAP::ValueArg<std::string> calibOutputArg("","calib-output","Output calibration file (from stereolabs SDK)",false,"","Calib filename", cmd);

		TCLAP::ValueArg<std::string> compressionArg("","compression","",false,"snappy","SVO filename", cmd);

		TCLAP::ValueArg<std::string> imageOutputArg("","image-output","",false,"","SVO filename", cmd);
		TCLAP::ValueArg<std::string> videoOutputArg("","video-output","",false,"","SVO filename", cmd);

		TCLAP::ValueArg<int> skipArg("","skip","",false,1,"", cmd);

		TCLAP::ValueArg<std::string> statisticsOutputArg("","statistics-output","",false,"","", cmd);

		// TCLAP::SwitchArg noGuiSwitch("","no-gui","Don't show a GUI", cmd, false);

		TCLAP::SwitchArg depthSwitch("","depth","", cmd, false);
		TCLAP::SwitchArg rightSwitch("","right","", cmd, false);

		TCLAP::SwitchArg guiSwitch("","display","", cmd, false);


		TCLAP::ValueArg<int> durationArg("","duration","Duration",false,0,"seconds", cmd);

		cmd.parse(argc, argv );

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
		if( !svoOutputArg.isSet() && !videoOutputArg.isSet() && !imageOutputArg.isSet() && !loggerOutputArg.isSet() && !guiSwitch.isSet() ) {
			LOG(WARNING) << "No output options set.";
			exit(-1);
		}

		zed_recorder::Display display( guiSwitch.getValue() );
		zed_recorder::ImageOutput imageOutput( imageOutputArg.getValue() );


		const sl::zed::ZEDResolution_mode zedResolution = parseResolution( resolutionArg.getValue() );
		const sl::zed::MODE zedMode = sl::zed::MODE::NONE;
		const int whichGpu = -1;
		const bool verboseInit = true;

		DataSource *dataSource = NULL;

		sl::zed::Camera *camera = NULL;

		if( logInputArg.isSet() ) {
			LOG(INFO) << "Loading logger data from " << logInputArg.getValue();
			dataSource = new LoggerSource( logInputArg.getValue() );

			LOG_IF(FATAL, depthSwitch.getValue() && !dataSource->hasDepth() ) << "Depth requested but log file doesn't have depth data.";
			LOG_IF(FATAL, rightSwitch.getValue() && dataSource->numImages() < 2 ) << "Depth requested but log file doesn't have depth data.";

			if( calibOutputArg.isSet() )
				LOG(WARNING) << "Can't create calibration file from a log file.";

		} else {

			LOG_IF( FATAL, calibOutputArg.isSet() && svoInputArg.isSet() ) << "Calibration data isn't stored in SVO input files.";
			LOG_IF( FATAL, calibOutputArg.isSet() && svoOutputArg.isSet() ) << "Calibration data is only generated when using live video, not when recording to SVO.";

			if( svoInputArg.isSet() )	{
				LOG(INFO) << "Loading SVO file " << svoInputArg.getValue();
				camera = new sl::zed::Camera( svoInputArg.getValue() );
			} else  {
				LOG(INFO) << "Using live Zed data";
				camera = new sl::zed::Camera( zedResolution, fpsArg.getValue() );
			}

			sl::zed::ERRCODE err;
			if( svoOutputArg.isSet() ) {
				err = camera->initRecording( svoOutputArg.getValue() );
			} else {
#ifdef ZED_1_0
				sl::zed::InitParams initParams;
				initParams.mode = zedMode;
				initParams.verbose = verboseInit;
        sl::zed::ERRCODE err = camera->init( initParams );
#else
				sl::zed::ERRCODE err = camera->init( zedMode, whichGpu, verboseInit );
#endif
			}

			if (err != sl::zed::SUCCESS) {
				LOG(WARNING) << "Unable to init the zed: " << errcode2str(err);
				delete camera;
				exit(-1);
			}

			dataSource = new ZedSource( camera, depthSwitch.getValue() );

			if( calibOutputArg.isSet() ) {
					LOG(INFO) << "Saving calibration to \"" << calibOutputArg.getValue() << "\"";
					lsd_slam::UndistorterLogger::calibrationFromZed( camera, calibOutputArg.getValue() );
			}
		}

		int numFrames = dataSource->numFrames();
		float fps = dataSource->fps();

		CHECK( fps >= 0 );

		logger::LogWriter logWriter( compressLevel );
		logger::FieldHandle_t leftHandle, rightHandle = -1, depthHandle = -1;
		if( loggerOutputArg.isSet() ) {
			sl::zed::resolution res( camera->getImageSize() );
			cv::Size sz( res.width, res.height);

			leftHandle = logWriter.registerField( "left", sz, logger::FIELD_BGRA_8C );
			if( depthSwitch.getValue() ) depthHandle = logWriter.registerField( "depth", sz, logger::FIELD_DEPTH_32F );
			if( rightSwitch.getValue() ) rightHandle = logWriter.registerField( "right", sz, logger::FIELD_BGRA_8C );

			if( !logWriter.open( loggerOutputArg.getValue() ) ) {
				LOG(FATAL) << "Unable to open file " << loggerOutputArg.getValue() << " for logging.";
			}
		}

		imageOutput.registerField( leftHandle, "left" );
		imageOutput.registerField( rightHandle, "right" );
		imageOutput.registerField( depthHandle, "depth" );

		zed_recorder::VideoOutput videoOutput( videoOutputArg.getValue(), fps > 0 ? fps : 30 );

		int dt_us = (fps > 0) ? (1e6/fps) : 0;
		const float sleepFudge = 1.0;
		dt_us *= sleepFudge;

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

		int count = 0, skip = skipArg.getValue();
		while( keepGoing ) {
			if( count > 0 && (count % 100)==0 ) LOG(INFO) << count << " frames";

			std::chrono::steady_clock::time_point loopStart( std::chrono::steady_clock::now() );

			if( (duration > 0) && (loopStart > end) ) { keepGoing = false;  break; }

			if( svoOutputArg.isSet() ) {

				if( camera->record() ) {
					LOG(WARNING) << "Error occured while recording from camera";
				} else if( count % skip == 0 ) {
					// According to the docs, this:
					//		Get[s] the current side by side YUV 4:2:2 frame, CPU buffer.
					sl::zed::Mat slRawImage( camera->getCurrentRawRecordedFrame() );
					// Make a copy before enqueueing
					Mat rawCopy;
					sl::zed::slMat2cvMat( slRawImage ).reshape( 2, 0 ).copyTo( rawCopy );
					display.showRawStereoYUV( rawCopy );
				}


			} else {

				if( dataSource->grab() ) {

					cv::Mat left;
					dataSource->getImage( 0, left );

					imageOutput.write( leftHandle, left );
					videoOutput.write( left );

					if( loggerOutputArg.isSet() ) {
						logWriter.newFrame();
						// This makes a copy of the data to send it to the compressor
						logWriter.addField( leftHandle, left );
					}

					if( count % skip == 0 ) {
						display.showLeft( left );
					}

					if( rightSwitch.getValue() ) {
						cv::Mat right;
						dataSource->getImage( 1, right );

						imageOutput.write( rightHandle, right );

						if( loggerOutputArg.isSet() ) {
							logWriter.addField( rightHandle, right.data );
						}

						if( count % skip == 0 )
							display.showRight( right );

					}

					if( depthSwitch.getValue() ) {
						cv::Mat depth;
						dataSource->getDepth( depth );
						imageOutput.write( depthHandle, depth );

						if( loggerOutputArg.isSet() ) {
							logWriter.addField( depthHandle, depth.data );
						}

						if( count % skip == 0 )
							display.showDepth( depth );
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

			if( count % skip == 0 )
				display.waitKey();


			if( dt_us > 0 ) {
				std::chrono::steady_clock::time_point sleepTarget( loopStart + std::chrono::microseconds( dt_us ) );
				//if( std::chrono::steady_clock::now() < sleepTarget )
				std::this_thread::sleep_until( sleepTarget );
			}


			count++;

			if( numFrames > 0 && count >= numFrames ) {
				keepGoing = false;
			}
		}


		LOG(INFO) << "Cleaning up...";
		if( camera ) camera->stopRecording();

		std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );

		LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
		LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";

		std::string fileName("");
		if( svoOutputArg.isSet() ) {
			fileName = svoOutputArg.getValue();
		} else if( loggerOutputArg.isSet() ) {
			logWriter.close();
			fileName = loggerOutputArg.getValue();
		}

		if( !fileName.empty() ) {
			unsigned int fileSize = fs::file_size( fs::path(svoOutputArg.getValue() ));
			float fileSizeMB = float(fileSize) / (1024*1024);
			LOG(INFO) << "Resulting file is " << fileSizeMB << " MB";
			LOG(INFO) << "     " << fileSizeMB/dur.count() << " MB/sec";
			LOG(INFO) << "     " << fileSizeMB/count << " MB/frame";

			if( statisticsOutputArg.isSet() ) {
				ofstream out( statisticsOutputArg.getValue(), ios_base::out | ios_base::ate | ios_base::app );
				if( out.is_open() ) {
					out << resolutionToString( zedResolution ) << "," << fps << "," << (guiSwitch.isSet() ? "display" : "") << "," << count << "," << dur.count() << ","
							<< fileSizeMB << endl;
				}
			}
		}



		if( dataSource ) delete dataSource;
		if( camera ) delete camera;

	} catch (TCLAP::ArgException &e)  // catch any exceptions
	{
		LOG(WARNING) << "error: " << e.error() << " for arg " << e.argId();
		exit(-1);
	}



	return 0;
}
