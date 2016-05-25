
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <zed/Camera.hpp>

#include "active_object/active.h"

#include <tclap/CmdLine.h>

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>
#include "util/G3LogSinks.h"
#include "util/ZedUtils.h"
#include "util/DataSource.h"
#include "util/Undistorter.h"

#include "logger/LogWriter.h"

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


class Display {
public:
	Display( bool doDisplay )
		: _doDisplay( doDisplay ),
			_displaySize( 640, 480 ),
			_active( active_object::Active::createActive() )
	{}

	void showRGB( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowRGB, this, img )); }

	void showStereoYUV( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowStereoYUV, this, img )); }

protected:

	void onShowRGB( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img, resized, _displaySize );
		imshow( "Display", resized );
		cv::waitKey(1);
	}

	void onShowStereoYUV( const Mat &img )
	{
		cv::Mat leftRoi( img, cv::Rect(0,0, img.cols/2, img.rows ));
		cv::Mat leftBgr;
		cv::resize( leftRoi, leftBgr, _displaySize );
		cv::cvtColor( leftBgr, leftBgr, cv::COLOR_YUV2BGRA_YUYV );
		imshow( "Display", leftBgr );
		cv::waitKey(1);
	}

	bool _doDisplay;
	cv::Size _displaySize;
	std::unique_ptr<active_object::Active> _active;
};



int main( int argc, char** argv )
{
	auto worker = g3::LogWorker::createLogWorker();
	auto stderrHandle = worker->addSink(std::unique_ptr<ColorStderrSink>( new ColorStderrSink ),
	&ColorStderrSink::ReceiveLogMessage);
	g3::initializeLogging(worker.get());

	signal( SIGINT, signal_handler );

	bool doDepth = false, doRight = false, doGui = false;

	try {
		TCLAP::CmdLine cmd("LSDRecorder", ' ', "0.1");

		TCLAP::ValueArg<std::string> resolutionArg("r","resolution","",false,"hd1080","", cmd);
		TCLAP::ValueArg<float> fpsArg("f","fps","",false,0.0,"", cmd);

		TCLAP::ValueArg<std::string> logInputArg("","log-input","Input Logger file",false,"","Logger filename", cmd);
		TCLAP::ValueArg<std::string> svoInputArg("i","svo-input","Input SVO file",false,"","Filename", cmd);
		TCLAP::ValueArg<std::string> svoOutputArg("s","svo-output","Output SVO file",false,"","SVO filename", cmd);
		TCLAP::ValueArg<std::string> loggerOutputArg("l","log-output","Output Logger filename",false,"","SVO filename", cmd);
		TCLAP::ValueArg<std::string> calibOutputArg("","calib-output","Output calibration file (from stereolabs SDK)",false,"","Calib filename", cmd);

		TCLAP::ValueArg<std::string> compressionArg("","compression","",false,"snappy","SVO filename", cmd);

		TCLAP::ValueArg<std::string> imageOutputArg("","image-output","",false,"","SVO filename", cmd);

		TCLAP::ValueArg<std::string> statisticsOutputArg("","statistics-output","",false,"","", cmd);

		// TCLAP::SwitchArg noGuiSwitch("","no-gui","Don't show a GUI", cmd, false);

		TCLAP::SwitchArg depthSwitch("","depth","", cmd, false);
		TCLAP::SwitchArg rightSwitch("","right","", cmd, false);

		TCLAP::SwitchArg guiSwitch("","display","", cmd, false);


		TCLAP::ValueArg<int> durationArg("","duration","Duration",false,0,"seconds", cmd);

		cmd.parse(argc, argv );

		doDepth = depthSwitch.getValue();
		doRight = rightSwitch.getValue();
		doGui = guiSwitch.getValue();

		Display display( guiSwitch.getValue() );

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
		if( !svoOutputArg.isSet() && !imageOutputArg.isSet() && !loggerOutputArg.isSet() && !doGui ) {
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
		const sl::zed::MODE zedMode = sl::zed::MODE::NONE;
		const int whichGpu = -1;
		const bool verboseInit = true;

		DataSource *dataSource = NULL;

		sl::zed::Camera *camera = NULL;

		if( logInputArg.isSet() ) {
			LOG(INFO) << "Loading logger data from " << logInputArg.getValue();
			dataSource = new LoggerSource( logInputArg.getValue() );

			LOG_IF(FATAL, doDepth && !dataSource->hasDepth() ) << "Depth requested but log file doesn't have depth data.";
			LOG_IF(FATAL, doRight && dataSource->numImages() < 2 ) << "Depth requested but log file doesn't have depth data.";

			if( calibOutputArg.isSet() )
				LOG(WARNING) << "Can't create calibration file from a log file.";

		} else {
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
				err = camera->init( sl::zed::PERFORMANCE, -1, true );
			}

			dataSource = new ZedSource( camera, doDepth );

			if (err != sl::zed::SUCCESS) {
				LOG(WARNING) << "Unable to init the zed: " << errcode2str(err);
				delete camera;
				exit(-1);
			}

			// Does this need to be called after a grab()?
			if( calibOutputArg.isSet() ) {
				if( svoInputArg.isSet() )	{
					LOG(INFO) << "Calibration not loaded when logging to SVO?";
				} else {
					LOG(INFO) << "Saving calibration to \"" << calibOutputArg.getValue() << "\"";
					lsd_slam::UndistorterLogger::calibrationFromZed( camera, calibOutputArg.getValue() );
				}
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
			if( doDepth ) depthHandle = logWriter.registerField( "depth", sz, logger::FIELD_DEPTH_32F );
			if( doRight ) rightHandle = logWriter.registerField( "right", sz, logger::FIELD_BGRA_8C );

			if( !logWriter.open( loggerOutputArg.getValue() ) ) {
				LOG(FATAL) << "Unable to open file " << loggerOutputArg.getValue() << " for logging.";
			}
		}


		int dt_us = (fps > 0) ? (1e6/fps) : 0;
		const float sleepWiggle = 0.9;
		dt_us *= sleepWiggle;

		LOG(INFO) << "Input is at " << resolutionToString( zedResolution ) << " at nominal " << fps << "FPS";

		std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
		int duration = durationArg.getValue();
		std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );

		std::vector< int > guiDuration;

		if( duration > 0 )
			LOG(INFO) << "Will log for " << duration << " seconds or press CTRL-C to stop.";
		else
			LOG(INFO) << "Logging now, press CTRL-C to stop.";

		// Wait for the auto exposure and white balance
		std::this_thread::sleep_for(std::chrono::seconds(1));

		int count = 0;
		while( keepGoing ) {
			if( count > 0 && (count % 100)==0 ) LOG(INFO) << count << " frames";

			std::chrono::steady_clock::time_point loopStart( std::chrono::steady_clock::now() );

			if( (duration > 0) && (loopStart > end) ) { keepGoing = false;  break; }

			if( svoOutputArg.isSet() ) {

				if( camera->record() ) {
					LOG(WARNING) << "Error occured while recording from camera";
				} else {
					// if( doGui ) {

//						std::chrono::steady_clock::time_point guiStart( std::chrono::steady_clock::now() );
						// According to the docs, this:
						//		Get[s] the current side by side YUV 4:2:2 frame, CPU buffer.
						sl::zed::Mat slRawImage( camera->getCurrentRawRecordedFrame() );
						// LOG(INFO) << slRawImage.width << " x " << slRawImage.height;
						// LOG(INFO) << slRawImage.channels << " " << slRawImage.data_type << " " << slRawImage.type;

						// Zed presents YUV data as 4 channels at {resolution} (e.g. 640x480)
						// despite actually showing both Left and Right (e.g. 1280x480)
						// This actually makes sense at YUV uses 4 bytes to show 2 pixels
						// presumably the channels are ordered [U, Y1, V, Y2]
						//
						// OpenCV expects two channels of [U,Y1], [V,Y2] at the actual
						// image resoluton (1280x480)
						// If the byte ordering (U,Y1,V,Y2) is the same, then a simple
						// reshape (2 channel, rows=0 means retain # of rows) should suffice
						Mat rawCopy;
						sl::zed::slMat2cvMat( slRawImage ).reshape( 2, 0 ).copyTo( rawCopy );
						display.showStereoYUV( rawCopy );

						// LOG(INFO) << "Raw image is now " << rawImage.cols << " x " << rawImage.rows;
						// LOG(INFO) << rawImage.channels() << " " <<rawImage.depth() << " " << ((rawImage.type() == CV_8UC2) ? "CV_8UC2" : "not CV_8UC2");

						// cv::Mat leftRoi( rawImage, cv::Rect(0,0, rawImage.cols/2, rawImage.rows ));
						//
						// cv::Size displaySize( 640, 480);
						// cv::Mat leftBgr;
						// cv::resize( leftRoi, leftBgr, displaySize );
						// cv::cvtColor( leftBgr, leftBgr, cv::COLOR_YUV2BGRA_YUYV );

						//cv::imshow( "Left", leftBgr );
						//cv::waitKey(1);

						// std::chrono::duration<float> dt(std::chrono::steady_clock::now() - guiStart);
						// guiDuration.push_back( dt.count() * 1e6 );

						// Canned routine from Stereolabs
						//camera->displayRecorded();
					//}
				}


			} else {


				if( dataSource->grab() ) {

					cv::Mat left;
					dataSource->getImage( 0, left );

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
						cv::Mat right;
						dataSource->getImage( 1, right );
						if( imageOutputArg.isSet() ) {
							char filename[80];
							snprintf(filename, 79, "right_%06d.png", count );
							cv::imwrite( (imageOutputDir / filename).string(), left );
						} else if( loggerOutputArg.isSet() ) {
							logWriter.addField( rightHandle, right.data );
						}

						if( doGui ) {
							if( right.empty() ) {
								LOG(WARNING) << "Right image is empty, not displaying";
							} else {
								cv::imshow("Right", right);
							}
						}
					}

					if( doDepth ) {
						cv::Mat depth;
						dataSource->getDepth( depth );
						if( imageOutputArg.isSet() ) {
							char filename[80];
							snprintf(filename, 79, "depth_%06d.png", count );
							cv::imwrite( (imageOutputDir / filename).string(), left );
						} else if( loggerOutputArg.isSet() ) {
							logWriter.addField( depthHandle, depth.data );
						}

						if( doGui ) {
							if( depth.empty() ) {
								LOG(WARNING) << "Depth image is empty, not displaying";
							} else {
								cv::imshow("Depth", depth);
							}
						}
					}

					if( doGui ) {
						if( left.empty() ) {
							LOG(WARNING) << "Left image is empty, not displaying";
						} else {
							cv::imshow("Left", left);
							cv::waitKey(1);
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
		camera->stopRecording();

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
					out << resolutionToString( zedResolution ) << "," << fps << "," << (doGui ? "display" : "") << "," << count << "," << dur.count() << ","
							<< fileSizeMB << endl;
				}
			}
		}


		if( guiDuration.size() > 0 ) {
			int total = std::accumulate( guiDuration.begin(), guiDuration.end(), 0 );

			LOG(INFO) << "Gui required " << float(total)/guiDuration.size() << " us avg.";
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
