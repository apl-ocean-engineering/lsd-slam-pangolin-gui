#pragma once

#include "active_object/active.h"

#include <opencv2/opencv.hpp>

namespace zed_recorder {

using namespace cv;

class Display {
public:
	Display( bool doDisplay )
		: _doDisplay( doDisplay ),
			_displaySize( 640, 480 ),
			_active( active_object::Active::createActive() )
	{}

	void showLeft( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowLeft, this, img )); }

	void showDepth( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowDepth, this, img )); }

	void showRight( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowRight, this, img )); }

	void showRawStereoYUV( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowRawStereoYUV, this, img )); }

	void waitKey( int wk = 1 )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onWaitKey, this, wk )); }

protected:

	void onShowLeft( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img, resized, _displaySize );
		imshow( "Left", resized );
	}

	void onShowDepth( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img*255, resized, _displaySize );
		imshow( "Display", resized );
	}

	void onShowRight( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img, resized, _displaySize );
		imshow( "Right", resized );
	}

	void onShowRawStereoYUV( const Mat &img )
	{
		// Zed presents YUV data as 4 channels at {resolution} (e.g. 640x480)
		// despite actually showing both Left and Right (e.g. 1280x480)
		// This actually makes sense at YUV uses 4 bytes to show 2 pixels
		// presumably the channels are ordered [U, Y1, V, Y2]
		//
		// OpenCV expects two channels of [U,Y1], [V,Y2] at the actual
		// image resoluton (1280x480)
		// If the byte ordering (U,Y1,V,Y2) is the same, then a simple
		// reshape (2 channel, rows=0 means retain # of rows) should suffice

		cv::Mat leftRoi( img, cv::Rect(0,0, img.cols/2, img.rows ));
		cv::Mat leftBgr;
		cv::resize( leftRoi, leftBgr, _displaySize );
		cv::cvtColor( leftBgr, leftBgr, cv::COLOR_YUV2BGRA_YUYV );
		imshow( "RawLeft", leftBgr );
	}

	void onWaitKey( unsigned int k )
	{
		cv::waitKey(k);
	}

	bool _doDisplay;
	cv::Size _displaySize;
	std::unique_ptr<active_object::Active> _active;
};

}
