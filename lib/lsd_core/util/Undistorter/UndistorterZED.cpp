

#include "util/Undistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


namespace lsd_slam
{

UndistorterZED::UndistorterZED( sl::zed::Camera *camera  )
	: UndistorterLogger( camera->getImageSize(), camera->getParameters() )
{
	unsigned int h = camera->getImageSize().height,
							 w = camera->getImageSize().width;

	// This is inflexible.   On the other hand, the Zed resolutions don't
	// scale directly to LSD-SLAM compatible resolutions (divisible by 16, etc)
	if(  h==1080 && w ==1920 ) {
		_cropSize = ImageSize( 1920, 1056 );
		_finalSize = SlamImageSize( _cropSize.width / 2, _cropSize.height / 2 );
	} else if( h==720 && w==1280 ) {
		_cropSize = ImageSize( 1280, 704 );
		_finalSize = SlamImageSize( _cropSize.width / 2, _cropSize.height / 2 );
	} else if( h==480 && w == 640 ) {
		_cropSize = ImageSize( 640, 480 );
		_finalSize = SlamImageSize( _cropSize.width, _cropSize.height );
	} else {
		LOG(FATAL) << "Don't know how to handle Zed resolution " << w << " x " << h;
	}
}

UndistorterZED::~UndistorterZED()
{
}

// void UndistorterZED::undistort(const cv::Mat& image, cv::OutputArray result) const
// {
// 	cv::Mat imageROI( image, cv::Rect( cv::Point(0,0), _cropSize.cvSize() ) );
//
// 	// Convert to greyscale
// 	cv::Mat imageGray( _cropSize.cvSize(), CV_8UC1 );
// 	cv::cvtColor( imageROI, imageGray, cv::COLOR_BGRA2GRAY );
//
// 	cv::resize( imageGray, result, _finalSize.cvSize() );
//
// 	CHECK( result.type() == CV_8U );
// 	// CHECK( (result.rows == _finalSize.height) && (result.cols == _finalSize.width) );
// }
//
// void UndistorterZED::undistortDepth(const cv::Mat& depth, cv::OutputArray result) const
// {
// 	cv::Mat depthROI( depth, cv::Rect( cv::Point(0,0), _cropSize.cvSize() ) );
// 	cv::resize( depthROI, result, _finalSize.cvSize() );
//
// 	CHECK( result.type() == CV_32F );
// 	// CHECK( (result.rows == _finalSize.height) && (result.cols == _finalSize.width) );
// }

}
