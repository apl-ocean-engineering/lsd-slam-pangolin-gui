

#include "util/Undistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


namespace lsd_slam
{

UndistorterZED::UndistorterZED( sl::zed::Camera *camera, const ImageSize &cropSize, const ImageSize &finalSize )
	: UndistorterLogger( camera->getImageSize(), cropSize, finalSize, camera->getParameters() )
{
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
