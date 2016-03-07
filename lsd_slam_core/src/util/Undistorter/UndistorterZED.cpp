

#include "util/Undistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>


namespace lsd_slam
{

UndistorterZED::UndistorterZED( sl::zed::Camera *camera, const ImageSize &cropSize, const ImageSize &finalSize )
	: _inputSize( camera->getImageSize() ),
		_cropSize( cropSize ),
		_finalSize( finalSize ),
		_originalCamera( camera->getParameters() )
{
	;
}

UndistorterZED::~UndistorterZED()
{
}

void UndistorterZED::undistort(const cv::Mat& image, cv::OutputArray result) const
{
	cv::Mat imageROI( image, cv::Rect( cv::Point(0,0), _cropSize.cvSize() ) );

	// Convert to greyscale
	cv::Mat imageGray( _cropSize.cvSize(), CV_8UC1 );
	cv::cvtColor( imageROI, imageGray, cv::COLOR_BGRA2GRAY );

	cv::resize( imageGray, result, _finalSize.cvSize() );

	CHECK( result.type() == CV_8U );
	// CHECK( (result.rows == _finalSize.height) && (result.cols == _finalSize.width) );

}

const Camera UndistorterZED::getCamera() const
{
	float xscale = getOutputWidth() * 1.0f / getInputWidth();
	float yscale = getOutputHeight() * 1.0f / getInputHeight();

	return _originalCamera.scale( xscale, yscale );
}

const cv::Mat UndistorterZED::getK() const
{
	cv::Mat out;
	cv::eigen2cv(getCamera().K, out);
	std::cout << out << std::endl;
	return out;
}

Camera UndistorterZED::getOriginalCamera() const
{
	return _originalCamera;
}

const cv::Mat UndistorterZED::getOriginalK() const
{
	cv::Mat out;
	cv::eigen2cv(getOriginalCamera().K, out);
	return out;
}

int UndistorterZED::getOutputWidth() const
{
	return _finalSize.width;
}

int UndistorterZED::getOutputHeight() const
{
	return _finalSize.height;
}

int UndistorterZED::getInputWidth() const
{
	return _inputSize.width;
}

int UndistorterZED::getInputHeight() const
{
	return _inputSize.height;
}


bool UndistorterZED::isValid() const
{
	return true;
}

}
