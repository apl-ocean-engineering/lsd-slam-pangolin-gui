
#include "util/Undistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace lsd_slam
{


UndistorterLogger::UndistorterLogger( const ImageSize &inputSize, const Camera &cam  )
	: _inputSize( inputSize ),
		_cropSize( inputSize ),
		_finalSize( inputSize ),
		_originalCamera( cam ),
		_valid( true )
{
	;
}

UndistorterLogger::UndistorterLogger(const char* configFileName)
	 : _valid( false )
{
	// read parameters
	std::ifstream infile(configFileName);
	assert(infile.good());

	std::string l1, l2, l3, l4;

	std::getline(infile,l1);
	std::getline(infile,l2);
	std::getline(infile,l3);
	std::getline(infile,l4);

	// l1 & l2
	float fx, fy, cx, cy;
	int in_width, in_height;

	if(std::sscanf(l1.c_str(), "%f %f %f %f", &fx, &fy, &cx, &cy ) == 4 &&
			std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
	{
		_inputSize.width = in_width;
		_inputSize.height = in_height;

		_originalCamera = Camera( fx, fy, cx, cy );

		LOG(INFO) << "Input resolution " << _inputSize.width << ", " << _inputSize.height;
		// printf("In: %f %f %f %f %f %f %f %f\n",
		// 		inputCalibration[0], inputCalibration[1], inputCalibration[2], inputCalibration[3], inputCalibration[4],
		// 		inputCalibration[5], inputCalibration[6], inputCalibration[7]);
	}
	else
	{
		LOG(WARNING) << "Failed to read camera calibration (invalid format?) from calibration file: " << configFileName;
		_valid = false;
		return;
	}




	int crop_width, crop_height;
	if(std::sscanf(l3.c_str(), "%d %d", &crop_width, &crop_height) == 2)
	{
		_cropSize.width = crop_width;
		_cropSize.height = crop_height;
		LOG(INFO) << "Crop resolution " << _cropSize.width << ", " << _cropSize.height;
	} else {
		LOG(WARNING) << "Failed to read camera calibration (invalid format?) from calibration file: " << configFileName;
		_valid = false;
		return;
	}

	int final_width, final_height;
	if(std::sscanf(l4.c_str(), "%d %d", &final_width, &final_height) == 2)
	{
		_finalSize.width = final_width;
		_finalSize.height = final_height;
		LOG(INFO) << "Final resolution " << _finalSize.width << ", " << _finalSize.height;
	} else {
		LOG(WARNING) << "Failed to read camera calibration (invalid format?) from calibration file: " << configFileName;
		_valid = false;
		return;
	}

	_valid = true;
}

UndistorterLogger::~UndistorterLogger()
{
}

void UndistorterLogger::undistort(const cv::Mat& image, cv::OutputArray result) const
{
	cv::Mat imageROI( image, cv::Rect( cv::Point(0,0), _cropSize.cvSize() ) );

	// Convert to greyscale
	if( imageROI.channels() > 1 ) {
		cv::Mat imageGray( _cropSize.cvSize(), CV_8UC1 );
		cv::cvtColor( imageROI, imageGray, cv::COLOR_BGRA2GRAY );
		cv::resize( imageGray, result, _finalSize.cvSize() );
	} else {
		cv::resize( imageROI, result, _finalSize.cvSize() );
	}

	CHECK( result.type() == CV_8U );
}

void UndistorterLogger::undistortDepth(const cv::Mat& depth, cv::OutputArray result) const
{
	cv::Mat depthROI( depth, cv::Rect( cv::Point(0,0), _cropSize.cvSize() ) );
	cv::resize( depthROI, result, _finalSize.cvSize() );

	CHECK( result.type() == CV_32F );
	// CHECK( (result.rows == _finalSize.height) && (result.cols == _finalSize.width) );
}

const Camera UndistorterLogger::getCamera() const
{
	float xscale = getOutputWidth() * 1.0f / getInputWidth();
	float yscale = getOutputHeight() * 1.0f / getInputHeight();

	return _originalCamera.scale( xscale, yscale );
}

const cv::Mat UndistorterLogger::getK() const
{
	cv::Mat out;
	cv::eigen2cv(getCamera().K, out);
	return out;
}

const cv::Mat UndistorterLogger::getOriginalK() const
{
	cv::Mat out;
	cv::eigen2cv(getOriginalCamera().K, out);
	return out;
}


#ifdef USE_ZED
bool UndistorterLogger::calibrationFromZed( sl::zed::Camera *camera, const std::string &filename )
{
	std::ofstream out( filename );
	if( !out.is_open() ) {
		LOG(WARNING) << "Unable to write to calibration file \"" << filename << "\"";
		return false;
	}

	sl::zed::StereoParameters *params = camera->getParameters();

	const sl::zed::CamParameters &left( params->LeftCam );
	out << left.fx << " " << left.fy << " " << left.cx << " " << left.cy << std::endl;

	sl::zed::resolution res( camera->getImageSize() );
	out << res.width << " " << res.height << std::endl;

	return true;
}


#endif


}
