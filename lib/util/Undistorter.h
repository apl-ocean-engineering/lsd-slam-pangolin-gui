/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _UNDISTORTER_HPP_
#define _UNDISTORTER_HPP_

#include <opencv2/core/core.hpp>

#include "Configuration.h"

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif


namespace lsd_slam
{

class Undistorter
{
public:
	virtual ~Undistorter();

	/**
	 * Undistorts the given image and returns the result image.
	 */
	virtual void undistort(const cv::Mat &image, cv::OutputArray result) const = 0;

	virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const { depth.copyTo( result ); }

	/**
	 * Returns the intrinsic parameter matrix of the undistorted images.
	 */
	virtual const cv::Mat getK() const = 0;

	virtual const Camera getCamera() const = 0;

	/**
	 * Returns the intrinsic parameter matrix of the original images,
	 */
	virtual const cv::Mat getOriginalK() const = 0;

	virtual ImageSize outputImageSize( void ) const
		{ return ImageSize( getOutputWidth(), getOutputHeight() ); }

	/**
	 * Returns the width of the undistorted images in pixels.
	 */
	virtual int getOutputWidth() const = 0;

	/**
	 * Returns the height of the undistorted images in pixels.
	 */
	virtual int getOutputHeight() const = 0;

	virtual ImageSize inputImageSize( void ) const
		{ return ImageSize( getInputWidth(), getInputHeight() ); }

	/**
	 * Returns the width of the input images in pixels.
	 */
	virtual int getInputWidth() const = 0;

	/**
	 * Returns the height of the input images in pixels.
	 */
	virtual int getInputHeight() const = 0;


	/**
	 * Returns if the undistorter was initialized successfully.
	 */
	virtual bool isValid() const = 0;

	/**
	 * Creates and returns an Undistorter of the type used by the given
	 * configuration file. If the format is not recognized, returns nullptr.
	 */
	static Undistorter* getUndistorterForFile(const std::string &configFilename);
};

class UndistorterPTAM : public Undistorter
{
public:
	/**
	 * Creates an Undistorter by reading the distortion parameters from a file.
	 *
	 * The file format is as follows:
	 * d1 d2 d3 d4 d5
	 * inputWidth inputHeight
	 * crop / full / none
	 * outputWidth outputHeight
	 */
	UndistorterPTAM(const char* configFileName);

	/**
	 * Destructor.
	 */
	~UndistorterPTAM();

	UndistorterPTAM(const UndistorterPTAM&) = delete;
	UndistorterPTAM& operator=(const UndistorterPTAM&) = delete;

	/**
	 * Undistorts the given image and returns the result image.
	 */
	void undistort(const cv::Mat &image, cv::OutputArray result) const;

	/**
	 * Returns the intrinsic parameter matrix of the undistorted images.
	 */
	const cv::Mat getK() const;

	virtual const Camera getCamera() const;


	/**
	 * Returns the intrinsic parameter matrix of the original images,
	 */
	const cv::Mat getOriginalK() const;

	/**
	 * Returns the width of the undistorted images in pixels.
	 */
	int getOutputWidth() const;

	/**
	 * Returns the height of the undistorted images in pixels.
	 */
	int getOutputHeight() const;

	/**
	 * Returns the width of the input images in pixels.
	 */
	int getInputWidth() const;

	/**
	 * Returns the height of the input images in pixels.
	 */
	int getInputHeight() const;


	/**
	 * Returns if the undistorter was initialized successfully.
	 */
	bool isValid() const;

private:
	cv::Mat K_;
	cv::Mat originalK_;

	float inputCalibration[5];
	float outputCalibration[5];
	int out_width, out_height;
	int in_width, in_height;
	float* remapX;
	float* remapY;


	/// Is true if the undistorter object is valid (has been initialized with
	/// a valid configuration)
	bool valid;
};

class UndistorterOpenCV : public Undistorter
{
public:
	/**
	 * Creates an Undistorter by reading the distortion parameters from a file.
	 *
	 * The file format is as follows:
	 * fx fy cx cy d1 d2 d3 d4 d5 d6
	 * inputWidth inputHeight
	 * crop / full / none
	 * outputWidth outputHeight
	 */
	UndistorterOpenCV(const char* configFileName);

	/**
	 * Destructor.
	 */
	~UndistorterOpenCV();

	UndistorterOpenCV(const UndistorterOpenCV&) = delete;
	UndistorterOpenCV& operator=(const UndistorterOpenCV&) = delete;

	/**
	 * Undistorts the given image and returns the result image.
	 */
	void undistort(const cv::Mat &image, cv::OutputArray result) const;

	/**
	 * Returns the intrinsic parameter matrix of the undistorted images.
	 */
	const cv::Mat getK() const;

	virtual const Camera getCamera() const;

	/**
	 * Returns the intrinsic parameter matrix of the original images,
	 */
	const cv::Mat getOriginalK() const;

	/**
	 * Returns the width of the undistorted images in pixels.
	 */
	int getOutputWidth() const;

	/**
	 * Returns the height of the undistorted images in pixels.
	 */
	int getOutputHeight() const;


	/**
	 * Returns the width of the input images in pixels.
	 */
	int getInputWidth() const;

	/**
	 * Returns the height of the input images in pixels.
	 */
	int getInputHeight() const;

	/**
	 * Returns if the undistorter was initialized successfully.
	 */
	bool isValid() const;

private:
	cv::Mat K_;
	cv::Mat originalK_;

	float inputCalibration[10];
	float outputCalibration;
	int out_width, out_height;
	int in_width, in_height;
	cv::Mat map1, map2;

	/// Is true if the undistorter object is valid (has been initialized with
	/// a valid configuration)
	bool valid;
};


class UndistorterLogger : public Undistorter
{
public:
	/**
	 * Creates an Undistorter by reading the distortion parameters from a file.
	 *
	 * The file format is as follows:
	 * fx fy cx cy
	 * inputWidth inputHeight
	 * cropWidth cropHeight
	 * outputWidth outputHeight
	 */
	UndistorterLogger(const char* configFileName);

	/**
	 * Destructor.
	 */
	~UndistorterLogger();

	UndistorterLogger(const UndistorterLogger&) = delete;
	UndistorterLogger& operator=(const UndistorterLogger&) = delete;

	void undistort(const cv::Mat &image, cv::OutputArray result) const;
	virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const;


	const cv::Mat getK() const;
	virtual const Camera getCamera() const;
	virtual const Camera getOriginalCamera( void ) const { return _originalCamera; }


	const cv::Mat getOriginalK() const;
	int getOutputWidth() const						{ return _finalSize.width; }
	int getOutputHeight() const						{ return _finalSize.height; }
	int getInputWidth() const							{ return _inputSize.width; }
	int getInputHeight() const						{ return _inputSize.height; }


	bool isValid() const { return _valid; }

#ifdef USE_ZED
	static bool calibrationFromZed( sl::zed::Camera *camera, const std::string &filename );
#endif

protected:

	UndistorterLogger( const ImageSize &inputSize, const Camera &cam  );

	ImageSize _inputSize, _cropSize, _finalSize;
	Camera _originalCamera;

	bool _valid;
};

#ifdef USE_ZED

class UndistorterZED : public UndistorterLogger
{
public:
	/**
	 * Creates an Undistorter which wraps a Zed camera.  Doesn't
   * take any further parameters, as the Zed does its own undistorion.
	 * for a Zed camera.  Determines cropped and final size automatically
	 * from input resolution.
	 */
	UndistorterZED( sl::zed::Camera *camera );

	/**
	 * Destructor.
	 */
	~UndistorterZED();

	UndistorterZED(const UndistorterZED&) = delete;
	UndistorterZED& operator=(const UndistorterZED&) = delete;

	// Until a Zed-optimized version is written, devolve these to UndistortLogger
	//
	// void undistort(const cv::Mat &image, cv::OutputArray result) const;
	// virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const;


private:


};

#endif



}
#endif
