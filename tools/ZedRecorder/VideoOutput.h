#pragma once

#include <string>
#include <map>

#include <opencv2/highgui.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "logger/LogFields.h"

namespace zed_recorder {


class VideoOutput {
public:
	VideoOutput( const string &filename, float fps )
		: _file( filename ),
			_active( false ),
			_writer(),
			_fps( fps )
	{
		if( !_file.empty() ) {
			LOG(INFO) << "Recording to video file " << _file.string();
			_active = true;
		}
	}

	// void registerField( logger::FieldHandle_t handle, const string &name )
	// {
	// 	if( handle >= 0 ) {
	// 		_names[handle] = name;
	// 	}
	// }

	// bool write( logger::FieldHandle_t handle, const Mat &img, int frame = -1 )
	bool write( const Mat &img )
	{
		// Eager-create the writer
		if( !_writer ) {
			_writer.reset( new cv::VideoWriter(_file.string(), CV_FOURCC('A','V','C','1'), _fps, cv::Size(img.cols, img.rows)));
		}

	_writer->write( img );

		return true;
	}

protected:

	fs::path _file;
	bool _active;

	std::unique_ptr< cv::VideoWriter > _writer;
	float _fps;
	//unsigned int _count;

	//std::map< logger::FieldHandle_t, std::string > _names;
};


}
