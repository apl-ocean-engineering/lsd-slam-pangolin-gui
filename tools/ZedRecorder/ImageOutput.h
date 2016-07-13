#pragma once

#include <string>
#include <map>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "logger/LogFields.h"

namespace zed_recorder {


class ImageOutput {
public:
	ImageOutput( const string &path )
		: _path( path ),
			_active( false )
	{
		if( !_path.empty() ) {
			LOG(INFO) << "Recording to directory " << _path.string();

			if( !is_directory( _path ) ) {
				LOG(WARNING) << "Making directory " << _path.string();
				create_directory( _path );
			}

			_active = true;
		}
	}

	void registerField( logger::FieldHandle_t handle, const string &name )
	{
		if( handle >= 0 ) {
			_names[handle] = name;
			_count[handle] = 0;
		}
	}

	bool write( logger::FieldHandle_t handle, const Mat &img, int frame = -1 )
	{
		if( !_active ) return true;
		if( _names.count(handle) == 0 ) return  false;
		char buf[80];
		snprintf(buf, 79, "%s_%06d.png", _names[handle].c_str(), (frame < 0 ? _count[handle] : frame ) );
		fs::path imgPath( _path );
		imgPath /= buf;

		imwrite( imgPath.string(), img );
		_count[handle]++;
		return true;
	}

protected:

	fs::path _path;
	bool _active;
	std::map< logger::FieldHandle_t, unsigned int > _count;

	std::map< logger::FieldHandle_t, std::string > _names;
};


}
