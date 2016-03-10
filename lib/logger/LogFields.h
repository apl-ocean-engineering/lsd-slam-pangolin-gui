

#pragma once

#include <memory>

#include <zlib.h>

#include <g3log/g3log.hpp>


namespace logger {

enum FieldType_t {
	FIELD_BGRA_8C = 0,
	FIELD_DEPTH_32F = 1
};

typedef unsigned int FieldHandle_t;

struct Field {
	Field( const std::string &nm, const cv::Size &sz, FieldType_t tp )
		: name(nm), size(sz), type(tp)
	{;}

	std::string name;
	cv::Size size;
	FieldType_t type;

	unsigned int compressedBytes( void ) const {
		unsigned int bytes = nBytes();
		if( bytes > 0 )
			return compressBound( nBytes() );
		return 0;
	}

	unsigned int nBytes( void ) const {
		if( type == FIELD_BGRA_8C ) {
			return size.height * size.width * 4 * sizeof( unsigned char );
		} else if( type == FIELD_DEPTH_32F ) {
			return size.height * size.width * 1 * sizeof( float );
		} else {
			return 0;
		}
	}

};
typedef std::vector<Field> Fields;

struct Chunk {
	Chunk() = delete;

	Chunk( unsigned int _sz )
		: data( new char[_sz] ), size(_sz), capacity( _sz )
	{ memset( data.get(), 0, size );}

	Chunk( const void *_data, unsigned int _sz )
		: data( new char[_sz] ), size(_sz), capacity( _sz )
	{ memcpy( data.get(), _data, size );}

	unsigned int set( void *_data, unsigned int _sz )
	{
		CHECK( _sz >= size );
		memcpy( data.get(), _data, _sz );
		return size = _sz;
	}

	std::unique_ptr<char[]> data;
	unsigned int size, capacity;
};

}
