

#pragma once

#include <memory>

#include <zlib.h>

#include <g3log/g3log.hpp>


namespace logger {

	extern const uint16_t LogFormatVersion;

	enum FeatureFlags {
		ZLIB_COMPRESSION = (1 << 0),
		SNAPPY_COMPRESSION = (1 << 1)
	};


enum FieldType_t {
	FIELD_BGRA_8C = 0,
	FIELD_DEPTH_32F = 1
};

typedef int FieldHandle_t;

struct Field {
	Field( const std::string &nm, const cv::Size &sz, FieldType_t tp )
		: name(nm), size(sz), type(tp)
	{;}

	std::string name;
	cv::Size size;
	FieldType_t type;

	unsigned int compressedBytes( void ) const {
		unsigned int bytes = nBytes();
		if( bytes > 0 ) {
#ifdef USE_GOOGLE_SNAPPY
			return std::max( compressBound( bytes ), snappy::MaxCompressedLength( bytes ) );
#else
			return compressBound( bytes );
#endif
		}
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

	 int cvType( void ) const {
			if( type == FIELD_BGRA_8C ) {
				return CV_8UC4;
			} else if( type == FIELD_DEPTH_32F ) {
				return CV_32FC1;
			} else {
				return 0;
			}
		}

};
typedef std::vector<Field> Fields;

struct Chunk {
	Chunk() = delete;

	Chunk( unsigned long _sz )
		: size(_sz), _capacity( _sz ), data( new char[_capacity] )
	{ memset( data.get(), 0, size );}

	Chunk( const void *_data, unsigned long _sz )
		: size(_sz), _capacity( _sz ), data( new char[_capacity] )
	{ memcpy( data.get(), _data, size );}

	unsigned int set( void *_data, unsigned int _sz )
	{
		CHECK( _sz >= capacity() );
		memcpy( data.get(), _data, _sz );
		return size = _sz;
	}

private:

	const unsigned long _capacity;
public:

	unsigned long size;
	std::unique_ptr<char[]> data;

	unsigned long capacity( void ) const { return _capacity; }

};

}
