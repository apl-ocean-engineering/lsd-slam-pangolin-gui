

#include "LogWriter.h"

#ifdef USE_SNAPPY
#include <snappy.h>
#endif
#include <zlib.h>


namespace logger {

	const uint16_t LogFormatVersion = 1;

	using active_object::Active;

LogWriter::LogWriter( int level )
	: _writer( Active::createActive() ),
		fp( NULL ),
		numFrames( 0 ),
		_compressionLevel( level )
{
#ifdef USE_SNAPPY
	CHECK( (_compressionLevel == SnappyCompressLevel) || (_compressionLevel == Z_DEFAULT_COMPRESSION) || (_compressionLevel >= Z_NO_COMPRESSION && _compressionLevel <= Z_BEST_COMPRESSION ) ) << "Unknown compression level " << _compressionLevel;

	if( _compressionLevel == SnappyCompressLevel ) {
		LOG(INFO) << "Using Snappy compression";
	} else {
		LOG(INFO) << "Using Zlib compression level " << _compressionLevel;
	}

#else
	if( _compressionLevel == SnappyCompressLevel ) {
			LOG(FATAL) << "Not compiled with Snappy compression";
	}
	CHECK( (_compressionLevel == Z_DEFAULT_COMPRESSION) || (_compressionLevel >= Z_NO_COMPRESSION && _compressionLevel <= Z_BEST_COMPRESSION) ) << "Unknown compression level " << _compressionLevel;
#endif
}

LogWriter::~LogWriter()
{}

FieldHandle_t LogWriter::registerField( const std::string &name, const cv::Size &sz, FieldType_t type )
{
	_fields.emplace_back( name, sz, type );

	const Field &f( _fields.back() );
	CHECK( f.size.width != 0 && f.size.height != 0 && f.nBytes() != 0 );

	_compressorOutput.emplace_back( new Chunk(f.compressedBytes()) );
	_compressorMutex.emplace_back();
	_compressorDone.push_back( true );
	_fieldUpdated.push_back( false );
	_compressors.push_back( Active::createActive() );

	return _fields.size()-1;
}

bool LogWriter::open( const std::string &filename )
{
	if( fp ) {
		LOG(WARNING) << "Already logging, can't open a new file.";
		return false;
	}
	numFrames = 0;

	fp = fopen(filename.c_str(), "wb+");

	if( !fp ) {
		LOG(WARNING) << "Unable to open file " << filename << " for logging." << strerror( errno );
		return false;
	}

	writeHeader();
	return true;
}

bool LogWriter::close( void )
{
	if( !fp ) {
		LOG(WARNING) << "Calling close() when file hasn't been opened.";
		return false;
	}

	// Wait for all threads to complete.
	_compressors.clear();				// Destructor for includes join();
	delete _writer.release();

	fseek(fp, 2*sizeof(uint16_t), SEEK_SET);
  fwrite(&numFrames, sizeof(int32_t), 1, fp);

  fflush(fp);
  fclose(fp);

	fp = NULL;

	numFrames = 0;
	return true;
}


int LogWriter::newFrame( void )
{
	for( unsigned int handle = 0; handle < _fields.size(); ++handle ) {
		_fieldUpdated[handle] = false;
	}
	return 0;
}

void LogWriter::addField( FieldHandle_t handle, const void *data )
{
	CHECK( handle >= 0 && handle < _fields.size() );

	{
		std::lock_guard< std::mutex > lock( _compressorMutex[handle] );

		// This is a pathological condition
		while( !_compressorDone[handle]) { std::this_thread::sleep_for( std::chrono::milliseconds(100) ); }
			_compressorDone[handle] = false;
		}

	if( _fields[handle].type == FIELD_BGRA_8C ) {
		bgCompressPng( handle, data );
	} else if( _fields[handle].type == FIELD_DEPTH_32F ) {
		bgCompressPng( handle, data );
	}

	_fieldUpdated[handle] = true;
}

void LogWriter::addField( FieldHandle_t handle, const cv::Mat &mat )
{
	CHECK( handle >= 0 && handle < _fields.size() );
	CHECK( mat.rows == _fields[handle].size.height && mat.cols == _fields[handle].size.width );

	if( _fields[handle].type == FIELD_DEPTH_32F ) {
		CHECK( mat.type() == CV_32FC1 );
	} else if( _fields[handle].type == FIELD_BGRA_8C ){
		CHECK( mat.type() == CV_8UC4 );
	}

	CHECK( mat.isContinuous() );

	addField( handle, mat.ptr<void>(0) );
}


bool LogWriter::allCompressionDone( void )
{
	for( unsigned int handle = 0; handle < _fields.size(); ++handle ) {
			std::lock_guard< std::mutex > lock( _compressorMutex[handle] );

			if( _compressorDone[handle] == false ) return false;
	}
	return true;
}

bool LogWriter::writeFrame( bool doBlock )
{
	// Wait for all frames to be done
	while( !allCompressionDone() ) {
 	// 	LOG(INFO) << "Waiting for compressors....";
		std::this_thread::sleep_for( std::chrono::milliseconds(1) );
	}

	for( unsigned int handle = 0; handle < _fields.size(); ++handle ) {
		if( !_fieldUpdated[handle] ) return false;
	}

	// LOG(INFO) << "Compressors done, writing frame." << numFrames;

	// Write per-frame headers if needed...

	for( unsigned int handle = 0; handle < _fields.size(); ++handle ) {

		// Should be unnecessary, there should be no compressors running.
		std::lock_guard< std::mutex > lock( _compressorMutex[handle] );

		if( doBlock ){
			writeData( _compressorOutput[handle]  );
		} else {
			// Push into the background queue
			// This causes a copy into a new chunk which is queued to the file writer...
			bgWriteData( _compressorOutput[handle] );
		}
	}

	// LOG(INFO) << "Wrote frame " << numFrames;
	++numFrames;
	return true;
}


void LogWriter::compressPng( unsigned int handle, std::shared_ptr<Chunk> chunk )
{
	{
		std::lock_guard< std::mutex > lock( _compressorMutex[handle] );

		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
#ifdef USE_SNAPPY
		if( _compressionLevel == SnappyCompressLevel ) {
			size_t destSz = _compressorOutput[handle]->capacity();
			snappy::RawCompress( chunk->data.get(), chunk->size, (char *)_compressorOutput[handle]->data.get(),  &destSz );
			_compressorOutput[handle]->size = destSz;
		}
		else
#endif
		{
			uLongf destSz = _compressorOutput[handle]->capacity();
			int status = compress2( (Bytef *)_compressorOutput[handle]->data.get(), &destSz, (Bytef *)chunk->data.get(), chunk->size, _compressionLevel );
			CHECK( status == Z_OK ) << "Compress status: " << status;
			_compressorOutput[handle]->size = destSz;
		}
		LOG(DEBUG) << "PNG compression required " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << " ms";
		_compressorDone[handle] = true;
	}
}

// The two functions are currently identical
// void LogWriter::compressDepth( unsigned int handle, std::shared_ptr<Chunk> chunk )
// {
// 	{
// 		std::lock_guard< std::mutex > lock( _compressorMutex[handle] );
//
// 		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
//
// 		#ifdef USE_SNAPPY
// 				if( _compressionLevel == SnappyCompressLevel ) {
// 					size_t destSz = _compressorOutput[handle]->capacity();
// 					snappy::RawCompress( chunk->data.get(), chunk->size, (char *)_compressorOutput[handle]->data.get(),  &destSz );
// 					_compressorOutput[handle]->size = destSz;
// 				}
// 				else
// 		#endif
// 				{
// 					uLongf destSz = _compressorOutput[handle]->capacity();
// 					int status = compress2( (Bytef *)_compressorOutput[handle]->data.get(), &destSz, (Bytef *)chunk->data.get(), chunk->size, _compressionLevel );
// 					CHECK( status == Z_OK ) << "Compress status: " << status;
// 					_compressorOutput[handle]->size = destSz;
// 				}
//
// 		LOG(DEBUG) << "Depth compression required " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << " ms";
// 		_compressorDone[handle] = true;
// 	}
// }

void LogWriter::writeHeader( void )
{
	// unsigned int bufferSize = 2 * sizeof(int32_t);
	// for( const Field &field : _fields ) {
	// 	bufferSize += 4 * sizeof( int32_t ) + field.name.size();
	// }

	fwrite( &LogFormatVersion, sizeof(int16_t), 1, fp );

	uint16_t featureFlags = 0;

	if( _compressionLevel == SnappyCompressLevel ) {
		featureFlags |= SNAPPY_COMPRESSION;
	} else {
		featureFlags |= ZLIB_COMPRESSION;
	}

	LOG(DEBUG) << "Feature flags: " << featureFlags;
	fwrite( &featureFlags, sizeof(int16_t), 1, fp );

	LOG(DEBUG) << "Num frames: " << numFrames;
	fwrite(&numFrames, sizeof(int32_t), 1, fp);

	int32_t val = _fields.size();
	fwrite(&val, sizeof(int32_t), 1, fp);

	for( const Field &field : _fields ) {
		int32_t h( field.size.height ), w( field.size.width ), type( field.type ), len( field.name.length() );

		LOG(DEBUG) << "Write field \"" << field.name << "\"";

		fwrite(&h, sizeof(int32_t), 1, fp);
		fwrite(&w, sizeof(int32_t), 1, fp);
		fwrite(&type, sizeof(int32_t), 1, fp);
		fwrite(&len, sizeof(int32_t), 1, fp);
		fwrite((field.name.data()), sizeof(std::string::value_type), len, fp);

	}
}

}
