

#include "LogWriter.h"


namespace logger {

LogWriter::LogWriter( void )
	: _writer( Active::createActive() ),
		fp( NULL ),
		numFrames( 0 )
{}

LogWriter::~LogWriter()
{}

FieldHandle_t LogWriter::registerField( const std::string &name, const cv::Size &sz, FieldType_t type )
{
	_fields.emplace_back( name, sz, type );

	const Field &f( _fields.back() );
	CHECK( f.size.width != 0 && f.size.height != 0 && f.nBytes() != 0 );

	_compressors.push_back( Active::createActive() );
	_compressorOutput.push_back( f.compressedBytes() );
	_compressorMutex.emplace_back();
	_compressorDone.push_back( true );
	_fieldUpdated.push_back( false );

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

	fseek(fp, 0, SEEK_SET);
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
		compressPng( handle, data );
	} else if( _fields[handle].type == FIELD_DEPTH_32F ) {
		compressDepth( handle, data );
	}

	_fieldUpdated[handle] = true;
}

void LogWriter::addField( FieldHandle_t handle, const cv::Mat &mat )
{
	CHECK( handle >= 0 && handle < _fields.size() );
	CHECK( mat.rows == _fields[handle].size.height && mat.cols == _fields[handle].size.width );
	if( _fields[handle].type == FIELD_DEPTH_32F ) CHECK( mat.type() == CV_32FC1 );
	else if( _fields[handle].type == FIELD_BGRA_8C ) CHECK( mat.type() == CV_8UC4 );
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
		// This causes another copy into a a chunk which goes to the file writer...
			if( fp ) fwrite( _compressorOutput[handle].data.get(), _compressorOutput[handle].size, 1, fp );
			else LOG(WARNING) << "Trying to write but fp doesn't exist!";
		} else {
			// Push into the background queue
			writeData( _compressorOutput[handle].data.get(), _compressorOutput[handle].size );
		}
	}



	// LOG(INFO) << "Wrote frame " << numFrames;
	++numFrames;
	return true;
}


void LogWriter::bgCompressPng( unsigned int handle, std::shared_ptr<Chunk> chunk )
{
	// Do some non-compression
	{
		std::lock_guard< std::mutex > lock( _compressorMutex[handle] );

		uLongf destSz = _compressorOutput[handle].capacity;
		CHECK( compress2( (Bytef *)_compressorOutput[handle].data.get(), &destSz, (Bytef *)chunk->data.get(), chunk->size, CompressLevel )  == Z_OK );
		_compressorOutput[handle].size = destSz;
		_compressorDone[handle] = true;
	}
}

void LogWriter::bgCompressDepth( unsigned int handle, std::shared_ptr<Chunk> chunk )
{
	// Do some non-compression
	{
		std::lock_guard< std::mutex > lock( _compressorMutex[handle] );

		uLongf destSz = _compressorOutput[handle].capacity;
		CHECK( compress2( (Bytef *)_compressorOutput[handle].data.get(), &destSz, (Bytef *)chunk->data.get(), chunk->size, CompressLevel )  == Z_OK );
		_compressorOutput[handle].size = destSz;
		_compressorDone[handle] = true;
	}
}

void LogWriter::writeHeader( void )
{
	unsigned int bufferSize = 2 * sizeof(int32_t);
	for( const Field &field : _fields ) {
		bufferSize += 4 * sizeof( int32_t ) + field.name.size();
	}

	fwrite(&numFrames, sizeof(int32_t), 1, fp);
	int32_t val = _fields.size();
	fwrite(&val, sizeof(int32_t), 1, fp);

	for( const Field &field : _fields ) {
		int32_t h( field.size.height ), w( field.size.width ), type( field.type ), len( field.name.length() );

		fwrite(&h, sizeof(int32_t), 1, fp);
		fwrite(&w, sizeof(int32_t), 1, fp);
		fwrite(&type, sizeof(int32_t), 1, fp);
		fwrite(&len, sizeof(int32_t), 1, fp);
		fwrite((field.name.data()), sizeof(std::string::value_type), len, fp);

	}
}

}
