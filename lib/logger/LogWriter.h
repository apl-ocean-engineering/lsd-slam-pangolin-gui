/*
 * Was once based roughly on Thomas Whelan's RawLogReader.h
 */

#pragma once

#include <iostream>
#include <string>
#include <deque>
#include <memory>

#include <stdio.h>

#include <g3log/g3log.hpp>
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "active_object/active.h"
#include "LogFields.h"

namespace logger {

class LogWriter
{
    public:

      static const int SnappyCompressLevel = 100;
#ifdef USE_SNAPPY
      static const int DefaultCompressLevel = SnappyCompressLevel;
#else
      static const int DefaultCompressLevel = Z_DEFAULT_COMPRESSION;
#endif


        LogWriter( int compressLevel = DefaultCompressLevel );
        virtual ~LogWriter();

        FieldHandle_t registerField( const std::string &name, const cv::Size &sz, FieldType_t type );

        bool open( const std::string &filename );
        bool close( void );

        int newFrame( void );
        void addField( FieldHandle_t handle, const void *data );
        void addField( FieldHandle_t handle, const cv::Mat &mat );

        bool writeFrame( bool doBlock = false );

        // void getNext();
        //
        // int getNumFrames();
        //
        // bool hasMore();
        //
        // const std::string getFile();
        //
        // unsigned short minVal, maxVal;
        // int64_t timestamp;
        //
        // unsigned short * depth;
        // unsigned char * rgb;

    private:

        Fields _fields;

        std::unique_ptr<active_object::Active> _writer;
        std::deque< std::unique_ptr<active_object::Active> > _compressors;
        std::deque< std::shared_ptr<Chunk> > _compressorOutput;
        std::deque< bool > _compressorDone, _fieldUpdated;
        std::deque< std::mutex > _compressorMutex;


        // One backgroundable task
        void writeData( std::shared_ptr<Chunk> chunk )
        {
          if( fp ) {
            uint32_t sz = chunk->size;
            fwrite( &sz, sizeof(uint32_t), 1, fp );
            fwrite( chunk->data.get(), sizeof(unsigned char), chunk->size, fp );
          }
          else LOG(WARNING) << "Trying to write but fp doesn't exist!";
        }

        void bgWriteData( std::shared_ptr<Chunk> chunk )
        {
          std::shared_ptr<Chunk> ptrChunk( new Chunk( chunk->data.get(), chunk->size ) );
          _writer->send( std::bind( &LogWriter::writeData, this, ptrChunk ) );
        }

        void bgWriteData( char *data, unsigned int sz )
        {
          // This copies data in to the chunk
          std::shared_ptr<Chunk> ptrChunk( new Chunk( data, sz ) );
          _writer->send( std::bind( &LogWriter::writeData, this, ptrChunk ) );
        }

        // void writeData( std::unique_ptr<char[]> &data, unsigned int sz )
        // {
        //   std::shared_ptr<Chunk> ptrChunk( new Chunk( data, sz ) );
        //   _writer->send( std::bind( &LogWriter::bgWriteData, this, ptrChunk ) );
        // }

        // Backgroundable Image compression
        void compressPng( unsigned int handle, std::shared_ptr<Chunk> chunk );

        void bgCompressPng( FieldHandle_t handle, const void *data )
        {
          // This will do an extra copy (data -> Chunk)
          // is this avoidable?
          std::shared_ptr<Chunk> ptrChunk( new Chunk( data, _fields[handle].nBytes() ) );
          _compressors[handle]->send( std::bind( &LogWriter::compressPng, this, handle, ptrChunk ) );
        }

        // Backgroundable depth compression -- currently just use compressPng
        // void compressDepth( unsigned int handle, std::shared_ptr<Chunk> chunk );
        //
        // void bgCompressDepth( FieldHandle_t handle, const void *data )
        // {
        //   // This will do an extra copy (data -> Chunk)
        //   // is this avoidable?
        //   std::shared_ptr<Chunk> ptrChunk( new Chunk( data, _fields[handle].nBytes() ) );
        //   _compressors[handle]->send( std::bind( &LogWriter::compressDepth, this, handle, ptrChunk ) );
        // }


        void writeHeader( void );

        bool allCompressionDone( void );


        // Bytef *& decompressionBuffer;
        // IplImage *& deCompImage;
        // unsigned char * depthReadBuffer;
        // unsigned char * imageReadBuffer;
        // int32_t depthSize;
        // int32_t imageSize;
        //
        // const std::string file;


        FILE * fp;
        int32_t numFrames;
        int _compressionLevel;


        // int currentFrame;
        // int width;
        // int height;
        // int numPixels;
};

}
