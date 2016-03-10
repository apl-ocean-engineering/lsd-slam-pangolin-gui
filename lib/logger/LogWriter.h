/*
 * RawLogReader.h
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
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

#include "active.h"
#include "LogFields.h"

namespace logger {

class LogWriter
{
    public:

      static const int CompressLevel = 6;

        LogWriter();
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

        std::unique_ptr<logger::Active> _writer;
        std::deque< std::unique_ptr<logger::Active> > _compressors;
        std::deque< Chunk > _compressorOutput;
        std::deque< bool > _compressorDone, _fieldUpdated;
        std::deque< std::mutex > _compressorMutex;


        // One backgroundable task
        void bgWriteData( std::shared_ptr<Chunk> chunk )
        {
          if( fp ) {
            fwrite( &(chunk->size), sizeof( uint32_t), 1, fp );
            fwrite( chunk->data.get(), chunk->size, 1, fp );
          }
          else LOG(WARNING) << "Trying to write but fp doesn't exist!";
        }

        void writeData( std::shared_ptr<Chunk> ptrChunk )
        {
          _writer->send( std::bind( &LogWriter::bgWriteData, this, ptrChunk ) );
        }

        void writeData( char *data, unsigned int sz )
        {
          // This copies data in to the chunk
          std::shared_ptr<Chunk> ptrChunk( new Chunk( data, sz ) );
          _writer->send( std::bind( &LogWriter::bgWriteData, this, ptrChunk ) );
        }

        // void writeData( std::unique_ptr<char[]> &data, unsigned int sz )
        // {
        //   std::shared_ptr<Chunk> ptrChunk( new Chunk( data, sz ) );
        //   _writer->send( std::bind( &LogWriter::bgWriteData, this, ptrChunk ) );
        // }

        // Backgroundable Image compression
        void bgCompressPng( unsigned int handle, std::shared_ptr<Chunk> chunk );

        void compressPng( FieldHandle_t handle, const void *data )
        {
          // This will do an extra copy (data -> Chunk)
          // is this avoidable?
          std::shared_ptr<Chunk> ptrChunk( new Chunk( data, _fields[handle].nBytes() ) );
          _compressors[handle]->send( std::bind( &LogWriter::bgCompressPng, this, handle, ptrChunk ) );
        }

        // Backgroundable depth compression
        void bgCompressDepth( unsigned int handle, std::shared_ptr<Chunk> chunk );

        void compressDepth( FieldHandle_t handle, const void *data )
        {
          // This will do an extra copy (data -> Chunk)
          // is this avoidable?
          std::shared_ptr<Chunk> ptrChunk( new Chunk( data, _fields[handle].nBytes() ) );
          _compressors[handle]->send( std::bind( &LogWriter::bgCompressDepth, this, handle, ptrChunk ) );
        }


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

        // int currentFrame;
        // int width;
        // int height;
        // int numPixels;
};

}
