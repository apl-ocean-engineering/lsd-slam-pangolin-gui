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
#include <zlib.h>
#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "active.h"

namespace logger {

class LogWriter
{
    public:

      static const int CompressLevel = 6;

      typedef unsigned int FieldHandle_t;

        LogWriter();
        virtual ~LogWriter();

        enum FieldType_t {
          FIELD_BGRA_8C = 0,
          FIELD_DEPTH_32F = 1
        };

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

        struct Field {
          Field( const std::string &nm, const cv::Size &sz, FieldType_t tp )
            : name(nm), size(sz), type(tp)
          {;}

          std::string name;
          cv::Size size;
          FieldType_t type;

          unsigned int nBytes( void ) const {
            if( type == FIELD_BGRA_8C ) {
              unsigned long bytes = size.height * size.width * 4 * sizeof( unsigned char );
              return compressBound( bytes );
            } else if( type == FIELD_DEPTH_32F ) {
              unsigned long bytes = size.height * size.width * 1 * sizeof( float );
              return compressBound( bytes );
            } else {
              return 0;
            }
          }

        };
        typedef std::vector<Field> Fields;
        Fields _fields;

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
            size = _sz;
          }

          std::unique_ptr<char[]> data;
          unsigned int size, capacity;
        };

        std::unique_ptr<logger::Active> _writer;
        std::deque< std::unique_ptr<logger::Active> > _compressors;
        std::deque< Chunk > _compressorOutput;
        std::deque< bool > _compressorDone, _fieldUpdated;
        std::deque< std::mutex > _compressorMutex;


        // One backgroundable task
        void bgWriteData( std::shared_ptr<Chunk> chunk )
        {
          if( fp ) fwrite( chunk->data.get(), chunk->size, 1, fp );
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

        // Backgroundable PNG compression
        void bgCompressPng( unsigned int handle, std::shared_ptr<Chunk> chunk );

        void compressPng( FieldHandle_t handle, const void *data )
        {
          // This will do an extra copy (data -> Chunk)
          // is this avoidable?
          std::shared_ptr<Chunk> ptrChunk( new Chunk( data, _fields[handle].nBytes() ) );
          _compressors[handle]->send( std::bind( &LogWriter::bgCompressPng, this, handle, ptrChunk ) );
        }

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
