/*
 * LogReader.cpp
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
 */

#include "LogReader.h"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <g3log/g3log.hpp>

namespace logger {

LogReader::LogReader( void )
  : currentFrame( 0 ),
    numFrames( 0 )
{
    // CHECK(fs::exists(file.c_str()));
    //
    // fp = fopen(file.c_str(), "rb");
    //
    // currentFrame = 0;
    //
    // assert(fread(&numFrames, sizeof(int32_t), 1, fp));
    //
    // depthReadBuffer = new unsigned char[numPixels * 2];
    // imageReadBuffer = new unsigned char[numPixels * 3];
}

LogReader::~LogReader()
{
  close();

    // delete [] depthReadBuffer;
    // delete [] imageReadBuffer;
    //
    // if(deCompImage != 0)
    // {
    //     cvReleaseImage(&deCompImage);
    // }
    //
    // fclose(fp);
}

bool LogReader::open( const std::string &filename )
{
  CHECK(fs::exists(fs::path(filename)));

  fp = fopen(filename.c_str(), "rb");
  if( !fp ) {
    LOG(WARNING) << "Couldn't open log file " << filename;
    return false;
  }

  currentFrame = 0;

  fread(&numFrames, sizeof(int32_t), 1, fp);
	int32_t numFields = 0;
	fread(&numFields, sizeof(int32_t), 1, fp);

  _fields.clear();
	for( unsigned int i = 0; i < numFields; ++i ) {
		int32_t h, w, type, len;
    char buf[80];

		fread(&h, sizeof(int32_t), 1, fp);
		fread(&w, sizeof(int32_t), 1, fp);
		fread(&type, sizeof(int32_t), 1, fp);
		fread(&len, sizeof(int32_t), 1, fp);
    len = std::min(len,80);
		fread(buf, sizeof(std::string::value_type), len, fp);

    _fields.emplace_back( std::string( buf, len), cv::Size( w, h ), static_cast<FieldType_t>(type) );

    const Field &f( _fields.back() );
    _data.emplace_back( f.nBytes() );
	}

  // Read header
  return true;
}

bool LogReader::close( void )
{
  fclose( fp );
  fp = NULL;
}


void LogReader::grab()
{
  currentFrame++;

//     assert(fread(&timestamp, sizeof(int64_t), 1, fp));
//
//     assert(fread(&depthSize, sizeof(int32_t), 1, fp));
//     assert(fread(&imageSize, sizeof(int32_t), 1, fp));
//
//     assert(fread(depthReadBuffer, depthSize, 1, fp));
//
//     if(imageSize > 0)
//     {
//         assert(fread(imageReadBuffer, imageSize, 1, fp));
//     }
//
//     if(depthSize == numPixels * 2)
//     {
//         memcpy(&decompressionBuffer[0], depthReadBuffer, numPixels * 2);
//     }
//     else
//     {
//         unsigned long decompLength = numPixels * 2;
//         uncompress(&decompressionBuffer[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);
//     }
//
//     unsigned short * depthBuffer = (unsigned short *)&decompressionBuffer[0];
//     unsigned short maxVal = 0;
//     unsigned short minVal = std::numeric_limits<unsigned short>::max();
//
//     #pragma omp parallel for reduction(max : maxVal) reduction(min : minVal)
//     for(int i = 0; i < numPixels; i++)
//     {
//         if(depthBuffer[i] > maxVal)
//         {
//             maxVal = depthBuffer[i];
//         }
//
//         if(depthBuffer[i] < minVal && depthBuffer[i] != 0)
//         {
//             minVal = depthBuffer[i];
//         }
//     }
//
//     this->maxVal = maxVal;
//     this->minVal = minVal;
//
//     if(deCompImage != 0)
//     {
//         cvReleaseImage(&deCompImage);
//     }
//
//     CvMat tempMat = cvMat(1, imageSize, CV_8UC1, (void *)imageReadBuffer);
//
//     if(imageSize == numPixels * 3)
//     {
//         deCompImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
//
//         memcpy(deCompImage->imageData, imageReadBuffer, numPixels * 3);
//     }
//     else if(imageSize > 0)
//     {
//         deCompImage = cvDecodeImage(&tempMat);
//     }
//     else
//     {
//         deCompImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
//         memset(deCompImage->imageData, 0, numPixels * 3);
//     }
//
//     depth = (unsigned short *)decompressionBuffer;
//     rgb = (unsigned char *)deCompImage->imageData;
//
}

cv::Mat LogReader::retrieve( FieldHandle_t handle )
{
  CHECK( handle >= 0 && handle < _fields.size() );

  return cv::Mat();
}


// bool LogReader::hasMore()
// {
//     return currentFrame + 1 < numFrames;
// }
//
// const std::string LogReader::getFile()
// {
//     return file;
// }

 }
