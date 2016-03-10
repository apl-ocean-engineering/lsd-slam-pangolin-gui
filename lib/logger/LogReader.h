/*
 * LogReader.h
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
 */

#pragma once

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

#include "LogFields.h"

namespace logger {

class LogReader
{
    public:
        LogReader(void);
        virtual ~LogReader();

        bool open( const std::string &file );
        bool close( void );

        void grab();

        cv::Mat retrieve( FieldHandle_t handle );

        int getNumFrames() const { return numFrames; }

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

        // Bytef *& decompressionBuffer;
        // IplImage *& deCompImage;
        // unsigned char * depthReadBuffer;
        // unsigned char * imageReadBuffer;
        // int32_t depthSize;
        // int32_t imageSize;

        // const std::string file;
        FILE * fp;
        int32_t numFrames;
        int currentFrame;

        Fields _fields;
        std::deque< Chunk > _data;
        std::deque< Chunk > _compressed;

        // int width;
        // int height;
        // int numPixels;
};

}
