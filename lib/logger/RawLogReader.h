/*
 * RawLogReader.h
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
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace logger {

class RawLogReader
{
    public:
        RawLogReader(const cv::Size &sz,
                     Bytef *& decompressionBuffer,
                     IplImage *& deCompImage,
                     std::string file);

        virtual ~RawLogReader();

        void getNext();

        int getNumFrames();

        bool hasMore();

        const std::string getFile();

        unsigned short minVal, maxVal;
        int64_t timestamp;

        unsigned short * depth;
        unsigned char * rgb;

    private:

        Bytef *& decompressionBuffer;
        IplImage *& deCompImage;
        unsigned char * depthReadBuffer;
        unsigned char * imageReadBuffer;
        int32_t depthSize;
        int32_t imageSize;

        const std::string file;
        FILE * fp;
        int32_t numFrames;
        int currentFrame;
        int width;
        int height;
        int numPixels;
};

}
