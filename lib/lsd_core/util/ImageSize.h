
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#pragma once

namespace lsd_slam {

struct ImageSize {
  ImageSize( void ) : width(0), height(0) {;}

  ImageSize( int w, int h )
    : width(w), height(h)
  {;}

  ImageSize( const cv::Size &sz )
    : width(sz.width), height(sz.height)
  {;}


#ifdef USE_ZED
  ImageSize( const sl::zed::resolution &res )
    : width( res.width ), height( res.height )
  {;}
#endif

  long int area( void ) const { return width*height; }
  float aspectRatio( void ) const { return float(width)/float(height); }

  cv::Size cvSize( void ) const { return cv::Size( width, height); }
  cv::Size operator()( void ) const { return cv::Size( width, height); }

  int width, height;
};

// Specialization of ImageSize with additional input checking
struct SlamImageSize : public ImageSize {
  SlamImageSize( void ) : ImageSize() {;}

  SlamImageSize( int w, int h )
    : ImageSize( w, h )
  {
    CHECK(w%16 == 0 && h%16 == 0) << "SLAM image dimensions must be multiples of 16! Please crop your images / video accordingly.";
    CHECK(     w!=0 && h!=0 ) << "Height or width set to zero!";
  }

  SlamImageSize &operator=(const ImageSize &other) { width = other.width; height = other.height; return *this; }
};

}
