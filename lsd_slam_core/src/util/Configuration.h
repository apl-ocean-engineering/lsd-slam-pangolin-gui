
#include <opencv2/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#include "SophusUtil.h"

#pragma once

namespace lsd_slam {

struct ImageSize {
  ImageSize( void ) : width(0), height(0) {;}

  ImageSize( int w, int h )
    : width(w), height(h)
  {;}

  long int area( void ) const { return width*height; }
  float aspectRatio( void ) const { return float(width)/float(height); }

  cv::Size cvSize( void ) const { return cv::Size( width, height); }

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
};

struct Camera {
  Camera( void )
    : fx(1), fy(1), cx(0), cy(0)
  {
    buildK();
  }

  Camera( const cv::Mat &kin )
    : fx( kin.at<double>(0, 0) ),
      fy( kin.at<double>(1, 1) ),
      cx( kin.at<double>(2, 0) ),
      cy( kin.at<double>(2, 1) )
  {
    buildK();
  }

  Camera( float fxin, float fyin, float cxin, float cyin )
    : fx( fxin ),
      fy( fyin ),
      cx( cxin ),
      cy( cyin )
  {
    buildK();
  }

  Camera scale( float scale )
  {
    return Camera( fx*scale, fy*scale,
                    (cx+0.5) * scale - 0.5,
                    (cy+0.5) * scale - 0.5);
}


  Sophus::Matrix3f K, Kinv;
  float fx, fy, cx, cy;
  float fxi, fyi, cxi, cyi;

private:

  void buildK( void )
  {
    K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    Kinv = K.inverse();

  	fxi = Kinv(0,0);
  	fyi = Kinv(1,1);
  	cxi = Kinv(0,2);
  	cyi = Kinv(1,2);
  }

};

class Configuration {
public:

  Configuration() {;}

  ImageSize inputImage;
  SlamImageSize slamImage;
  Camera camera;

protected:

};

}
