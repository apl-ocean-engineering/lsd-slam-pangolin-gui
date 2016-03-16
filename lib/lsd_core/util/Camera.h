
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#include "SophusUtil.h"

#pragma once

namespace lsd_slam {

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

#ifdef USE_ZED
  Camera( sl::zed::StereoParameters *params )
    : fx( params->LeftCam.fx ),
	    fy( params->LeftCam.fy ),
	    cx( params->LeftCam.cx ),
	    cy( params->LeftCam.cy )
  {
    buildK();
  }
#endif

  Camera scale( float scale ) const
  {
    return Camera( fx*scale, fy*scale,
                    (cx+0.5) * scale - 0.5,
                    (cy+0.5) * scale - 0.5);
  }

  Camera scale( float xscale, float yscale ) const
  {
    return Camera( fx*xscale, fy*yscale,
                    (cx+0.5) * xscale - 0.5,
                    (cy+0.5) * yscale - 0.5);
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

}
