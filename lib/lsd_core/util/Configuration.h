
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#include "SophusUtil.h"

#include "ImageSize.h"
#include "Camera.h"

#pragma once

namespace lsd_slam {


class Configuration {
public:

  Configuration() :
    doDepth( NO_STEREO ),
    stopOnFailedRead( true ),
    SLAMEnabled( true ),
    doKFReActivation( true ),
    doMapping( true )
  {;}

  ImageSize inputImage;
  SlamImageSize slamImage;
  Camera camera;

  enum { NO_STEREO = 0, STEREO_ZED } doDepth;

  bool stopOnFailedRead;
  bool SLAMEnabled;
  bool doKFReActivation;
  bool doMapping;


protected:

};

}
