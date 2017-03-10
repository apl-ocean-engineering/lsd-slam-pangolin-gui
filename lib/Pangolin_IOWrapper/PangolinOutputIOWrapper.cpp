/*
 * PangolinOutputIOWrapper.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "PangolinOutputIOWrapper.h"

// #include "util/SophusUtil.h"
// #include "util/settings.h"
// #include "DataStructures/Frame.h"
// #include "GlobalMapping/KeyFrameGraph.h"
// #include "sophus/sim3.hpp"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{

PangolinOutputIOWrapper::PangolinOutputIOWrapper( const Configuration &conf, GUI & gui)
 : _conf( conf ),
   _gui(gui)
{

}

PangolinOutputIOWrapper::~PangolinOutputIOWrapper()
{

}

void PangolinOutputIOWrapper::updateFrameNumber( int runningIdx )
{
  _gui.updateFrameNumber( runningIdx );
}

void PangolinOutputIOWrapper::updateLiveImage( const cv::Mat &img )
{
  _gui.updateLiveImage( img.data );
}

}
