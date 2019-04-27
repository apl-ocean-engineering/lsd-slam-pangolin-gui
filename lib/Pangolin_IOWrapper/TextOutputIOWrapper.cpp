/*
 * TextOutputIOWrapper.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "TextOutputIOWrapper.h"

#include "util/SophusUtil.h"
#include "util/settings.h"
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam {

using namespace std;

TextOutputIOWrapper::TextOutputIOWrapper()
 : _poseFile()
{
  _poseFile.open("pose.txt");
  CHECK( _poseFile.is_open() ) << "Unable to open posefile";
}

TextOutputIOWrapper::~TextOutputIOWrapper()
{

}

// void TextOutputIOWrapper::publishPose( const Sophus::Sim3f &pose )
// {
//   _gui.pose.set( pose );
// }

// void TextOutputIOWrapper::updateDepthImage(unsigned char * data)
// {
//     _gui.updateDepthImage(data);
// }

void TextOutputIOWrapper::publishKeyframe(const Frame::SharedPtr &f)
{
}

void TextOutputIOWrapper::publishTrackedFrame(const Frame::SharedPtr &kf)
{
  CHECK( bool(kf->trackingParent()) );
  CHECK( kf->pose != nullptr );
  CHECK( _poseFile.is_open() ) << "Posefile isn't open!";

  LOG(DEBUG) << "Writing pose for frame " << kf->id();

  // _poseFile << kf->id() << "," << kf->trackingParent()->id();
  //
  // {
  //   const auto pose = kf->getCamToWorld();
  //   const auto trans = pose.translation();
  //
  //   _poseFile << "," << trans.x() << "," << trans.y() << "," << trans.z();
  // }
  //
  // {
  //   const auto pose = kf->pose->thisToParent_raw;
  //   const auto trans = pose.translation();
  //
  //   _poseFile << "," << trans.x() << "," << trans.y() << "," << trans.z();
  // }
  //
  // _poseFile << endl;

}

}
