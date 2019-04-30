/*
 * PangolinOutputIOWrapper.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "PangolinOutputIOWrapper.h"

#include "util/SophusUtil.h"
#include "util/settings.h"
#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{

PangolinOutputIOWrapper::PangolinOutputIOWrapper( GUI & gui)
 : publishLvl(0),
   _gui(gui)
{

}

PangolinOutputIOWrapper::~PangolinOutputIOWrapper()
{

}

void PangolinOutputIOWrapper::publishPose( const Sophus::Sim3f &pose )
{
  _gui.pose.set( pose );
}

void PangolinOutputIOWrapper::updateDepthImage(unsigned char * data)
{
    _gui.updateDepthImage(data);
}

void PangolinOutputIOWrapper::publishKeyframe(const Frame::SharedPtr &f)
{
    LOG(DEBUG) << "Received keyFrame " << f->id() << " at address " << std::hex << f.get();
    // LOG(DEBUG) << "KeyFrame timestamp " << f->timestamp();
    // LOG(DEBUG) << "Frame::SharedPtr has " << f.use_count() << " references";

    Keyframe * fMsg = new Keyframe;

    boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();

    fMsg->id = f->id();
    fMsg->time = f->timestamp();
    fMsg->isKeyframe = true;

    int w = f->width(publishLvl);
    int h = f->height(publishLvl);

    fMsg->camToWorld = f->getCamToWorld().cast<float>();

    fMsg->fx = f->fx(publishLvl);
    fMsg->fy = f->fy(publishLvl);
    fMsg->cx = f->cx(publishLvl);
    fMsg->cy = f->cy(publishLvl);

    fMsg->width = w;
    fMsg->height = h;

    fMsg->pointData = new unsigned char[w * h * sizeof(InputPointDense)];

    InputPointDense * pc = (InputPointDense*)fMsg->pointData;

    // Handles a pathological case where the frame is a copy that does
    // not have depth information...
    if( f->hasIDepthBeenSet() ) {

      int validCount = 0;

      const float* idepth = f->idepth(publishLvl);
      const float* idepthVar = f->idepthVar(publishLvl);
      const float* color = f->image(publishLvl);

      for(int idx = 0;idx < w * h; idx++)
      {
        if( idepth[idx] > 0.0 ) validCount++;

          pc[idx].idepth = idepth[idx];
          pc[idx].idepth_var = idepthVar[idx];
          pc[idx].color[0] = color[idx];
          pc[idx].color[1] = color[idx];
          pc[idx].color[2] = color[idx];
          pc[idx].color[3] = color[idx];
      }

      //LOG(DEBUG) << "  Copied " << validCount << " valid depths";
    } else {
      LOG(WARNING) << "Frame " << f->id() << " does not appear to have depth information";
    }

    lock.unlock();

    _gui.addKeyframe(fMsg);
}

void PangolinOutputIOWrapper::publishTrackedFrame(const Frame::SharedPtr &kf)
{
  // TODO.  Get working again...
//    lsd_slam_viewer::keyframeMsg fMsg;
//
//
//    fMsg.id = kf->id();
//    fMsg.time = kf->timestamp();
//    fMsg.isKeyframe = false;
//
//
//    memcpy(fMsg.camToWorld.data(),kf->getScaledCamToWorld().cast<float>().data(),sizeof(float)*7);
//    fMsg.fx = kf->fx(publishLvl);
//    fMsg.fy = kf->fy(publishLvl);
//    fMsg.cx = kf->cx(publishLvl);
//    fMsg.cy = kf->cy(publishLvl);
//    fMsg.width = kf->width(publishLvl);
//    fMsg.height = kf->height(publishLvl);
//
//    fMsg.pointcloud.clear();
//
//    liveframe_publisher.publish(fMsg);
//
//
//    SE3 camToWorld = se3FromSim3(kf->getScaledCamToWorld());
//
//    geometry_msgs::PoseStamped pMsg;
//
//    pMsg.pose.position.x = camToWorld.translation()[0];
//    pMsg.pose.position.y = camToWorld.translation()[1];
//    pMsg.pose.position.z = camToWorld.translation()[2];
//    pMsg.pose.orientation.x = camToWorld.so3().unit_quaternion().x();
//    pMsg.pose.orientation.y = camToWorld.so3().unit_quaternion().y();
//    pMsg.pose.orientation.z = camToWorld.so3().unit_quaternion().z();
//    pMsg.pose.orientation.w = camToWorld.so3().unit_quaternion().w();
//
//    if (pMsg.pose.orientation.w < 0)
//    {
//        pMsg.pose.orientation.x *= -1;
//        pMsg.pose.orientation.y *= -1;
//        pMsg.pose.orientation.z *= -1;
//        pMsg.pose.orientation.w *= -1;
//    }
//
//    pMsg.header.stamp = ros::Time(kf->timestamp());
//    pMsg.header.frame_id = "world";
//    pose_publisher.publish(pMsg);
}

void PangolinOutputIOWrapper::publishKeyframeGraph(const std::shared_ptr<KeyFrameGraph> &graph)
{
    graph->keyframesAllMutex.lock_shared();

    int num = graph->keyframesAll.size();

    unsigned char * buffer = new unsigned char[num * sizeof(GraphFramePose)];

    GraphFramePose* framePoseData = (GraphFramePose*)buffer;

    for(unsigned int i = 0; i < graph->keyframesAll.size(); i++)
    {
        framePoseData[i].id = graph->keyframesAll[i]->id();
        memcpy(framePoseData[i].camToWorld, graph->keyframesAll[i]->frame()->getCamToWorld().cast<float>().data(), sizeof(float) * 7);
    }

    graph->keyframesAllMutex.unlock_shared();

    _gui.updateKeyframePoses(framePoseData, num);

    delete [] buffer;
}

void PangolinOutputIOWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
    //TODO
}

void PangolinOutputIOWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
    //TODO
}

void PangolinOutputIOWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
    //TODO
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
