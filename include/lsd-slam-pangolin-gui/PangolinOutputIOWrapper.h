/*
 * PangolinOutputIOWrapper.h
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#pragma once

#include "lsd-slam-pangolin-gui/GUI.h"
#include "IOWrapper/OutputIOWrapper.h"
#include "util/Configuration.h"

// #include "lsd_slam_msgs/keyframeGraphMsg.h"
// #include "lsd_slam_msgs/keyframeMsg.h"


namespace lsd_slam {

class Frame;
class KeyFrameGraph;

struct GraphConstraint {
  int from;
  int to;
  float err;
};

class PangolinOutputIOWrapper : public OutputIOWrapper {
public:
  PangolinOutputIOWrapper(GUI &gui);
  PangolinOutputIOWrapper(GUI &gui, float &pointSize);
  virtual ~PangolinOutputIOWrapper();

  virtual void publishPose(const Sophus::Sim3f &pose);

  virtual void publishKeyframeGraph(const std::shared_ptr<KeyFrameGraph> &graph);

  // Publish pointcloud from graph
  //virtual void publishPointCloud(const Frame::SharedPtr &kf) {}

  // publishes a keyframe. if that frame already existis, it is overwritten,
  // otherwise it is added.
  virtual void publishKeyframe(const Frame::SharedPtr &kf);

  virtual void publishFrame(const Frame::SharedPtr &kf,
                            const Eigen::MatrixXf G);

  virtual void publishFrame(const Frame::SharedPtr &kf, const Eigen::MatrixXf G,
                            const DepthMap::SharedPtr &depthMap) {}

  virtual void updateDepthImage(unsigned char *data);

  // published a tracked frame that did not become a keyframe (i.e. has no
  // depth data)
  virtual void publishTrackedFrame(const Frame::SharedPtr &kf);

  // publishes graph and all constraints, as well as updated KF poses.
  virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory,
                                          std::string identifier);

  virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt,
                                          std::string identifier);

  virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);

  // virtual void
  // keyFrameCallback(const lsd_slam_msgs::keyframeMsg::ConstPtr &kfMsg) {}

  virtual void updateFrameNumber(int);

  virtual void updateLiveImage(const cv::Mat &img);

  // Functions only in pangolin OW
  // virtual void publishKeyframe(const Frame::SharedPtr &kf, const PointCloud &pc,
  //                              const std::vector<float> idepthvar,
  //                              const Sophus::Sim3f &pose);

  virtual void updateLiveImage(unsigned char *data);

  virtual void updateKeyframePoses(GraphFramePose *framePoseData,
                                   const int num);


  virtual void updateGui();

  virtual void dumpPoints(std::string time_now);

  int publishLvl;

private:
  GUI &_gui;
  float _pointSize;
  float _scale;
};

} // namespace lsd_slam
