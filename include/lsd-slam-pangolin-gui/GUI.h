/*
 * GUI.h
 *
 *  Created on: 15 Aug 2014
 *      Author: thomas
 */

#pragma once

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <map>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/pangolin.h>

#include "libvideoio/types/Camera.h"
#include "libvideoio/types/ImageSize.h"

#include "DataStructures/Frame.h"
#include "Keyframe.h"
#include "util/ThreadMutexObject.h"

#include "IOWrapper/OutputIOWrapper.h"

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI : public lsd_slam::OutputIOWrapper  {
public:
  GUI(const libvideoio::ImageSize &sz, const libvideoio::Camera &camera);

  virtual ~GUI();

  void initImages();

  // The master roll-up of all of the updating
  void update(void);

  void preCall();
  void drawKeyframes();
  void drawImages();
  void postCall();


  void setRescaleFactor(float rescaleFactor);

  ThreadMutexObject<Sophus::Sim3f> pose;


  //===== OutputIOWrapper Interface functions =====

  virtual void publishPose(const Sophus::Sim3f &pose)
  {;}

  virtual void publishKeyframeGraph(const std::shared_ptr<lsd_slam::KeyFrameGraph> &graph)
  {;}

  // Publish pointcloud from graph
  virtual void publishPointCloud(const lsd_slam::Frame::SharedPtr &kf)
  {;}

  // publishes a keyframe. if that frame already existis, it is overwritten,
  // otherwise it is added.
  virtual void publishKeyframe(const lsd_slam::Frame::SharedPtr &kf);

  virtual void publishFrame(const lsd_slam::Frame::SharedPtr &kf,
                            const Eigen::MatrixXf G)
   {;}

  virtual void publishFrame(const lsd_slam::Frame::SharedPtr &kf, const Eigen::MatrixXf G,
                            const lsd_slam::DepthMap::SharedPtr &depthMap)
   {;}

  // published a tracked frame that did not become a keyframe (i.e. has no
  // depth data)
  virtual void publishTrackedFrame(const lsd_slam::Frame::SharedPtr &kf)
  {;}

  // publishes graph and all constraints, as well as updated KF poses.
  virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory,
                                          std::string identifier)
  {;}

  virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt,
                                          std::string identifier)
  {;}

  virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
  {;}

  // virtual void
  // keyFrameCallback(const lsd_slam_msgs::keyframeMsg::ConstPtr &kfMsg) {}

  virtual void updateFrameNumber(int);

  virtual void updateDepthImage(unsigned char *data);
  virtual void updateLiveImage(const cv::Mat &img);

  // Functions only in pangolin OW
  // virtual void publishKeyframe(const Frame::SharedPtr &kf, const PointCloud &pc,
  //                              const std::vector<float> idepthvar,
  //                              const Sophus::Sim3f &pose);

  virtual void updateKeyframePoses(GraphFramePose *framePoseData,
                                   const int num);


  virtual void updateGui() {;}

  virtual void dumpPoints(std::string time_now);


protected:

  void saveStateCallback();
  void resetPointCloudCallback();

private:
  libvideoio::ImageSize _imageSize;
  libvideoio::Camera _camera;

  void drawGrid();

  pangolin::GlTexture *liveImg;
  pangolin::GlTexture *depthImg;

  ThreadMutexObject<unsigned char *> liveImgBuffer;
  ThreadMutexObject<unsigned char *> depthImgBuffer;

  pangolin::Var<int> *frameNumber;

  pangolin::Var<std::string> *totalPoints;

  pangolin::Var< std::function<void(void)> > *_saveState;
  pangolin::Var< std::function<void(void)> > *_resetPointCloud;


  pangolin::OpenGlRenderState s_cam;

  ThreadMutexObject< std::map<int, std::shared_ptr<Keyframe> > > keyframes;

  float _reScaleFactor;
};
