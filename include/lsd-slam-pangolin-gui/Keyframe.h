/*
 * Keyframe.h
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#pragma once

#include "sophus/sim3.hpp"
#include "util/settings.h"
#include <GL/glew.h>
#include <iostream>

#include "DataStructures/Frame.h"
#include "util/Configuration.h"

#ifdef USE_ROS
  #include <pcl/filters/voxel_grid.h>
  // #include <pcl/io/pcd_io.h>
  #include <pcl/point_types.h>

  #include <ros/ros.h>

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
#endif

struct GraphFramePose {
  unsigned int id;
  float camToWorld[7];
};

class Keyframe {
public:

  struct PointCloudPoint {
    float xImg, yImg;           // xImg, yImg are pixel point locations.
    float x, y, z, depth;       // x,y are real world locations
    float idepth_var;
    unsigned char color[3];     // R,G,B
  };

  struct GLVertexColorStruct {
    float point[3];
    unsigned char color[4];
  };

  Keyframe( const Keyframe & ) = delete;

  Keyframe();

  virtual ~Keyframe();

  //-- hidden() accessors --
  void hide()           { _hidden = true; }
  void unhide()         { _hidden = false; }
  bool isHidden() const { return _hidden; }

  const std::vector< PointCloudPoint > &points() const { return _points; }
  int numPoints() const { return _points.size(); }

  void update(const lsd_slam::Frame::SharedPtr &kf);

  void computeVbo();

  void drawPoints();
  void drawCamera();
  void reset();

  void setPointSize(float size) { pointSize = size; }

  int id;
  int initId;
  uint64_t time;
  bool isKeyframe;

  Sophus::Sim3f camToWorld;

  float fx;
  float fy;
  float cx;
  float cy;
  int height;
  int width;

  std::vector< PointCloudPoint > _points;

  int _glPointCount;

  bool hasVbo;
  GLuint vbo;

  bool needsUpdate;

  float pointSize;

  float scale;
  float scaledTh;
  float absTh;
  int nearSupport;
  int sparisityFactor;


  bool _hidden;

  int   _publishLvl;
  float _pointSize;

};
