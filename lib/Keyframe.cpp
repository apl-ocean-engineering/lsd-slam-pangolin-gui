
#include "lsd-slam-pangolin-gui/Keyframe.h"


using namespace lsd_slam;

Keyframe::Keyframe()
    : hasVbo(false), needsUpdate(false),
      pointSize(0.5), scale(10), scaledTh(1e-3), absTh(1e-1), nearSupport(9),
      sparisityFactor(1), _hidden(false),
      _publishLvl(0), _pointSize(0.5)
{;}

Keyframe::~Keyframe() {
  if (hasVbo)
    glDeleteBuffers(1, &vbo);
}

void Keyframe::update(const Frame::SharedPtr &kf) {

  LOG(DEBUG) << "Received keyFrame " << kf->id() << " at " << std::hex << kf.get();
  // LOG(DEBUG) << "KeyFrame timestamp " << f->timestamp();
  // LOG(DEBUG) << "Frame::SharedPtr has " << f.use_count() << " references";

  boost::shared_lock<boost::shared_mutex> lock = kf->getActiveLock();

  id = kf->id();
  time = kf->timestamp();
  isKeyframe = true;

  width = kf->width(_publishLvl);
  height = kf->height(_publishLvl);

  camToWorld = kf->getCamToWorld().cast<float>();

  fx = kf->fx(_publishLvl);
  fy = kf->fy(_publishLvl);
  cx = kf->cx(_publishLvl);
  cy = kf->cy(_publishLvl);

  const float fxi = 1.0/fx;
  const float fyi = 1.0/fy;
  const float cxi = -cx * fxi;
  const float cyi = -cy * fyi;


  _points.resize( width*height );

  // Handles a pathological case where the frame is a copy that does
  // not have depth information...
  if (kf->hasIDepthBeenSet()) {

    const float *idepth = kf->idepth(_publishLvl);
    const float *idepthVar = kf->idepthVar(_publishLvl);
    const float *color = kf->image(_publishLvl);

    int idx = 0;
    for (int y = 0; y < height; ++y ) {
    for( int x = 0; x < width; ++x, ++idx  ) {

      // In this version, need to project points...
      const float depth = 1.0/idepth[idx];
      _points[idx].depth = depth;

      _points[idx].xImg = x;
      _points[idx].yImg = y;

      _points[idx].x = (x * fxi + cxi) * depth;
      _points[idx].y = (y * fyi + cyi) * depth;
      _points[idx].z = _points[idx].depth;

      _points[idx].idepth_var = idepthVar[idx];
      _points[idx].color[0] = color[idx];
      _points[idx].color[1] = color[idx];
      _points[idx].color[2] = color[idx];
      //pc[idx].color[3] = color[idx];
    }
  }
  } else {
    LOG(WARNING) << "Frame " << kf->id()
                 << " does not appear to have depth information";
  }

  lock.unlock();

  needsUpdate = true;
}


void Keyframe::computeVbo() {
  assert(!(hasVbo && !needsUpdate));

  if(hasVbo && needsUpdate) {
    glDeleteBuffers(1, &vbo);
    _glPointCount = 0;
  }

  GLVertexColorStruct *glBuffer = new GLVertexColorStruct[_points.size()];

  float my_scale = lsd_slam::Conf().scale; // camToWorld.scale();
  float my_scale4 = my_scale * my_scale;
  my_scale4 *= my_scale4;
  float my_scaledTH = lsd_slam::Conf().scaledTh;
  float my_absTH = lsd_slam::Conf().absTh;
  int my_minNearSupport = lsd_slam::Conf().nearSupport;
  int my_sparsifyFactor = lsd_slam::Conf().sparisityFactor;

  float fxi = 1 / fx;
  float fyi = 1 / fy;
  float cxi = -cx / fx;
  float cyi = -cy / fy;

  int runningSparistyFailureCount = 0;
  int runningabsTHFailureCount = 0;
  int runningscaledTHFailureCount = 0;
  int runningNearSupportFailureCount = 0;

  // PointCloud::Ptr cloud(new PointCloud);
  // PointCloud::Ptr cloud_filtered(new PointCloud);
  float x;
  float y;
  float depth;

  int glPoints = 0;

  for (int idx = 2; idx < _points.size() - 2; idx++) {
    bool fail = false;
    x = _points[idx].x;
    y = _points[idx].y;
    depth = _points[idx].depth;

    if (lsd_slam::Conf().useVarianceFiltering) {
      float depth4 = depth * depth;

      float _x = _points[idx].xImg;
      float _y = _points[idx].xImg;

      depth4 *= depth4;

      if (depth <= 0)
        continue;

      if (my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0) {
        // std::cout << "failed my_sparsifyFactor: " << std::endl;
        // continue;
        fail = true;
        runningSparistyFailureCount++;
      }

      if (_points[idx].idepth_var * depth4 > my_scaledTH) {
        // std::cout << "failed my_scaledTH: "
        //           << originalInput[idx].idepth_var * depth4
        //           << "scale: " << scale << std::endl;
        // continue;
        fail = true;
        runningscaledTHFailureCount++;
      } else {
        // std::cout << originalInput[idx].idepth_var * depth4 << std::endl;
      }

      // if (originalInput[idx].idepth_var * depth4 > my_absTH) {
      //
      //   // continue;
      //   fail = true;
      //   runningabsTHFailureCount++;
      // }

      if (my_minNearSupport > 1) {
        int nearSupport = 0;
        for (int dx = -1; dx < 2; dx++) {
          for (int dy = -1; dy < 2; dy++) {
            int _idx = idx + dx + dy;
            if (_points[_idx].depth > 0) {
              float diff = 1 / _points[_idx].depth - 1.0f / depth;
              if (diff * diff <
                  2 * my_scale * _points[idx].idepth_var) {
                // if (diff * diff > 1e-15) {
                nearSupport++;
              } else {
                // std::cout << "diff^2 :" << diff * diff << "idepth_var: "
                //           << 2 * my_scale * originalInput[idx].idepth_var
                //           << std::endl;
                // std::cout << "failed nearSupport" << std::endl;
              }
            } else {
              // std::cout << "depth: " << originalInput[_idx].depth <<
              // std::endl;
            }
          }
        }

        if (nearSupport < my_minNearSupport) {
          // std::cout << "failed nearSupport: " << nearSupport << std::endl;
          // continue;
          fail = true;
          runningNearSupportFailureCount++;
        }
      }
    }
    // std::cout << fail << std::endl;

    if (!fail) {
      // if (lsd_slam::Conf().useVoxelFilter) {
      //   PointT point;
      //   point.x = x;
      //   point.y = y;
      //   point.z = depth;
      //   point.r = originalInput[idx].color[0];
      //   point.g = originalInput[idx].color[1];
      //   point.b = originalInput[idx].color[2];
      //   cloud->push_back(point);
      // } else {
        glBuffer[glPoints].point[0] = x;
        glBuffer[glPoints].point[1] = y;
        glBuffer[glPoints].point[2] = depth;
        glBuffer[glPoints].color[3] = 100;
        glBuffer[glPoints].color[2] = _points[idx].color[0];
        glBuffer[glPoints].color[1] = _points[idx].color[1];
        glBuffer[glPoints].color[0] = _points[idx].color[2];
      // }

      glPoints++;
    }
  }
  // if (lsd_slam::Conf().useVoxelFilter) {
  //   pcl::VoxelGrid<PointT> sor;
  //   sor.setInputCloud(cloud);
  //   sor.setLeafSize(lsd_slam::Conf().pclLeafSize,
  //                   lsd_slam::Conf().pclLeafSize,
  //                   lsd_slam::Conf().pclLeafSize);
  //   sor.filter(*cloud_filtered);
  //
  //   for (int p = 0; p < cloud_filtered->size(); p++) {
  //     PointT point = cloud_filtered->at(p);
  //     glBuffer[p].point[0] = point.x;
  //     glBuffer[p].point[1] = point.y;
  //     glBuffer[p].point[2] = point.z;
  //     glBuffer[p].color[3] = 100;
  //     glBuffer[p].color[2] = point.b;
  //     glBuffer[p].color[1] = point.g;
  //     glBuffer[p].color[0] = point.r;
  //   }
  // }

  // LOGF_IF(INFO, lsd_slam::Conf().printGUIinfo,
  //         "running sparisty failure count: %i, running absTH failure "
  //         "count: %i, running scaledTH failure count: %i, "
  //         "running near support failure count: %i",
  //         runningSparistyFailureCount, runningabsTHFailureCount,
  //         runningscaledTHFailureCount, runningNearSupportFailureCount);

  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertexColorStruct) * glPoints, glBuffer,
               GL_STATIC_DRAW);
  _glPointCount = glPoints;

  delete[] glBuffer;

  // delete[] pointData;
  // pointData = 0;

  hasVbo = true;
  needsUpdate = false;
}

void Keyframe::drawPoints() {

  if( _hidden ) return;

  assert(hasVbo);
  GLfloat mi(0.0);
  glPointParameterf(GL_POINT_SIZE_MIN, mi);
  glEnable(GL_PROGRAM_POINT_SIZE);
  GLfloat s(lsd_slam::Conf().pointcloudSize);
  glPointSize(s);
  // GLfloat *min;
  // glGetFloatv(GL_POINT_SIZE_MIN, min);
  // ROS_WARN("min: %f", *min);

  glPushMatrix();

  Sophus::Matrix4f m = camToWorld.matrix();
  glMultMatrixf((GLfloat *)m.data());

  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glVertexPointer(3, GL_FLOAT, sizeof(GLVertexColorStruct), 0);
  glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(GLVertexColorStruct),
                 (const void *)(3 * sizeof(float)));

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glDrawArrays(GL_POINTS, 0, _glPointCount);

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glPopMatrix();
}

void Keyframe::drawCamera() {

  if( _hidden ) return;

  glPushMatrix();
  float size = 0.2;
  Sophus::Matrix4f m = camToWorld.matrix();
  glMultMatrixf((GLfloat *)m.data());

  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(size * (0 - cx) / fx, size * (0 - cy) / fy, size);
  glVertex3f(0, 0, 0);
  glVertex3f(size * (0 - cx) / fx, size * (height - 1 - cy) / fy, size);
  glVertex3f(0, 0, 0);
  glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy,
             0.05);
  glVertex3f(0, 0, 0);
  glVertex3f(size * (width - 1 - cx) / fx, size * (0 - cy) / fy, size);
  glVertex3f(size * (width - 1 - cx) / fx, size * (0 - cy) / fy, size);
  glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy,
             size);
  glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy,
             size);
  glVertex3f(size * (0 - cx) / fx, size * (height - 1 - cy) / fy, size);
  glVertex3f(size * (0 - cx) / fx, size * (height - 1 - cy) / fy, size);
  glVertex3f(size * (0 - cx) / fx, size * (0 - cy) / fy, size);
  glVertex3f(size * (0 - cx) / fx, size * (0 - cy) / fy, size);
  glVertex3f(size * (width - 1 - cx) / fx, size * (0 - cy) / fy, size);
  glEnd();
  glPopMatrix();
  glColor3f(1, 1, 1);
}

void Keyframe::reset() {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
