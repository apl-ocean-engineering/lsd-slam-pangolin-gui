
#include "lsd-slam-pangolin-gui/GuiKeyframe.h"


using namespace lsd_slam;

Keyframe::Keyframe()
    : hasVbo(false), needsUpdate(false),
      pointSize(0.5), scale(10), scaledTh(1e-3), absTh(1e-1), nearSupport(9),
      sparisityFactor(1), _hidden(false),
      _publishLvl(0), _pointSize(0.5)
{;}

Keyframe::~Keyframe() {
  if (hasVbo) glDeleteBuffers(1, &vbo);
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

  _points.reserve( width*height );
  _points.clear();

  // Handles a pathological case where the frame is a copy that does
  // not have depth information...
  if (kf->hasIDepthBeenSet()) {

    const float *idepth = kf->idepth(_publishLvl);
    const float *idepthVar = kf->idepthVar(_publishLvl);
    const float *color = kf->image(_publishLvl);

    int idx = 0;
    for (int y = 0; y < height; ++y ) {
      for( int x = 0; x < width; ++x, ++idx  ) {
        // If iDepth is not defined
        if( idepth[idx] <= 0 ) continue;

        // If all we got is a frame, need to project the points ourselves.
        const float depth = 1.0/idepth[idx];

        PointCloudPoint pt;
        pt.xImg = x;
        pt.yImg = y;

        pt.x = (x * fxi + cxi) * depth;
        pt.y = (y * fyi + cyi) * depth;
        pt.z = depth;

        pt.idepth_var = idepthVar[idx];
        pt.color[0] = color[idx];
        pt.color[1] = color[idx];
        pt.color[2] = color[idx];

        _points.push_back( pt );
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

  //assert(!(hasVbo && !needsUpdate));

  if(hasVbo && needsUpdate) {
    glDeleteBuffers(1, &vbo);
    _glPointCount = 0;
  }

  LOG(INFO) << "Creating glBuffer of size " << _points.size();
  GLVertexColorStruct *glBuffer = new GLVertexColorStruct[_points.size()];

  const float my_scale = lsd_slam::Conf().scale; // camToWorld.scale();
  const float my_scale2 = my_scale * my_scale;
  const float my_scale4 = my_scale2 * my_scale2;

  const float my_scaledTH = lsd_slam::Conf().scaledTh;
  const float my_absTH = lsd_slam::Conf().absTh;
  const int my_minNearSupport = lsd_slam::Conf().nearSupport;
  const int my_sparsifyFactor = lsd_slam::Conf().sparisityFactor;

  const float fxi = 1 / fx;
  const float fyi = 1 / fy;
  const float cxi = -cx / fx;
  const float cyi = -cy / fy;

  int runningSparistyFailureCount = 0;
  int runningabsTHFailureCount = 0;
  int runningscaledTHFailureCount = 0;
  int runningNearSupportFailureCount = 0;

  // PointCloud::Ptr cloud(new PointCloud);
  // PointCloud::Ptr cloud_filtered(new PointCloud);

  int glPoints = 0;

  for (int idx = 2; idx < _points.size() - 2; idx++) {

    if( glPoints >= _points.size() ) break;

    bool fail = false;


    if (lsd_slam::Conf().useVarianceFiltering) {

      const float _x = _points[idx].xImg;
      const float _y = _points[idx].yImg;

      const float depth = _points[idx].z;
      const float depth2 = depth * depth;
      const float depth4 = depth2 * depth2;

      if (depth <= 0)
        continue;

      if (my_sparsifyFactor > 1 && rand() % my_sparsifyFactor != 0) {
        fail = true;
        runningSparistyFailureCount++;
      }

      if (_points[idx].idepth_var * depth4 > my_scaledTH) {
        fail = true;
        runningscaledTHFailureCount++;
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
            if (_points[_idx].z > 0) {
              float diff = 1 / _points[_idx].z - 1.0f / depth;
              if (diff * diff <
                  2 * my_scale * _points[idx].idepth_var) {
                // if (diff * diff > 1e-15) {
                nearSupport++;
              }
            }

          }
        }

        if (nearSupport < my_minNearSupport) {
          fail = true;
          runningNearSupportFailureCount++;
        }
      }
    }

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

        glBuffer[glPoints].point[0] = _points[idx].x;
        glBuffer[glPoints].point[1] = _points[idx].y;
        glBuffer[glPoints].point[2] = _points[idx].z;
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
  glBufferData(GL_ARRAY_BUFFER, sizeof(GLVertexColorStruct) * glPoints, glBuffer, GL_STATIC_DRAW);
  _glPointCount = glPoints;
  LOG(INFO) << "Final VBO buffer had " << _glPointCount << " (" << glPoints << ") points";

  delete[] glBuffer;

  // delete[] pointData;
  // pointData = 0;

  hasVbo = true;
  needsUpdate = false;
}

void Keyframe::drawPoints() {

  if( _hidden ) return;

  if (!hasVbo || needsUpdate) computeVbo();

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
  glVertexPointer(GLVertexColorStruct::NumPoints, GL_FLOAT, sizeof(GLVertexColorStruct), 0);
  glColorPointer(GLVertexColorStruct::NumColors, GL_UNSIGNED_BYTE, sizeof(GLVertexColorStruct),
                 (const void *)(GLVertexColorStruct::NumPoints * sizeof(float)));

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
  glVertex3f(size * (width - 1 - cx) / fx, size * (height - 1 - cy) / fy, size);
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
