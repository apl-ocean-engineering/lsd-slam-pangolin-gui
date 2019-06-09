/*
 * GUI.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

 #include <pangolin/display/display.h>
#include <opencv2/imgproc.hpp>

#include "lsd-slam-pangolin-gui/GUI.h"
#include "lsd-slam-pangolin-gui/StateSaver.h"


using namespace libvideoio;
using namespace lsd_slam;

GUI::GUI(const ImageSize &sz, const Camera &camera)
    : OutputIOWrapper(),
      _imageSize(sz), _camera(camera), liveImg(NULL), depthImg(NULL),
      liveImgBuffer(NULL), depthImgBuffer(NULL), _reScaleFactor(0.5),
      _saveState( NULL )
{
  const int initialWidth = 800, initialHeight = 800;

  pangolin::CreateWindowAndBind(
      "Main", initialWidth, initialHeight); //, GLUT_DOUBLE | GLUT_RGBA |
                                            // GLUT_DEPTH | GLUT_MULTISAMPLE);

  glDisable(GL_MULTISAMPLE);
  glEnable(GL_PROGRAM_POINT_SIZE_EXT);

  glEnable(GL_DEPTH_TEST);

  pangolin::View& _depthImgDisplay = pangolin::Display("DepthImage").SetAspect(_imageSize.aspectRatio());
  pangolin::View& _liveImgDisplay  = pangolin::Display("LiveImage").SetAspect(_imageSize.aspectRatio());

  pangolin::Display("multi")
      .SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, pangolin::Attach::Pix(180), 1.0)
      .SetLayout(pangolin::LayoutEqualHorizontal)
      .AddDisplay(_liveImgDisplay)
      .AddDisplay(_depthImgDisplay);

  s_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
      pangolin::ModelViewLookAt(-1, 0, 0, 0, 0, 0, pangolin::AxisY));

  pangolin::Display("cam")
      .SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
      .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

  frameNumber = new pangolin::Var<int>("ui.Frame number", 0);

  totalPoints = new pangolin::Var<std::string>("ui.Total points", "0");

  _saveState = new pangolin::Var< std::function<void(void)> >( "ui.Save state",  std::bind(&GUI::saveStateCallback, this) );
  _resetPointCloud = new pangolin::Var< std::function<void(void)> >( "ui.(R)eset point cloud",  std::bind(&GUI::resetPointCloudCallback, this) );

  // Keyboard shortcuts
  pangolin::RegisterKeyPressCallback('r', std::bind(&GUI::resetPointCloudCallback, this) );


  initImages();
}

GUI::~GUI() {

  // Delete UI elements
  if (depthImg) delete depthImg;
  if (depthImgBuffer.getValue()) delete[] depthImgBuffer.getValue();

  if (liveImg) delete liveImg;
  if (liveImgBuffer.getValue()) delete[] liveImgBuffer.getValue();

  if( totalPoints)  delete totalPoints;
  if( frameNumber)  delete frameNumber;
  if( _saveState)   delete _saveState;

  // {
  //   std::lock_guard<std::mutex> lock(keyframes.mutex());
  //
  //   for( auto i : keyframes ) {
  //     delete i->second;
  //   }

    keyframes.getReference().clear();
  // }

}

void GUI::initImages() {
  liveImg = new pangolin::GlTexture(_imageSize.width, _imageSize.height,
                                    GL_RGB, true, 0, GL_RGB, GL_UNSIGNED_BYTE);
  liveImgBuffer.assignValue(new unsigned char[_imageSize.area() * 3]);

  depthImg = new pangolin::GlTexture(_imageSize.width, _imageSize.height,
                                     GL_RGB, true, 0, GL_RGB, GL_UNSIGNED_BYTE);
  depthImgBuffer.assignValue(new unsigned char[_imageSize.area() * 3]);
}

//===== Interfaces to OutputIOWrapper =====


void GUI::updateDepthImage(unsigned char *data) {
  std::lock_guard<std::mutex> lock(depthImgBuffer.mutex());
  memcpy(depthImgBuffer.getReference(), data, _imageSize.area() * 3);
}

void GUI::updateLiveImage(const cv::Mat &img) {
  CHECK( img.type() == CV_8UC3 );

  std::lock_guard<std::mutex> lock(liveImgBuffer.mutex());
  memcpy(liveImgBuffer.getReference(), img.data, _imageSize.area() * 3);
}

void GUI::updateFrameNumber(int fn) {
  frameNumber->operator=(fn);
}


void GUI::publishKeyframe(const Frame::SharedPtr &kf) {
  std::lock_guard<std::mutex> lock(keyframes.mutex());

  std::map<int, std::shared_ptr<Keyframe> > &kframes( keyframes.getReference() );

  if( kframes.find( kf->id() ) != kframes.end() ) {
    kframes.at(kf->id())->update(kf);
  } else {
    std::shared_ptr<Keyframe> newKF(new Keyframe);
    newKF->update(kf);

    kframes.insert( std::make_pair(kf->id(), newKF) );
  }
}


void GUI::updateKeyframePoses(GraphFramePose *framePoseData, int num) {
  std::lock_guard<std::mutex> lock(keyframes.mutex());
  LOG(DEBUG) << "updating key frame poses";
  for (int i = 0; i < num; i++) {

    if (keyframes.getReference().find(framePoseData[i].id) !=
        keyframes.getReference().end()) {
      memcpy(keyframes.getReference()[framePoseData[i].id]->camToWorld.data(),
             &framePoseData[i].camToWorld[0], sizeof(float) * 7);
      // std::cout << framePoseData[i].id << std::endl;
      // for (int j = 0; j < 7; j++) {
      //   std::cout << keyframes.getReference()[framePoseData[i].id]
      //                    ->camToWorld.data()[j]
      //             << std::endl;
      // }
    }
  }
}

//== Actual draw/render functions ==

void GUI::update(void) {
  preCall();
  drawImages();

  drawKeyframes();
  postCall();
}


void GUI::preCall() {
  // glClearColor(0.05, 0.05, 0.3, 0.0f);
  glClearColor(0.2, 0.2, 0.8, 0.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // pangolin::Display("cam").Activate(s_cam);
  // drawGrid();
}

void GUI::drawKeyframes() {

  pangolin::Display("cam").Activate(s_cam);

  std::lock_guard<std::mutex> lock(keyframes.mutex());

  glEnable(GL_MULTISAMPLE);
  glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

  for( auto &i : keyframes.getReference() ) {

    // Don't render first five, according to original code
    // if(i->second->initId >= 5)
    // {

    i.second->drawPoints();
    i.second->drawCamera();

    // }
  }

  glDisable(GL_MULTISAMPLE);
}

void GUI::drawImages() {

  {
    std::lock_guard<std::mutex> lock(liveImgBuffer.mutex());
    liveImg->Upload(liveImgBuffer.getReference(), GL_RGB, GL_UNSIGNED_BYTE);
  }
  pangolin::Display("LiveImage").Activate();
  liveImg->RenderToViewport(true);

  {
    std::lock_guard<std::mutex> lock(depthImgBuffer.mutex());
    depthImg->Upload(depthImgBuffer.getReference(), GL_RGB, GL_UNSIGNED_BYTE);
  }
  pangolin::Display("DepthImage").Activate();
  depthImg->RenderToViewport(true);
}

void GUI::drawGrid() {
  // set pose
  glPushMatrix();

  Eigen::Matrix4f m;
  m << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
  glMultTransposeMatrixf((float *)m.data());

  glLineWidth(1);

  glBegin(GL_LINES);

    // Draw a larger grid around the outside..
    double dGridInterval = 0.1;

    double dMin = -100.0 * dGridInterval;
    double dMax = 100.0 * dGridInterval;

    double height = -4;

    for (int x = -10; x <= 10; x += 1) {
      if (x == 0)
        glColor3f(1, 1, 1);
      else
        glColor3f(0.3, 0.3, 0.3);
      glVertex3d((double)x * 10 * dGridInterval, dMin, height);
      glVertex3d((double)x * 10 * dGridInterval, dMax, height);
    }

    for (int y = -10; y <= 10; y += 1) {
      if (y == 0)
        glColor3f(1, 1, 1);
      else
        glColor3f(0.3, 0.3, 0.3);
      glVertex3d(dMin, (double)y * 10 * dGridInterval, height);
      glVertex3d(dMax, (double)y * 10 * dGridInterval, height);
    }

  glEnd();

  glBegin(GL_LINES);
    dMin = -10.0 * dGridInterval;
    dMax = 10.0 * dGridInterval;

    for (int x = -10; x <= 10; x++) {
      if (x == 0)
        glColor3f(1, 1, 1);
      else
        glColor3f(0.5, 0.5, 0.5);

      glVertex3d((double)x * dGridInterval, dMin, height);
      glVertex3d((double)x * dGridInterval, dMax, height);
    }

    for (int y = -10; y <= 10; y++) {
      if (y == 0)
        glColor3f(1, 1, 1);
      else
        glColor3f(0.5, 0.5, 0.5);
      glVertex3d(dMin, (double)y * dGridInterval, height);
      glVertex3d(dMax, (double)y * dGridInterval, height);
    }

    glColor3f(1, 0, 0);
    glVertex3d(0, 0, height);
    glVertex3d(1, 0, height);
    glColor3f(0, 1, 0);
    glVertex3d(0, 0, height);
    glVertex3d(0, 1, height);
    glColor3f(1, 1, 1);
    glVertex3d(0, 0, height);
    glVertex3d(0, 0, height + 1);
  glEnd();

  glPopMatrix();
}

void GUI::postCall() {
  pangolin::FinishFrame();
  glFinish();
}

void GUI::setRescaleFactor(float rescaleFactor) {
  _reScaleFactor = rescaleFactor;
}

void GUI::dumpPoints(std::string time_now) {

  // Construct filename from current datetime
  std::string pcdFilename( "pointcloud" );
  pcdFilename += time_now;
  pcdFilename += ".pcd";

  {
    std::lock_guard<std::mutex> guard( keyframes.mutex() );
    PangolinGui::StateSaver::SaveState( pcdFilename, keyframes.getReference() );
  }

}

//=== Callbacks ===
void GUI::saveStateCallback() {
  LOG(INFO) << "In saveStateCallback";

  time_t rawtime;
  struct tm * timeinfo;

  time(&rawtime);
  timeinfo = gmtime(&rawtime);
  char dateStr[80];

  strftime( dateStr, 79, "%Y%m%d-%H%M%S", timeinfo);

  dumpPoints( dateStr );
}


void GUI::resetPointCloudCallback() {
  std::lock_guard<std::mutex> guard( keyframes.mutex() );
  for( const auto kf_pair : keyframes.getReference() ) {
      kf_pair.second->hide();
  }
}
