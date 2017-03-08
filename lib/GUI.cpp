/*
 * GUI.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "GUI.h"

#include <pangolin/display/display.h>

GUI::GUI( const lsd_slam::Configuration &conf )
 : _conf( conf ),
    liveImg(NULL),
   depthImg(NULL),
   liveImgBuffer(NULL),
   depthImgBuffer(NULL)
{
    const int initialWidth = 800, initialHeight = 800;

    pangolin::CreateWindowAndBind("Main", initialWidth, initialHeight ); //, GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);

    glDisable(GL_MULTISAMPLE);

    glEnable(GL_DEPTH_TEST);

    s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                        pangolin::ModelViewLookAt(-1, -5, -1, 0, 0, 0, pangolin::AxisNegY));

    pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    LOG(INFO) << "AR: " << _conf.slamImage.aspectRatio();
    pangolin::Display("LiveImage").SetAspect( _conf.slamImage.aspectRatio() );
    pangolin::Display("DepthImage").SetAspect( _conf.slamImage.aspectRatio() );

    pangolin::Display("multi").SetBounds(pangolin::Attach::Pix(0), 1 / 4.0f, pangolin::Attach::Pix(180), 1.0)
                              .SetLayout(pangolin::LayoutEqualHorizontal)
                              .AddDisplay(pangolin::Display("LiveImage"))
                              .AddDisplay(pangolin::Display("DepthImage"));

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

    gpuMem = new pangolin::Var<int>("ui.GPU memory free", 0);
    frameNumber = new pangolin::Var<int>("ui.Frame number", 0);

    totalPoints = new pangolin::Var<std::string>("ui.Total points", "0");

    initImages();
}

GUI::~GUI()
{
    if(depthImg)
        delete depthImg;

    if(depthImgBuffer.getValue())
        delete [] depthImgBuffer.getValue();

    if(liveImg)
        delete liveImg;

    if(liveImgBuffer.getValue())
        delete [] liveImgBuffer.getValue();

    {
      std::lock_guard<std::mutex> lock(keyframes.mutex());

      for(std::map<int, Keyframe *>::iterator i = keyframes.getReference().begin(); i != keyframes.getReference().end(); ++i)
      {
          delete i->second;
      }

      keyframes.getReference().clear();
    }

    delete totalPoints;
    delete gpuMem;
    delete frameNumber;
}

void GUI::initImages()
{
    depthImg = new pangolin::GlTexture(_conf.slamImage.width, _conf.slamImage.height, GL_RGB, true, 0, GL_RGB, GL_UNSIGNED_BYTE);
    depthImgBuffer.assignValue(new unsigned char[_conf.slamImage.area() * 3]);

    liveImg = new pangolin::GlTexture(_conf.slamImage.width, _conf.slamImage.height, GL_LUMINANCE, true, 0, GL_RGB, GL_UNSIGNED_BYTE);
    liveImgBuffer.assignValue(new unsigned char[_conf.slamImage.area()]);
}

void GUI::update( void )
{
  preCall();
  drawKeyframes();
  drawFrustum();
  drawImages();
  postCall();
}


void GUI::updateDepthImage(unsigned char * data)
{
  std::lock_guard<std::mutex> lock(depthImgBuffer.mutex());
  memcpy(depthImgBuffer.getReference(), data, _conf.slamImage.area() * 3);
}

// Expects CV_8UC1 data
void GUI::updateLiveImage(unsigned char * data)
{
  std::lock_guard<std::mutex> lock(liveImgBuffer.mutex());
  memcpy(liveImgBuffer.getReference(), data, _conf.slamImage.area() );
}

void GUI::updateFrameNumber( int fn )
{
  frameNumber->operator=(fn);
}

void GUI::addKeyframe(Keyframe * newFrame)
{
  std::lock_guard<std::mutex> lock(keyframes.mutex());

  //Exists
  if(keyframes.getReference().find(newFrame->id) != keyframes.getReference().end())
  {
      keyframes.getReference()[newFrame->id]->updatePoints(newFrame);

      delete newFrame;
  }
  else
  {
      newFrame->initId = keyframes.getReference().size();
      keyframes.getReference()[newFrame->id] = newFrame;
  }
}

void GUI::updateKeyframePoses(GraphFramePose* framePoseData, int num)
{
  std::lock_guard<std::mutex> lock(keyframes.mutex());

  for(int i = 0; i < num; i++)
  {
      if(keyframes.getReference().find(framePoseData[i].id) != keyframes.getReference().end())
      {
          memcpy(keyframes.getReference()[framePoseData[i].id]->camToWorld.data(), &framePoseData[i].camToWorld[0], sizeof(float) * 7);
      }
  }
}

//== Actual draw/render functions ==

void GUI::preCall()
{
    glClearColor(0.05, 0.05, 0.3, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    pangolin::Display("cam").Activate(s_cam);

    drawGrid();
}

void GUI::drawImages()
{
    {
      std::lock_guard<std::mutex> lock(depthImgBuffer.mutex());
      depthImg->Upload(depthImgBuffer.getReference(), GL_RGB, GL_UNSIGNED_BYTE);
    }

    pangolin::Display("DepthImage").Activate();
    depthImg->RenderToViewport(true);

    {
      std::lock_guard<std::mutex> lock(liveImgBuffer.mutex());
      liveImg->Upload(liveImgBuffer.getReference(), GL_LUMINANCE, GL_UNSIGNED_BYTE);
    }

    pangolin::Display("LiveImage").Activate();
    liveImg->RenderToViewport(true);
}

void GUI::drawKeyframes()
{
   std::lock_guard<std::mutex> lock(keyframes.mutex());

    glEnable(GL_MULTISAMPLE);
    glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

    for(std::map<int, Keyframe *>::iterator i = keyframes.getReference().begin(); i != keyframes.getReference().end(); ++i)
    {
        // Don't render first five, according to original code
        if(i->second->initId >= 5)
        {
            if(!i->second->hasVbo || i->second->needsUpdate)
            {
                i->second->computeVbo();
            }
            i->second->drawPoints();
            i->second->drawCamera();
        }
    }

    glDisable(GL_MULTISAMPLE);
}

void GUI::drawFrustum()
{
  lsd_slam::Camera c( _conf.camera );
  lsd_slam::ImageSize img( _conf.slamImage );

  glPushMatrix();
  Sophus::Matrix4f m = pose.getValue().matrix();
  glMultMatrixf((GLfloat*) m.data());
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(0.05 * (0 - c.cx) / c.fx, 0.05 * (0 - c.cy) / c.fy, 0.05);
      glVertex3f(0, 0, 0);
      glVertex3f(0.05 * (0 - c.cx) / c.fx, 0.05 * (img.height - 1 - c.cy) / c.fy, 0.05);
      glVertex3f(0, 0, 0);
      glVertex3f(0.05 * (img.width - 1 - c.cx) / c.fx, 0.05 * (img.height - 1 - c.cy) / c.fy, 0.05);
      glVertex3f(0, 0, 0);
      glVertex3f(0.05 * (img.width - 1 - c.cx) / c.fx, 0.05 * (0 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (img.width - 1 - c.cx) / c.fx, 0.05 * (0 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (img.width - 1 - c.cx) / c.fx, 0.05 * (img.height - 1 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (img.width - 1 - c.cx) / c.fx, 0.05 * (img.height - 1 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (0 - c.cx) / c.fx, 0.05 * (img.height - 1 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (0 - c.cx) / c.fx, 0.05 * (img.height - 1 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (0 - c.cx) / c.fx, 0.05 * (0 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (0 - c.cx) / c.fx, 0.05 * (0 - c.cy) / c.fy, 0.05);
      glVertex3f(0.05 * (img.width - 1 - c.cx) / c.fx, 0.05 * (0 - c.cy) / c.fy, 0.05);
  glEnd();
  glPopMatrix();
  glColor3f(1, 1, 1);
}

void GUI::drawGrid()
{
    //set pose
    glPushMatrix();

    Eigen::Matrix4f m;
    m <<  0,  0, 1, 0,
         -1,  0, 0, 0,
          0, -1, 0, 0,
          0,  0, 0, 1;
    glMultTransposeMatrixf((float*)m.data());

    glLineWidth(1);

    glBegin(GL_LINES);

    // Draw a larger grid around the outside..
    double dGridInterval = 0.1;

    double dMin = -100.0 * dGridInterval;
    double dMax = 100.0 * dGridInterval;

    double height = -4;

    for(int x = -10; x <= 10; x += 1)
    {
        if(x == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.3, 0.3, 0.3);
        glVertex3d((double) x * 10 * dGridInterval, dMin, height);
        glVertex3d((double) x * 10 * dGridInterval, dMax, height);
    }

    for(int y = -10; y <= 10; y += 1)
    {
        if(y == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.3, 0.3, 0.3);
        glVertex3d(dMin, (double) y * 10 * dGridInterval, height);
        glVertex3d(dMax, (double) y * 10 * dGridInterval, height);
    }

    glEnd();

    glBegin(GL_LINES);
    dMin = -10.0 * dGridInterval;
    dMax = 10.0 * dGridInterval;

    for(int x = -10; x <= 10; x++)
    {
        if(x == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.5, 0.5, 0.5);

        glVertex3d((double) x * dGridInterval, dMin, height);
        glVertex3d((double) x * dGridInterval, dMax, height);
    }

    for(int y = -10; y <= 10; y++)
    {
        if(y == 0)
            glColor3f(1, 1, 1);
        else
            glColor3f(0.5, 0.5, 0.5);
        glVertex3d(dMin, (double) y * dGridInterval, height);
        glVertex3d(dMax, (double) y * dGridInterval, height);
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

void GUI::postCall()
{
    GLint cur_avail_mem_kb = 0;
    glGetIntegerv(GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX, &cur_avail_mem_kb);

    int memFree = cur_avail_mem_kb / 1024;

    gpuMem->operator=(memFree);

    pangolin::FinishFrame();

    glFinish();
}
