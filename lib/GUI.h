/*
 * GUI.h
 *
 *  Created on: 15 Aug 2014
 *      Author: thomas
 */

#pragma once

#define GLM_FORCE_RADIANS

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <map>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "libvideoio/types/ImageSize.h"
#include "libvideoio/types/Camera.h"

#include "Pangolin_IOWrapper/Keyframe.h"
#include "util/ThreadMutexObject.h"
#include "DataStructures/Frame.h"

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049

class GUI
{
    public:
        GUI( const libvideoio::ImageSize &sz, const libvideoio::Camera &camera );

        virtual ~GUI();

        void initImages();

        void preCall();

        void drawFrustum();

        void postCall();

        void addKeyframe(Keyframe * newFrame);

        void updateLiveImage(unsigned char * data);
        void updateDepthImage(unsigned char * data);

        void updateKeyframePoses(GraphFramePose* framePoseData, int num);

        void drawKeyframes();

        void drawImages();

        void updateFrameNumber( int frameNumber );

        // The master roll-up of all of the updating
        void update( void );

        ThreadMutexObject<Sophus::Sim3f> pose;

    private:

      libvideoio::ImageSize _imageSize;
      libvideoio::Camera    _camera;

      void drawGrid();

      pangolin::GlTexture *liveImg;
      pangolin::GlTexture *depthImg;

      ThreadMutexObject<unsigned char * > liveImgBuffer;
      ThreadMutexObject<unsigned char * > depthImgBuffer;

      pangolin::Var<int> * gpuMem;
      pangolin::Var<int> * frameNumber;

      pangolin::Var<std::string> * totalPoints;

      pangolin::OpenGlRenderState s_cam;

      ThreadMutexObject<std::map<int, Keyframe *> > keyframes;
};
