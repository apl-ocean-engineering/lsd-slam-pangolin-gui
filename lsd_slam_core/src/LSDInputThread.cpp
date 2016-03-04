

#include "LSD.h"

using namespace lsd_slam;


void run(SlamSystem * system, DataSource *dataSource, Undistorter* undistorter )
{
    // get HZ
    float fps = dataSource->fps();
    long int dt_us = (fps > 0) ? (1e6/fps) : 0;

    lsdReady.notify();
    startAll.wait();

    int numFrames = dataSource->numFrames();
    LOG_IF( INFO, numFrames > 0 ) << "Running for " << numFrames << " frames";

    cv::Mat image = cv::Mat(system->conf().slamImage.cvSize(), CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; (numFrames < 0) || (i < numFrames); ++i)
    {
        if(lsdDone.getValue()) break;

        std::chrono::time_point<std::chrono::steady_clock> start(std::chrono::steady_clock::now());

        cv::Mat imageDist = cv::Mat( system->conf().inputImage.cvSize(), CV_8U);

        if( dataSource->grab() ) {

          dataSource->getImage( imageDist );
          undistorter->undistort(imageDist, image);

          CHECK(image.type() == CV_8U);

          if(runningIDX == 0)
          {
              system->randomInit(image.data, fakeTimeStamp, runningIDX);
          }
          else
          {
              system->trackFrame(image.data, runningIDX, fps == 0, fakeTimeStamp);
          }

          if( gui ){
            gui->pose.assignValue(system->getCurrentPoseEstimateScale());
            gui->updateFrameNumber( runningIDX );
            gui->updateLiveImage( image.data );
          }

          runningIDX++;
          fakeTimeStamp += (fps > 0) ? (1.0/fps) : 0.03;

          if(fullResetRequested)
          {
              SlamSystem *newSystem = new SlamSystem( system->conf(), system->SLAMEnabled );
              newSystem->set3DOutputWrapper( system->get3DOutputWrapper() );

              LOG(WARNING) << "FULL RESET!";
              delete system;

              system = newSystem;

              fullResetRequested = false;
              runningIDX = 0;
          }

        } else {
          if( system->conf().stopOnFailedRead ) break;
        }

        if( dt_us > 0 ) std::this_thread::sleep_until( start + std::chrono::microseconds( dt_us ) );
    }

    LOG(INFO) << "Have processed all input frames.";
    lsdDone.assignValue(true);
}
