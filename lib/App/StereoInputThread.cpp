

#include "App/StereoInputThread.h"
#include "util/globalFuncs.h"

namespace lsd_slam {


  StereoInputThread::StereoInputThread(  std::shared_ptr<lsd_slam::SlamSystem> &sys,
                              std::shared_ptr<libvideoio::ImageSource> &src,
                              std::shared_ptr<libvideoio::Undistorter> &und,
                              std::shared_ptr<libvideoio::Undistorter> &rightUnd,
                               const Sophus::SE3d &rightToLeft )
    : InputThread( sys, src, und ),
      rightUndistorter( rightUnd ),
      _rightToLeft( rightToLeft )
    {}

    void StereoInputThread::operator()() {
      // get HZ

      LOG(INFO) << " !!! Starting StereoInputThread !!!!";

      float fps = dataSource->fps();

      if( Conf().runRealTime && fps == 0 ) {
        LOG(WARNING) << "Input FPS not provided, using 30fps.";
        fps = 30;
      }

      long int dt_us = (fps > 0) ? (1e6/fps) : 0;
      long int dt_fudge = 0;


      inputReady.notify();
      startAll.wait();

      int numFrames = dataSource->numFrames();
      LOG_IF( INFO, numFrames > 0 ) << "Running for " << numFrames << " frames at " << fps << " fps";

      cv::Mat stereoImage = cv::Mat();
      int runningIdx=0;
      float fakeTimeStamp = 0;

      for(unsigned int i = 0; (numFrames < 0) || (i < (unsigned int)numFrames); ++i)
      {
        if(inputDone.getValue()) break;

        std::chrono::time_point<std::chrono::steady_clock> start(std::chrono::steady_clock::now());

        if( dataSource->grab() ) {
          if( dataSource->getImage( stereoImage ) >= 0 ) {
            CHECK(stereoImage.type() == CV_8UC1);

            // Get ROIs
            cv::Size imgSize = stereoImage.size();
            const int halfWidth = imgSize.width/2;

            cv::Mat leftROI( stereoImage, cv::Rect(0,0, halfWidth, imgSize.height) );
            cv::Mat rightROI( stereoImage, cv::Rect(halfWidth,0, halfWidth, imgSize.height) );


            cv::Mat leftUndist;
            if( _doRotate ) {
              cv::Mat rotated;
              cv::rotate( leftROI, rotated, cv::ROTATE_180 );
              undistorter->undistort( rotated, leftUndist );
            } else {
              undistorter->undistort(leftROI, leftUndist);
            }
            CHECK(leftUndist.data != nullptr) << "Undistorted left image is nullptr";
            CHECK(leftUndist.type() == CV_8UC1);

            cv::Mat rightUndist;
            if( _doRotate ) {
              cv::Mat rotated;
              cv::rotate( rightROI, rotated, cv::ROTATE_180 );
              undistorter->undistort( rotated, rightUndist );
            } else {
              undistorter->undistort(rightROI, rightUndist);
            }
            CHECK(rightUndist.data != nullptr) << "Undistorted right image is nullptr";
            CHECK(rightUndist.type() == CV_8UC1);


            ImageSet::SharedPtr set( new ImageSet(runningIdx, leftUndist, undistorter->getCamera() ) );
            set->addFrame( rightUndist, rightUndistorter->getCamera(), _rightToLeft );

            // This will block if system->conf().runRealTime is false
            system->nextImageSet( set );

            runningIdx++;
            fakeTimeStamp += (fps > 0) ? (1.0/fps) : 0.03;

            if( output ) {
              output->updateFrameNumber( runningIdx );
              output->updateLiveImage( leftUndist );
            }

          }

          if(fullResetRequested)
          {
            LOG(WARNING) << "FULL RESET!";
            system.reset( system->fullReset() );

            fullResetRequested = false;
            runningIdx = 0;
          }

        } else {
          LOG(INFO) << "Bad read, still running...";
          if( Conf().stopOnFailedRead ) break;
        }

        if( Conf().runRealTime && dt_us > 0 ) std::this_thread::sleep_until( start + std::chrono::microseconds( dt_us + dt_fudge ) );
      }

      LOG(INFO) << "Have processed all input frames.";
      inputDone.assignValue(true);
    }

  }