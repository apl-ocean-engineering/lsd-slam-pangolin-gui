/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "IOWrapper/Pangolin/PangolinOutput3DWrapper.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

#include "util/Undistorter.h"
#include "util/RawLogReader.h"

#include "util/FileUtils.h"

#include "opencv2/opencv.hpp"

#include "GUI.h"

std::vector<std::string> files;
int w, h, w_inp, h_inp;
ThreadMutexObject<bool> lsdDone(false);
GUI gui( 640.0f/480.0f );
RawLogReader * logReader = 0;
int numFrames = 0;

using namespace lsd_slam;

void run(SlamSystem * system, Undistorter* undistorter, Output3DWrapper* outputWrapper, Sophus::Matrix3f K)
{
    // get HZ
    double hz = 30;

    cv::Mat image = cv::Mat(h, w, CV_8U);
    int runningIDX=0;
    float fakeTimeStamp = 0;

    for(unsigned int i = 0; i < numFrames; i++)
    {
        if(lsdDone.getValue())
            break;

        cv::Mat imageDist = cv::Mat(h, w, CV_8U);

        if(logReader)
        {
            logReader->getNext();

            cv::Mat3b img(h, w, (cv::Vec3b *)logReader->rgb);

            cv::cvtColor(img, imageDist, CV_RGB2GRAY);
        }
        else
        {
            imageDist = cv::imread(files[i], CV_LOAD_IMAGE_GRAYSCALE);

            if(imageDist.rows != h_inp || imageDist.cols != w_inp)
            {
                if(imageDist.rows * imageDist.cols == 0)
                    printf("failed to load image %s! skipping.\n", files[i].c_str());
                else
                    printf("image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
                            files[i].c_str(),
                            w,h,imageDist.cols, imageDist.rows);
                continue;
            }
        }

        CHECK(imageDist.type() == CV_8U);

        undistorter->undistort(imageDist, image);

        CHECK(image.type() == CV_8U);

        if(runningIDX == 0)
        {
            system->randomInit(image.data, fakeTimeStamp, runningIDX);
        }
        else
        {
            system->trackFrame(image.data, runningIDX, hz == 0, fakeTimeStamp);
        }

        gui.pose.assignValue(system->getCurrentPoseEstimateScale());
        gui.updateFrameNumber( runningIDX );
        gui.updateLiveImage( image.data );

        runningIDX++;
        fakeTimeStamp+=0.03;

        if(fullResetRequested)
        {
            printf("FULL RESET!\n");
            delete system;

            system = new SlamSystem(w, h, K, doSlam);
            system->set3DOutputWrapper(outputWrapper);

            fullResetRequested = false;
            runningIDX = 0;
        }
    }

    lsdDone.assignValue(true);
}

int main( int argc, char** argv )
{
  auto worker = g3::LogWorker::createLogWorker();
  auto handle = worker->addDefaultLogger(argv[0], ".");
  g3::initializeLogging(worker.get());
  std::future<std::string> log_file_name = handle->call(&g3::FileSink::fileName);
  std::cout << "*\n*   Log file: [" << log_file_name.get() << "]\n\n" << std::endl;

  LOG(INFO) << "Starting log.";

	// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;

	if(Parse::arg(argc, argv, "-c", calibFile) > 0)
	{
		 undistorter = Undistorter::getUndistorterForFile(calibFile.c_str());
	}

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using -c FILE)\n");
		exit(0);
	}

	w = undistorter->getOutputWidth();
	h = undistorter->getOutputHeight();

	w_inp = undistorter->getInputWidth();
	h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	Resolution::getInstance(w, h);
	Intrinsics::getInstance(fx, fy, cx, cy);

	gui.initImages();

	Output3DWrapper* outputWrapper = new PangolinOutput3DWrapper(w, h, gui);

	// make slam system
	SlamSystem * system = new SlamSystem(w, h, K, doSlam);
	system->set3DOutputWrapper(outputWrapper);


	// open image files: first try to open as file.
	std::string source;
	if(!(Parse::arg(argc, argv, "-f", source) > 0))
	{
		printf("need source files! (set using -f FOLDER or KLG)\n");
		exit(0);
	}

	Bytef * decompressionBuffer = new Bytef[Resolution::getInstance().numPixels() * 2];
    IplImage * deCompImage = 0;

    if(source.substr(source.find_last_of(".") + 1) == "klg")
    {
        logReader = new RawLogReader(decompressionBuffer,
                                     deCompImage,
                                     source);

        numFrames = logReader->getNumFrames();
    }
    else
    {
        if(getdir(source, files) >= 0)
        {
            printf("found %d image files in folder %s!\n", (int)files.size(), source.c_str());
        }
        else if(getFile(source, files) >= 0)
        {
            printf("found %d image files in file %s!\n", (int)files.size(), source.c_str());
        }
        else
        {
            printf("could not load file list! wrong path / file?\n");
        }

        numFrames = (int)files.size();
    }

	boost::thread lsdThread(run, system, undistorter, outputWrapper, K);

	while(!pangolin::ShouldQuit())
	{
    if(lsdDone.getValue() && !system->finalized)
    {
        system->finalize();
    }

    gui.preCall();

    gui.drawKeyframes();

    gui.drawFrustum();

    gui.drawImages();

    gui.postCall();
	}

	lsdDone.assignValue(true);

	lsdThread.join();

  delete system;
  delete undistorter;
  delete outputWrapper;
  return 0;
}
