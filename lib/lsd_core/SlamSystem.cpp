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

#include "SlamSystem.h"

#include <boost/thread/shared_lock_guard.hpp>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include <opencv2/opencv.hpp>

#include "DataStructures/Frame.h"
// #include "Tracking/SE3Tracker.h"
// #include "Tracking/Sim3Tracker.h"
// #include "DepthEstimation/DepthMap.h"
// #include "Tracking/TrackingReference.h"
#include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"
// #include "IOWrapper/ImageDisplay.h"
// #include "IOWrapper/Output3DWrapper.h"
// #include <g2o/core/robust_kernel_impl.h>

#include "DataStructures/FrameMemory.h"
// #include "deque"

// for mkdir
// #include <sys/types.h>
// #include <sys/stat.h>

#include "SlamSystem/MappingThread.h"
#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/OptimizationThread.h"
#include "SlamSystem/TrackingThread.h"

using namespace lsd_slam;



SlamSystem::SlamSystem( const Configuration &conf )
: finalized(false),
	perf(),
	_conf( conf ),
	keyFrameGraph( new KeyFrameGraph ),
	trackableKeyFrameSearch( new TrackableKeyFrameSearch( keyFrameGraph, conf ) )
{

	// Because some of these rely on conf(), need to explicitly call after
 	// static initialization
	optThread =        new OptimizationThread( *this, conf.SLAMEnabled );
	mapThread =        new MappingThread( *this );
	constraintThread = new ConstraintSearchThread( *this, conf.SLAMEnabled );
	trackingThread =   new TrackingThread( *this );

	// this->width = w;
	// this->height = h;
	// this->K = K;
	// trackingIsGood = true;

	// keyFrameGraph = new KeyFrameGraph();

	// createNewKeyFrame = false;

	// map =  new DepthMap( conf );

	// newConstraintAdded = false;
	//haveUnmergedOptimizationOffset = false;


	// tracker = new SE3Tracker( _conf.slamImage );
	// // Do not use more than 4 levels for odometry tracking
	// for (int level = 4; level < PYRAMID_LEVELS; ++level)
	// 	tracker->settings.maxItsPerLvl[level] = 0;

	//mappingTrackingReference = new TrackingReference();


	// if(SLAMEnabled)
	// {
	// 	trackableKeyFrameSearch = new TrackableKeyFrameSearch(keyFrameGraph,conf);
	// 	constraintTracker = new Sim3Tracker( _conf.slamImage );
	// 	constraintSE3Tracker = new SE3Tracker( _conf.slamImage );
	// 	newKFTrackingReference = new TrackingReference();
	// 	candidateTrackingReference = new TrackingReference();
	// }
	// else
	// {
	// 	constraintSE3Tracker = 0;
	// 	trackableKeyFrameSearch = 0;
	// 	constraintTracker = 0;
	// 	newKFTrackingReference = 0;
	// 	candidateTrackingReference = 0;
	// }


	// outputWrapper = 0;

	// keepRunning = true;
	// depthMapScreenshotFlag = false;
	// lastTrackingClosenessScore = 0;

	// thread_mapping = std::thread(&SlamSystem::mappingThreadLoop, this);

	timeLastUpdate.start();
}


SlamSystem::~SlamSystem()
{
	// keepRunning = false;

	// make sure no-one is waiting for something.
	LOG(INFO) << "... waiting for all threads to exit";
	// newFrameMapped.notify();
	// unmappedTrackedFrames.notifyAll();
	// // unmappedTrackedFramesSignal.notify_all();
	//
	// newKeyFrames.notifyAll();

	// newConstraintCreatedSignal.notify_all();

	delete mapThread;
	delete constraintThread;
	delete optThread;
	delete trackingThread;
	LOG(INFO) << "DONE waiting for all threads to exit";

	// delete keyframe graph
	delete keyFrameGraph;

	delete trackableKeyFrameSearch;

	FrameMemory::getInstance().releaseBuffes();

	// Util::closeAllWindows();
}

void SlamSystem::finalize()
{
	finalized = true;

	LOG(INFO) << "Finalizing Graph... adding final constraints!!";

	// This happens in the foreground
	constraintThread->doFullReConstraintTrack();
	constraintThread->fullReConstraintTrackComplete.wait();

	LOG(INFO) << "Finalizing Graph... optimizing!!";
	// This happens in the foreground
	// This will kick off a final map publication with the newly optimized offsets (also in foreground)
	optThread->doFinalOptimization();

	optThread->finalOptimizationComplete.wait();
	mapThread->optimizationUpdateMerged.wait();


	// doFinalOptimization = true;
	// newConstraintMutex.lock();
	// newConstraintAdded = true;
	// newConstraintCreatedSignal.notify_all();
	// newConstraintMutex.unlock();

	// while(doFinalOptimization)
	// {
	// 	usleep(200000);
	// }

	//printf("Finalizing Graph... publishing!!\n");
	//unmappedTrackedFrames.notifyAll();
	// unmappedTrackedFramesMutex.lock();
	// unmappedTrackedFramesSignal.notify_one();
	// unmappedTrackedFramesMutex.unlock();

	// while(doFinalOptimization)
	// {
	// 	usleep(200000);
	// }

	// newFrameMapped.wait();
	// newFrameMapped.wait();

	// usleep(200000);
	LOG(INFO) << "Done Finalizing Graph.!!";
}


// void SlamSystem::requestDepthMapScreenshot(const std::string& filename)
// {
// 	depthMapScreenshotFilename = filename;
// 	depthMapScreenshotFlag = true;
// }

void SlamSystem::gtDepthInit( std::shared_ptr<Frame> frame )
{
	// For a newly-imported frame, this will only be true if the depth
	// has been set explicitly
	CHECK( frame->hasIDepthBeenSet() );

	currentKeyFrame.set( frame );
	mapThread->gtDepthInit( frame );
	keyFrameGraph->addFrame( frame.get() );

	if( conf().SLAMEnabled) {
		boost::lock_guard<boost::shared_mutex> lock( keyFrameGraph->idToKeyFrameMutex );
		keyFrameGraph->idToKeyFrame.insert(std::make_pair( currentKeyFrame()->id(), currentKeyFrame() ));
	}

	if(continuousPCOutput) publishKeyframe( currentKeyFrame().get() );

}

void SlamSystem::randomInit(uchar* image, int id, double timeStamp)
{
	randomInit( std::shared_ptr<Frame>( new Frame(id, _conf, timeStamp, image) ) );
}

void SlamSystem::randomInit( std::shared_ptr<Frame> frame )
{
	LOG(INFO) << "Doing Random initialization!";

	if(! conf().doMapping)
		LOG(FATAL) << "WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.";

		currentKeyFrame.set( frame );

		mapThread->randomInit( frame );
		keyFrameGraph->addFrame( frame.get() );

		if( conf().SLAMEnabled) {
			boost::lock_guard<boost::shared_mutex> lock( keyFrameGraph->idToKeyFrameMutex );
			keyFrameGraph->idToKeyFrame.insert( std::make_pair(currentKeyFrame()->id(), currentKeyFrame() ) );
		}

	if(continuousPCOutput ) publishKeyframe( currentKeyFrame().get() );

	LOG(INFO) << "Done Random initialization!";
}

// Passthrough to TrackingThread
void SlamSystem::trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp )
{
	LOG(DEBUG) << "Tracking frame; " << ( blockUntilMapped ? "WILL" : "won't") << " block";
	trackingThread->trackFrame( std::shared_ptr<lsd_slam::Frame>(new Frame(frameID, _conf, timestamp, image)), blockUntilMapped );
}

void SlamSystem::trackFrame(std::shared_ptr<Frame> trackingNewFrame, bool blockUntilMapped )
{
	LOG(DEBUG) << "Tracking frame; " << ( blockUntilMapped ? "WILL" : "won't") << " block";
	trackingThread->trackFrame( trackingNewFrame, blockUntilMapped );


	//TODO: At present only happens at frame rate.  Push to a thread?
	addTimingSamples();
}


//=== Keyframe maintenance functions ====

void SlamSystem::changeKeyframe( std::shared_ptr<Frame> candidate, bool noCreate, bool force, float maxScore)
{
	Frame* newReferenceKF=0;

	if( conf().doKFReActivation && conf().SLAMEnabled )
	{
		Timer timer;
		newReferenceKF = trackableKeyFrameSearch->findRePositionCandidate( candidate.get(), maxScore );
		perf.findReferences.update( timer );
	}

	if(newReferenceKF != 0) {
		LOG(INFO) << "Reloading existing key frame " << newReferenceKF->id();
		loadNewCurrentKeyframe(newReferenceKF);
	} else {
		if(force)
		{
			if(noCreate)
			{
				LOG(INFO) << "mapping is disabled & moved outside of known map. Starting Relocalizer!";
				trackingThread->setTrackingIsBad();
				//nextRelocIdx = -1; /// What does this do?
			}
			else
			{
				createNewCurrentKeyframe( candidate );
			}
		}
	}
	// createNewKeyFrame = false;
}

void SlamSystem::loadNewCurrentKeyframe(Frame* keyframeToLoad)
{
	std::lock_guard< std::mutex > lock( currentKeyFrame.mutex() );

	// LOG_IF(DEBUG, enablePrintDebugInfo && printThreadingInfo ) << "RE-ACTIVATE KF " << keyframeToLoad->id();

	mapThread->map->setFromExistingKF(keyframeToLoad);

	LOG_IF(DEBUG, enablePrintDebugInfo && printRegularizeStatistics ) << "re-activate frame " << keyframeToLoad->id() << "!";

	currentKeyFrame.set( keyFrameGraph->idToKeyFrame.find(keyframeToLoad->id())->second );
	currentKeyFrame()->depthHasBeenUpdatedFlag = false;
}


void SlamSystem::createNewCurrentKeyframe( SharedFramePtr newKeyframeCandidate)
{
	LOG_IF(INFO, printThreadingInfo) << "CREATE NEW KF " << newKeyframeCandidate->id() << ", replacing " << currentKeyFrame()->id();

	if( conf().SLAMEnabled)
	{
		boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph->idToKeyFrameMutex );
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
	}

	// propagate & make new.
	mapThread->map->createKeyFrame(newKeyframeCandidate.get());
	currentKeyFrame.set( newKeyframeCandidate );								// Locking



	// if(outputWrapper && printPropagationStatistics)
	// {
	//
	// 	Eigen::Matrix<float, 20, 1> data;
	// 	data.setZero();
	// 	data[0] = runningStats.num_prop_attempts / ((float)_conf.slamImage.area());
	// 	data[1] = (runningStats.num_prop_created + runningStats.num_prop_merged) / (float)runningStats.num_prop_attempts;
	// 	data[2] = runningStats.num_prop_removed_colorDiff / (float)runningStats.num_prop_attempts;
	//
	// 	outputWrapper->publishDebugInfo(data);
	// }

	// currentKeyFrameMutex.lock();
	// currentKeyFrameMutex.unlock();
}




//===== Debugging output functions =====


void SlamSystem::addTimingSamples()
{
	mapThread->map->addTimingSample();

	float sPassed = timeLastUpdate.reset();
	if(sPassed > 1.0f)
	{

		LOGF_IF(INFO, enablePrintDebugInfo && printOverallTiming, "MapIt: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
					mapThread->map->_perf.update.ms(), mapThread->map->_perf.update.rate(),
					trackingThread->perf.ms(), trackingThread->perf.rate(),
					mapThread->map->_perf.create.ms()+mapThread->map->_perf.finalize.ms(), mapThread->map->_perf.create.rate(),
					perf.findReferences.ms(), perf.findReferences.rate(),
					0.0, 0.0,
					//trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
					optThread->perf.ms(), optThread->perf.rate(),
					perf.findConstraint.ms(), perf.findConstraint.rate() );
	}

}

void SlamSystem::updateDisplayDepthMap()
{
	if( !displayDepthMap ) return;  //&& !depthMapScreenshotFlag)

	mapThread->map->debugPlotDepthMap();
	double scale = 1;
	if( currentKeyFrame().get() != nullptr )
		scale = currentKeyFrame()->getScaledCamToWorld().scale();

	// debug plot depthmap
	char buf1[200];
	char buf2[200];

	snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d",
			mapThread->map->_perf.update.ms(), mapThread->map->_perf.update.rate(),
			trackingThread->perf.ms(), trackingThread->perf.rate(),
			currentKeyFrame()->numFramesTrackedOnThis, currentKeyFrame()->numMappedOnThis ); //, (int)unmappedTrackedFrames().size());

	// snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
	// 		100*currentKeyFrame->numPoints/(float)(conf().slamImage.area()),
	// 		100*tracking_lastGoodPerBad,
	// 		scale,
	// 		tracking_lastResidual,
	// 		100*tracking_lastUsage,
	// 		(int)keyFrameGraph->allFramePoses.size(),
	// 		keyFrameGraph->totalVertices,
	// 		(int)keyFrameGraph->edgesAll.size(),
	// 		1e-6 * (float)keyFrameGraph->totalPoints);


	if(onSceenInfoDisplay)
		printMessageOnCVImage(mapThread->map->debugImageDepth, buf1, buf2);

	CHECK( mapThread->map->debugImageDepth.data != NULL );
	publishDepthImage( mapThread->map->debugImageDepth.data );
}



SE3 SlamSystem::getCurrentPoseEstimate()
{
	boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph->allFramePosesMutex );
	if( keyFrameGraph->allFramePoses.size() > 0)
		return se3FromSim3(keyFrameGraph->allFramePoses.back()->getCamToWorld());

	return Sophus::SE3();
}

Sophus::Sim3f SlamSystem::getCurrentPoseEstimateScale()
{
	boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph->allFramePosesMutex );
	if(keyFrameGraph->allFramePoses.size() > 0)
		return keyFrameGraph->allFramePoses.back()->getCamToWorld().cast<float>();

	return Sophus::Sim3f();
}

std::vector<FramePoseStruct*> SlamSystem::getAllPoses()
{
	return keyFrameGraph->allFramePoses;
}
