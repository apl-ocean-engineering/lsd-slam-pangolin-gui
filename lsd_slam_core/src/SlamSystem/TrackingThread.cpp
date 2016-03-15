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

#include <boost/thread/shared_lock_guard.hpp>

#include "TrackingThread.h"

#include "SlamSystem.h"

// #include "DataStructures/Frame.h"
#include "Tracking/SE3Tracker.h"
// #include "Tracking/Sim3Tracker.h"
// #include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"
// #include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"
// #include "IOWrapper/ImageDisplay.h"
// #include "IOWrapper/Output3DWrapper.h"
// #include <g2o/core/robust_kernel_impl.h>
// #include "DataStructures/FrameMemory.h"
// #include "deque

#include "SlamSystem/MappingThread.h"

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"

using namespace lsd_slam;



TrackingThread::TrackingThread( SlamSystem &system )
: _system( system ),
	_currentKeyFrame( system.currentKeyFrame ),
	_tracker( new SE3Tracker( system.conf().slamImage ) ),
	_trackingReference( new TrackingReference() ),
	_trackingIsGood( true )
{


	// this->width = w;
	// this->height = h;
	// this->K = K;
	// trackingIsGood = true;

	// keyFrameGraph = new KeyFrameGraph();

	// createNewKeyFrame = false;

	// map =  new DepthMap( conf );

	// newConstraintAdded = false;
	//haveUnmergedOptimizationOffset = false;


	// Do not use more than 4 levels for odometry tracking
	for (int level = 4; level < PYRAMID_LEVELS; ++level)
		_tracker->settings.maxItsPerLvl[level] = 0;

	// trackingReference = new TrackingReference();
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
	lastTrackingClosenessScore = 0;

	timeLastUpdate.start();
}


TrackingThread::~TrackingThread()
{

	delete _trackingReference;
	delete _tracker;
}


void TrackingThread::trackFrame(std::shared_ptr<Frame> newFrame, bool blockUntilMapped )
{
	// Create new frame
	// std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, _conf, timestamp, image));

	LOG(INFO) << "In trackFrame";

	if(!_trackingIsGood)
	{
		// Prod mapping to check the relocalizer
		_system.mapThread->relocalizer.updateCurrentFrame(newFrame);
		_system.mapThread->doIteration();

//		unmappedTrackedFrames.notifyAll();

		// {
		// 	std::lock_guard< std::mutex > lock( unmappedTrackedFramesMutex );
		// 	unmappedTrackedFramesSignal.notify_one();
		// }
		return;
	}

	// bool my_createNewKeyframe = createNewKeyFrame;	// pre-save here, to make decision afterwards.
	{
		std::lock_guard< std::mutex > lock( _currentKeyFrame.mutex() );

		if(_trackingReference->frameID != _currentKeyFrame->id() || _currentKeyFrame->depthHasBeenUpdatedFlag)
		{
			LOG(DEBUG) << "Setting new tracking reference from frame " << _currentKeyFrame->id();
			_trackingReference->importFrame( _currentKeyFrame.get() );
			_currentKeyFrame->depthHasBeenUpdatedFlag = false;
			_trackingReferenceFrameSharedPT = _currentKeyFrame.ptr();
		}

	}

	FramePoseStruct &trackingReferencePose( *_trackingReference->keyframe->pose);


	// DO TRACKING & Show tracking result.
	LOG_IF(DEBUG, enablePrintDebugInfo && printThreadingInfo) << "TRACKING frame " << newFrame->id() << " onto ref. " << _trackingReference->frameID;


	SE3 frameToReference_initialEstimate;
	{
		boost::shared_lock_guard<boost::shared_mutex> lock( _system.poseConsistencyMutex );
		frameToReference_initialEstimate = se3FromSim3( trackingReferencePose.getCamToWorld().inverse() * _system.keyFrameGraph->allFramePoses.back()->getCamToWorld());
	}


	Timer timer;

	SE3 newRefToFrame_poseUpdate = _tracker->trackFrame(
																	_trackingReference,
																	newFrame.get(),
																	frameToReference_initialEstimate);

	perf.update( timer );

	tracking_lastResidual = _tracker->lastResidual;
	tracking_lastUsage = _tracker->pointUsage;
	tracking_lastGoodPerBad = _tracker->lastGoodCount / (_tracker->lastGoodCount + _tracker->lastBadCount);
	tracking_lastGoodPerTotal = _tracker->lastGoodCount / (newFrame->width(SE3TRACKING_MIN_LEVEL)*newFrame->height(SE3TRACKING_MIN_LEVEL));


	if(manualTrackingLossIndicated || _tracker->diverged ||
		(_system.keyFrameGraph->keyframesAll.size() > INITIALIZATION_PHASE_COUNT && !_tracker->trackingWasGood))
	{
		LOGF(WARNING, "TRACKING LOST for frame %d (%1.2f%% good Points, which is %1.2f%% of available points, %s)!\n",
				newFrame->id(),
				100*tracking_lastGoodPerTotal,
				100*tracking_lastGoodPerBad,
				_tracker->diverged ? "DIVERGED" : "NOT DIVERGED");

		_trackingReference->invalidate();

		setTrackingIsBad();
		//nextRelocIdx = -1;  // What does this do?

		// Kick over the mapping thread
		_system.mapThread->doIteration();
		// unmappedTrackedFrames.notifyAll();

		// unmappedTrackedFramesMutex.lock();
		// unmappedTrackedFramesSignal.notify_one();
		// unmappedTrackedFramesMutex.unlock();

		manualTrackingLossIndicated = false;
		return;
	}



	// if(plotTracking)
	// {
	// 	Eigen::Matrix<float, 20, 1> data;
	// 	data.setZero();
	// 	data[0] = _tracker->lastResidual;
	//
	// 	data[3] = _tracker->lastGoodCount / (tracker->lastGoodCount + _tracker->lastBadCount);
	// 	data[4] = 4*tracker->lastGoodCount / (float)_conf.slamImage.area();
	// 	data[5] = _tracker->pointUsage;
	//
	// 	data[6] = _tracker->affineEstimation_a;
	// 	data[7] = _tracker->affineEstimation_b;
	// 	outputWrapper->publishDebugInfo(data);
	// }

	_system.keyFrameGraph->addFrame(newFrame.get());


	//Sim3 lastTrackedCamToWorld = mostCurrentTrackedFrame->getScaledCamToWorld();
//  mostCurrentTrackedFrame->TrackingParent->getScaledCamToWorld() * sim3FromSE3(mostCurrentTrackedFrame->thisToParent_SE3TrackingResult, 1.0);

	_system.publishTrackedFrame(newFrame.get());


	// Keyframe selection
	// latestTrackedFrame = trackingNewFrame;
	//if (!my_createNewKeyframe && _map.currentKeyFrame()->numMappedOnThisTotal > MIN_NUM_MAPPED)
	if (_currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED)
	{
		Sophus::Vector3d dist = newRefToFrame_poseUpdate.translation() * _currentKeyFrame->meanIdepth;
		float minVal = fmin(0.2f + _system.keyFrameGraph->keyframesAll.size() * 0.8f / INITIALIZATION_PHASE_COUNT,1.0f);

		if(_system.keyFrameGraph->keyframesAll.size() < INITIALIZATION_PHASE_COUNT)	minVal *= 0.7;

		lastTrackingClosenessScore = _system.trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), _tracker->pointUsage);

		if (lastTrackingClosenessScore > minVal)
		{
			_system.mapThread->createNewKeyFrame( newFrame );
			// createNewKeyFrame = true;

			LOGF_IF( INFO, enablePrintDebugInfo && printKeyframeSelectionInfo,
							"SELECT %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",newFrame->id(),newFrame->getTrackingParent()->id(), dist.dot(dist), _tracker->pointUsage, _system.trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), _tracker->pointUsage));
		}
		else
		{
			LOGF_IF( INFO, enablePrintDebugInfo && printKeyframeSelectionInfo,
							"SKIPPD %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",newFrame->id(),newFrame->getTrackingParent()->id(), dist.dot(dist), _tracker->pointUsage, _system.trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), _tracker->pointUsage));

		}
	}

	_system.mapThread->pushUnmappedTrackedFrame( newFrame );

	// unmappedTrackedFrames.notifyAll();
		// unmappedTrackedFramesSignal.notify_one();
	// }

	// If blocking is requested...
	if(blockUntilMapped && trackingIsGood() ){
		while( _system.mapThread->unmappedTrackedFrames().size() > 0 ) {
			LOG(INFO) << "Waiting for mapping to be done...";
			_system.mapThread->unmappedTrackedFrames.wait( );
		}
	}

	// {
	// 	int utf = 0;
	// 	{
	// 		std::unique_lock< std::mutex > lock( unmappedTrackedFrames.mutex() );
	// 		utf = unmappedTrackedFrames().size();
	// 	}
	//
	// 	// std::unique_lock<std::mutex> lock(newFrameMappedMutex);
	// 	while(utf > 0)
	// 	{
	// 		LOGF(DEBUG, "TRACKING IS BLOCKING, waiting for %d frames to finish mapping.", (int)utf);
	// 		_mapThread.newFrameMapped.wait();
	//
	// 		{
	// 			std::unique_lock< std::mutex > lock( unmappedTrackedFrames.mutex() );
	// 			utf = unmappedTrackedFrames().size();
	// 		}
	// 	}
	// }
}



// n.b. this function will be called from the mapping thread.  Ensure
// locking is in place.
void TrackingThread::takeRelocalizeResult( const RelocalizerResult &result  )
{

	// Frame* keyframe;
	// int succFrameID;
	// SE3 succFrameToKF_init;
	// std::shared_ptr<Frame> succFrame;
	//
	// relocalizer.stop();
	// relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);
	// assert(keyframe != 0);

	{
		std::lock_guard<std::mutex> lock( _currentKeyFrame.mutex() );
		_trackingReference->importFrame( _currentKeyFrame.get());
		_trackingReferenceFrameSharedPT = _currentKeyFrame.ptr();
	}

	_tracker->trackFrame(
			_trackingReference,
			result.successfulFrame.get(),
			result.successfulFrameToKeyframe );

	if(!_tracker->trackingWasGood || _tracker->lastGoodCount / (_tracker->lastGoodCount + _tracker->lastBadCount) < 1-0.75f*(1-MIN_GOODPERGOODBAD_PIXEL))
	{
		LOG_IF(DEBUG, enablePrintDebugInfo && printRelocalizationInfo) << "RELOCALIZATION FAILED BADLY! discarding result.";
		_trackingReference->invalidate();
	}
	else
	{
		_system.keyFrameGraph->addFrame(result.successfulFrame.get());

		_system.mapThread->pushUnmappedTrackedFrame( result.successfulFrame );

		// {
		// 	std::lock_guard<std::mutex> lock( currentKeyFrameMutex );
			// createNewKeyFrame = false;
			setTrackingIsGood();
		//}
	}
}
