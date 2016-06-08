/**
* This file is derived from Jakob Engel's original LSD-SLAM code.
* His original copyright notice follows:
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

#pragma once
// #include <vector>
//#include <mutex>
// #include <thread>
// #include <condition_variable>
// #include <memory>
// #include <chrono>
//
// #include <boost/thread/shared_mutex.hpp>
//
// #include "util/settings.h"
// #include "IOWrapper/Timestamp.h"
// #include "opencv2/core/core.hpp"
//
// #include "util/SophusUtil.h"
// #include "util/Configuration.h"
// #include "util/Timer.h"
// #include "util/ThreadMutexObject.h"

#include "DataStructures/CurrentKeyFrame.h"
#include "Tracking/Relocalizer.h"
#include "util/MovingAverage.h"

namespace lsd_slam
{

class SlamSystem;
class TrackingReference;
// class KeyFrameGraph;
class SE3Tracker;
// class Sim3Tracker;
// class DepthMap;
// class Frame;
// class DataSet;
// class LiveSLAMWrapper;
// class Output3DWrapper;
// class FramePoseStruct;
//struct KFConstraintStruct;


typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class TrackingThread {
friend class IntegrationTest;
public:

	TrackingThread( SlamSystem &system );

	TrackingThread( const TrackingThread&) = delete;
	TrackingThread& operator=(const TrackingThread&) = delete;
	~TrackingThread();

	// tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
	void trackFrame(std::shared_ptr<Frame> newFrame, bool blockUntilMapped );
	void trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp );


	/** Sets the visualization where point clouds and camera poses will be sent to. */


	int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	//void changeKeyframe(std::shared_ptr<Frame> candidate, bool noCreate, bool force, float maxScore);

	void takeRelocalizeResult( const RelocalizerResult &result );

 	float lastTrackingClosenessScore;

	bool trackingIsGood( void ) const { return _trackingIsGood; }
	bool setTrackingIsBad( void )  { return _trackingIsGood = false; }
	bool setTrackingIsGood( void ) { return _trackingIsGood = true; }


	MsRateAverage perf;

	Timer timeLastUpdate;



private:

	SlamSystem &_system;
	CurrentKeyFrame &_currentKeyFrame;

	bool _trackingIsGood;

	// ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
	TrackingReference* _trackingReference; // tracking reference for current keyframe. only used by tracking.
	std::shared_ptr<Frame> _trackingReferenceFrameSharedPT;	// only used in odometry-mode, to keep a keyframe alive until it is deleted. ONLY accessed whithin currentKeyFrameMutex lock.

	SE3Tracker* _tracker;


	//
	//
	// // ============= EXCLUSIVELY MAPPING THREAD (+ init) =============
	//
	//
	//
	// // ============= EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) =============
	//
	//
	//
	//
	// // ============= SHARED ENTITIES =============
	float tracking_lastResidual;
	float tracking_lastUsage;
	float tracking_lastGoodPerBad;
	float tracking_lastGoodPerTotal;

	//
	// int lastNumConstraintsAddedOnFullRetrack;
	// bool doFinalOptimization;
	//
	// // for sequential operation. Set in Mapping, read in Tracking.
	// // std::condition_variable  newFrameMappedSignal;
	// // std::mutex newFrameMappedMutex;
	//
	//
	//

	//
	//
	//
	//
	//
	// // Tracking: if (!create) set candidate, set create.
	// // Mapping: if (create) use candidate, reset create.
	// // => no locking required.
	// std::shared_ptr<Frame> latestTrackedFrame;
	// bool createNewKeyFrame;
	//
	//
	//
	// // PUSHED in tracking, READ & CLEARED in mapping
	// // std::deque< std::shared_ptr<Frame> > unmappedTrackedFrames;
	// // ThreadSynchronizer unmappedTrackedFramesSynchro;
	// // std::mutex unmappedTrackedFramesMutex;
	// // std::condition_variable  unmappedTrackedFramesSignal;
	//
	//
	// // PUSHED by Mapping, READ & CLEARED by constraintFinder
	// ThreadMutexObject< std::deque< Frame* > > newKeyFrames;
	// // std::deque< Frame* > newKeyFrames;
	// // std::mutex newKeyFrameMutex;
	// // std::condition_variable newKeyFrameCreatedSignal;
	//
	//
	//
	//
	// // threads
	// // std::thread thread_mapping;
	// // std::thread thread_constraint_search;
	// //std::thread thread_optimization;
	//
	// // bool keepRunning; // used only on destruction to signal threads to finish.
	//
	//
	//
	// // optimization thread
	// // bool newConstraintAdded;
	// // std::mutex newConstraintMutex;
	// // std::condition_variable newConstraintCreatedSignal;
	//
	//
	//
	//
	//
	//
	//
	//
	// bool depthMapScreenshotFlag;
	// std::string depthMapScreenshotFilename;
	//

	//

	//
	//
	// void changeKeyframe(bool noCreate, bool force, float maxScore);
	// void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);
	// void loadNewCurrentKeyframe(Frame* keyframeToLoad);
	//

	//
	// void constraintSearchThreadLoop();

	// /** Calculates a scale independent error norm for reciprocal tracking results a and b with associated information matrices. */


	//void optimizationThreadLoop();



};

}
