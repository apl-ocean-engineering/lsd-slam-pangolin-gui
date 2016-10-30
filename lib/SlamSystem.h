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

#pragma once
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>
#include <chrono>

#include <boost/thread/shared_mutex.hpp>

#include "util/settings.h"
#include "IOWrapper/Timestamp.h"
#include "opencv2/core/core.hpp"

#include "IOWrapper/Output3DWrapper.h"

#include "util/SophusUtil.h"
#include "util/MovingAverage.h"
#include "util/Configuration.h"
#include "util/Timer.h"
#include "util/ThreadMutexObject.h"

#include "Tracking/Relocalizer.h"
#include "DataStructures/CurrentKeyFrame.h"

namespace lsd_slam
{

// class TrackingReference;
class KeyFrameGraph;
// class SE3Tracker;
// class Sim3Tracker;
// class DepthMap;
// class Frame;
// class DataSet;
// class LiveSLAMWrapper;
class Output3DWrapper;
class FramePoseStruct;
class TrackableKeyFrameSearch;
// struct KFConstraintStruct;

	class TrackingThread;
	class OptimizationThread;
	class MappingThread;
	class ConstraintSearchThread;


typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class SlamSystem
{
friend class IntegrationTest;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// settings. Constant from construction onward.
	// int width;
	// int height;
	// Eigen::Matrix3f K;
	// const bool SLAMEnabled;

	// bool trackingIsGood;

	bool finalized;

	SlamSystem( const Configuration &conf );

	SlamSystem( const SlamSystem&) = delete;
	SlamSystem& operator=(const SlamSystem&) = delete;

	~SlamSystem();


	void randomInit(uchar* image, int id, double timeStamp);
	void randomInit( SharedFramePtr frame );
	void gtDepthInit( SharedFramePtr frame );


	// tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
	void trackFrame(SharedFramePtr newFrame, bool blockUntilMapped );
	void trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp );


	// finalizes the system, i.e. blocks and does all remaining loop-closures etc.
	void finalize();

	/** Does an offline optimization step. */
	// void optimizeGraph();

	// inline Frame* getCurrentKeyframe() {return currentKeyFrame.get();}	// not thread-safe!

	/** Returns the current pose estimate. */
	SE3 getCurrentPoseEstimate();

	Sophus::Sim3f getCurrentPoseEstimateScale();

	//==== KeyFrame maintenance functions ====
	void changeKeyframe( SharedFramePtr frame, bool noCreate, bool force, float maxScore);
	void loadNewCurrentKeyframe(Frame* keyframeToLoad);
	void createNewCurrentKeyframe( SharedFramePtr newKeyframeCandidate );


	// void requestDepthMapScreenshot(const std::string& filename);

	// int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	std::vector<FramePoseStruct*> getAllPoses();

	struct PerformanceData {
		PerformanceData( void ) {;}

		MsRateAverage findConstraint, findReferences;
	} perf;

	Timer timeLastUpdate;

	// Output3DWrapper *get3DOutputWrapper( void ) const { return outputWrapper; }
	const Configuration &conf( void ) const     { return _conf; }

	//=== Debugging output functions =====
	Output3DWrapper *outputWrapper( void )      { return _outputWrapper; }
	void set3DOutputWrapper(Output3DWrapper* outputWrapper) {	_outputWrapper = outputWrapper; }

	void publishTrackedFrame( Frame *frame )      { if( _outputWrapper ) _outputWrapper->publishTrackedFrame( frame ); }
	void publishKeyframeGraph( void )             { if( _outputWrapper ) _outputWrapper->publishKeyframeGraph( keyFrameGraph ); }
	void publishKeyframe(  Frame *frame )         { if( _outputWrapper ) _outputWrapper->publishKeyframe( frame ); }
	void publishDepthImage( unsigned char* data  ) { if( _outputWrapper ) _outputWrapper->updateDepthImage( data ); }


	void updateDisplayDepthMap();



	TrackingThread *trackingThread;
	OptimizationThread *optThread;
	MappingThread *mapThread;
	ConstraintSearchThread *constraintThread;

	// mutex to lock frame pose consistency. within a shared lock of this, *->getScaledCamToWorld() is
	// GUARANTEED to give the same result each call, and to be compatible to each other.
	// locked exclusively during the pose-update by Mapping.
	boost::shared_mutex poseConsistencyMutex;

	KeyFrameGraph* keyFrameGraph;	  // has own locks

	CurrentKeyFrame currentKeyFrame;

	TrackableKeyFrameSearch* trackableKeyFrameSearch;


private:

	const Configuration &_conf;

	// Individual / no locking
	Output3DWrapper* _outputWrapper;	// no lock required



	// ======= Functions =====

	void addTimingSamples();




	// ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
	// TrackingReference* trackingReference; // tracking reference for current keyframe. only used by tracking.
	// SE3Tracker* tracker;



	// ============= EXCLUSIVELY MAPPING THREAD (+ init) =============



	// ============= EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) =============


	//
	//
	// // ============= SHARED ENTITIES =============
	// float tracking_lastResidual;
	// float tracking_lastUsage;
	// float tracking_lastGoodPerBad;
	// float tracking_lastGoodPerTotal;
	//
	// int lastNumConstraintsAddedOnFullRetrack;
	// bool doFinalOptimization;
	// float lastTrackingClosenessScore;
	//
	// // for sequential operation. Set in Mapping, read in Tracking.
	// // std::condition_variable  newFrameMappedSignal;
	// // std::mutex newFrameMappedMutex;
	//
	//
	//
	// // USED DURING RE-LOCALIZATION ONLY
	// Relocalizer relocalizer;
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
	// const Configuration &_conf;
	//
	//
	// /** Merges the current keyframe optimization offset to all working entities. */
	// void mergeOptimizationOffset();
	//
	//
	// // void mappingThreadLoop();
	//
	// void finishCurrentKeyframe();
	// void discardCurrentKeyframe();
	//
	// void changeKeyframe(bool noCreate, bool force, float maxScore);
	// void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);
	// void loadNewCurrentKeyframe(Frame* keyframeToLoad);
	//
	//
	// bool updateKeyframe();
	//
	//
	// void debugDisplayDepthMap();
	//
	// void takeRelocalizeResult();
	//
	// void constraintSearchThreadLoop();
	// /** Calculates a scale independent error norm for reciprocal tracking results a and b with associated information matrices. */
	// float tryTrackSim3(
	// 		TrackingReference* A, TrackingReference* B,
	// 		int lvlStart, int lvlEnd,
	// 		bool useSSE,
	// 		Sim3 &AtoB, Sim3 &BtoA,
	// 		KFConstraintStruct* e1=0, KFConstraintStruct* e2=0);
	//
	// void testConstraint(
	// 		Frame* candidate,
	// 		KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
	// 		Sim3 candidateToFrame_initialEstimate,
	// 		float strictness);
	//
	// //void optimizationThreadLoop();



};

}
