

#pragma once

#include <mutex>
#include <memory>

#include <boost/thread/shared_mutex.hpp>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

#include "DataStructures/CurrentKeyFrame.h"
#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"

#include "Tracking/Relocalizer.h"


namespace lsd_slam {

	class KeyFrameGraph;
	class SlamSystem;

class MappingThread {
public:

	// TODO.  Don't like passing reference to Mutex.  Another way to do it?

	MappingThread( SlamSystem &system );
	~MappingThread();

	//=== Callbacks into the thread ===
	void pushUnmappedTrackedFrame( const std::shared_ptr<Frame> &frame )
	{
		{
			std::lock_guard<std::mutex> lock(unmappedTrackedFrames.mutex() );
			unmappedTrackedFrames().push_back( frame );
		}

		if( _thread ) {
			_thread->send( std::bind( &MappingThread::callbackUnmappedTrackedFrames, this ));
			LOG(INFO) << "Mq now " << _thread->size();
		}
	}

	void doIteration( void )
	{ if( _thread ) _thread->send( std::bind( &MappingThread::callbackIdle, this )); }

	void mergeOptimizationUpdate( void )
	{ optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::callbackMergeOptimizationOffset, this )); }

	void createNewKeyFrame( const SharedFramePtr &frame )
	{
		if( _newKeyFrame.get() != nullptr ) LOG(WARNING) << "Asked to make " << frame->id() << " a keyframe when " << _newKeyFrame()->id() << " is already pending";
		_newKeyFrame = frame;
		//if( _thread ) {
		//		_thread->send( std::bind( &MappingThread::callbackCreateNewKeyFrame, this, frame ));
		//		LOG(INFO) << "Mq now " << _thread->size();
		//}
	}

	bool newKeyFramePending( void )
	{
			return _newKeyFrame.get() != nullptr;
	}

	void gtDepthInit( SharedFramePtr frame );
	void randomInit( SharedFramePtr frame );


	// SET & READ EVERYWHERE
	// std::mutex currentKeyFrameMutex;

	MutexObject< std::deque< SharedFramePtr > > unmappedTrackedFrames;

	DepthMap* map;
	TrackingReference* mappingTrackingReference;

	ThreadSynchronizer optimizationUpdateMerged;


	// during re-localization used
	Relocalizer relocalizer;

private:

	SlamSystem &_system;
	CurrentKeyFrame &_currentKeyFrame;

	CurrentKeyFrame _newKeyFrame;

	// == Thread callbacks ==
	void callbackIdle( void );
	void callbackUnmappedTrackedFrames( void );
	//void callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame );

	// == Local functions ==
	void callbackMergeOptimizationOffset();

	bool doMappingIteration();

	bool updateKeyframe();

	void addTimingSamples();

	void finishCurrentKeyframe();
	void discardCurrentKeyframe();


	void debugDisplayDepthMap();


	// std::vector<Frame*> KFForReloc;
	// //int nextRelocIdx;
	// std::shared_ptr<Frame> latestFrameTriedForReloc;

	std::unique_ptr<active_object::ActiveIdle> _thread;

};


}
