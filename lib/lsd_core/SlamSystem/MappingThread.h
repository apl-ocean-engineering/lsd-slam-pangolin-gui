

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
	void pushUnmappedTrackedFrame( std::shared_ptr<Frame> frame )
	{	if( _thread ) _thread->send( std::bind( &MappingThread::callbackUnmappedTrackedFrames, this, frame )); }

	void doIteration( void )
	{ if( _thread ) _thread->send( std::bind( &MappingThread::callbackIdle, this )); }

	void mergeOptimizationUpdate( void )
	{ optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::callbackMergeOptimizationOffset, this )); }

	void createNewKeyFrame( std::shared_ptr<Frame> frame )
	{ 	if( _thread ) _thread->send( std::bind( &MappingThread::callbackCreateNewKeyFrame, this, frame )); }


	void gtDepthInit( std::shared_ptr<Frame> frame );
	void randomInit( std::shared_ptr<Frame> frame );


	// SET & READ EVERYWHERE
	// std::mutex currentKeyFrameMutex;

	MutexObject< std::deque< std::shared_ptr<Frame> > > unmappedTrackedFrames;

	DepthMap* map;
	TrackingReference* mappingTrackingReference;

	ThreadSynchronizer optimizationUpdateMerged;


	// during re-localization used
	Relocalizer relocalizer;

private:

	SlamSystem &_system;
	CurrentKeyFrame &_currentKeyFrame;

	// == Thread callbacks ==
	void callbackIdle( void );
	void callbackUnmappedTrackedFrames( std::shared_ptr<Frame> frame );
	void callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame );

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
