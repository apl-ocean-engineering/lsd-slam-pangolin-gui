
#include "MappingThread.h"

#include <g3log/g3log.hpp>

#include <boost/thread/shared_lock_guard.hpp>

#include "GlobalMapping/KeyFrameGraph.h"
#include "DataStructures/Frame.h"
#include "Tracking/TrackingReference.h"

#include "SlamSystem/TrackingThread.h"
#include "SlamSystem/ConstraintSearchThread.h"

#include "SlamSystem.h"
#include "util/settings.h"

namespace lsd_slam {

using active_object::ActiveIdle;

// static const bool depthMapScreenshotFlag = true;


MappingThread::MappingThread( SlamSystem &system )
	: _system(system ),
		_currentKeyFrame( system.currentKeyFrame ),
		relocalizer( system.conf() ),
		map( new DepthMap( system.conf() ) ),
		mappingTrackingReference( new TrackingReference() ),
		_thread( ActiveIdle::createActiveIdle( std::bind( &MappingThread::callbackIdle, this ), std::chrono::milliseconds(200)) )
{
	LOG(INFO) << "Started Mapping thread";
}

MappingThread::~MappingThread()
{
	if( _thread ) delete _thread.release();

	delete mappingTrackingReference;
	delete map;

	// make sure to reset all shared pointers to all frames before deleting the keyframegraph!
	unmappedTrackedFrames().clear();

	// latestFrameTriedForReloc.reset();
	// latestTrackedFrame.reset();

	// currentKeyFrame().reset();
	//trackingReferenceFrameSharedPT.reset();

	//LOG(INFO) << "Exited Mapping thread";
}

//==== Callbacks ======


void MappingThread::callbackIdle( void )
{
	//LOG(INFO) << "Mapping thread idle callback";
	//while( doMappingIteration() ) {
	//	unmappedTrackedFrames.notifyAll();
	//}
}

void MappingThread::callbackUnmappedTrackedFrames( void )
{
	bool nMapped = false;
	size_t sz = 0;
	{
		std::lock_guard<std::mutex> lock(unmappedTrackedFrames.mutex() );
		nMapped = unmappedTrackedFrames().back()->getTrackingParent()->numMappedOnThisTotal < 10;
		sz = unmappedTrackedFrames().size();
	}

	LOG(DEBUG) << "In unmapped tracked frames callback with " << sz << " frames";

	if(sz < 50 ||
	  (sz < 100 && nMapped) ) {

		while( doMappingIteration() ) { ; }

		// TODO:  Was originally in the while() loop above.  However, there are
		// circumstances (if there's one untracked thread in the queue referenced
		// to an older keyframe) where doMappingIteration will return false, and this
		// notify never happens.   If that happens, TrackingThread will stop if it's
		// waiting on the signal.
		//
		// This should be called once per callback otherwise TrackingThread might get hung up?
		unmappedTrackedFrames.notifyAll();
		//}
	}
}

void MappingThread::callbackMergeOptimizationOffset()
{
	LOG(DEBUG) << "Merging optimization offset";

	// Bool lets us put the publishKeyframeGraph outside the mutex lock
	bool didUpdate = false;

	// if(_optThread->haveUnmergedOptimizationOffset())
	{
		boost::shared_lock_guard< boost::shared_mutex > pose_lock(_system.poseConsistencyMutex);
		boost::shared_lock_guard< boost::shared_mutex > kfLock( _system.keyFrameGraph->keyframesAllMutex);

		for(unsigned int i=0;i<_system.keyFrameGraph->keyframesAll.size(); i++)
			_system.keyFrameGraph->keyframesAll[i]->pose->applyPoseGraphOptResult();

		// _optThread->clearUnmergedOptimizationOffset();

		didUpdate = true;
	}

	if ( didUpdate ) _system.publishKeyframeGraph();

	optimizationUpdateMerged.notify();
}

void MappingThread::callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame )
{
	LOG(INFO) << "Set " << frame->id() << " as new key frame";
	finishCurrentKeyframe();
	_system.changeKeyframe(frame, false, true, 1.0f);

	_system.updateDisplayDepthMap();
}

//==== Actual meat ====

void MappingThread::gtDepthInit( std::shared_ptr<Frame> frame )
{
	// For a newly-imported frame, this will only be true if the depth
	// has been set explicitly
	CHECK( frame->hasIDepthBeenSet() );

	map->initializeFromGTDepth( _currentKeyFrame.get() );

	_system.updateDisplayDepthMap();
}


void MappingThread::randomInit( std::shared_ptr<Frame> frame )
{
	map->initializeRandomly( frame.get() );

	_system.updateDisplayDepthMap();
}


bool MappingThread::doMappingIteration()
{
	// If there's no keyframe, then give up
	if(_currentKeyFrame.empty() ) {
		LOG(INFO) << "Nothing to map: no keyframe";
		return false;
	}

		// TODO:  Don't know under what circumstances this if happens
	// if(!doMapping && currentKeyFrame()->idxInKeyframes < 0)
	// {
	// 	if(currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
	// 		finishCurrentKeyframe();
	// 	else
	// 		discardCurrentKeyframe();
	//
	// 	map->invalidate();
	// 	LOGF(INFO, "Finished KF %d as Mapping got disabled!\n",currentKeyFrame()->id());
	//
	// 	changeKeyframe(true, true, 1.0f);
	// }

	//callbackMergeOptimizationOffset();
	//addTimingSamples();

	// if(dumpMap)
	// {
	// 	keyFrameGraph->dumpMap(packagePath+"/save");
	// 	dumpMap = false;
	// }


	// set mappingFrame
	if( _system.trackingThread->trackingIsGood() )
	{
		// TODO:  Don't know under what circumstances doMapping = false
		// if(!doMapping)
		// {
		// 	//printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
		// 	if(_system.trackingThread->lastTrackingClosenessScore > 1)
		// 		changeKeyframe(true, false, _system.trackingThread->lastTrackingClosenessScore * 0.75);
		//
		// 	if (displayDepthMap || depthMapScreenshotFlag)
		// 		debugDisplayDepthMap();
		//
		// 	return false;
		// }


		// if (createNewKeyFrame)
		// {
		//
		// }
		// else
		// {
			bool didSomething = updateKeyframe();

			_system.updateDisplayDepthMap();

			LOG(DEBUG) << "Tracking is good, updating key frame, " << (didSomething ? "DID" : "DIDN'T") << " do something";

			return didSomething;
		// 	if(!didSomething)
		// 		return false;
		// // }
		//
		// return true;
	}
	else
	{
		LOG(INFO) << "Tracking is bad";

		// invalidate map if it was valid.
		if(map->isValid())
		{
			if( _currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
				finishCurrentKeyframe();
			else
				discardCurrentKeyframe();

			map->invalidate();
		}

		// start relocalizer if it isnt running already
		if(!relocalizer.isRunning)
			relocalizer.start(_system.keyFrameGraph->keyframesAll);

		// did we find a frame to relocalize with?
		if(relocalizer.waitResult(50)) {

						// Frame* keyframe;
						// int succFrameID;
						// SE3 succFrameToKF_init;
						// std::shared_ptr<Frame> succFrame;
						//
						// relocalizer.stop();
						// relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);

			relocalizer.stop();
			RelocalizerResult result( relocalizer.getResult() );

			_system.loadNewCurrentKeyframe(result.keyframe);

			_system.trackingThread->takeRelocalizeResult( result );
		}



		return true;
	}
}




bool MappingThread::updateKeyframe()
{
	std::shared_ptr<Frame> reference = nullptr;
	std::deque< std::shared_ptr<Frame> > references;

	unmappedTrackedFrames.lock();

	// remove frames that have a different tracking parent.
	while(unmappedTrackedFrames().size() > 0 &&
			(!unmappedTrackedFrames().front()->hasTrackingParent() ||
					unmappedTrackedFrames().front()->getTrackingParent() != _currentKeyFrame.get()))
	{
		unmappedTrackedFrames().front()->clear_refPixelWasGood();
		unmappedTrackedFrames().pop_front();
	}

	// clone list
	if(unmappedTrackedFrames().size() > 0)
	{
		for(unsigned int i=0;i<unmappedTrackedFrames().size(); i++)
			references.push_back(unmappedTrackedFrames()[i]);

		std::shared_ptr<Frame> popped = unmappedTrackedFrames().front();
		unmappedTrackedFrames().pop_front();
		unmappedTrackedFrames.unlock();

		if(enablePrintDebugInfo && printThreadingInfo)
			printf("MAPPING %d on %d to %d (%d frames)\n", _currentKeyFrame->id(), references.front()->id(), references.back()->id(), (int)references.size());

		map->updateKeyframe(references);

		popped->clear_refPixelWasGood();
		references.clear();
	}
	else
	{
		unmappedTrackedFrames.unlock();
		return false;
	}


	// if( outputWrapper ) {
	//
	// if( enablePrintDebugInfo && printRegularizeStatistics)
	// {
	// 	Eigen::Matrix<float, 20, 1> data;
	// 	data.setZero();
	// 	data[0] = runningStats.num_reg_created;
	// 	data[2] = runningStats.num_reg_smeared;
	// 	data[3] = runningStats.num_reg_deleted_secondary;
	// 	data[4] = runningStats.num_reg_deleted_occluded;
	// 	data[5] = runningStats.num_reg_blacklisted;
	//
	// 	data[6] = runningStats.num_observe_created;
	// 	data[7] = runningStats.num_observe_create_attempted;
	// 	data[8] = runningStats.num_observe_updated;
	// 	data[9] = runningStats.num_observe_update_attempted;
	//
	//
	// 	data[10] = runningStats.num_observe_good;
	// 	data[11] = runningStats.num_observe_inconsistent;
	// 	data[12] = runningStats.num_observe_notfound;
	// 	data[13] = runningStats.num_observe_skip_oob;
	// 	data[14] = runningStats.num_observe_skip_fail;
	//
	// 	outputWrapper->publishDebugInfo(data);
	// }



	if( continuousPCOutput && !_currentKeyFrame.empty() ) _system.publishKeyframe( _currentKeyFrame.get() );

	return true;
}




void MappingThread::finishCurrentKeyframe()
{
	LOG_IF(DEBUG, enablePrintDebugInfo && printThreadingInfo) << "FINALIZING KF " << _currentKeyFrame->id();

	map->finalizeKeyFrame();

	if(_system.conf().SLAMEnabled)
	{
		mappingTrackingReference->importFrame(_currentKeyFrame.get());
		_currentKeyFrame->setPermaRef(mappingTrackingReference);
		mappingTrackingReference->invalidate();

		if(_currentKeyFrame->idxInKeyframes < 0)
		{
			_system.keyFrameGraph->keyframesAllMutex.lock();
			_currentKeyFrame->idxInKeyframes = _system.keyFrameGraph->keyframesAll.size();
			_system.keyFrameGraph->keyframesAll.push_back(_currentKeyFrame.get() );
			_system.keyFrameGraph->totalPoints += _currentKeyFrame->numPoints;
			_system.keyFrameGraph->totalVertices ++;
			_system.keyFrameGraph->keyframesAllMutex.unlock();

			_system.constraintThread->newKeyFrame( _currentKeyFrame.get() );
		}
	}

	_system.publishKeyframe(_currentKeyFrame.get());
}

void MappingThread::discardCurrentKeyframe()
{
	LOG_IF(DEBUG, enablePrintDebugInfo && printThreadingInfo) << "DISCARDING KF " << _currentKeyFrame->id();

	if(_currentKeyFrame->idxInKeyframes >= 0)
	{
		LOG(WARNING) << "WARNING: trying to discard a KF that has already been added to the graph... finalizing instead.";
		finishCurrentKeyframe();
		return;
	}


	map->invalidate();

	{
		boost::shared_lock_guard< boost::shared_mutex > lock( _system.keyFrameGraph->allFramePosesMutex );
		for(FramePoseStruct* p : _system.keyFrameGraph->allFramePoses)
		{
			if(p->trackingParent != 0 && p->trackingParent->frameID == _currentKeyFrame->id() )
				p->trackingParent = 0;
		}
	}

	{
		boost::shared_lock_guard< boost::shared_mutex > lock(_system.keyFrameGraph->idToKeyFrameMutex);
		_system.keyFrameGraph->idToKeyFrame.erase(_currentKeyFrame->id());
	}

}


}
