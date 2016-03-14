

#pragma once

#include "active_object/active.h"

#include "util/Configuration.h"
#include "util/ThreadMutexObject.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/Sim3Tracker.h"
#include "Tracking/TrackingReference.h"

namespace lsd_slam {

	class SlamSystem;

	struct KFConstraintStruct;

class ConstraintSearchThread {
public:
	ConstraintSearchThread( SlamSystem &system, bool enabled );
	~ConstraintSearchThread();

	void doFullReConstraintTrack( void )
	{ fullReConstraintTrackComplete.reset();
		if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::callbackDoFullReConstraintTrack, this )); }

	void newKeyFrame( Frame *frame )
	{ if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::callbackNewKeyFrame, this, frame )); }

	ThreadSynchronizer fullReConstraintTrackComplete;

private:

	SlamSystem &_system;

	Sim3Tracker* constraintTracker;
	SE3Tracker* constraintSE3Tracker;
	TrackingReference* newKFTrackingReference;
	TrackingReference* candidateTrackingReference;

	int _failedToRetrack;
	int lastNumConstraintsAddedOnFullRetrack;

	//=== Callbacks ===
	void callbackIdle( void );
	int callbackDoFullReConstraintTrack( void );
	void callbackNewKeyFrame( Frame *frame );

	//=== Internal functions ====
	int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH);

	std::unique_ptr<active_object::ActiveIdle> _thread;

	float tryTrackSim3(
			TrackingReference* A, TrackingReference* B,
			int lvlStart, int lvlEnd,
			bool useSSE,
			Sim3 &AtoB, Sim3 &BtoA,
			KFConstraintStruct* e1=0, KFConstraintStruct* e2=0);

	void testConstraint(
			Frame* candidate,
			KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
			Sim3 candidateToFrame_initialEstimate,
			float strictness);

};


}
