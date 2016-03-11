

#pragma once

#include <mutex>
#include <memory>

#include <boost/thread/shared_mutex.hpp>

#include "active_object/active.h"

#include "util/MovingAverage.h"

namespace lsd_slam {

	class KeyFrameGraph;

class OptimizationThread {
public:

	// TODO.  Don't like passing reference to Mutex.  Another way to do it?

	OptimizationThread( KeyFrameGraph* graph,
											boost::shared_mutex &poseConsistencyMutex,
											bool idle );
	~OptimizationThread();

	// Public interfaces to kick off events
	void bgNewConstraint( void );
	void fgFinalOptimization( void );

	std::mutex g2oGraphAccessMutex;
	std::mutex newConstraintMutex;

	// optimization merging. SET in Optimization, merged in Mapping.
	bool haveUnmergedOptimizationOffset;

	MsRateAverage perf;

private:

	void callbackIdle( void );
	void callbackFinalOptimization( void );

	bool optimizationIteration(int itsPerTry, float minChange);

	// Function didn't appear to be called ... (?)
	// void OptimizationThread::optimizeGraph( void );

	KeyFrameGraph *keyFrameGraph;
	boost::shared_mutex &_poseConsistencyMutex;

	std::unique_ptr<active_object::ActiveIdle> _thread;

};


}
