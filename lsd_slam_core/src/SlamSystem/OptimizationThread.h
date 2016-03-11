

#pragma once

namespace lsd_slam {


class OptimizationThread {
public:

	OptimizationThread( bool idle );
	~OptimizationThread();

	// Public interfaces to kick off events
	void bgNewConstraint( void );
	void fgFinalOptimizaiton( void );

	std::mutex g2oGraphAccessMutex;
	std::mutex newConstraintMutex;

	MsRateAverage perf;

private:

	void callbackIdle( void );
	void callbackFinalOptimization( void );

	bool optimizationIteration(int itsPerTry, float minChange);

	// void OptimizationThread::optimizeGraph( void );



	unique_ptr<active_object::ActiveIdle> _thread;

}


}
