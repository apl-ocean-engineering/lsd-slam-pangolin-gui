
#include "OptimizationThread.h"

namespace logger {

OptimizationThread( bool idle )
	: _thread( idle ? ActiveIdle::createActiveIdle( std::bind( &OptimizationThread::callbackIdle, this ), std::chrono::milliseconds(2000)) : NULL )
{
	LOG(INFO) << "Started optimization thread";
}

OptimizationThread::~OptimizationThread()
{
	if( _thread ) delete _thread.release();
	LOG(INFO) << "Exited optimization thread";
}

void OptimizationThread::bgNewConstraint( void )
{
	// Currently the same as onIdle
	if( _thread ) _thread.send( std::bind(&OptimizationThread::bgIdle, this) );
}

OptimizationThread::fgFinalOptimization( void )
{
	// It was waiting for this to finish in the main loop anyway
	//_thread.send( std::bind(&OptimizationThread::bgFinalOptimization, this) );
	if( _thread ) bgFinalOptimization();
}

//===== Callbacks for ActiveObject ======
void OptimizationThread::callbackIdle( void )
{
	while(optimizationIteration(5, 0.02));
}

void OptimizationThread::callbackFinalOptimization( void )
{
	LOG(INFO) << "doing final optimization iteration!";
	optimizationIteration(50, 0.001);
}


//=== Actual meat ====



bool OptimizationThread::optimizationIteration(int itsPerTry, float minChange)
{

	Timer timer;

	{
		std::lock_guard< std::mutex > lock(g2oGraphAccessMutex);

		// lock new elements buffer & take them over.
		{
			std::lock_guard< std::mutex > newConstraintMutex;
			keyFrameGraph->addElementsFromBuffer();
		}


		// Do the optimization. This can take quite some time!
		int its = keyFrameGraph->optimize(itsPerTry);


		// save the optimization result.
		poseConsistencyMutex.lock_shared();
		keyFrameGraph->keyframesAllMutex.lock_shared();
		float maxChange = 0;
		float sumChange = 0;
		float sum = 0;
		for(size_t i=0;i<keyFrameGraph->keyframesAll.size(); i++)
		{
			// set edge error sum to zero
			keyFrameGraph->keyframesAll[i]->edgeErrorSum = 0;
			keyFrameGraph->keyframesAll[i]->edgesNum = 0;

			if(!keyFrameGraph->keyframesAll[i]->pose->isInGraph) continue;



			// get change from last optimization
			Sim3 a = keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate();
			Sim3 b = keyFrameGraph->keyframesAll[i]->getScaledCamToWorld();
			Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();


			for(int j=0;j<7;j++)
			{
				float d = fabsf((float)(diff[j]));
				if(d > maxChange) maxChange = d;
				sumChange += d;
			}
			sum +=7;

			// set change
			keyFrameGraph->keyframesAll[i]->pose->setPoseGraphOptResult(
					keyFrameGraph->keyframesAll[i]->pose->graphVertex->estimate());

			// add error
			for(auto edge : keyFrameGraph->keyframesAll[i]->pose->graphVertex->edges())
			{
				keyFrameGraph->keyframesAll[i]->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
				keyFrameGraph->keyframesAll[i]->edgesNum++;
			}
		}

		haveUnmergedOptimizationOffset = true;
		keyFrameGraph->keyframesAllMutex.unlock_shared();
		poseConsistencyMutex.unlock_shared();

	} // Unlock g2oGraphAccessMutex

	LOGF_IF(DEBUG, enablePrintDebugInfo && printOptimizationInfo, "did %d optimization iterations. Max Pose Parameter Change: %f; avgChange: %f. %s\n", its, maxChange, sumChange / sum,
			maxChange > minChange && its == itsPerTry ? "continue optimizing":"Waiting for addition to graph.");

	perf.update( timer );

	return maxChange > minChange && its == itsPerTry;
}

// void OptimizationThread::optimizeGraph( void )
// {
// 	std::unique_lock<std::mutex> g2oLock(g2oGraphAccessMutex);
// 	keyFrameGraph->optimize(1000);
// 	g2oLock.unlock();
// 	mergeOptimizationOffset();
// }



}
