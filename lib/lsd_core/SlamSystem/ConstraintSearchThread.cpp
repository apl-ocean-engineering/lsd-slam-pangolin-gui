
#include "ConstraintSearchThread.h"

#include <boost/thread/shared_lock_guard.hpp>

#include <g2o/core/robust_kernel_impl.h>

#include "SlamSystem.h"
#include "GlobalMapping/KeyFrameGraph.h"

#include "SlamSystem/OptimizationThread.h"

namespace lsd_slam {

	using active_object::ActiveIdle;


ConstraintSearchThread::ConstraintSearchThread( SlamSystem &system, bool enabled )
	: _system( system ),
		_failedToRetrack( 0 ),
		constraintTracker( new Sim3Tracker( system.conf().slamImage ) ),
		constraintSE3Tracker(  new SE3Tracker( system.conf().slamImage )  ),
		newKFTrackingReference(  new TrackingReference()  ),
		candidateTrackingReference(  new TrackingReference()  ),
	_thread( enabled ? ActiveIdle::createActiveIdle( std::bind( &ConstraintSearchThread::callbackIdle, this ), std::chrono::milliseconds(500)) : NULL )
{
}

ConstraintSearchThread::~ConstraintSearchThread( void )
{
	if( _thread) delete _thread.release();

	if(constraintTracker )          delete constraintTracker;
	if(constraintSE3Tracker )       delete constraintSE3Tracker;
	if(newKFTrackingReference )     delete newKFTrackingReference;
	if(candidateTrackingReference ) delete candidateTrackingReference;
}


//=== callbacks ====


void ConstraintSearchThread::callbackIdle( void )
{
	bool doneSomething = false;

	{
		std::lock_guard< std::mutex > lock( _system.keyFrameGraph->keyframesForRetrackMutex );

		if(_system.keyFrameGraph->keyframesForRetrack.size() > 10) {
			std::deque< Frame* >::iterator toReTrack = _system.keyFrameGraph->keyframesForRetrack.begin() + (rand() % (_system.keyFrameGraph->keyframesForRetrack.size()/3));
			Frame* toReTrackFrame = *toReTrack;

			_system.keyFrameGraph->keyframesForRetrack.erase(toReTrack);
			_system.keyFrameGraph->keyframesForRetrack.push_back(toReTrackFrame);

			// lock.unlock();

			int found = findConstraintsForNewKeyFrames(toReTrackFrame, false, false, 2.0);
			if(found == 0)
				_failedToRetrack++;
			else
				_failedToRetrack=0;

			if(_failedToRetrack < (int)_system.keyFrameGraph->keyframesForRetrack.size() - 5)
				doneSomething = true;
		}
	}

	// If you did something, go again immediately
	if( doneSomething ) _thread->send( std::bind( &ConstraintSearchThread::callbackIdle, this ) );

			// if(!doneSomething)
			// {
			// 	LOG_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo) << "nothing to re-track... waiting.";
			// 	newKeyFrames.cv().wait_for(lock,std::chrono::milliseconds(500));
			//
			// }
}

int ConstraintSearchThread::callbackDoFullReConstraintTrack( void )
{
	// 	std::unique_lock<std::mutex> lock(newKeyFrames.mutex());
	LOG(INFO) << "Optimizing Full Map!";

	int added = 0;
	for(unsigned int i=0;i<_system.keyFrameGraph->keyframesAll.size();i++)
	{
		if(_system.keyFrameGraph->keyframesAll[i]->pose->isInGraph)
			added += findConstraintsForNewKeyFrames(_system.keyFrameGraph->keyframesAll[i], false, false, 1.0);
	}

	LOG(INFO) << "Done optimizing Full Map! Added " << added << " constraints.";

	// doFullReConstraintTrack = false;
	fullReConstraintTrackComplete.notify();

	return lastNumConstraintsAddedOnFullRetrack = added;
}

void ConstraintSearchThread::callbackNewKeyFrame( Frame *newKF )
{
	{
		Timer timer;

		findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
		_failedToRetrack=0;

		_system.perf.findConstraint.update( timer );
	}

	FrameMemory::getInstance().pruneActiveFrames();
}



//=== Meat =====


// void ConstraintSearchThread::constraintSearchThreadLoop()
// {
// 	LOG(INFO) << "Started constraint search thread!";
//
// 	std::unique_lock<std::mutex> lock(newKeyFrames.mutex());
// 	int failedToRetrack = 0;
//
// 	while(keepRunning)
// 	{
// 		if(newKeyFrames().size() == 0)
// 		{
// 			lock.unlock();
// 			keyFrameGraph->keyframesForRetrackMutex.lock();
// 			bool doneSomething = false;
// 			if(keyFrameGraph->keyframesForRetrack.size() > 10)
// 			{
// 				std::deque< Frame* >::iterator toReTrack = keyFrameGraph->keyframesForRetrack.begin() + (rand() % (keyFrameGraph->keyframesForRetrack.size()/3));
// 				Frame* toReTrackFrame = *toReTrack;
//
// 				keyFrameGraph->keyframesForRetrack.erase(toReTrack);
// 				keyFrameGraph->keyframesForRetrack.push_back(toReTrackFrame);
//
// 				keyFrameGraph->keyframesForRetrackMutex.unlock();
//
// 				int found = findConstraintsForNewKeyFrames(toReTrackFrame, false, false, 2.0);
// 				if(found == 0)
// 					failedToRetrack++;
// 				else
// 					failedToRetrack=0;
//
// 				if(failedToRetrack < (int)keyFrameGraph->keyframesForRetrack.size() - 5)
// 					doneSomething = true;
// 			}
// 			else
// 				keyFrameGraph->keyframesForRetrackMutex.unlock();
//
// 			lock.lock();
//
// 			if(!doneSomething)
// 			{
// 				LOG_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo) << "nothing to re-track... waiting.";
// 				newKeyFrames.cv().wait_for(lock,std::chrono::milliseconds(500));
//
// 			}
// 		}
// 		else
// 		{
// 			Frame* newKF = newKeyFrames().front();
// 			newKeyFrames().pop_front();
// 			lock.unlock();
//
//
// 			{
// 				Timer timer;
//
// 				findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
// 				failedToRetrack=0;
//
// 				_perf.findConstraint.update( timer );
// 			}
//
// 			FrameMemory::getInstance().pruneActiveFrames();
// 			lock.lock();
// 		}
//
//
// 		if(doFullReConstraintTrack)
// 		{
// 			lock.unlock();
// 			LOG(INFO) << "Optimizing Full Map!";
//
// 			int added = 0;
// 			for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size();i++)
// 			{
// 				if(keyFrameGraph->keyframesAll[i]->pose->isInGraph)
// 					added += findConstraintsForNewKeyFrames(keyFrameGraph->keyframesAll[i], false, false, 1.0);
// 			}
//
// 			LOG(INFO) << "Done optimizing Full Map! Added " << added << " constraints.";
//
// 			doFullReConstraintTrack = false;
//
// 			lastNumConstraintsAddedOnFullRetrack = added;
// 			lock.lock();
// 		}
//
//
//
// 	}
//
// 	LOG(INFO) << "Exited constraint search thread";
// }





int ConstraintSearchThread::findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH)
{
	if(!newKeyFrame->hasTrackingParent()) {
		// {
		// 	std::lock_guard<std::mutex> lock( _system.optThread->newConstraintMutex );
			_system.keyFrameGraph->addKeyFrame(newKeyFrame);
		// }
		_system.optThread->doNewConstraint();
		return 0;
	}

	if(!forceParent && (newKeyFrame->lastConstraintTrackedCamToWorld * newKeyFrame->getScaledCamToWorld().inverse()).log().norm() < 0.01)
		return 0;


	newKeyFrame->lastConstraintTrackedCamToWorld = newKeyFrame->getScaledCamToWorld();

	// =============== get all potential candidates and their initial relative pose. =================
	std::vector<KFConstraintStruct*> constraints;
	Frame* fabMapResult = 0;
	std::unordered_set<Frame*> candidates = _system.trackableKeyFrameSearch->findCandidates(newKeyFrame, fabMapResult, useFABMAP, closeCandidatesTH);
	std::map< Frame*, Sim3 > candidateToFrame_initialEstimateMap;


	// erase the ones that are already neighbours.
	for(std::unordered_set<Frame*>::iterator c = candidates.begin(); c != candidates.end();)
	{
		if(newKeyFrame->neighbors.find(*c) != newKeyFrame->neighbors.end())
		{
			LOGF_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo,"SKIPPING %d on %d cause it already exists as constraint.\n", (*c)->id(), newKeyFrame->id());
			c = candidates.erase(c);
		}
		else
			++c;
	}

	std::unordered_map<Frame*, int> distancesToNewKeyFrame;
	{
		boost::shared_lock_guard<boost::shared_mutex> lock(_system.poseConsistencyMutex);
		for (Frame* candidate : candidates)
		{
			Sim3 candidateToFrame_initialEstimate = newKeyFrame->getScaledCamToWorld().inverse() * candidate->getScaledCamToWorld();
			candidateToFrame_initialEstimateMap[candidate] = candidateToFrame_initialEstimate;
		}

		if(newKeyFrame->hasTrackingParent())
			_system.keyFrameGraph->calculateGraphDistancesToFrame(newKeyFrame->getTrackingParent(), &distancesToNewKeyFrame);
	}





	// =============== distinguish between close and "far" candidates in Graph =================
	// Do a first check on trackability of close candidates.
	std::unordered_set<Frame*> closeCandidates;
	std::vector<Frame*> farCandidates;
	Frame* parent = newKeyFrame->hasTrackingParent() ? newKeyFrame->getTrackingParent() : 0;

	int closeFailed = 0;
	int closeInconsistent = 0;

	SO3 disturbance = SO3::exp(Sophus::Vector3d(0.05,0,0));

	for (Frame* candidate : candidates)
	{
		if (candidate->id() == newKeyFrame->id())
			continue;
		if(!candidate->pose->isInGraph)
			continue;
		if(newKeyFrame->hasTrackingParent() && candidate == newKeyFrame->getTrackingParent())
			continue;
		if(candidate->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
			continue;

		SE3 c2f_init = se3FromSim3(candidateToFrame_initialEstimateMap[candidate].inverse()).inverse();
		c2f_init.so3() = c2f_init.so3() * disturbance;
		SE3 c2f = constraintSE3Tracker->trackFrameOnPermaref(candidate, newKeyFrame, c2f_init);
		if(!constraintSE3Tracker->trackingWasGood) {closeFailed++; continue;}


		SE3 f2c_init = se3FromSim3(candidateToFrame_initialEstimateMap[candidate]).inverse();
		f2c_init.so3() = disturbance * f2c_init.so3();
		SE3 f2c = constraintSE3Tracker->trackFrameOnPermaref(newKeyFrame, candidate, f2c_init);
		if(!constraintSE3Tracker->trackingWasGood) {closeFailed++; continue;}

		if((f2c.so3() * c2f.so3()).log().norm() >= 0.09) {closeInconsistent++; continue;}

		closeCandidates.insert(candidate);
	}



	int farFailed = 0;
	int farInconsistent = 0;
	for (Frame* candidate : candidates)
	{
		if (candidate->id() == newKeyFrame->id())
			continue;
		if(!candidate->pose->isInGraph)
			continue;
		if(newKeyFrame->hasTrackingParent() && candidate == newKeyFrame->getTrackingParent())
			continue;
		if(candidate->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
			continue;

		if(candidate == fabMapResult)
		{
			farCandidates.push_back(candidate);
			continue;
		}

		if(distancesToNewKeyFrame.at(candidate) < 4)
			continue;

		farCandidates.push_back(candidate);
	}




	int closeAll = closeCandidates.size();
	int farAll = farCandidates.size();

	// erase the ones that we tried already before (close)
	for(std::unordered_set<Frame*>::iterator c = closeCandidates.begin(); c != closeCandidates.end();)
	{
		if(newKeyFrame->trackingFailed.find(*c) == newKeyFrame->trackingFailed.end())
		{
			++c;
			continue;
		}
		auto range = newKeyFrame->trackingFailed.equal_range(*c);

		bool skip = false;
		Sim3 f2c = candidateToFrame_initialEstimateMap[*c].inverse();
		for (auto it = range.first; it != range.second; ++it)
		{
			if((f2c * it->second).log().norm() < 0.1)
			{
				skip=true;
				break;
			}
		}

		if(skip)
		{
			LOGF_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo,
						"SKIPPING %d on %d (NEAR), cause we already have tried it.\n", (*c)->id(), newKeyFrame->id());
			c = closeCandidates.erase(c);
		}
		else
			++c;
	}

	// erase the ones that are already neighbours (far)
	for(unsigned int i=0;i<farCandidates.size();i++)
	{
		if(newKeyFrame->trackingFailed.find(farCandidates[i]) == newKeyFrame->trackingFailed.end())
			continue;

		auto range = newKeyFrame->trackingFailed.equal_range(farCandidates[i]);

		bool skip = false;
		for (auto it = range.first; it != range.second; ++it)
		{
			if((it->second).log().norm() < 0.2)
			{
				skip=true;
				break;
			}
		}

		if(skip)
		{
			LOGF_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo,
						"SKIPPING %d on %d (FAR), cause we already have tried it.\n", farCandidates[i]->id(), newKeyFrame->id());
			farCandidates[i] = farCandidates.back();
			farCandidates.pop_back();
			i--;
		}
	}



	LOGF_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo,
				"Final Loop-Closure Candidates: %d / %d close (%d failed, %d inconsistent) + %d / %d far (%d failed, %d inconsistent) = %d\n",
				(int)closeCandidates.size(),closeAll, closeFailed, closeInconsistent,
				(int)farCandidates.size(), farAll, farFailed, farInconsistent,
				(int)closeCandidates.size() + (int)farCandidates.size());



	// =============== limit number of close candidates ===============
	// while too many, remove the one with the highest connectivity.
	while((int)closeCandidates.size() > maxLoopClosureCandidates)
	{
		Frame* worst = 0;
		int worstNeighbours = 0;
		for(Frame* f : closeCandidates)
		{
			int neightboursInCandidates = 0;
			for(Frame* n : f->neighbors)
				if(closeCandidates.find(n) != closeCandidates.end())
					neightboursInCandidates++;

			if(neightboursInCandidates > worstNeighbours || worst == 0)
			{
				worst = f;
				worstNeighbours = neightboursInCandidates;
			}
		}

		closeCandidates.erase(worst);
	}


	// =============== limit number of far candidates ===============
	// delete randomly
	int maxNumFarCandidates = (maxLoopClosureCandidates +1) / 2;
	if(maxNumFarCandidates < 5) maxNumFarCandidates = 5;
	while((int)farCandidates.size() > maxNumFarCandidates)
	{
		int toDelete = rand() % farCandidates.size();
		if(farCandidates[toDelete] != fabMapResult)
		{
			farCandidates[toDelete] = farCandidates.back();
			farCandidates.pop_back();
		}
	}







	// =============== TRACK! ===============

	// make tracking reference for newKeyFrame.
	newKFTrackingReference->importFrame(newKeyFrame);


	for (Frame* candidate : closeCandidates)
	{
		KFConstraintStruct* e1=0;
		KFConstraintStruct* e2=0;

		testConstraint(
				candidate, e1, e2,
				candidateToFrame_initialEstimateMap[candidate],
				loopclosureStrictness);

		LOG_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo) << " CLOSE (" << distancesToNewKeyFrame.at(candidate) << ")";

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);

			// delete from far candidates if it's in there.
			for(unsigned int k=0;k<farCandidates.size();k++)
			{
				if(farCandidates[k] == candidate)
				{
					LOGF_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo,
						" DELETED %d from far, as close was successful!\n", candidate->id());

					farCandidates[k] = farCandidates.back();
					farCandidates.pop_back();
				}
			}
		}
	}


	for (Frame* candidate : farCandidates)
	{
		KFConstraintStruct* e1=0;
		KFConstraintStruct* e2=0;

		testConstraint(
				candidate, e1, e2,
				Sim3(),
				loopclosureStrictness);

		LOG_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo) << " FAR (" << distancesToNewKeyFrame.at(candidate) << ")";

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);
		}
	}



	if(parent != 0 && forceParent)
	{
		KFConstraintStruct* e1=0;
		KFConstraintStruct* e2=0;
		testConstraint(
				parent, e1, e2,
				candidateToFrame_initialEstimateMap[parent],
				100);
		LOG_IF(DEBUG, enablePrintDebugInfo && printConstraintSearchInfo) << " PARENT (0)";

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);
		}
		else
		{
			float downweightFac = 5;
			const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness) / downweightFac;
			LOG(WARNING) << "warning: reciprocal tracking on new frame failed badly, added odometry edge (Hacky).";

			_system.poseConsistencyMutex.lock_shared();
			constraints.push_back(new KFConstraintStruct());
			constraints.back()->firstFrame = newKeyFrame;
			constraints.back()->secondFrame = newKeyFrame->getTrackingParent();
			constraints.back()->secondToFirst = constraints.back()->firstFrame->getScaledCamToWorld().inverse() * constraints.back()->secondFrame->getScaledCamToWorld();
			constraints.back()->information  <<
					0.8098,-0.1507,-0.0557, 0.1211, 0.7657, 0.0120, 0,
					-0.1507, 2.1724,-0.1103,-1.9279,-0.1182, 0.1943, 0,
					-0.0557,-0.1103, 0.2643,-0.0021,-0.0657,-0.0028, 0.0304,
					 0.1211,-1.9279,-0.0021, 2.3110, 0.1039,-0.0934, 0.0005,
					 0.7657,-0.1182,-0.0657, 0.1039, 1.0545, 0.0743,-0.0028,
					 0.0120, 0.1943,-0.0028,-0.0934, 0.0743, 0.4511, 0,
					0,0, 0.0304, 0.0005,-0.0028, 0, 0.0228;
			constraints.back()->information *= (1e9/(downweightFac*downweightFac));

			constraints.back()->robustKernel = new g2o::RobustKernelHuber();
			constraints.back()->robustKernel->setDelta(kernelDelta);

			constraints.back()->meanResidual = 10;
			constraints.back()->meanResidualD = 10;
			constraints.back()->meanResidualP = 10;
			constraints.back()->usage = 0;

			_system.poseConsistencyMutex.unlock_shared();
		}
	}


	// newConstraintMutex.lock();

	//TODO: Surely something should be locked here...

	{
		// std::lock_guard< std::mutex > lock(_system.keyFrameGraph->newConstraintMutex);
		_system.keyFrameGraph->addKeyFrame(newKeyFrame);
		for(unsigned int i=0;i<constraints.size();i++)
			_system.keyFrameGraph->insertConstraint(constraints[i]);
	}

	_system.optThread->doNewConstraint();

	// newConstraintAdded = true;
	// newConstraintCreatedSignal.notify_all();
	// newConstraintMutex.unlock();

	newKFTrackingReference->invalidate();
	candidateTrackingReference->invalidate();

	return constraints.size();
}

float ConstraintSearchThread::tryTrackSim3(
		TrackingReference* A, TrackingReference* B,
		int lvlStart, int lvlEnd,
		bool useSSE,
		Sim3 &AtoB, Sim3 &BtoA,
		KFConstraintStruct* e1, KFConstraintStruct* e2 )
{
	BtoA = constraintTracker->trackFrameSim3(
			A,
			B->keyframe,
			BtoA,
			lvlStart,lvlEnd);
	Matrix7x7 BtoAInfo = constraintTracker->lastSim3Hessian;
	float BtoA_meanResidual = constraintTracker->lastResidual;
	float BtoA_meanDResidual = constraintTracker->lastDepthResidual;
	float BtoA_meanPResidual = constraintTracker->lastPhotometricResidual;
	float BtoA_usage = constraintTracker->pointUsage;


	if (constraintTracker->diverged ||
		BtoA.scale() > 1 / Sophus::SophusConstants<sophusType>::epsilon() ||
		BtoA.scale() < Sophus::SophusConstants<sophusType>::epsilon() ||
		BtoAInfo(0,0) == 0 ||
		BtoAInfo(6,6) == 0)
	{
		return 1e20;
	}


	AtoB = constraintTracker->trackFrameSim3(
			B,
			A->keyframe,
			AtoB,
			lvlStart,lvlEnd);
	Matrix7x7 AtoBInfo = constraintTracker->lastSim3Hessian;
	float AtoB_meanResidual = constraintTracker->lastResidual;
	float AtoB_meanDResidual = constraintTracker->lastDepthResidual;
	float AtoB_meanPResidual = constraintTracker->lastPhotometricResidual;
	float AtoB_usage = constraintTracker->pointUsage;


	if (constraintTracker->diverged ||
		AtoB.scale() > 1 / Sophus::SophusConstants<sophusType>::epsilon() ||
		AtoB.scale() < Sophus::SophusConstants<sophusType>::epsilon() ||
		AtoBInfo(0,0) == 0 ||
		AtoBInfo(6,6) == 0)
	{
		return 1e20;
	}

	// Propagate uncertainty (with d(a * b) / d(b) = Adj_a) and calculate Mahalanobis norm
	Matrix7x7 datimesb_db = AtoB.cast<float>().Adj();
	Matrix7x7 diffHesse = (AtoBInfo.inverse() + datimesb_db * BtoAInfo.inverse() * datimesb_db.transpose()).inverse();
	Vector7 diff = (AtoB * BtoA).log().cast<float>();


	float reciprocalConsistency = (diffHesse * diff).dot(diff);


	if(e1 != 0 && e2 != 0)
	{
		e1->firstFrame = A->keyframe;
		e1->secondFrame = B->keyframe;
		e1->secondToFirst = BtoA;
		e1->information = BtoAInfo.cast<double>();
		e1->meanResidual = BtoA_meanResidual;
		e1->meanResidualD = BtoA_meanDResidual;
		e1->meanResidualP = BtoA_meanPResidual;
		e1->usage = BtoA_usage;

		e2->firstFrame = B->keyframe;
		e2->secondFrame = A->keyframe;
		e2->secondToFirst = AtoB;
		e2->information = AtoBInfo.cast<double>();
		e2->meanResidual = AtoB_meanResidual;
		e2->meanResidualD = AtoB_meanDResidual;
		e2->meanResidualP = AtoB_meanPResidual;
		e2->usage = AtoB_usage;

		e1->reciprocalConsistency = e2->reciprocalConsistency = reciprocalConsistency;
	}

	return reciprocalConsistency;
}


void ConstraintSearchThread::testConstraint(
		Frame* candidate,
		KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
		Sim3 candidateToFrame_initialEstimate,
		float strictness)
{
	candidateTrackingReference->importFrame(candidate);

	Sim3 FtoC = candidateToFrame_initialEstimate.inverse(), CtoF = candidateToFrame_initialEstimate;
	Matrix7x7 FtoCInfo, CtoFInfo;

	float err_level3 = tryTrackSim3(
			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
			SIM3TRACKING_MAX_LEVEL-1, 3,
			USESSE,
			FtoC, CtoF);

	if(err_level3 > 3000*strictness)
	{
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf("FAILE %d -> %d (lvl %d): errs (%.1f / - / -).",
				newKFTrackingReference->frameID, candidateTrackingReference->frameID,
				3,
				sqrtf(err_level3));

		e1_out = e2_out = 0;

		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
		return;
	}

	float err_level2 = tryTrackSim3(
			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
			2, 2,
			USESSE,
			FtoC, CtoF);

	if(err_level2 > 4000*strictness)
	{
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf("FAILE %d -> %d (lvl %d): errs (%.1f / %.1f / -).",
				newKFTrackingReference->frameID, candidateTrackingReference->frameID,
				2,
				sqrtf(err_level3), sqrtf(err_level2));

		e1_out = e2_out = 0;
		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
		return;
	}

	e1_out = new KFConstraintStruct();
	e2_out = new KFConstraintStruct();


	float err_level1 = tryTrackSim3(
			newKFTrackingReference, candidateTrackingReference,	// A = frame; b = candidate
			1, 1,
			USESSE,
			FtoC, CtoF, e1_out, e2_out);

	if(err_level1 > 6000*strictness)
	{
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf("FAILED %d -> %d (lvl %d): errs (%.1f / %.1f / %.1f).",
					newKFTrackingReference->frameID, candidateTrackingReference->frameID,
					1,
					sqrtf(err_level3), sqrtf(err_level2), sqrtf(err_level1));

		delete e1_out;
		delete e2_out;
		e1_out = e2_out = 0;
		newKFTrackingReference->keyframe->trackingFailed.insert(std::pair<Frame*,Sim3>(candidate, candidateToFrame_initialEstimate));
		return;
	}


	if(enablePrintDebugInfo && printConstraintSearchInfo)
		printf("ADDED %d -> %d: errs (%.1f / %.1f / %.1f).",
			newKFTrackingReference->frameID, candidateTrackingReference->frameID,
			sqrtf(err_level3), sqrtf(err_level2), sqrtf(err_level1));


	const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness);
	e1_out->robustKernel = new g2o::RobustKernelHuber();
	e1_out->robustKernel->setDelta(kernelDelta);
	e2_out->robustKernel = new g2o::RobustKernelHuber();
	e2_out->robustKernel->setDelta(kernelDelta);
}


}
