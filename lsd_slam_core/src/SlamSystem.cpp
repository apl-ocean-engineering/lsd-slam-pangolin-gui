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

#include <boost/thread/shared_lock_guard.hpp>

#include "SlamSystem.h"

#include "DataStructures/Frame.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/Sim3Tracker.h"
#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"
#include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "GlobalMapping/g2oTypeSim3Sophus.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
#include <g2o/core/robust_kernel_impl.h>
#include "DataStructures/FrameMemory.h"
#include "deque"

// for mkdir
#include <sys/types.h>
#include <sys/stat.h>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"

using namespace lsd_slam;



SlamSystem::SlamSystem( const Configuration &conf, bool enableSLAM )
: SLAMEnabled(enableSLAM), finalized(false), _perf(), relocalizer( conf ),
	_conf( conf )
{


	// this->width = w;
	// this->height = h;
	// this->K = K;
	trackingIsGood = true;


	currentKeyFrame =  nullptr;
	trackingReferenceFrameSharedPT = nullptr;
	keyFrameGraph = new KeyFrameGraph();
	createNewKeyFrame = false;

	map =  new DepthMap( conf );

	newConstraintAdded = false;
	haveUnmergedOptimizationOffset = false;


	tracker = new SE3Tracker( _conf.slamImage );
	// Do not use more than 4 levels for odometry tracking
	for (int level = 4; level < PYRAMID_LEVELS; ++level)
		tracker->settings.maxItsPerLvl[level] = 0;
	trackingReference = new TrackingReference();
	mappingTrackingReference = new TrackingReference();


	if(SLAMEnabled)
	{
		trackableKeyFrameSearch = new TrackableKeyFrameSearch(keyFrameGraph,conf);
		constraintTracker = new Sim3Tracker( _conf.slamImage );
		constraintSE3Tracker = new SE3Tracker( _conf.slamImage );
		newKFTrackingReference = new TrackingReference();
		candidateTrackingReference = new TrackingReference();
	}
	else
	{
		constraintSE3Tracker = 0;
		trackableKeyFrameSearch = 0;
		constraintTracker = 0;
		newKFTrackingReference = 0;
		candidateTrackingReference = 0;
	}


	outputWrapper = 0;

	keepRunning = true;
	doFinalOptimization = false;
	depthMapScreenshotFlag = false;
	lastTrackingClosenessScore = 0;

	thread_mapping = std::thread(&SlamSystem::mappingThreadLoop, this);

	if(SLAMEnabled)
	{
		thread_constraint_search = std::thread(&SlamSystem::constraintSearchThreadLoop, this);
		thread_optimization = std::thread(&SlamSystem::optimizationThreadLoop, this);
	}

	timeLastUpdate.start();
}


SlamSystem::~SlamSystem()
{
	keepRunning = false;

	// make sure no-one is waiting for something.
	printf("... waiting for SlamSystem's threads to exit\n");
	newFrameMappedSignal.notify_all();
	unmappedTrackedFramesSignal.notify_all();
	newKeyFrameCreatedSignal.notify_all();
	newConstraintCreatedSignal.notify_all();

	thread_mapping.join();
	thread_constraint_search.join();
	thread_optimization.join();
	printf("DONE waiting for SlamSystem's threads to exit\n");

	if(trackableKeyFrameSearch != 0) delete trackableKeyFrameSearch;
	if(constraintTracker != 0) delete constraintTracker;
	if(constraintSE3Tracker != 0) delete constraintSE3Tracker;
	if(newKFTrackingReference != 0) delete newKFTrackingReference;
	if(candidateTrackingReference != 0) delete candidateTrackingReference;

	delete mappingTrackingReference;
	delete map;
	delete trackingReference;
	delete tracker;

	// make shure to reset all shared pointers to all frames before deleting the keyframegraph!
	unmappedTrackedFrames.clear();
	latestFrameTriedForReloc.reset();
	latestTrackedFrame.reset();
	currentKeyFrame.reset();
	trackingReferenceFrameSharedPT.reset();

	// delte keyframe graph
	delete keyFrameGraph;

	FrameMemory::getInstance().releaseBuffes();


	Util::closeAllWindows();
}

void SlamSystem::set3DOutputWrapper(Output3DWrapper* outputWrapper)
{
	this->outputWrapper = outputWrapper;
}

void SlamSystem::mergeOptimizationOffset()
{
	// update all vertices that are in the graph!
	poseConsistencyMutex.lock();

	bool needPublish = false;
	if(haveUnmergedOptimizationOffset)
	{
		keyFrameGraph->keyframesAllMutex.lock_shared();
		for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size(); i++)
			keyFrameGraph->keyframesAll[i]->pose->applyPoseGraphOptResult();
		keyFrameGraph->keyframesAllMutex.unlock_shared();

		haveUnmergedOptimizationOffset = false;
		needPublish = true;
	}

	poseConsistencyMutex.unlock();

	if(needPublish)
		publishKeyframeGraph();
}



void SlamSystem::mappingThreadLoop()
{
	printf("Started mapping thread!\n");
	while(keepRunning)
	{
		if (!doMappingIteration())
		{
			std::unique_lock<std::mutex> lock(unmappedTrackedFramesMutex);
			unmappedTrackedFramesSignal.wait_for(lock,std::chrono::milliseconds(200));	// slight chance of deadlock otherwise
		}


		// By my reading, this mutex lock is unnecessary
		//newFrameMappedMutex.lock();
		newFrameMappedSignal.notify_all();
		//newFrameMappedMutex.unlock();
	}
	printf("Exited mapping thread \n");
}

void SlamSystem::finalize()
{
    finalized = true;

	printf("Finalizing Graph... finding final constraints!!\n");

	lastNumConstraintsAddedOnFullRetrack = 1;
	while(lastNumConstraintsAddedOnFullRetrack != 0)
	{
		doFullReConstraintTrack = true;
		usleep(200000);
	}


	printf("Finalizing Graph... optimizing!!\n");
	doFinalOptimization = true;
	newConstraintMutex.lock();
	newConstraintAdded = true;
	newConstraintCreatedSignal.notify_all();
	newConstraintMutex.unlock();

	while(doFinalOptimization)
	{
		usleep(200000);
	}

	printf("Finalizing Graph... publishing!!\n");
	unmappedTrackedFramesMutex.lock();
	unmappedTrackedFramesSignal.notify_one();
	unmappedTrackedFramesMutex.unlock();

	while(doFinalOptimization)
	{
		usleep(200000);
	}

	std::unique_lock<std::mutex> lock(newFrameMappedMutex);
	newFrameMappedSignal.wait(lock);
	newFrameMappedSignal.wait(lock);

	usleep(200000);
	printf("Done Finalizing Graph.!!\n");
}


void SlamSystem::constraintSearchThreadLoop()
{
	LOG(INFO) << "Started constraint search thread!";

	std::unique_lock<std::mutex> lock(newKeyFrameMutex);
	int failedToRetrack = 0;

	while(keepRunning)
	{
		if(newKeyFrames.size() == 0)
		{
			lock.unlock();
			keyFrameGraph->keyframesForRetrackMutex.lock();
			bool doneSomething = false;
			if(keyFrameGraph->keyframesForRetrack.size() > 10)
			{
				std::deque< Frame* >::iterator toReTrack = keyFrameGraph->keyframesForRetrack.begin() + (rand() % (keyFrameGraph->keyframesForRetrack.size()/3));
				Frame* toReTrackFrame = *toReTrack;

				keyFrameGraph->keyframesForRetrack.erase(toReTrack);
				keyFrameGraph->keyframesForRetrack.push_back(toReTrackFrame);

				keyFrameGraph->keyframesForRetrackMutex.unlock();

				int found = findConstraintsForNewKeyFrames(toReTrackFrame, false, false, 2.0);
				if(found == 0)
					failedToRetrack++;
				else
					failedToRetrack=0;

				if(failedToRetrack < (int)keyFrameGraph->keyframesForRetrack.size() - 5)
					doneSomething = true;
			}
			else
				keyFrameGraph->keyframesForRetrackMutex.unlock();

			lock.lock();

			if(!doneSomething)
			{
				if(enablePrintDebugInfo && printConstraintSearchInfo)
					printf("nothing to re-track... waiting.\n");
				newKeyFrameCreatedSignal.wait_for(lock,std::chrono::milliseconds(500));

			}
		}
		else
		{
			Frame* newKF = newKeyFrames.front();
			newKeyFrames.pop_front();
			lock.unlock();


			{
				Timer timer;

				findConstraintsForNewKeyFrames(newKF, true, true, 1.0);
				failedToRetrack=0;

				_perf.findConstraint.update( timer );
			}

			FrameMemory::getInstance().pruneActiveFrames();
			lock.lock();
		}


		if(doFullReConstraintTrack)
		{
			lock.unlock();
			LOG(INFO) << "Optizing Full Map!";

			int added = 0;
			for(unsigned int i=0;i<keyFrameGraph->keyframesAll.size();i++)
			{
				if(keyFrameGraph->keyframesAll[i]->pose->isInGraph)
					added += findConstraintsForNewKeyFrames(keyFrameGraph->keyframesAll[i], false, false, 1.0);
			}

			LOG(INFO) << "Done optizing Full Map! Added " << added << " constraints.";

			doFullReConstraintTrack = false;

			lastNumConstraintsAddedOnFullRetrack = added;
			lock.lock();
		}



	}

	LOG(INFO) << "Exited constraint search thread";
}

void SlamSystem::optimizationThreadLoop()
{
	printf("Started optimization thread \n");

	while(keepRunning)
	{
		std::unique_lock<std::mutex> lock(newConstraintMutex);
		if(!newConstraintAdded)
			newConstraintCreatedSignal.wait_for(lock,std::chrono::milliseconds(2000));	// slight chance of deadlock otherwise
		newConstraintAdded = false;
		lock.unlock();

		if(doFinalOptimization)
		{
			printf("doing final optimization iteration!\n");
			optimizationIteration(50, 0.001);
			doFinalOptimization = false;
		}
		while(optimizationIteration(5, 0.02));
	}

	printf("Exited optimization thread \n");
}

void SlamSystem::publishKeyframeGraph()
{
	if (outputWrapper != nullptr)
		outputWrapper->publishKeyframeGraph(keyFrameGraph);
}

void SlamSystem::requestDepthMapScreenshot(const std::string& filename)
{
	depthMapScreenshotFilename = filename;
	depthMapScreenshotFlag = true;
}

void SlamSystem::finishCurrentKeyframe()
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("FINALIZING KF %d\n", currentKeyFrame->id());

	map->finalizeKeyFrame();

	if(SLAMEnabled)
	{
		mappingTrackingReference->importFrame(currentKeyFrame.get());
		currentKeyFrame->setPermaRef(mappingTrackingReference);
		mappingTrackingReference->invalidate();

		if(currentKeyFrame->idxInKeyframes < 0)
		{
			keyFrameGraph->keyframesAllMutex.lock();
			currentKeyFrame->idxInKeyframes = keyFrameGraph->keyframesAll.size();
			keyFrameGraph->keyframesAll.push_back(currentKeyFrame.get());
			keyFrameGraph->totalPoints += currentKeyFrame->numPoints;
			keyFrameGraph->totalVertices ++;
			keyFrameGraph->keyframesAllMutex.unlock();

			newKeyFrameMutex.lock();
			newKeyFrames.push_back(currentKeyFrame.get());
			newKeyFrameCreatedSignal.notify_all();
			newKeyFrameMutex.unlock();
		}
	}

	if(outputWrapper!= 0)
		outputWrapper->publishKeyframe(currentKeyFrame.get());
}

void SlamSystem::discardCurrentKeyframe()
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("DISCARDING KF %d\n", currentKeyFrame->id());

	if(currentKeyFrame->idxInKeyframes >= 0)
	{
		printf("WARNING: trying to discard a KF that has already been added to the graph... finalizing instead.\n");
		finishCurrentKeyframe();
		return;
	}


	map->invalidate();

	keyFrameGraph->allFramePosesMutex.lock();
	for(FramePoseStruct* p : keyFrameGraph->allFramePoses)
	{
		if(p->trackingParent != 0 && p->trackingParent->frameID == currentKeyFrame->id())
			p->trackingParent = 0;
	}
	keyFrameGraph->allFramePosesMutex.unlock();

	{
		boost::shared_lock_guard< boost::shared_mutex > lock(keyFrameGraph->idToKeyFrameMutex);
		keyFrameGraph->idToKeyFrame.erase(currentKeyFrame->id());
	}

}

void SlamSystem::createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate)
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("CREATE NEW KF %d from %d\n", newKeyframeCandidate->id(), currentKeyFrame->id());


	if(SLAMEnabled)
	{
		// add NEW keyframe to id-lookup
		keyFrameGraph->idToKeyFrameMutex.lock();
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
		keyFrameGraph->idToKeyFrameMutex.unlock();
	}

	// propagate & make new.
	map->createKeyFrame(newKeyframeCandidate.get());

	if(printPropagationStatistics)
	{

		Eigen::Matrix<float, 20, 1> data;
		data.setZero();
		data[0] = runningStats.num_prop_attempts / ((float)_conf.slamImage.area());
		data[1] = (runningStats.num_prop_created + runningStats.num_prop_merged) / (float)runningStats.num_prop_attempts;
		data[2] = runningStats.num_prop_removed_colorDiff / (float)runningStats.num_prop_attempts;

		outputWrapper->publishDebugInfo(data);
	}

	currentKeyFrameMutex.lock();
	currentKeyFrame = newKeyframeCandidate;
	currentKeyFrameMutex.unlock();
}

void SlamSystem::loadNewCurrentKeyframe(Frame* keyframeToLoad)
{
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("RE-ACTIVATE KF %d\n", keyframeToLoad->id());

	map->setFromExistingKF(keyframeToLoad);

	if(enablePrintDebugInfo && printRegularizeStatistics)
		printf("re-activate frame %d!\n", keyframeToLoad->id());

	currentKeyFrameMutex.lock();
	currentKeyFrame = keyFrameGraph->idToKeyFrame.find(keyframeToLoad->id())->second;
	currentKeyFrame->depthHasBeenUpdatedFlag = false;
	currentKeyFrameMutex.unlock();
}

void SlamSystem::changeKeyframe(bool noCreate, bool force, float maxScore)
{
	Frame* newReferenceKF=0;
	std::shared_ptr<Frame> newKeyframeCandidate = latestTrackedFrame;
	if(doKFReActivation && SLAMEnabled)
	{
		Timer timer;
		newReferenceKF = trackableKeyFrameSearch->findRePositionCandidate(newKeyframeCandidate.get(), maxScore);
		_perf.findReferences.update( timer );
	}

	if(newReferenceKF != 0)
		loadNewCurrentKeyframe(newReferenceKF);
	else
	{
		if(force)
		{
			if(noCreate)
			{
				trackingIsGood = false;
				nextRelocIdx = -1;
				printf("mapping is disabled & moved outside of known map. Starting Relocalizer!\n");
			}
			else
				createNewCurrentKeyframe(newKeyframeCandidate);
		}
	}


	createNewKeyFrame = false;
}

bool SlamSystem::updateKeyframe()
{
	std::shared_ptr<Frame> reference = nullptr;
	std::deque< std::shared_ptr<Frame> > references;

	unmappedTrackedFramesMutex.lock();

	// remove frames that have a different tracking parent.
	while(unmappedTrackedFrames.size() > 0 &&
			(!unmappedTrackedFrames.front()->hasTrackingParent() ||
					unmappedTrackedFrames.front()->getTrackingParent() != currentKeyFrame.get()))
	{
		unmappedTrackedFrames.front()->clear_refPixelWasGood();
		unmappedTrackedFrames.pop_front();
	}

	// clone list
	if(unmappedTrackedFrames.size() > 0)
	{
		for(unsigned int i=0;i<unmappedTrackedFrames.size(); i++)
			references.push_back(unmappedTrackedFrames[i]);

		std::shared_ptr<Frame> popped = unmappedTrackedFrames.front();
		unmappedTrackedFrames.pop_front();
		unmappedTrackedFramesMutex.unlock();

		if(enablePrintDebugInfo && printThreadingInfo)
			printf("MAPPING %d on %d to %d (%d frames)\n", currentKeyFrame->id(), references.front()->id(), references.back()->id(), (int)references.size());

		map->updateKeyframe(references);

		popped->clear_refPixelWasGood();
		references.clear();
	}
	else
	{
		unmappedTrackedFramesMutex.unlock();
		return false;
	}


	if(enablePrintDebugInfo && printRegularizeStatistics)
	{
		Eigen::Matrix<float, 20, 1> data;
		data.setZero();
		data[0] = runningStats.num_reg_created;
		data[2] = runningStats.num_reg_smeared;
		data[3] = runningStats.num_reg_deleted_secondary;
		data[4] = runningStats.num_reg_deleted_occluded;
		data[5] = runningStats.num_reg_blacklisted;

		data[6] = runningStats.num_observe_created;
		data[7] = runningStats.num_observe_create_attempted;
		data[8] = runningStats.num_observe_updated;
		data[9] = runningStats.num_observe_update_attempted;


		data[10] = runningStats.num_observe_good;
		data[11] = runningStats.num_observe_inconsistent;
		data[12] = runningStats.num_observe_notfound;
		data[13] = runningStats.num_observe_skip_oob;
		data[14] = runningStats.num_observe_skip_fail;

		outputWrapper->publishDebugInfo(data);
	}



	if(outputWrapper != 0 && continuousPCOutput && currentKeyFrame != 0)
		outputWrapper->publishKeyframe(currentKeyFrame.get());

	return true;
}


void SlamSystem::addTimingSamples()
{
	map->addTimingSample();

	float sPassed = timeLastUpdate.reset();
	if(sPassed > 1.0f)
	{

		LOGF_IF(INFO, enablePrintDebugInfo && printOverallTiming, "MapIt: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
					map->_perf.update.ms(), map->_perf.update.rate(),
					_perf.trackFrame.ms(), _perf.trackFrame.rate(),
					map->_perf.create.ms()+map->_perf.finalize.ms(), map->_perf.create.rate(),
					_perf.findReferences.ms(), _perf.findReferences.rate(),
					trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
					_perf.optimization.ms(), _perf.optimization.rate(),
					_perf.findConstraint.ms(), _perf.findConstraint.rate() );
	}

}


void SlamSystem::debugDisplayDepthMap()
{


	map->debugPlotDepthMap();
	double scale = 1;
	if(currentKeyFrame != 0 && currentKeyFrame != 0)
		scale = currentKeyFrame->getScaledCamToWorld().scale();
	// debug plot depthmap
	char buf1[200];
	char buf2[200];


	snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d / %d",
			map->_perf.update.ms(), map->_perf.update.rate(),
			_perf.trackFrame.ms(), _perf.trackFrame.rate(),
			currentKeyFrame->numFramesTrackedOnThis, currentKeyFrame->numMappedOnThis, (int)unmappedTrackedFrames.size());

	snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
			100*currentKeyFrame->numPoints/(float)(_conf.slamImage.area()),
			100*tracking_lastGoodPerBad,
			scale,
			tracking_lastResidual,
			100*tracking_lastUsage,
			(int)keyFrameGraph->allFramePoses.size(),
			keyFrameGraph->totalVertices,
			(int)keyFrameGraph->edgesAll.size(),
			1e-6 * (float)keyFrameGraph->totalPoints);


	if(onSceenInfoDisplay)
		printMessageOnCVImage(map->debugImageDepth, buf1, buf2);

	CHECK( map->debugImageDepth.data != NULL );
	outputWrapper->updateDepthImage(map->debugImageDepth.data);

	// int pressedKey = Util::waitKey(1);
	// handleKey(pressedKey);
}


void SlamSystem::takeRelocalizeResult()
{
	Frame* keyframe;
	int succFrameID;
	SE3 succFrameToKF_init;
	std::shared_ptr<Frame> succFrame;
	relocalizer.stop();
	relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);
	assert(keyframe != 0);

	loadNewCurrentKeyframe(keyframe);

	{
		std::lock_guard<std::mutex> lock( currentKeyFrameMutex );
		trackingReference->importFrame(currentKeyFrame.get());
		trackingReferenceFrameSharedPT = currentKeyFrame;
	}

	tracker->trackFrame(
			trackingReference,
			succFrame.get(),
			succFrameToKF_init);

	if(!tracker->trackingWasGood || tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount) < 1-0.75f*(1-MIN_GOODPERGOODBAD_PIXEL))
	{
		LOG_IF(DEBUG, enablePrintDebugInfo && printRelocalizationInfo) << "RELOCALIZATION FAILED BADLY! discarding result.";
		trackingReference->invalidate();
	}
	else
	{
		keyFrameGraph->addFrame(succFrame.get());

		{
			std::lock_guard<std::mutex> lock( unmappedTrackedFramesMutex );
			if(unmappedTrackedFrames.size() < 50)
				unmappedTrackedFrames.push_back(succFrame);
		}

		{
			std::lock_guard<std::mutex> lock( currentKeyFrameMutex );
			createNewKeyFrame = false;
			trackingIsGood = true;
		}
	}
}

bool SlamSystem::doMappingIteration()
{
	if(currentKeyFrame == 0)
		return false;

	if(!doMapping && currentKeyFrame->idxInKeyframes < 0)
	{
		if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
			finishCurrentKeyframe();
		else
			discardCurrentKeyframe();

		map->invalidate();
		LOGF(INFO, "Finished KF %d as Mapping got disabled!\n",currentKeyFrame->id());

		changeKeyframe(true, true, 1.0f);
	}

	mergeOptimizationOffset();
	addTimingSamples();

	if(dumpMap)
	{
		keyFrameGraph->dumpMap(packagePath+"/save");
		dumpMap = false;
	}


	// set mappingFrame
	if(trackingIsGood)
	{
		if(!doMapping)
		{
			//printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
			if(lastTrackingClosenessScore > 1)
				changeKeyframe(true, false, lastTrackingClosenessScore * 0.75);

			if (displayDepthMap || depthMapScreenshotFlag)
				debugDisplayDepthMap();

			return false;
		}


		if (createNewKeyFrame)
		{
			finishCurrentKeyframe();
			changeKeyframe(false, true, 1.0f);


			if (displayDepthMap || depthMapScreenshotFlag)
				debugDisplayDepthMap();
		}
		else
		{
			bool didSomething = updateKeyframe();

			if (displayDepthMap || depthMapScreenshotFlag)
				debugDisplayDepthMap();
			if(!didSomething)
				return false;
		}

		return true;
	}
	else
	{
		// invalidate map if it was valid.
		if(map->isValid())
		{
			if(currentKeyFrame->numMappedOnThisTotal >= MIN_NUM_MAPPED)
				finishCurrentKeyframe();
			else
				discardCurrentKeyframe();

			map->invalidate();
		}

		// start relocalizer if it isnt running already
		if(!relocalizer.isRunning)
			relocalizer.start(keyFrameGraph->keyframesAll);

		// did we find a frame to relocalize with?
		if(relocalizer.waitResult(50))
			takeRelocalizeResult();


		return true;
	}
}


void SlamSystem::gtDepthInit(uchar* image, float* depth, double timeStamp, int id)
{
	LOG(INFO) << "Doing GT initialization!";

	{
		std::lock_guard<std::mutex> lock( currentKeyFrameMutex );

		currentKeyFrame.reset(new Frame(id, _conf, timeStamp, image));
		currentKeyFrame->setDepthFromGroundTruth(depth);

		map->initializeFromGTDepth(currentKeyFrame.get());
		keyFrameGraph->addFrame(currentKeyFrame.get());
	}

	if(doSlam) {
		boost::lock_guard<boost::shared_mutex> lock( keyFrameGraph->idToKeyFrameMutex );

		keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
	}
	if(continuousPCOutput && outputWrapper != 0) outputWrapper->publishKeyframe(currentKeyFrame.get());

	LOG(INFO) << "Done GT initialization!";
}


void SlamSystem::randomInit(uchar* image, double timeStamp, int id)
{
	printf("Doing Random initialization!\n");

	if(!doMapping)
		printf("WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.\n");

	{
		std::lock_guard<std::mutex> lock(currentKeyFrameMutex);

		currentKeyFrame.reset(new Frame(id, _conf, timeStamp, image));
		map->initializeRandomly(currentKeyFrame.get());
		keyFrameGraph->addFrame(currentKeyFrame.get());
	}

	if(doSlam)
	{
		keyFrameGraph->idToKeyFrameMutex.lock();
		keyFrameGraph->idToKeyFrame.insert(std::make_pair(currentKeyFrame->id(), currentKeyFrame));
		keyFrameGraph->idToKeyFrameMutex.unlock();
	}
	if(continuousPCOutput && outputWrapper != 0) outputWrapper->publishKeyframe(currentKeyFrame.get());

	if (displayDepthMap || depthMapScreenshotFlag)
		debugDisplayDepthMap();

	printf("Done Random initialization!\n");

}

void SlamSystem::trackStereoFrame(uchar* image, float *depth, unsigned int frameID, bool blockUntilMapped, double timestamp)
{
	std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, _conf, timestamp, image));
	trackingNewFrame->setDepthFromGroundTruth( depth );
	trackFrame( trackingNewFrame, blockUntilMapped );
}

void SlamSystem::trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp )
{
	std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, _conf, timestamp, image));
	trackFrame( trackingNewFrame, blockUntilMapped );
}

void SlamSystem::trackFrame(std::shared_ptr<Frame> trackingNewFrame, bool blockUntilMapped )
{
	// Create new frame
	// std::shared_ptr<Frame> trackingNewFrame(new Frame(frameID, _conf, timestamp, image));

	if(!trackingIsGood)
	{
		relocalizer.updateCurrentFrame(trackingNewFrame);

		{
			std::lock_guard< std::mutex > lock( unmappedTrackedFramesMutex );
			unmappedTrackedFramesSignal.notify_one();
		}
		return;
	}

	currentKeyFrameMutex.lock();
	bool my_createNewKeyframe = createNewKeyFrame;	// pre-save here, to make decision afterwards.
	if(trackingReference->keyframe != currentKeyFrame.get() || currentKeyFrame->depthHasBeenUpdatedFlag)
	{
		trackingReference->importFrame(currentKeyFrame.get());
		currentKeyFrame->depthHasBeenUpdatedFlag = false;
		trackingReferenceFrameSharedPT = currentKeyFrame;
	}

	FramePoseStruct* trackingReferencePose = trackingReference->keyframe->pose;
	currentKeyFrameMutex.unlock();

	// DO TRACKING & Show tracking result.
	if(enablePrintDebugInfo && printThreadingInfo)
		printf("TRACKING %d on %d\n", trackingNewFrame->id(), trackingReferencePose->frameID);


	SE3 frameToReference_initialEstimate;
	{
		boost::shared_lock_guard<boost::shared_mutex> lock( poseConsistencyMutex );
		frameToReference_initialEstimate = se3FromSim3( trackingReferencePose->getCamToWorld().inverse() * keyFrameGraph->allFramePoses.back()->getCamToWorld());
	}


	Timer timer;

	SE3 newRefToFrame_poseUpdate = tracker->trackFrame(
			trackingReference,
			trackingNewFrame.get(),
			frameToReference_initialEstimate);

	_perf.trackFrame.update( timer );

	tracking_lastResidual = tracker->lastResidual;
	tracking_lastUsage = tracker->pointUsage;
	tracking_lastGoodPerBad = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
	tracking_lastGoodPerTotal = tracker->lastGoodCount / (trackingNewFrame->width(SE3TRACKING_MIN_LEVEL)*trackingNewFrame->height(SE3TRACKING_MIN_LEVEL));


	if(manualTrackingLossIndicated || tracker->diverged || (keyFrameGraph->keyframesAll.size() > INITIALIZATION_PHASE_COUNT && !tracker->trackingWasGood))
	{
		printf("TRACKING LOST for frame %d (%1.2f%% good Points, which is %1.2f%% of available points, %s)!\n",
				trackingNewFrame->id(),
				100*tracking_lastGoodPerTotal,
				100*tracking_lastGoodPerBad,
				tracker->diverged ? "DIVERGED" : "NOT DIVERGED");

		trackingReference->invalidate();

		trackingIsGood = false;
		nextRelocIdx = -1;

		unmappedTrackedFramesMutex.lock();
		unmappedTrackedFramesSignal.notify_one();
		unmappedTrackedFramesMutex.unlock();

		manualTrackingLossIndicated = false;
		return;
	}



	if(plotTracking)
	{
		Eigen::Matrix<float, 20, 1> data;
		data.setZero();
		data[0] = tracker->lastResidual;

		data[3] = tracker->lastGoodCount / (tracker->lastGoodCount + tracker->lastBadCount);
		data[4] = 4*tracker->lastGoodCount / (float)_conf.slamImage.area();
		data[5] = tracker->pointUsage;

		data[6] = tracker->affineEstimation_a;
		data[7] = tracker->affineEstimation_b;
		outputWrapper->publishDebugInfo(data);
	}

	keyFrameGraph->addFrame(trackingNewFrame.get());


	//Sim3 lastTrackedCamToWorld = mostCurrentTrackedFrame->getScaledCamToWorld();
//  mostCurrentTrackedFrame->TrackingParent->getScaledCamToWorld() * sim3FromSE3(mostCurrentTrackedFrame->thisToParent_SE3TrackingResult, 1.0);

	if (outputWrapper != 0)
	{
		outputWrapper->publishTrackedFrame(trackingNewFrame.get());
	}


	// Keyframe selection
	latestTrackedFrame = trackingNewFrame;
	if (!my_createNewKeyframe && currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED)
	{
		Sophus::Vector3d dist = newRefToFrame_poseUpdate.translation() * currentKeyFrame->meanIdepth;
		float minVal = fmin(0.2f + keyFrameGraph->keyframesAll.size() * 0.8f / INITIALIZATION_PHASE_COUNT,1.0f);

		if(keyFrameGraph->keyframesAll.size() < INITIALIZATION_PHASE_COUNT)	minVal *= 0.7;

		lastTrackingClosenessScore = trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage);

		if (lastTrackingClosenessScore > minVal)
		{
			createNewKeyFrame = true;

			LOGF_IF( INFO, enablePrintDebugInfo && printKeyframeSelectionInfo,
							"SELECT %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage));
		}
		else
		{
			LOGF_IF( INFO, enablePrintDebugInfo && printKeyframeSelectionInfo,
							"SKIPPD %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",trackingNewFrame->id(),trackingNewFrame->getTrackingParent()->id(), dist.dot(dist), tracker->pointUsage, trackableKeyFrameSearch->getRefFrameScore(dist.dot(dist), tracker->pointUsage));

		}
	}

	{
		std::lock_guard< std::mutex > lock( unmappedTrackedFramesMutex );
		if(unmappedTrackedFrames.size() < 50 || (unmappedTrackedFrames.size() < 100 && trackingNewFrame->getTrackingParent()->numMappedOnThisTotal < 10))
				unmappedTrackedFrames.push_back(trackingNewFrame);
		unmappedTrackedFramesSignal.notify_one();
	}

	// implement blocking
	if(blockUntilMapped && trackingIsGood)
	{
		std::unique_lock<std::mutex> lock(newFrameMappedMutex);
		while(unmappedTrackedFrames.size() > 0)
		{
			printf("TRACKING IS BLOCKING, waiting for %d frames to finish mapping.\n", (int)unmappedTrackedFrames.size());
			newFrameMappedSignal.wait(lock);
		}
	}
}


float SlamSystem::tryTrackSim3(
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


void SlamSystem::testConstraint(
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

int SlamSystem::findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH)
{
	if(!newKeyFrame->hasTrackingParent()) {
		std::lock_guard<std::mutex> lock( newConstraintMutex );
		keyFrameGraph->addKeyFrame(newKeyFrame);
		newConstraintAdded = true;
		newConstraintCreatedSignal.notify_all();
		return 0;
	}

	if(!forceParent && (newKeyFrame->lastConstraintTrackedCamToWorld * newKeyFrame->getScaledCamToWorld().inverse()).log().norm() < 0.01)
		return 0;


	newKeyFrame->lastConstraintTrackedCamToWorld = newKeyFrame->getScaledCamToWorld();

	// =============== get all potential candidates and their initial relative pose. =================
	std::vector<KFConstraintStruct*> constraints;
	Frame* fabMapResult = 0;
	std::unordered_set<Frame*> candidates = trackableKeyFrameSearch->findCandidates(newKeyFrame, fabMapResult, useFABMAP, closeCandidatesTH);
	std::map< Frame*, Sim3 > candidateToFrame_initialEstimateMap;


	// erase the ones that are already neighbours.
	for(std::unordered_set<Frame*>::iterator c = candidates.begin(); c != candidates.end();)
	{
		if(newKeyFrame->neighbors.find(*c) != newKeyFrame->neighbors.end())
		{
			if(enablePrintDebugInfo && printConstraintSearchInfo)
				printf("SKIPPING %d on %d cause it already exists as constraint.\n", (*c)->id(), newKeyFrame->id());
			c = candidates.erase(c);
		}
		else
			++c;
	}

	std::unordered_map<Frame*, int> distancesToNewKeyFrame;
	{
		boost::shared_lock_guard<boost::shared_mutex> lock(poseConsistencyMutex);
		for (Frame* candidate : candidates)
		{
			Sim3 candidateToFrame_initialEstimate = newKeyFrame->getScaledCamToWorld().inverse() * candidate->getScaledCamToWorld();
			candidateToFrame_initialEstimateMap[candidate] = candidateToFrame_initialEstimate;
		}

		if(newKeyFrame->hasTrackingParent())
			keyFrameGraph->calculateGraphDistancesToFrame(newKeyFrame->getTrackingParent(), &distancesToNewKeyFrame);
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
			if(enablePrintDebugInfo && printConstraintSearchInfo)
				printf("SKIPPING %d on %d (NEAR), cause we already have tried it.\n", (*c)->id(), newKeyFrame->id());
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
			if(enablePrintDebugInfo && printConstraintSearchInfo)
				printf("SKIPPING %d on %d (FAR), cause we already have tried it.\n", farCandidates[i]->id(), newKeyFrame->id());
			farCandidates[i] = farCandidates.back();
			farCandidates.pop_back();
			i--;
		}
	}



	if (enablePrintDebugInfo && printConstraintSearchInfo)
		printf("Final Loop-Closure Candidates: %d / %d close (%d failed, %d inconsistent) + %d / %d far (%d failed, %d inconsistent) = %d\n",
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

		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf(" CLOSE (%d)\n", distancesToNewKeyFrame.at(candidate));

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);

			// delete from far candidates if it's in there.
			for(unsigned int k=0;k<farCandidates.size();k++)
			{
				if(farCandidates[k] == candidate)
				{
					if(enablePrintDebugInfo && printConstraintSearchInfo)
						printf(" DELETED %d from far, as close was successful!\n", candidate->id());

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

		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf(" FAR (%d)\n", distancesToNewKeyFrame.at(candidate));

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
		if(enablePrintDebugInfo && printConstraintSearchInfo)
			printf(" PARENT (0)\n");

		if(e1 != 0)
		{
			constraints.push_back(e1);
			constraints.push_back(e2);
		}
		else
		{
			float downweightFac = 5;
			const float kernelDelta = 5 * sqrt(6000*loopclosureStrictness) / downweightFac;
			printf("warning: reciprocal tracking on new frame failed badly, added odometry edge (Hacky).\n");

			poseConsistencyMutex.lock_shared();
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

			poseConsistencyMutex.unlock_shared();
		}
	}


	newConstraintMutex.lock();

	keyFrameGraph->addKeyFrame(newKeyFrame);
	for(unsigned int i=0;i<constraints.size();i++)
		keyFrameGraph->insertConstraint(constraints[i]);


	newConstraintAdded = true;
	newConstraintCreatedSignal.notify_all();
	newConstraintMutex.unlock();

	newKFTrackingReference->invalidate();
	candidateTrackingReference->invalidate();



	return constraints.size();
}




bool SlamSystem::optimizationIteration(int itsPerTry, float minChange)
{

	Timer timer;

	g2oGraphAccessMutex.lock();

	// lock new elements buffer & take them over.
	newConstraintMutex.lock();
	keyFrameGraph->addElementsFromBuffer();
	newConstraintMutex.unlock();


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

	g2oGraphAccessMutex.unlock();

	if(enablePrintDebugInfo && printOptimizationInfo)
		printf("did %d optimization iterations. Max Pose Parameter Change: %f; avgChange: %f. %s\n", its, maxChange, sumChange / sum,
				maxChange > minChange && its == itsPerTry ? "continue optimizing":"Waiting for addition to graph.");



	_perf.optimization.update( timer );

	return maxChange > minChange && its == itsPerTry;
}

void SlamSystem::optimizeGraph()
{
	std::unique_lock<std::mutex> g2oLock(g2oGraphAccessMutex);
	keyFrameGraph->optimize(1000);
	g2oLock.unlock();
	mergeOptimizationOffset();
}


SE3 SlamSystem::getCurrentPoseEstimate()
{
	SE3 camToWorld = SE3();
	keyFrameGraph->allFramePosesMutex.lock_shared();
	if(keyFrameGraph->allFramePoses.size() > 0)
		camToWorld = se3FromSim3(keyFrameGraph->allFramePoses.back()->getCamToWorld());
	keyFrameGraph->allFramePosesMutex.unlock_shared();
	return camToWorld;
}

Sophus::Sim3f SlamSystem::getCurrentPoseEstimateScale()
{
    Sophus::Sim3f camToWorld = Sophus::Sim3f();
    keyFrameGraph->allFramePosesMutex.lock_shared();
    if(keyFrameGraph->allFramePoses.size() > 0)
        camToWorld = keyFrameGraph->allFramePoses.back()->getCamToWorld().cast<float>();
    keyFrameGraph->allFramePosesMutex.unlock_shared();
    return camToWorld;
}

std::vector<FramePoseStruct*> SlamSystem::getAllPoses()
{
	return keyFrameGraph->allFramePoses;
}
