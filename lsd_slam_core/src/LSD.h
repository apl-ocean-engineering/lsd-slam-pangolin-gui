

#pragma once

#include <condition_variable>
#include <mutex>

#include "SlamSystem.h"
#include "GUI.h"
#include "util/DataSource.h"


#include "util/Undistorter.h"


extern ThreadMutexObject<bool> lsdDone, guiDone;

extern ThreadSynchronizer startAll, lsdReady, guiReady;

extern GUI *gui;

extern void runGui(lsd_slam::SlamSystem * system );
extern void run(lsd_slam::SlamSystem * system, lsd_slam::DataSource *dataSource, lsd_slam::Undistorter* undistorter );
