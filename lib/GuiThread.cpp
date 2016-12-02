
#include "LSD.h"

#include "Pangolin_IOWrapper/PangolinOutput3DWrapper.h"


using namespace lsd_slam;

GUI *gui = NULL;

void runGui(SlamSystem * system )
{
	gui = new GUI( system->conf() );
	gui->initImages();
	PangolinOutput3DWrapper *outputWrapper = new PangolinOutput3DWrapper( system->conf(), *gui );
	system->set3DOutputWrapper( outputWrapper );

	guiReady.notify();
	startAll.wait();

	while(!pangolin::ShouldQuit())
	{
		if(guiDone.getValue()) break;

		gui->preCall();

		gui->drawKeyframes();

		gui->drawFrustum();

		gui->drawImages();

		gui->postCall();
	}

	guiDone.assignValue(true);

}
