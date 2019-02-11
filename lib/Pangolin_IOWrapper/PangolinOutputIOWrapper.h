/*
 * PangolinOutput3DWrapper.h
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#pragma once

#include <IOWrapper/OutputIOWrapper.h>
#include <util/Configuration.h>

#include "GUI.h"

namespace lsd_slam
{


class PangolinOutputIOWrapper : public OutputIOWrapper
{
    public:
        PangolinOutputIOWrapper( GUI & gui);
        virtual ~PangolinOutputIOWrapper();

      	virtual void updateFrameNumber( int );
      	virtual void updateLiveImage( const cv::Mat &img );

    private:
        GUI & _gui;
};
}
