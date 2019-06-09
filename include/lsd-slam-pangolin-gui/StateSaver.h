#pragma once

#include <string>

#include "lsd-slam-pangolin-gui/GuiKeyframe.h"
#include "util/ThreadMutexObject.h"

namespace PangolinGui {
namespace StateSaver {

    extern bool SaveState( const std::string &filename,  const std::map<int, std::shared_ptr<Keyframe> > &keyframes );

}
}
