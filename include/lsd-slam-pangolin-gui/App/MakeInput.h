#pragma once

#include <string>
#include <vector>
#include <memory>

#include "libvideoio/ImageSource.h"

namespace lsd_slam {

  namespace input {

    extern std::shared_ptr<libvideoio::ImageSource> MakeInput( const std::vector<std::string> &inputs );

  }

}
