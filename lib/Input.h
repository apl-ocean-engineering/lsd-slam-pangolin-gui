#pragma once

#include <string>
#include <vector>
#include <memory>

#include "libvideoio/ImageSource.h"

namespace lsd_slam {

  class Input {
  public:

    Input() = delete;
    Input(const Input &) = delete;

    static std::shared_ptr<libvideoio::ImageSource> makeImageSource( const std::vector<std::string> &inputs );

  };

}
