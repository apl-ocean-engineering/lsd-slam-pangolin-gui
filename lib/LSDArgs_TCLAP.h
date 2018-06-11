#pragma once

#include <memory>

#include "libvideoio/DataSource.h"
#include "libvideoio/Undistorter.h"

namespace lsd_slam {

using namespace libvideoio;

struct LSDArgs {

  LSDArgs( int argc, char **argv );

  std::shared_ptr<DataSource> dataSource;
  std::shared_ptr<Undistorter> undistorter;
  bool doGui;

  bool verbose() { return _verbose; }


private:

  bool _verbose;
};


}
