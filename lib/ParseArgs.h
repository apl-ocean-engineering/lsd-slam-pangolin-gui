#pragma once

#include <memory>

#include "util/DataSource.h"
#include "util/Undistorter.h"

namespace lsd_slam {

struct ParseArgs {

  ParseArgs( int argc, char **argv );

  std::shared_ptr<DataSource> dataSource;
  std::shared_ptr<Undistorter> undistorter;
  bool doGui;

};


}
