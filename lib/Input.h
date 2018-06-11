#pragma once

#include <string>
#include <vector>
#include <memory>

#include "libvideoio/DataSource.h"

std::shared_ptr<libvideoio::DataSource> makeInput( const std::vector<std::string> &inputs );
