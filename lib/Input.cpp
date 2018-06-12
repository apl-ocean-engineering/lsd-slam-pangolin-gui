
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Input.h"

#ifdef VIDEOIO_GO
#include <libvideoio-go/GoSource.h>
#endif

using namespace libvideoio;

namespace lsd_slam {

std::shared_ptr<ImageSource> Input::makeInput( const std::vector<std::string> &inputs ) {

  if( inputs.size() == 0 ) return nullptr;

  // Convert first input to files
  fs::path asPath( inputs.front() );

  if( !exists(asPath) ) {
    LOG(WARNING) << "Input file " << asPath << " doesn't exist";
    return nullptr;
  }

  if( asPath.extension() == ".json" || asPath.extension() == ".mov" ) {
#ifdef VIDEOIO_GO
  auto src( new GoSource( asPath.string() ) );

  if( src->isOpened() ) {
    LOG(INFO) << " Success opening " << asPath << " as a Go source";
    return std::shared_ptr<ImageSource>(src);
  }
#else
    LOG(FATAL) << "Trying to open .json or .mov without Videoio-go installed.";
#endif
}



  // Attempt to intuit the input type from the first  entry in the vector
  return std::shared_ptr<ImageSource>(new ImageFilesSource( inputs ));

  //  return std::shared_ptr<DataSource>( nullptr );
}

}
