
#include <fstream>

#include "lsd-slam-pangolin-gui/StateSaver.h"

namespace PangolinGui {
namespace StateSaver {

  /// From the PCD documentation:
  // The header entries must be specified precisely in the above order, that is:
  //     VERSION
  //     FIELDS
  //     SIZE
  //     TYPE
  //     COUNT
  //     WIDTH
  //     HEIGHT
  //     VIEWPOINT
  //     POINTS
  //     DATA

  bool SaveState( const std::string &filename,  const std::map<int, std::shared_ptr<Keyframe> > &keyframes ) {

    LOG(INFO) << "Saving state to " << filename;

     std::ofstream outf( filename );

     if( !outf.is_open() ) return false;

     outf << "VERSION 0.7" << std::endl;
     outf << "FIELDS x y z rgb" << std::endl;
     outf << "SIZE 4 4 4 4" << std::endl;       // Store all fields as floats
     outf << "TYPE F F F F" << std::endl;
     outf << "COUNT 1 1 1 1" << std::endl;

     unsigned int numPoints = 0;

     for( const auto kf_pair : keyframes ) {
       const int kfNum = kf_pair.first;
       const std::shared_ptr<Keyframe> &kf( kf_pair.second );

       // Do something useful with keyframe
       numPoints += kf->numPoints();
     }

     outf << "WIDTH " << numPoints << std::endl;
     outf << "HEIGHT 1" << std::endl;
     outf << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;

     outf << "POINTS " << numPoints << std::endl;
     outf << "DATA ascii" << std::endl;

     // iterate a second time
     int i = 0;
     for( const auto kf_pair : keyframes ) {
       const int kfNum = kf_pair.first;
       const std::shared_ptr<Keyframe> &kf( kf_pair.second );

       for( auto const point : kf->points() ) {

         uint32_t rgb = ((uint32_t)point.color[0] << 16 | (uint32_t)point.color[1] << 8 | (uint32_t)point.color[2]);
         const float rgbFloat = float(rgb);

         outf << point.x << " " << point.y << " " << point.z << " " << rgbFloat << std::endl;

         ++i;
         if( i % 10000 == 0) LOG(INFO) << "Wrote " << i << " points";
       }
     }

     return true;

   }


}
}
