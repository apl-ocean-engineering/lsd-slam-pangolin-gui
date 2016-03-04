
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif
#include "util/FileUtils.h"


namespace lsd_slam {


class DataSource {
public:
  DataSource( void )
  {;}

  DataSource( const DataSource & ) = delete;
  DataSource &operator=( const DataSource & ) = delete;

  bool hasDepth( void ) const { return _hasDepth; }
  int numImages( void ) const { return _numImages; }

  virtual int numFrames( void ) const = 0;

  virtual int grab( void ) = 0;

  virtual int getImage( int i, cv::Mat &mat ) = 0;
  virtual int getImage( cv::Mat &mat ) { return getImage( 0, mat ); }


protected:

  int _numImages;
  bool _hasDepth;

};


class ImagesSource : public DataSource {
public:
  ImagesSource( const std::vector<std::string> &paths )
    : _idx( -1 )
  {
    for( std::string pathStr : paths ) {
      fs::path p( pathStr );
      if( fs::is_directory( p ) )
        getdir( p, _paths );
      else if( fs::is_regular_file( p ) )
        _paths.push_back( p );

      _hasDepth = false;
      _numImages = 1;
    }

  }

  virtual int numFrames( void ) const { return _paths.size(); }

  virtual int grab( void )
  {
    ++_idx;

    if( _idx >= _paths.size() ) return -1;

    return 0;
  }

  virtual int getImage( int i, cv::Mat &mat )
  {
    if( i != 0 ) return 0;

    if( _idx >= _paths.size() ) return -1;

    mat = cv::imread( _paths[_idx].string(), CV_LOAD_IMAGE_GRAYSCALE );
    return _idx;
  }


protected:

  std::vector<fs::path> _paths;
  int _idx;

};

#ifdef USE_ZED
class ZedSource : public DataSource {
public:

  ZedSource( sl::zed::Camera *camera, bool doComputeDepth = false, sl::zed::SENSING_MODE mode = sl::zed::RAW )
    :_cam( camera ),
     _mode( mode ),
     _computeDepth( doComputeDepth )

  {
    CHECK( _cam );
    _numImages = 2;
    _hasDepth = true;
  }

  ZedSource( const ZedSource & ) = delete;
  ZedSource &operator=( const ZedSource & ) = delete;

  virtual int numFrames( void ) const { return _cam->getSVONumberOfFrames(); };

  virtual int grab( void )
  {
    if( _cam->grab( _mode, _computeDepth, _computeDepth ) ) {
      LOG( WARNING ) << "Error from Zed::grab";
      return -1;
    }
  }

  virtual int getImage( int i, cv::Mat &mat )
  {
    if( i == 0 )
      mat = sl::zed::slMat2cvMat( _cam->getView( sl::zed::STEREO_LEFT ) );
    else if( i == 1 )
      mat = sl::zed::slMat2cvMat( _cam->getView( sl::zed::STEREO_RIGHT ) );

    return 0;
  }

protected:

  sl::zed::Camera *_cam;
  sl::zed::SENSING_MODE _mode;
  bool _computeDepth;


};
#endif

}
