
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include <g3log/g3log.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "util/FileUtils.h"


namespace lsd_slam {


class DataSource {
public:
  DataSource( void )
    : _fps( 0.0 )
  {;}

  DataSource( const DataSource & ) = delete;
  DataSource &operator=( const DataSource & ) = delete;

  bool hasDepth( void ) const { return _hasDepth; }
  int numImages( void ) const { return _numImages; }

  virtual int numFrames( void ) const = 0;

  virtual bool grab( void ) = 0;

  virtual int getImage( int i, cv::Mat &mat ) = 0;
  virtual int getImage( cv::Mat &mat ) { return getImage( 0, mat ); }

  virtual void getDepth( cv::Mat &mat ) { return; }

  float fps( void ) const { return _fps; }
  void setFPS( float f ) { _fps = f; }


protected:

  int _numImages;
  bool _hasDepth;
  float _fps;

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

  virtual bool grab( void )
  {
    ++_idx;

    if( _idx >= _paths.size() ) return false;

    return true;
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

}
