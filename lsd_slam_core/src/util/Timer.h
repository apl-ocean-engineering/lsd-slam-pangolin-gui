

#pragma once

#include <sys/time.h>

struct Timer {

  Timer( void )
  {
    start();
  }

  void start( void )
  {
    gettimeofday( &_start, NULL );
  }

  float reset( void )
  {
    float dt = stop();
    start();
    return dt;
  }

  float stop( void ) const
  {
    struct timeval end;
    gettimeofday( &end, NULL );

    return float( end.tv_sec - _start.tv_sec ) + float( end.tv_usec - _start.tv_usec) / 1e6;
  }

  struct timeval _start;

};
