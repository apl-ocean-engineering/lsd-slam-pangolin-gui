

#pragma once

#include "Timer.h"

struct MovingAverage {

  MovingAverage( const float alpha = 0.1 )
    :  _value(0), _alpha( alpha )
  {;}

  float value( void ) const { return _value; }
  float operator()( void ) const { return _value; }

protected:

  void update( float v )
  {
    _value = (1-_alpha)*_value + _alpha*v;
  }

  float _value;
  const float _alpha;
};

struct MsAverage : public MovingAverage {

  MsAverage( const float alpha = 0.1 )
    : MovingAverage( alpha )
  {;}

  void update( const Timer &timer )
  {
    MovingAverage::update( timer.stop() * 1000.0f );
  }

protected:
  // Deprecated
  void update( const struct timeval &tv_start, const struct timeval &tv_end )
  {
    MovingAverage::update( (tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f );
  }

};

struct CountRateAverage : public MovingAverage {
  CountRateAverage( const float alpha = 0.2 )
    : MovingAverage( alpha )
  {;}

  void increment( void )
  { ++_count; }

  void update( float dt )
  {
    MovingAverage::update( _count / dt );
    _count = 0;
  }

protected:
  int _count;
};

struct MsRateAverage {
  MsRateAverage( void )
  {;}

  void update( const Timer &timer )
  {
    _ms.update( timer );
    _rate.increment();
  }

  float ms( void ) const { return _ms.value(); }

  float rate( void  )
  {
    float dt = _timer.stop();
    if( dt > 1.0 ) {
      _timer.reset();
      _rate.update( dt );
    }

    return _rate.value();
  }

protected:

  MsAverage _ms;
  CountRateAverage _rate;
  Timer _timer;

};
