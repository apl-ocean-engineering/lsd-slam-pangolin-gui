

#pragma once


struct RollingAverage {

  RollingAverage( const float alpha = 0.1 )
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

struct MsAverage : public RollingAverage {

  MsAverage( const float alpha = 0.2 )
    : RollingAverage( alpha )
  {;}

  void update( const struct timeval &tv_start, const struct timeval &tv_end )
  {
    RollingAverage::update( (tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f );
  }

};

struct CountRateAverage : public RollingAverage {
  CountRateAverage( const float alpha = 0.1 )
    : RollingAverage( alpha )
  {;}

  void increment( void )
  { ++_count; }

  void update( float dt )
  {
    RollingAverage::update( _count / dt );
    _count = 0;
  }

protected:
  int _count;
};
