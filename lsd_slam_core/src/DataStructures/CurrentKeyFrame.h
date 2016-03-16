

#pragma once

#include <mutex>

#include "DataStructures/Frame.h"

namespace lsd_slam {


class CurrentKeyFrame  {
public:

	CurrentKeyFrame( void )
		: _ptr( NULL )
	{;}

	CurrentKeyFrame( const CurrentKeyFrame & ) = delete;
	CurrentKeyFrame &operator=( const CurrentKeyFrame & ) = delete;

	const std::shared_ptr<Frame>& set( Frame *f )
	{
		std::lock_guard< std::mutex > lock( _mutex );
		_ptr.reset( f );
	}

	const std::shared_ptr<Frame>& set( const std::shared_ptr<Frame> &f )
	{
		std::lock_guard< std::mutex > lock( _mutex );
		_ptr = f;
	}

	Frame *operator->( void )
	{
		return _ptr.get();
	}

	Frame *get( void )
	{
		return _ptr.get();
	}

	std::mutex &mutex( void )
	{
		return _mutex;
	}

	std::shared_ptr<Frame> &ptr( void )
	{ return _ptr; }


	// Convenience functions
	bool empty( void ) const { return _ptr.get() == NULL; }

private:

	std::shared_ptr<Frame> _ptr;
	std::mutex _mutex;


};

}
