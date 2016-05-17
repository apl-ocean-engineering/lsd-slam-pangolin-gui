// Test the active_idle class

#include "gtest/gtest.h"

#include "active_object/active.h"


using namespace active_object;

class Backgroundable {
public:

	Backgroundable( const std::chrono::milliseconds ms = std::chrono::milliseconds(0) )
		: _active( 0 ), _idle( 0 ),
			_thread( ActiveIdle::createActiveIdle( std::bind(&Backgroundable::onIdle, this), ms ) )
	{}

	~Backgroundable()
	{
		 delete _thread.release();
	}

	void doActive( void ) {
			_thread->send( std::bind(&Backgroundable::onActive, this) );
	}

	unsigned int active( void ) {
		std::lock_guard<std::mutex> lock(_mutex);
		return _active;
	}

	unsigned int idle( void ) {
		std::lock_guard<std::mutex> lock(_mutex);
		return _idle;
	}

private:

	void onIdle( void )
	{
		std::lock_guard<std::mutex> lock(_mutex);
		++_idle;
	}

	void onActive( void )
	{
		std::lock_guard<std::mutex> lock(_mutex);
		++_active;
	}

	unsigned int _active, _idle;

	std::mutex _mutex;
	std::unique_ptr<ActiveIdle> _thread;
};





TEST( ActiveIdleTest, TestConstructorDestructor ) {
	Backgroundable bg;
}

TEST( ActiveIdleTest, TestZeroTimeout ) {
	Backgroundable bg(std::chrono::milliseconds( 0 ) );
	std::this_thread::sleep_for( std::chrono::milliseconds( 525) );

	ASSERT_EQ( bg.active(), 0 );
	ASSERT_EQ( bg.idle(), 0 );
}

TEST( ActiveIdleTest, TestJustIdle ) {
	Backgroundable bg( std::chrono::milliseconds( 100 ) );
	std::this_thread::sleep_for( std::chrono::milliseconds( 550 ) );

	ASSERT_EQ( bg.active(), 0 );
	ASSERT_EQ( bg.idle(), 5 );
}

TEST( ActiveIdleTest, TestActiveAndIdle ) {
	Backgroundable bg( std::chrono::milliseconds( 100 ) );

	std::this_thread::sleep_for( std::chrono::milliseconds( 150 ) );		// One timeout
	bg.doActive();
	std::this_thread::sleep_for( std::chrono::milliseconds( 150 ) );		// One timeout
	bg.doActive();
	std::this_thread::sleep_for( std::chrono::milliseconds( 250 ) );		// Two timeouts

	ASSERT_EQ( bg.active(), 2 );
	ASSERT_EQ( bg.idle(), 4 );
}
