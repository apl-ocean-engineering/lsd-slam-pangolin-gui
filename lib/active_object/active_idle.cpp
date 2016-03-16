/** 2010 by KjellKod.cc. This is PUBLIC DOMAIN to use at your own risk and comes
* with no warranties. This code is yours to share, use and modify with no
* strings attached and no restrictions or obligations.
* ============================================================================
*
* Example of a ActiveIdle Object, using C++0x std::thread mechanisms to make it
* safe for thread communication.
*
* This was originally published at http://sites.google.com/site/kjellhedstrom2/ActiveIdle-object-with-cpp0x
* and inspired from Herb Sutter's C++0x ActiveIdle Object
* http://herbsutter.com/2010/07/12/effective-concurrency-prefer-using-ActiveIdle-objects-instead-of-naked-threads
*
* The code below uses JustSoftware Solutions Inc std::thread implementation
* http://www.justsoftwaresolutions.co.uk
*
* Last update 2011-06-23, by Kjell Hedstrom,
* e-mail: hedstrom at kjellkod dot cc
* linkedin: http://linkedin.com/se/kjellkod */


#include "active.h"

namespace active_object {

ActiveIdle::ActiveIdle( Callback idleCb, const std::chrono::milliseconds timeout )
  : done_(false), idleCb_(idleCb), timeout_( timeout )
{}

ActiveIdle::~ActiveIdle()
{
  Callback quit_token = std::bind(&ActiveIdle::doDone, this);
  send(quit_token); // tell thread to exit
  thd_.join();
}

// Add asynchronously a work-message to queue
void ActiveIdle::send(Callback msg_){
  mq_.push(msg_);
}


// Will wait for msgs if queue is empty
// A great explanation of how this is done (using Qt's library):
// http://doc.qt.nokia.com/stable/qwaitcondition.html
void ActiveIdle::run() {
  while (!done_) {
    Callback func;
    if( mq_.wait_for_pop( func, timeout_ ) )
      func();
    else
      idleCb_();
  }
}

// Factory: safe construction of object before thread start
std::unique_ptr<ActiveIdle> ActiveIdle::createActiveIdle( Callback idleCb, const std::chrono::milliseconds timeout ){
  std::unique_ptr<ActiveIdle> aPtr(new ActiveIdle( idleCb, timeout ));
  aPtr->thd_ = std::thread(&ActiveIdle::run, aPtr.get());
  return aPtr;
}

}
