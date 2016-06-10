/** ==========================================================================
* 2010 by KjellKod.cc. This is PUBLIC DOMAIN to use at your own risk and comes
* with no warranties. This code is yours to share, use and modify with no
* strings attached and no restrictions or obligations.
* ============================================================================
*
* Example of a Active Object, using C++0x std::thread mechanisms to make it
* safe for thread communication.
*
* This was originally published at http://sites.google.com/site/kjellhedstrom2/active-object-with-cpp0x
* and inspired from Herb Sutter's C++0x Active Object
* http://herbsutter.com/2010/07/12/effective-concurrency-prefer-using-active-objects-instead-of-naked-threads
*
* The code below uses JustSoftware Solutions Inc std::thread implementation
* http://www.justsoftwaresolutions.co.uk
*
* Last update 2011-06-23, by Kjell Hedstrom,
* e-mail: hedstrom at kjellkod dot cc
* linkedin: http://linkedin.com/se/kjellkod */

#ifndef ACTIVE_H_
#define ACTIVE_H_

#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <chrono>

#include "shared_queue.h"

// n.b. I've re-namespaced this from kjellkod not for nefarious purposes but
// to avoid conflict (or explicitly re-using) the version in g3log
namespace active_object {
typedef std::function<void()> Callback;

class Active {
private:
  Active(const Active&) = delete;
  Active& operator=(const Active&) = delete;

  Active();                               // Construction ONLY through factory createActive();

  void doDone(){done_ = true;}
  void run();

  shared_queue<Callback> mq_;
  std::thread thd_;
  bool done_;  // finished flag to be set through msg queue by ~Active

public:
  virtual ~Active();
  void send(Callback msg_);

  size_t size() const { return mq_.size(); }

  // std::unique_ptr< std::lock_guard< std::mutex > > lock_guard() { return mq_.lock_guard(); }

  static std::unique_ptr<Active> createActive(); // Factory: safe construction & thread start
};

class ActiveIdle {
private:
  ActiveIdle(const ActiveIdle&) = delete;
  ActiveIdle& operator=(const ActiveIdle&) = delete;

  // Construction ONLY through factory createActiveIdle();
  ActiveIdle( Callback idleCb, const std::chrono::milliseconds timeout);

  void doDone(){done_ = true;}
  void run();

  shared_queue<Callback> mq_;
  std::thread thd_;
  bool done_;  // finished flag to be set through msg queue by ~Active
  std::chrono::milliseconds timeout_;

  Callback idleCb_;

public:
  virtual ~ActiveIdle();
  void send(Callback msg_);

    size_t size() const { return mq_.size(); }

  // std::unique_ptr< std::lock_guard< std::mutex > > lock_guard() { return mq_.lock_guard(); }

  static std::unique_ptr<ActiveIdle> createActiveIdle( Callback idleCb, const std::chrono::milliseconds timeout	); // Factory: safe construction & thread start
};

} // end namespace kjellkod



#endif
