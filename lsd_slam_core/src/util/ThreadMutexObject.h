/*
 * ThreadMutexObject.h
 *
 *  Created on: 11 May 2012
 *      Author: thomas
 */

#pragma once

#include <mutex>
#include <condition_variable>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// #include <boost/thread/condition_variable.hpp>


template <class T>
class MutexObject
{
    public:

      typedef std::lock_guard<std::mutex> lock_guard;

        MutexObject()
        {}

        MutexObject(T initialValue)
         : object(initialValue)
        {}

        void set(const T &newValue)
        {
            lock_guard lock(_mutex);
            object = newValue;
        }

        void operator=( const T &newValue )
        {
          set( newValue );
        }

        void lock( void ) { _mutex.lock(); }
        void unlock( void ) { _mutex.unlock(); }

        std::mutex & mutex()
        {
            return _mutex;
        }

        // std::condition_variable_any &cv( void )
        // { return signal; }

        T &operator()( void )
        {
            return object;
        }

        const T &const_ref(void) const
        {
            return object;
        }

        void assignAndNotifyAll(const T &newValue)
        {
            set( newValue );
            _cv.notify_all();

        }

        void notifyAll()
        {
            // std::lock_guard lock(mutex);
            _cv.notify_all();
        }

        T get()
        {
            lock_guard lock(_mutex);
            return object;
        }

        template< class Rep, class Period >
        void wait_for( const std::chrono::duration<Rep, Period> &dur )
        {
          std::unique_lock<std::mutex> lk(_mutex);
          _cv.wait_for(lk, dur);
        }

        void wait( void )
        {
          std::unique_lock<std::mutex> lock(_mutex);
          _cv.wait(lock);
        }

        template < class Predicate >
        void wait( Predicate pred )
        {
          std::unique_lock<std::mutex> lock(_mutex);
          _cv.wait(lock, pred );
        }

    private:
        T object;
        std::mutex _mutex;
        std::condition_variable_any _cv;
};

template <class T>
class ThreadMutexObject
{
    public:

      typedef std::lock_guard<std::mutex> lock_guard;

        ThreadMutexObject()
        {}

        ThreadMutexObject(T initialValue)
         : object(initialValue),
           lastCopy(initialValue)
        {}

        void assignValue(T newValue)
        {
            lock_guard lock(_mutex);
            object = lastCopy = newValue;
        }

        void set(const T &newValue)
        {
            lock_guard lock(_mutex);
            object = lastCopy = newValue;
        }

        // std::mutex & getMutex()
        // {
        //     return _mutex;
        // }

        void lock( void ) { _mutex.lock(); }
        void unlock( void ) { _mutex.unlock(); }

        std::mutex & mutex()
        {
            return _mutex;
        }

        std::condition_variable_any &cv( void )
        { return signal; }

        T & getReference()
        {
            return object;
        }

        T &operator()( void )
        {
            return object;
        }

        void assignAndNotifyAll(T newValue)
        {
            {
              lock_guard lock(_mutex);
              object = newValue;
            }
            signal.notify_all();

        }

        void notifyAll()
        {
            // std::lock_guard lock(mutex);
            signal.notify_all();
        }

        T getValue()
        {
            lock_guard lock(_mutex);
            lastCopy = object;
            return lastCopy;
        }

        template< class Rep, class Period >
        void wait_for( const std::chrono::duration<Rep, Period> &dur )
        {
          std::unique_lock<std::mutex> lk(_mutex);
          signal.wait_for(lk, dur);
        }

        void wait( void )
        {
          lock_guard lock(_mutex);
          signal.wait(_mutex);
        }

        T waitForSignal()
        {
          lock_guard lock(_mutex);
          signal.wait(_mutex);
          lastCopy = object;
          return lastCopy;
        }

        T getValueWait(int wait = 33000)
        {
            boost::this_thread::sleep(boost::posix_time::microseconds(wait));
            lock_guard lock(_mutex);
            lastCopy = object;
            return lastCopy;
        }

        T & getReferenceWait(int wait = 33000)
        {
            boost::this_thread::sleep(boost::posix_time::microseconds(wait));
            lock_guard lock(_mutex);
            lastCopy = object;
            return lastCopy;
        }

        void operator++(int)
        {
            lock_guard lock(_mutex);
            object++;
        }

    private:
        T object;
        T lastCopy;
        std::mutex _mutex;
        std::condition_variable_any signal;
};

// Simplified version which only handles synchronization (no access to
//  stored boolean value)
class ThreadSynchronizer  {
public:
  ThreadSynchronizer( void )
    : _ready(false)
  {;}

  void lock( void ) { _mutex.lock(); }
  void unlock( void ) { _mutex.unlock(); }

  void notify( void )
  {
      {
        std::lock_guard<std::mutex> lk(_mutex);
        _ready = true;
      }
    _cv.notify_all();
  }

  void reset( void )
  {
    std::unique_lock<std::mutex> lk(_mutex);
    _ready = false;
  }

  // The extra while(!_ready) handles other circumstances which might break the
  // wait (signals?)
  void wait( void )
  {
    std::unique_lock<std::mutex> lk(_mutex);
    while(!_ready) {_cv.wait(lk); }
  }

  template< class Rep, class Period >
  void wait_for( const std::chrono::duration<Rep, Period> &dur )
  {
    {
      std::unique_lock<std::mutex> lk(_mutex);
      _cv.wait_for(lk, dur);
    }
  }

private:

  bool _ready;
  std::mutex _mutex;
  std::condition_variable _cv;

};
