/*
 * ThreadMutexObject.h
 *
 *  Created on: 11 May 2012
 *      Author: thomas
 */

#ifndef THREADMUTEXOBJECT_H_
#define THREADMUTEXOBJECT_H_

#include <mutex>
#include <condition_variable>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
// #include <boost/thread/condition_variable.hpp>

template <class T>
class ThreadMutexObject
{
    public:
        ThreadMutexObject()
        {}

        ThreadMutexObject(T initialValue)
         : object(initialValue),
           lastCopy(initialValue)
        {}

        typedef std::lock_guard<std::mutex> scoped_lock;

        void assignValue(T newValue)
        {
            scoped_lock lock(mutex);
            object = lastCopy = newValue;
        }

        std::mutex & getMutex()
        {
            return mutex;
        }

        T & getReference()
        {
            return object;
        }

        void assignAndNotifyAll(T newValue)
        {
            {
              scoped_lock lock(mutex);
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
            scoped_lock lock(mutex);
            lastCopy = object;
            return lastCopy;
        }

        T waitForSignal()
        {
          scoped_lock lock(mutex);
          signal.wait(mutex);
          lastCopy = object;
          return lastCopy;
        }

        T getValueWait(int wait = 33000)
        {
            boost::this_thread::sleep(boost::posix_time::microseconds(wait));
            scoped_lock lock(mutex);
            lastCopy = object;
            return lastCopy;
        }

        T & getReferenceWait(int wait = 33000)
        {
            boost::this_thread::sleep(boost::posix_time::microseconds(wait));
            scoped_lock lock(mutex);
            lastCopy = object;
            return lastCopy;
        }

        void operator++(int)
        {
            scoped_lock lock(mutex);
            object++;
        }

    private:
        T object;
        T lastCopy;
        std::mutex mutex;
        std::condition_variable_any signal;
};

// Simplified version which only handles synchronization (no access to
//  stored boolean value)
class ThreadSynchronizer  {
public:
  ThreadSynchronizer( void )
    : _ready(false)
  {;}

  void notify( void )
  {
      {
        std::lock_guard<std::mutex> lk(_mutex);
        _ready = true;
      }
    _cv.notify_all();
  }

  void wait( void )
  {
    std::unique_lock<std::mutex> lk(_mutex);
    while(!_ready) {_cv.wait(lk); }
  }

private:

  bool _ready;
  std::mutex _mutex;
  std::condition_variable _cv;

};


#endif /* THREADMUTEXOBJECT_H_ */
