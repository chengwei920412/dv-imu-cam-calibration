/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifdef _MSC_VER
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#include <aslam/Time.hpp>
#include <aslam/implementation/Time.hpp>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <limits>

#include <boost/thread/mutex.hpp>
#include <boost/io/ios_state.hpp>

/*********************************************************************
 ** Preprocessor
 *********************************************************************/

// Could probably do some better and more elaborate checking
// and definition here.
#define HAS_CLOCK_GETTIME (_POSIX_C_SOURCE >= 199309L)

/*********************************************************************
 ** Namespaces
 *********************************************************************/

namespace aslam {

/*********************************************************************
 ** Variables
 *********************************************************************/

const Duration DURATION_MAX(std::numeric_limits < int32_t > ::max(), 999999999);
const Duration DURATION_MIN(std::numeric_limits < int32_t > ::min(), 0);

const Time TIME_MAX(std::numeric_limits < uint32_t > ::max(), 999999999);
const Time TIME_MIN(0, 1);

/*********************************************************************
 ** Cross Platform Functions
 *********************************************************************/
/*
 * These have only internal linkage to this translation unit.
 * (i.e. not exposed to users of the time classes)
 */
void aslam_walltime(uint32_t& sec, uint32_t& nsec)
{
#ifndef WIN32
#if HAS_CLOCK_GETTIME
  timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  sec = start.tv_sec;
  nsec = start.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  sec = timeofday.tv_sec;
  nsec = timeofday.tv_usec * 1000;
#endif
#else
  // Win32 implementation
  // unless I've missed something obvious, the only way to get high-precision
  // time on Windows is via the QueryPerformanceCounter() call. However,
  // this is somewhat problematic in Windows XP on some processors, especially
  // AMD, because the Windows implementation can freak out when the CPU clocks
  // down to save power. Time can jump or even go backwards. Microsoft has
  // fixed this bug for most systems now, but it can still show up if you have
  // not installed the latest CPU drivers (an oxymoron). They fixed all these
  // problems in Windows Vista, and this API is by far the most accurate that
  // I know of in Windows, so I'll use it here despite all these caveats
  static LARGE_INTEGER cpu_freq, init_cpu_time;
  uint32_t start_sec = 0;
  uint32_t start_nsec = 0;
  if ( ( start_sec == 0 ) && ( start_nsec == 0 ) )
  {
    QueryPerformanceFrequency(&cpu_freq);
    if (cpu_freq.QuadPart == 0) {
      throw NoHighPerformanceTimersException();
    }
    QueryPerformanceCounter(&init_cpu_time);
    // compute an offset from the Epoch using the lower-performance timer API
    FILETIME ft;
    GetSystemTimeAsFileTime(&ft);
    LARGE_INTEGER start_li;
    start_li.LowPart = ft.dwLowDateTime;
    start_li.HighPart = ft.dwHighDateTime;
    // why did they choose 1601 as the time zero, instead of 1970?
    // there were no outstanding hard rock bands in 1601.
#ifdef _MSC_VER
    start_li.QuadPart -= 116444736000000000Ui64;
#else
    start_li.QuadPart -= 116444736000000000ULL;
#endif
    start_sec = (uint32_t)(start_li.QuadPart / 10000000);  // 100-ns units. odd.
    start_nsec = (start_li.LowPart % 10000000) * 100;
  }
  LARGE_INTEGER cur_time;
  QueryPerformanceCounter(&cur_time);
  LARGE_INTEGER delta_cpu_time;
  delta_cpu_time.QuadPart = cur_time.QuadPart - init_cpu_time.QuadPart;
  // todo: how to handle cpu clock drift. not sure it's a big deal for us.
  // also, think about clock wraparound. seems extremely unlikey, but possible
  double d_delta_cpu_time = delta_cpu_time.QuadPart / (double) cpu_freq.QuadPart;
  uint32_t delta_sec = (uint32_t) floor(d_delta_cpu_time);
  uint32_t delta_nsec = (uint32_t) boost::math::round((d_delta_cpu_time-delta_sec) * 1e9);

  int64_t sec_sum = (int64_t)start_sec + (int64_t)delta_sec;
  int64_t nsec_sum = (int64_t)start_nsec + (int64_t)delta_nsec;

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  sec = sec_sum;
  nsec = nsec_sum;
#endif
}
/**
 * @brief Simple representation of the rt library nanosleep function.
 */
int aslam_nanosleep(const uint32_t &sec, const uint32_t &nsec) {
#if defined(WIN32)
  HANDLE timer = NULL;
  LARGE_INTEGER sleepTime;
  sleepTime.QuadPart = -
  static_cast<int64_t>(sec)*10000000LL -
  static_cast<int64_t>(nsec) / 100LL;

  timer = CreateWaitableTimer(NULL, TRUE, NULL);
  if (timer == NULL)
  {
    return -1;
  }

  if (!SetWaitableTimer (timer, &sleepTime, 0, NULL, NULL, 0))
  {
    return -1;
  }

  if (WaitForSingleObject (timer, INFINITE) != WAIT_OBJECT_0)
  {
    return -1;
  }
  return 0;
#else
  timespec req = { sec, nsec };
  return nanosleep(&req, NULL);
#endif
}

/**
 * @brief Go to the wall!
 *
 * @todo Fully implement the win32 parts, currently just like a regular sleep.
 */
bool aslam_wallsleep(uint32_t sec, uint32_t nsec) {
#if defined(WIN32)
  aslam_nanosleep(sec,nsec);
#else
  timespec req = { sec, nsec };
  timespec rem = { 0, 0 };
  while (nanosleep(&req, &rem)) {
    req = rem;
  }
#endif
  return true;
}

/*********************************************************************
 ** Class Methods
 *********************************************************************/

bool Time::useSystemTime() {
  return true;
}

bool Time::isSimTime() {
  return false;
}

bool Time::isSystemTime() {
  return !isSimTime();
}

Time Time::now() {

  Time t;
  aslam_walltime(t.sec, t.nsec);

  return t;
}

void Time::setNow(const Time& new_now) {
  throw std::runtime_error("Unimplemented function");
}

void Time::init() {
}

void Time::shutdown() {
}

bool Time::isValid() {
  return true;
}

bool Time::waitForValid() {
  return waitForValid(aslam::WallDuration());
}

bool Time::waitForValid(const aslam::WallDuration& timeout) {
  return true;
}

std::ostream& operator<<(std::ostream& os, const Time &rhs) {
  boost::io::ios_all_saver s(os);
  os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
  return os;
}

std::ostream& operator<<(std::ostream& os, const Duration& rhs) {
  boost::io::ios_all_saver s(os);
  os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
  return os;
}

bool Time::sleepUntil(const Time& end) {
  if (Time::useSystemTime()) {
    Duration d(end - Time::now());
    if (d > Duration(0)) {
      return d.sleep();
    }

    return true;
  } else {
    Time start = Time::now();
    while ((Time::now() < end)) {
      aslam_nanosleep(0, 1000000);
      if (Time::now() < start) {
        return false;
      }
    }

    return true;
  }
}

bool WallTime::sleepUntil(const WallTime& end) {
  WallDuration d(end - WallTime::now());
  if (d > WallDuration(0)) {
    return d.sleep();
  }

  return true;
}

bool Duration::sleep() const {
  if (Time::useSystemTime()) {
    return aslam_wallsleep(sec, nsec);
  } else {
    Time start = Time::now();
    Time end = start + *this;
    if (start.isZero()) {
      end = TIME_MAX;
    }

    while ((Time::now() < end)) {
      aslam_wallsleep(0, 1000000);

      // If we started at time 0 wait for the first actual time to arrive before starting the timer on
      // our sleep
      if (start.isZero()) {
        start = Time::now();
        end = start + *this;
      }

      // If time jumped backwards from when we started sleeping, return immediately
      if (Time::now() < start) {
        return false;
      }
    }

    return true;
  }
}

std::ostream &operator<<(std::ostream& os, const WallTime &rhs) {
  boost::io::ios_all_saver s(os);
  os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
  return os;
}

WallTime WallTime::now() {
  WallTime t;
  aslam_walltime(t.sec, t.nsec);

  return t;
}

std::ostream &operator<<(std::ostream& os, const WallDuration& rhs) {
  boost::io::ios_all_saver s(os);
  os << rhs.sec << "." << std::setw(9) << std::setfill('0') << rhs.nsec;
  return os;
}

bool WallDuration::sleep() const {
  return aslam_wallsleep(sec, nsec);
}

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec) {
  uint64_t nsec_part = nsec % 1000000000UL;
  uint64_t sec_part = nsec / 1000000000UL;

  if (sec_part > UINT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec += sec_part;
  nsec = nsec_part;
}

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec) {
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t) sec64;
  nsec = (uint32_t) nsec64;
}

void normalizeSecNSecUnsigned(int64_t& sec, int64_t& nsec) {
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part >= 1000000000L) {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > INT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

template class TimeBase<Time, Duration> ;
template class TimeBase<WallTime, WallDuration> ;
}
