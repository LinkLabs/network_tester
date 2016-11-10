#include <stdio.h>
#include <time.h>

#if defined(WIN32) || defined(__MINGW32__)
#include <windows.h>
#elif defined(__APPLE__) || defined(__linux__) || defined(__CYGWIN__)
#include <sys/time.h>
#endif

#include "ll_ifc.h"
#include "ll_ifc_utils.h"

#if defined(WIN32) || defined(__MINGW32__)
#include <windows.h>
// Modified from http://stackoverflow.com/questions/5404277/porting-clock-gettime-to-windows
int32_t gettime(struct time *tp)
{
    __int64 wintime;

    GetSystemTimeAsFileTime((FILETIME*)&wintime);
    wintime -= 116444736000000000ll;  //1jan1601 to 1jan1970
    tp->tv_sec = wintime / 10000000ll;           //seconds
    tp->tv_nsec = wintime % 10000000ll * 100;      //nano-seconds
    return 0;
}
#elif defined(__APPLE__)
int32_t gettime(struct time *tp)
{
    struct timeval now;
    int ret = gettimeofday(&now, NULL);

    if (ret)
    {
        return ret;
    }
    tp->tv_sec = now.tv_sec;
    tp->tv_nsec = now.tv_usec * 1000;

    return 0;
}
#elif defined(__linux__) || defined(__CYGWIN__)
int32_t gettime(struct time *tp)
{
    // WARNING: gettimeofday is subject to clock jitter caused by NTP or other services
    // changing the wall-clock time.
    //
    // Using gettimeofday instead of clock_gettime to simplify the Makefile, so that
    // this code can run on system with glibc versions < 2.17 without having to 
    // link with -lrt because -lrt is not available on OS-X or Windows.
#if 1
    struct timeval now;
    int ret = gettimeofday(&now, NULL);

    if (ret)
    {
        return ret;
    }
    tp->tv_sec = now.tv_sec;
    tp->tv_nsec = now.tv_usec * 1000;

    return 0;
#else
    struct timespec now;
    int32_t ret;

    ret = clock_gettime(CLOCK_MONOTONIC, &now);
    tp->tv_sec = now.tv_sec;
    tp->tv_nsec = now.tv_nsec;
    return ret;
#endif
}
#endif

#if defined(WIN32) || defined (__MINGW32__)
int32_t sleep_ms(int32_t millis)
{
    Sleep(millis);
    return 0;
}
#elif defined(__APPLE__) || defined(__linux__) || defined(__CYGWIN__)
int32_t sleep_ms(int32_t millis)
{
    struct timespec time_sleep;

    time_sleep.tv_sec = millis / 1000;
    time_sleep.tv_nsec = (millis % 1000) * 1000;
    return nanosleep(&time_sleep, 0);
}
#endif
