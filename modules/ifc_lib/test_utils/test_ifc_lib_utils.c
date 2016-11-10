#include "ll_ifc.h"

#include <limits.h>
#include <string.h> // memset, memcpy
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <time.h>
#include <sys/time.h>
#include "cmockery.h"

struct time sudo_time = { 0 , 0 };

int32_t gettime(struct time *tp)
{
    tp->tv_sec  = sudo_time.tv_sec;
    tp->tv_nsec = sudo_time.tv_nsec;
}

void setUpRandom(void **state)
{
//    srand(time(NULL));
}

void tearDown(void **state)
{
    // Nothing to do
}

void test_utils_difftime_random_positive(void **state)
{
    for (unsigned int i = 0; i < 100; i++)
    {
        struct time now = {
            .tv_sec = rand() % LONG_MAX,
            .tv_nsec = rand() % LONG_MAX
        };

        struct time then = {
            .tv_sec = rand() % LONG_MAX,
            .tv_nsec = rand() % LONG_MAX
        };

        long sec;
        long nsec;

        if (then.tv_sec < now.tv_sec)
        {
            sec  = then.tv_sec - now.tv_sec;
            nsec = then.tv_nsec - now.tv_nsec;
            if (nsec < 0)
            {
                sec --;
            }
            nsec = (nsec + 1000) % 1000;

            assert_int_equal(ll_difftime(&now, &then), sec);
        }
        else
        {
            sec  = now.tv_sec - then.tv_sec;
            nsec = now.tv_nsec - then.tv_nsec;
            if (nsec < 0)
            {
                sec --;
            }
            nsec = (nsec + 1000) % 1000;

            assert_int_equal(ll_difftime(&then, &now), sec);
        }

        // printf("Passed iteration %i [ difftime = %i || now (%i:%i) || then (%i:%i) ]\n", i + 1, sec, now.tv_sec, now.tv_nsec, then.tv_sec, then.tv_nsec);
    }
}

void test_utils_difftime_random_negative(void **state)
{
    for (unsigned int i = 0; i < 100; i++)
    {
        struct time now = {
            .tv_sec = LONG_MIN + (rand() % LONG_MAX),
            .tv_nsec = LONG_MIN + (rand() % LONG_MAX)
        };

        struct time then = {
            .tv_sec = rand() % LONG_MAX,
            .tv_nsec = rand() % LONG_MAX
        };

        long sec;
        long nsec;

        if (then.tv_sec < now.tv_sec)
        {
            sec  = then.tv_sec - now.tv_sec;
            nsec = then.tv_nsec - now.tv_nsec;
            if (nsec < 0)
            {
                sec --;
            }
            nsec = (nsec + 1000) % 1000;

            assert_int_equal(ll_difftime(&now, &then), sec);
        }
        else
        {
            sec  = now.tv_sec - then.tv_sec;
            nsec = now.tv_nsec - then.tv_nsec;
            if (nsec < 0)
            {
                sec --;
            }
            nsec = (nsec + 1000) % 1000;

            assert_int_equal(ll_difftime(&then, &now), sec);
        }

        // printf("Passed iteration %i [ difftime = %i || now (%i:%i) || then (%i:%i) ]\n", i + 1, sec, now.tv_sec, now.tv_nsec, then.tv_sec, then.tv_nsec);
    }
}

void test_utils_difftime_limits(void **state)
{

}

void test_utils_difftime_now_random_positive(void **state)
{
    for (unsigned int i = 0; i < 100; i++)
    {
        struct time time1 = {
            .tv_sec = rand() % LONG_MAX,
            .tv_nsec = rand() % LONG_MAX
        };

        struct time time2 = {
            .tv_sec = rand() % LONG_MAX,
            .tv_nsec = rand() % LONG_MAX
        };

        long sec;
        long nsec;

        if (time1.tv_sec < time2.tv_sec)
        {
            sudo_time.tv_sec = time2.tv_sec;
            sudo_time.tv_nsec = time2.tv_nsec;

            sec = time1.tv_sec - sudo_time.tv_sec;
            nsec = time1.tv_nsec - sudo_time.tv_nsec;
        }
        else
        {
            sudo_time.tv_sec = time1.tv_nsec;
            sudo_time.tv_nsec = time1.tv_nsec;

            sec = time2.tv_sec - sudo_time.tv_sec;
            nsec = time2.tv_nsec - sudo_time.tv_nsec;
        }

        if (nsec < 0)
        {
            sec --;
        }
        nsec = (nsec + 1000) % 1000;

        if (time1.tv_sec < time2.tv_sec)
        {
            assert_int_equal(ll_difftime_from_now(&time1), sec);
            // printf("Passed iteration %i [ difftime = %i || now (%i:%i) || then (%i:%i) ]\n", i + 1, sec, sudo_time.tv_sec, sudo_time.tv_nsec, time1.tv_sec, time1.tv_nsec);
        }
        else
        {
            assert_int_equal(ll_difftime_from_now(&time2), sec);
            // printf("Passed iteration %i [ difftime = %i || now (%i:%i) || then (%i:%i) ]\n", i + 1, sec, sudo_time.tv_sec, sudo_time.tv_nsec, time2.tv_sec, time2.tv_nsec);
        }
    }
}

void test_utils_difftime_now_random_negative(void **state)
{
    for (unsigned int i = 0; i < 100; i++)
    {
        struct time time1 = {
            .tv_sec = LONG_MIN + (rand() % LONG_MAX),
            .tv_nsec = LONG_MIN + (rand() % LONG_MAX)
        };

        struct time time2 = {
            .tv_sec = LONG_MIN + (rand() % LONG_MAX),
            .tv_nsec = LONG_MIN + (rand() % LONG_MAX)
        };

        long sec;
        long nsec;

        if (time1.tv_sec < time2.tv_sec)
        {
            sudo_time.tv_sec = time1.tv_sec;
            sudo_time.tv_nsec = time1.tv_nsec;

            sec = time2.tv_sec - sudo_time.tv_sec;
            nsec = time2.tv_nsec - sudo_time.tv_nsec;
        }
        else
        {
            sudo_time.tv_sec = time2.tv_nsec;
            sudo_time.tv_nsec = time2.tv_nsec;

            sec = time1.tv_sec - sudo_time.tv_sec;
            nsec = time1.tv_nsec - sudo_time.tv_nsec;
        }

        if (nsec < 0)
        {
            sec --;
        }
        nsec = (nsec + 1000) % 1000;

        if (time1.tv_sec < time2.tv_sec)
        {
            assert_int_equal(ll_difftime_from_now(&time2), sec);
            // printf("Passed iteration %i [ difftime = %i || now (%i:%i) || then (%i:%i) ]\n", i + 1, sec, sudo_time.tv_sec, sudo_time.tv_nsec, time1.tv_sec, time1.tv_nsec);
        }
        else
        {
            assert_int_equal(ll_difftime_from_now(&time1), sec);
            // printf("Passed iteration %i [ difftime = %i || now (%i:%i) || then (%i:%i) ]\n", i + 1, sec, sudo_time.tv_sec, sudo_time.tv_nsec, time2.tv_sec, time2.tv_nsec);
        }
    }
}

void test_utils_difftime_now_limits(void **state)
{

}
