#include "utils.h"
#include <time.h>

float elapsed_time_in_s()
{
    struct timespec time_struct;

    clock_gettime(CLOCK_MONOTONIC, &time_struct);

    return (time_struct.tv_sec + time_struct.tv_nsec / 1.0e9);
}

inline float constrain(float value, float low, float high)
{
    if (value > high)
        return high;
    else if (value < low)
        return low;
    else
        return value;
}

float difference_wrap_180(float desired, float actual)
{
    float e = desired - actual;
    float abs_e;

    if (e < 0)
        abs_e = -e;
    if (360 - abs_e < abs_e) {
        if (e > 0)
            e = - (360 - e);
        else
            e = 360 + e;
    }
    return e;
}

float difference(float desired, float actual)
{
    return (desired - actual);
}
