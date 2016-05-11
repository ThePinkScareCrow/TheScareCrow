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
