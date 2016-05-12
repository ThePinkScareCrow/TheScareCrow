#ifndef _UTILS_H_
#define _UTILS_H_

float elapsed_time_in_s();
inline float constrain(float value, float low, float high);
float difference_wrap_180(float desired, float actual);
float difference(float desired, float actual);

#endif
