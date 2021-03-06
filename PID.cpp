#include "PID.hpp"
#include <iostream>
#include "utils.h"

using namespace std;

#define DEBUG_PID_OUTPUT 0

void PID::clear(void)
{
	set_point = 0.0;
	p_term = 0.0;
	i_term = 0.0;
	d_term = 0.0;
	last_error = 0.0;
	last_feedback = 0.0;
	windup_guard = 5.0;
	output = 0.0;
}

PID::PID(float (*error_func)(float desired, float actual),
         float kp_new, float ki_new, float kd_new)
{
	current_time = elapsed_time_in_s();
	last_time = current_time;

	this->clear();
        this->calculate_error = error_func;
	this->setKp(kp_new);
	this->setKi(ki_new);
	this->setKd(kd_new);
}

void PID::update(float desired_value, float feedback_value)
{
	current_time = elapsed_time_in_s();
        delta_time = current_time - last_time;

	set_point = desired_value;

        error = (*calculate_error)(set_point, feedback_value);
        delta_feedback = (*calculate_error)(last_feedback, feedback_value);

        p_term = Kp * error;
        i_term += Ki * error * delta_time;

        if (i_term < -windup_guard)
            i_term = -windup_guard;
        else if (i_term > windup_guard)
            i_term = windup_guard;

        d_term = 0.0;
        if (delta_time > 0)
            d_term = -Kd * (delta_feedback / delta_time);

        last_time = current_time;
        last_error = error;
        last_feedback = feedback_value;

        output = p_term + i_term + d_term;

#if DEBUG_PID_OUTPUT

        cout << p_term << " " << i_term << " " << d_term << " " << error;

#endif
}

void PID::setKp(float proportional_gain)
{
        Kp = proportional_gain;
}

void PID::setKi(float integral_gain)
{
        Ki = integral_gain;
}

void PID::setKd(float derivative_gain)
{
        Kd = derivative_gain;
}

void PID::setWindup(float windup)
{
        windup_guard = windup;
}
