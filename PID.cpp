#include "PID.h"

struct timespec time_struct;

long int millis()
{
	clock_gettime(CLOCK_MONOTONIC, &time_struct);
	return time_struct.tv_nsec / 1000000;
}

float calculate_error(float desired, float actual)
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

PID::clear()
{
	set_point = 0.0;
	p_term = 0.0;
	i_term = 0.0;
	d_term = 0.0;
	last_error = 0.0;
	last_feedback = 0.0;
	int_error = 0.0;
	windup_guard = 5.0;
	output = 0.0;
}

PID::PID(float kp_new, float ki_new, float kd_new)
{
	sample_time = 5;
	current_time = millis();
	last_time = current_time;

	this->clear();
	this->setKp(kp_new);
	this->setKi(ki_new);
	this->setKd(kd_new);
}

float PID::update(float feedback_value)
{
	current_time = millis();
        float delta_time = current_time - last_time;

        if (delta_time >= sample_time) {
		error = calculate_error(SetPoint, feedback_value);

		float delta_feedback = calculate_error(last_feedback, feedback_value);

		p_term = Kp * error;
		i_term += Ki * error * delta_time;

		if (i_term < -windup_guard)
			i_term = -windup_guard;
		else (i_term > windup_guard)
			     i_term = windup_guard;

		d_term = 0.0;
		if (delta_time > 0)
			d_term = delta_feedback / delta_time;

		last_time = current_time;
		last_error = error;
		last_feedback = feedback_value;

		output = PTerm + ITerm - (Kd * DTerm);
	}

	return output;
}

PID::setKp(float proportional_gain)
{
        Kp = proportional_gain;
}

PID::setKi(float integral_gain)
{
        Ki = integral_gain;
}

PID::setKd(float derivative_gain)
{
        Kd = derivative_gain;
}

PID::setWindup(float windup)
{
        windup_guard = windup;
}

PID::setSampleTime(float sample_time)
{
        sample_time = sample_time;
}
