#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

class PID
{
public:
	float Kp;
	float Ki;
	float Kd;

	float sample_time;
	float current_time;
	float last_time;

	float set_point;
	float p_term;
	float i_term;
	float d_term;
	float last_error;
	float last_feedback;
	float int_error;
	float windup_guard;
	float output;

	PID::clear();
	PID::PID(float kp_new, float ki_new, float kd_new);
	bool PID::update(float feedback_value);
	PID::setKp(float proportional_gain);
	PID::setKi(float integral_gain);
	PID::setKd(float derivative_gain);
	PID::setWindup(float windup);
	PID::setSampleTime(float sample_time);
};

#endif
