#ifndef PID_hpp
#define PID_hpp

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

	void clear(void);
	PID(float kp_new, float ki_new, float kd_new);
	bool update(float feedback_value);
	void setKp(float proportional_gain);
	void setKi(float integral_gain);
	void setKd(float derivative_gain);
	void setWindup(float windup);
	void setSampleTime(float sample_time);
};

#endif
