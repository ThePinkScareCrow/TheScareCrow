#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#define MAX_MAPPED_DUTY  100.0
#define MAX_PULSE_WIDTH  250   /* Full power is a 2.5ms pulse */
#define MIN_PULSE_WIDTH   50   /* Zero power is a 0.5ms pulse */

class Motor {
	float _power;
	static int _servo_fd;
	const char *_pin_name;
public:
	Motor(const char *pin_name);
	~Motor();
	float get_power();
	bool set_power(float power);
};

#endif
