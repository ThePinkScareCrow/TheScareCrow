#include "motor.hpp"
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int Motor::_servo_fd;

Motor::Motor(const char *pin_name)
{
	_pin_name = pin_name;

	/* static int _servo_fd will be initialized to 0 */
	if (!_servo_fd)
		_servo_fd = open("/dev/servoblaster", NULL);

	this->set_power(0);
	_power = 0;
}

Motor::~Motor()
{
	this->set_power(0);
}

/*
 * Return the power in the range [0, 100].
 */
float Motor::get_power()
{
	return _power;
}

/*
 * Set the power in the range [0, 100].
 */
bool Motor::set_power(float power)
{
	char buf[10];

	if (power < 0 || power > 100)
		return false;

	int p = _power / MAX_MAPPED_DUTY
		* (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
		+ MIN_PULSE_WIDTH;

	int n = sprintf(buf, "%s=%d",
			_pin_name,
			p);

	write(_servo_fd, buf, n);
}
