#include "motor.hpp"

Motor::Motor(BlackLib::pwmName pin)
{
	motor = new BlackLib::BlackPWM(pin);
	/* if new period value is less than the current duty value,
	 * the new period value setting operation won't execute. So
	 * firstly duty value is set to zero for safe steps. Duty can
	 * be set to 0 (the value returned by
	 * BlackLib::BlackPWM::getDutyValue()). This is achieved by
	 * setDutyPercent(100)
	 */
	motor->setDutyPercent(100);
	motor->setPeriodTime(PWM_PERIOD_MS, BlackLib::milisecond);
	this->set_power(0);
}

Motor::~Motor()
{
	this->set_power(0);
	free(this);
}

/*
 * Return the power in the range [0, 100].
 */
float Motor::get_power()
{
	return (motor->getNumericValue() - MIN_POWER_DUTY_PERC)
		* MAX_MAPPED_DUTY / (MAX_POWER_DUTY_PERC - MIN_POWER_DUTY_PERC);
}

/*
 * Set the power in the range [0, 100].
 */
bool Motor::set_power(float power)
{
	if (power < 0 || power > 100)
		return false;

	return motor->setDutyPercent(power / MAX_MAPPED_DUTY * (MAX_POWER_DUTY_PERC
								- MIN_POWER_DUTY_PERC)
				     + MIN_POWER_DUTY_PERC);
}
