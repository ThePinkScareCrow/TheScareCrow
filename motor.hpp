#include "BlackLib/BlackPWM.h"
#include <string>
#include <unistd.h>
#include <iostream>
#include <stdio.h>

#define MAX_MAPPED_DUTY     100.0
#define PWM_PERIOD_MS         4.0
#define MAX_POWER_DUTY_PERC   2.5 / PWM_PERIOD_MS * 100 /* Full power is a 2.5ms pulse */
#define MIN_POWER_DUTY_PERC   0.5 / PWM_PERIOD_MS * 100 /* Zero power is a 0.5ms pulse */

class Motor {
	BlackLib::BlackPWM *motor;
public:
	Motor(BlackLib::pwmName pin);
	~Motor();
	float get_power();
	bool set_power(float power);
};
