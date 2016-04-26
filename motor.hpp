#include "BlackLib/BlackPWM.h"
#include <string>
#include <unistd.h>
#include <iostream>
#include <stdio.h>

#define MIN_MAPPED_DUTY 0.0
#define MAX_MAPPED_DUTY 100.0
#define MAX_POWER_DUTY_PERC 12.5 /* Full power duty = 12.5% */
#define MIN_POWER_DUTY_PERC 2.5  /* Zero power duty =  2.5% */

class Motor {
	BlackLib::BlackPWM *motor;
public:
	Motor(BlackLib::pwmName pin);
	~Motor();
	float get_power();
	bool set_power(float power);
};
