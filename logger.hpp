#include "motor.hpp"
#include "PID.hpp"

#include <stdint.h>

#define BUFFER_SIZE 330

class Logger
{
	int fd;
	char data_buffer[BUFFER_SIZE];

public:
	Logger(const char *file_name);
	~Logger();
	void log(uint16_t fifo_count, float actual_ypr[3],
		 float desired_ypr[3], float throttle,
		 Motor *motors[4], PID *pids_ypr[3],
                 PID *rate_pids_ypr[3]);
};
