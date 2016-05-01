#include "motor.hpp"
#include "PID.hpp"
#include <stdint.h>

/* Ensure that all debug data fits into this. Approximating to about
 * 10 bytes a variable */
#define MAX_BUFFER_SIZE 330
#define MAX_WRITE_SIZE   32

class Logger
{
	int fd;
	char data_buffer[MAX_BUFFER_SIZE];
	/* cur_loc points to the location in data_buffer from where
	 * writing has to continue */
	int cur_loc;
	/* Holds the size of the string stored in data_buffer */
	int data_size;
	/* By default, the number of times a packet is written out
	 * through the radio or the number of MAX_WRITE_SIZE chunks
	 * written to file is 1. This can be changed to write more
	 * packets/chunks by passing the frequency during
	 * initialization */
	int freq;

public:
	Logger(int fd_in, int freq_in);
	~Logger();
	void update(uint16_t fifo_count, float actual_ypr[3],
		    float desired_ypr[3], float throttle,
		    Motor motors[4], PID pids_ypr[3]);
};
