#include "motor.hpp"
#include "PID.hpp"
#include <RF24/RF24.h>
#include <stdint.h>

/* Ensure that all debug data fits into this. Approximating to about
 * 10 bytes a variable */
#define MAX_BUFFER_SIZE            330
#define MAX_WRITE_SIZE              32
#define RADIO_BUFFER_WRITE_TIMEOUT   1
#define RADIO_PAYLOAD_SEND_TIMEOUT   3

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
	 * packets/chunks by passing the number of chunks during
	 * initialization */
	int nchunks;
	RF24 *radio;
	enum output_mode
	{
		file,
		radio
	} mode;

public:
	Logger(int fd_in, int nchunks_in);
	Logger(RF24 *radio_in, int nchunks_in);
	~Logger();
	void update(uint16_t fifo_count, float actual_ypr[3],
		    float desired_ypr[3], float throttle,
		    Motor motors[4], PID pids_ypr[3]);
};
