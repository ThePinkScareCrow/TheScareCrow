#include "logger.hpp"
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

Logger::Logger(int fd_in, int freq_in = 1)
{
	fd = fd_in;
	freq = freq_in;
	cur_loc = 0;
	mode = file;
}

Logger::Logger(RF24 *radio_in, int freq_in = 1)
{
	radio = radio_in;
	freq = freq_in;
	cur_loc = 0;
	mode = radio;
}

Logger::~Logger()
{
}

void Logger::update(uint16_t fifo_count, float actual_ypr[3],
		    float desired_ypr[3], float throttle,
		    Motor motors[4], PID pids_ypr[3])
{
	if (data_size - cur_loc > 0) {
		data_size = sprintf(data_buffer,
				    "%.3f "
				    "%.3f "
				    "%.3f %.3f %.3f "       /* Actual angles    */
				    "%.3f %.3f %.3f "       /* Desired angles   */
				    "%.1f "                 /* Throttle         */
				    "%.2f %.2f %.2f %.2f "  /* Motor outputs    */
				    "%.3f %.3f %.3f %.2f "  /* Yaw PID windup   */
				    "%.3f %.3f %.3f %.2f "  /* Pitch PID windup */
				    "%.3f %.3f %.3f %.2f "  /* Roll PID windup  */
				    "%.3f %.3f %.3f"        /* P, I, D terms    */
				    "%.3f %.3f %.3f"        /* P, I, D terms    */
				    "%.3f %.3f %.3f"        /* P, I, D terms    */
				    ,
				    elapsed_time_in_s(),
				    fifo_count,

				    actual_ypr[0], actual_ypr[1], actual_ypr[2],
				    desired_ypr[0], desired_ypr[1], desired_ypr[2],

				    throttle,

				    motors[0]->get_power(), motors[1]->get_power(),
				    motors[2]->get_power(), motors[3]->get_power(),

				    pids_ypr[0]->Kp, pids_ypr[0]->Ki,
				    pids_ypr[0]->Kd, pids_ypr[0]->windup_guard,

				    pids_ypr[1]->Kp, pids_ypr[1]->Ki,
				    pids_ypr[1]->Kd, pids_ypr[1]->windup_guard,

				    pids_ypr[2]->Kp, pids_ypr[2]->Ki,
				    pids_ypr[2]->Kd, pids_ypr[2]->windup_guard,


				    pids_ypr[0]->p_term, pids_ypr[0]->i_term, pids_ypr[0]->d_term,
				    pids_ypr[1]->p_term, pids_ypr[1]->i_term, pids_ypr[1]->d_term,
				    pids_ypr[2]->p_term, pids_ypr[2]->i_term, pids_ypr[2]->d_term
			);

		cur_loc = 0;	/* Start writing from beginning of new string */
	}

	for (int i = 0; i < freq; i++) {
		int size_to_write = (data_size - (cur_loc));
		if (size_to_write > MAX_WRITE_SIZE)
			size_to_write = MAX_WRITE_SIZE;

		if (mode == file) {
			int n = write(fd, data_buffer + cur_loc,
				      size_to_write);
			cur_loc += n;
		}
		else {
			bool write_ok = (radio->writeBlocking(data_buffer + cur_loc,
							size_to_write,
							RADIO_BUFFER_WRITE_TIMEOUT));
			if (write_ok && txStandBy(RADIO_PAYLOAD_SEND_TIMEOUT))
				cur_loc += size_to_write;
		}
	}
	if (mode == file)
		fsync(fd);
}
