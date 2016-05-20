#include "logger.hpp"
#include "utils.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>


Logger::Logger(const char *file_name)
{
	// fd = 1;
	fd = open(file_name, 0);
	if (fd == -1) {
		perror("Logger: ");
		exit(EXIT_FAILURE);
	}
}

Logger::~Logger()
{
	close(fd);
}

void Logger::log(uint16_t fifo_count, float actual_ypr[3],
		 float desired_ypr[3], float throttle,
		 Motor *motors[4], PID *pids_ypr[3])
{
	int size;
	int n = 0;

	size = sprintf(data_buffer,
		       "%.3f "
		       "%d "
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

	while (n < size) {
		n += write(fd, data_buffer + n, size - n);
	}
}
