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
		perror("Logger");
		exit(EXIT_FAILURE);
	}
}

Logger::~Logger()
{
	close(fd);
}

void Logger::log(uint16_t fifo_count, float actual_ypr[3],
		 float desired_ypr[3], float throttle,
		 Motor *motors[4], PID *stab_pids_ypr[3],
                 PID *rate_pids_ypr[3])
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

                       /* Stab PID */
                       "%.3f %.3f %.3f %.2f "  /* Yaw P, I, D windup   */
		       "%.3f %.3f %.3f %.2f "  /* Pitch P, I, D windup */
		       "%.3f %.3f %.3f %.2f "  /* Roll P, I, D windup  */

		       "%.3f %.3f %.3f"        /* P, I, D terms    */
		       "%.3f %.3f %.3f"        /* P, I, D terms    */
		       "%.3f %.3f %.3f"        /* P, I, D terms    */

                       /* Rate PID */
                       "%.3f %.3f %.3f %.2f "  /* Yaw P, I, D windup   */
		       "%.3f %.3f %.3f %.2f "  /* Pitch P, I, D windup */
		       "%.3f %.3f %.3f %.2f "  /* Roll P, I, D windup  */

		       "%.3f %.3f %.3f"        /* P, I, D terms    */
		       "%.3f %.3f %.3f"        /* P, I, D terms    */
		       "%.3f %.3f %.3f\n"        /* P, I, D terms    */
		       ,
		       elapsed_time_in_s(),
		       fifo_count,

		       actual_ypr[0], actual_ypr[1], actual_ypr[2],
		       desired_ypr[0], desired_ypr[1], desired_ypr[2],

		       throttle,

		       motors[0]->get_power(), motors[1]->get_power(),
		       motors[2]->get_power(), motors[3]->get_power(),

		       stab_pids_ypr[0]->Kp, stab_pids_ypr[0]->Ki,
		       stab_pids_ypr[0]->Kd, stab_pids_ypr[0]->windup_guard,

		       stab_pids_ypr[1]->Kp, stab_pids_ypr[1]->Ki,
		       stab_pids_ypr[1]->Kd, stab_pids_ypr[1]->windup_guard,

		       stab_pids_ypr[2]->Kp, stab_pids_ypr[2]->Ki,
		       stab_pids_ypr[2]->Kd, stab_pids_ypr[2]->windup_guard,


		       stab_pids_ypr[0]->p_term, stab_pids_ypr[0]->i_term, stab_pids_ypr[0]->d_term,
		       stab_pids_ypr[1]->p_term, stab_pids_ypr[1]->i_term, stab_pids_ypr[1]->d_term,
		       stab_pids_ypr[2]->p_term, stab_pids_ypr[2]->i_term, stab_pids_ypr[2]->d_term,

                       rate_pids_ypr[0]->Kp, rate_pids_ypr[0]->Ki,
		       rate_pids_ypr[0]->Kd, rate_pids_ypr[0]->windup_guard,

		       rate_pids_ypr[1]->Kp, rate_pids_ypr[1]->Ki,
		       rate_pids_ypr[1]->Kd, rate_pids_ypr[1]->windup_guard,

		       rate_pids_ypr[2]->Kp, rate_pids_ypr[2]->Ki,
		       rate_pids_ypr[2]->Kd, rate_pids_ypr[2]->windup_guard,


		       rate_pids_ypr[0]->p_term, rate_pids_ypr[0]->i_term, rate_pids_ypr[0]->d_term,
		       rate_pids_ypr[1]->p_term, rate_pids_ypr[1]->i_term, rate_pids_ypr[1]->d_term,
		       rate_pids_ypr[2]->p_term, rate_pids_ypr[2]->i_term, rate_pids_ypr[2]->d_term

		);

	while (n < size) {
		n += write(fd, data_buffer + n, size - n);
	}
}
