#include "MPU6050_6Axis_MotionApps20.h"
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <ctime>
#include <RF24/RF24.h>
#include "PID.hpp"
#include "motor.hpp"
#include <string.h>
#include <stdlib.h>
#include "BlackLib/BlackPWM.h"

#define DEBUG_MODE_MOTORS 0
#define DEBUG_MODE 0
#define DEBUG_MODE_WITH_PID 0

/*MPU Config */
#define MAX_PACKETS_IN_BUFFER 3
#define MAX_BUFFER_SIZE 1024

/*Radio Config*/
#define MAX_RADIO_MSG_SIZE 15
#define NUM_OF_RADIO_CHANNELS 4
#define TIME_BEFORE_RESENDING 15
#define TRIES_BEFORE_QUITTING 15
#define RADIO_CHANNEL 50

/*Motor Config*/
#define NUM_OF_MOTORS 4

using namespace std;

MPU6050 mpu;

uint16_t packet_size;
uint16_t fifo_count;     /* Count of all bytes currently in FIFO  */
uint8_t fifo_buffer[64]; /* FIFO storage buffer */

Quaternion q;           /* [w, x, y, z]         quaternion container */
VectorFloat gravity;    /* [x, y, z]            gravity vector */
float actual_ypr[3];    /* [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector */

PID *pids_ypr[3];
float desired_ypr[3] = {0, 0, 0};
float pids_output_ypr[3];
float pid_tunings[3][3] = {
	{0, 0, 0},
	{0, 0, 0},
	{0, 0, 0}
};

float throttle = 0;

RF24 radio(49, 0);
const uint8_t pipes[][6] = {"1Node","2Node"};
char radio_msg[32];

Motor *motors[4];
const enum BlackLib::pwmName motor_pins[4] = {
	BlackLib::EHRPWM1A,
	BlackLib::EHRPWM1B,
	BlackLib::EHRPWM2A,
	BlackLib::EHRPWM2B
};

/* Parse a control string and execute the command */
void parse_and_execute(char *control_string)
{
	char *command;
	char *value;
	float numeric_value;
	int ypr_update_index = -1;
	int pid_update_index = -1;

	/* control_string is modified */
	command = strtok(control_string, " ");
	value = strtok(NULL, " ");
	if (command && value) {
		char *endptr;
		numeric_value = strtof(value, &endptr);
		if (value == endptr) {
			fprintf(stderr, "Radio: Improper value received %s\n", value);
			return;
		}
		if (command[0] == 't')
			throttle = numeric_value;
		else if (!strcmp(command, "y"))
			desired_ypr[0] = numeric_value;
		else if (!strcmp(command, "p"))
			desired_ypr[1] = numeric_value;
		else if (!strcmp(command, "r"))
			desired_ypr[2] = numeric_value;
		else {
			/* PID tuning update.  In this case, the
			 * command would be of the form:
			 * "<y/p/r><p/i/d>"
			 * Therefore, "pi" would mean the integral
			 * tuning constant for pitch.
			 */
			switch(command[0]) {
			case 'y':
				ypr_update_index = 0;
				break;
			case 'p':
				ypr_update_index = 1;
				break;
			case 'r':
				ypr_update_index = 2;
				break;
			}
			switch(command[1]) {
			case 'p':
				pid_tunings[ypr_update_index][0] = numeric_value;
				pids_ypr[ypr_update_index]->setKp(pid_tunings[ypr_update_index][0]);
				break;
			case 'i':
				pid_tunings[ypr_update_index][1] = numeric_value;
				pids_ypr[ypr_update_index]->setKp(pid_tunings[ypr_update_index][1]);
				break;
			case 'd':
				pid_tunings[ypr_update_index][2] = numeric_value;
				pids_ypr[ypr_update_index]->setKp(pid_tunings[ypr_update_index][2]);
				break;
			default:
				fprintf(stderr, "Bad input");
			}
		}
	}
}

void setup()
{
	uint8_t dev_status;

	mpu.initialize();
	printf(mpu.testConnection() ?
	       "MPU6050 connection successful\n" :
	       "MPU6050 connection failed\n");
	printf("Initializing DMP...\n");
	dev_status = mpu.dmpInitialize();

	if (dev_status == 0) {
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);
		printf("DMP ready!\n");
		packet_size = mpu.dmpGetFIFOPacketSize();
	} else {
		printf("DMP Initialization failed (code %d)\n", dev_status);
		exit(0);
	}

	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(RADIO_CHANNEL);
	radio.setRetries(TIME_BEFORE_RESENDING,TRIES_BEFORE_QUITTING);
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);
	radio.enableDynamicPayloads();
	radio.startListening();

        /* Initialize PID controllers */
        for(int i = 0; i < 3; i++)
		pids_ypr[i] = new PID(pid_tunings[i][0], pid_tunings[i][1], pid_tunings[i][2]);

	for(int i = 0; i < 4; i++)
		motors[i] = new Motor(motor_pins[i]);
}

void loop()
{
	static float yaw_target = 0;
	char control_string[MAX_RADIO_MSG_SIZE] = {0};
	uint16_t channels[NUM_OF_RADIO_CHANNELS];
	float motor[NUM_OF_MOTORS];
	int max_fifo_count = 0;

	fifo_count = mpu.getFIFOCount();

	if ((fifo_count >= packet_size * MAX_PACKETS_IN_BUFFER
	     && fifo_count % packet_size == 0)
	    || fifo_count == MAX_BUFFER_SIZE) {
		mpu.resetFIFO();
		printf("FIFO Overflow\n");
		fifo_count = mpu.getFIFOCount();
	}

	while (fifo_count < packet_size)
		fifo_count = mpu.getFIFOCount();

	max_fifo_count = fifo_count;

	while ((fifo_count/packet_size) > 0) {
		mpu.getFIFOBytes(fifo_buffer, packet_size);
		mpu.dmpGetQuaternion(&q, fifo_buffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(actual_ypr, &q, &gravity);

		fifo_count -= packet_size;
	}

	/*  We need the angles in Cartesian format */
	for (int i = 0; i < 3; i++)
		actual_ypr[i] *= 180 / M_PI;

	/* Read from radio if data is available */
	if (radio.available()) {
		char control_string_copy[MAX_RADIO_MSG_SIZE] = {0};
		while (radio.available()) {
			int length = radio.getDynamicPayloadSize();
			length = MAX_RADIO_MSG_SIZE < length ? MAX_RADIO_MSG_SIZE : length;
			radio.read(control_string, length);
			strncpy(control_string_copy, control_string, length);
			parse_and_execute(control_string_copy);
		}
                /* TODO: Convert radio_msg into control_string cleanly. Consider strtok() */
	}

        if (throttle < 2) {
		/* Set power to zero regardless of PID output */
		for (int i = 0; i < 4; i++)
			motors[i]->set_power(0);
        } else {
		bool update_flag = false;

		for (int i = 0; i < 3; i++) {
			if (pids_ypr[i]->update(desired_ypr[i], actual_ypr[i]))
				update_flag = true;
		}

		if (update_flag) {
			/* Ensure that update_flag is set when throttle is changed */

			/* The output of the PID must be positive in
			 * the direction of positive rotation
			 * (right-hand rule. Also given on MPU)
			 *
			 * For yaw, motors 0 & 2 move anticlockwise from above.
			 */
			float m0, m1, m2, m3;

			m0 = throttle - pids_ypr[0]->output + pids_ypr[1]->output - pids_ypr[2]->output;
			m1 = throttle + pids_ypr[0]->output - pids_ypr[1]->output - pids_ypr[2]->output;
			m2 = throttle - pids_ypr[0]->output - pids_ypr[1]->output + pids_ypr[2]->output;
			m3 = throttle + pids_ypr[0]->output + pids_ypr[1]->output + pids_ypr[2]->output;

			motors[0]->set_power(m0);
			motors[1]->set_power(m1);
			motors[2]->set_power(m2);
			motors[3]->set_power(m3);

#if DEBUG_MODE_MOTORS
			cout << m0 << " " << m1 << " " << m2 << " " << m3 << " |";
#endif
		}
	}

#if DEBUG_MODE

	struct timespec time_struct;

	clock_gettime(CLOCK_MONOTONIC, &time_struct);

	printf("%d.%d | ", time_struct.tv_sec, time_struct.tv_nsec / 1000000);
	printf("%d | ", max_fifo_count);

	for (int i = 0; i < 3; i++)
		printf("%3.3f ", actual_ypr[i]);

	for (int i = 0; i < 3; i++)
		printf("%3.3f ", desired_ypr[i]);

	printf("%3.1f \"%s\" ", throttle, control_string);


#if DEBUG_MODE_WITH_PID

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			printf("%1.2f ", 4, pid_tunings[i][j]);
	}

#endif

	printf("\n");

#endif
}

int main(int argc, char *argv[])
{
	setup();

	while (1) {
		loop();
	}
	return 0;
}
