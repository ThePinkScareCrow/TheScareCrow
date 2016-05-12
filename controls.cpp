#include "BlackLib/BlackPWM.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID.hpp"
#include "motor.hpp"
#include "utils.h"

#include <RF24/RF24.h>
#include <ctime>
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* Debug modes */
#define DEBUG_MODE_MOTORS 0
#define DEBUG_MODE 0
#define DEBUG_MODE_WITH_PID 0

/* MPU Config */
#define MAX_PACKETS_IN_BUFFER 3
#define MAX_BUFFER_SIZE 1024

/* Radio Config */
#define MAX_RADIO_MSG_SIZE 15
#define NUM_OF_RADIO_CHANNELS 4
#define TIME_BEFORE_RESENDING 15
#define TRIES_BEFORE_QUITTING 15
#define RADIO_CHANNEL 50

/* Motor Config */
#define NUM_OF_MOTORS 4

/* Math constant */
#define MATH_180_BY_PI 57.2957795131

using namespace std;

MPU6050 mpu;

uint16_t packet_size;
uint16_t fifo_count;         /* Count of all bytes currently in FIFO  */
uint8_t fifo_buffer[64];     /* FIFO packet buffer */

Quaternion q;                /* [w, x, y, z]         quaternion container */
VectorFloat gravity;         /* [x, y, z]            gravity vector */

float actual_ypr[3];         /* Actual yaw/pitch/roll values obtained from the MPU */
float desired_ypr[3] = {0};  /* Desired yaw/pitch/roll values obtained from the user */
int16_t rate_ypr[3];         /* Actual angular velocity about yaw/pitch/roll in deg/s */
PID *stab_pids_ypr[3];       /* The PID to correct the orientation of the quadcopter (Angle) */
PID *rate_pids_ypr[3];

float throttle = 0;

RF24 radio(49, 0);           /* GPIO 49 = P9_23; Default CSN */
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

			enum {
				rate,
				stab
			} pid_type;

			switch(command[0]) {
			case 's':
				pid_type = stab;
			case 'r':
				pid_type = rate;
			default:
				fprintf(stderr, "Radio: Bad input");
				return;
			}
			switch(command[1]) {
			case 'y':
				ypr_update_index = 0;
				break;
			case 'p':
				ypr_update_index = 1;
				break;
			case 'r':
				ypr_update_index = 2;
				break;
			default:
				fprintf(stderr, "Radio: Bad input");
				return;
			}
			switch(command[2]) {
			case 'p':
				if (pid_type == stab)
					stab_pids_ypr[ypr_update_index]->setKp(numeric_value);
				else
					rate_pids_ypr[ypr_update_index]->setKp(numeric_value);
				break;
			case 'i':
				if (pid_type == stab)
					stab_pids_ypr[ypr_update_index]->setKi(numeric_value);
				else
					rate_pids_ypr[ypr_update_index]->setKi(numeric_value);
				break;
			case 'd':
				if (pid_type == stab)
					stab_pids_ypr[ypr_update_index]->setKd(numeric_value);
				else
					rate_pids_ypr[ypr_update_index]->setKd(numeric_value);
				break;
			case 'w':
				stab_pids_ypr[ypr_update_index]->setWindup(numeric_value);
				break;
			default:
				fprintf(stderr, "Radio: Bad input");
				return;
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
        for(int i = 0; i < 3; i++) {
		stab_pids_ypr[i] = new PID(&difference_wrap_180, 0, 0, 0);
		rate_pids_ypr[i] = new PID(&difference, 0, 0, 0);
	}
	/* Initialize Motors */
	for(int i = 0; i < 4; i++)
		motors[i] = new Motor(motor_pins[i]);

	/* Calibrate ESCs */
	cout << "Plug in battery now." << endl;
	sleep(5);
	cout << "Done calibrating motors." << endl;
}

void loop()
{
	static float yaw_target = 0;
	char control_string[MAX_RADIO_MSG_SIZE] = {0};
	uint16_t channels[NUM_OF_RADIO_CHANNELS];
	float motor[NUM_OF_MOTORS];
	int max_fifo_count = 0;
	int16_t rate_ypr_tmp[3];

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
		/* Read value from FIFO buffer */
		mpu.getFIFOBytes(fifo_buffer, packet_size);
		/* Calculate absolute angle */
		mpu.dmpGetQuaternion(&q, fifo_buffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(actual_ypr, &q, &gravity);
		mpu.dmpGetGyro(rate_ypr_tmp, fifo_buffer);

		/* Keep track of packets remaining */
		fifo_count -= packet_size;
	}

	/*  Convert radians to degrees */
	for (int i = 0; i < 3; i++)
		actual_ypr[i] *= 180 / M_PI;

	/*
	 * dmpGetGyro() gives us X/Y/Z values, map this to Yaw/Pitch/Roll.
	 * While doing this convert the obtained values from rad/s to deg/s.
	 */
	for (int i = 0; i < 3; i++)
		rate_ypr[i] = rate_ypr_tmp[2-i] * MATH_180_BY_PI;

	/* Read from radio if data is available */
	if (radio.available()) {
		char control_string_copy[MAX_RADIO_MSG_SIZE] = {0};
		while (radio.available()) {
			int length = radio.getDynamicPayloadSize();
			length = MAX_RADIO_MSG_SIZE < length ? MAX_RADIO_MSG_SIZE : length;
			radio.read(control_string, length);
			strncpy(control_string_copy, control_string, length);
			/* Update system values based on control input from pilot */
			parse_and_execute(control_string_copy);
		}
	}

        if (throttle < 2) {
		/* Set power to zero regardless of PID output */
		for (int i = 0; i < 4; i++)
			motors[i]->set_power(0);
        } else {
		for (int i = 0; i < 3; i++) {
			stab_pids_ypr[i]->update(desired_ypr[i], actual_ypr[i]);
			rate_pids_ypr[i]->update(stab_pids_ypr[i]->output, rate_ypr);
		}

		/* The output of the PID must be positive in
		 * the direction of positive rotation
		 * (right-hand rule. Also given on MPU)
		 *
		 * For yaw, motors 0 & 2 move anticlockwise from above.
		 */
		float m0, m1, m2, m3;

		m0 = throttle - rate_pids_ypr[0]->output + rate_pids_ypr[1]->output - rate_pids_ypr[2]->output;
		m1 = throttle + rate_pids_ypr[0]->output - rate_pids_ypr[1]->output - rate_pids_ypr[2]->output;
		m2 = throttle - rate_pids_ypr[0]->output - rate_pids_ypr[1]->output + rate_pids_ypr[2]->output;
		m3 = throttle + rate_pids_ypr[0]->output + rate_pids_ypr[1]->output + rate_pids_ypr[2]->output;

		motors[0]->set_power(m0);
		motors[1]->set_power(m1);
		motors[2]->set_power(m2);
		motors[3]->set_power(m3);

#if DEBUG_MODE_MOTORS
		cout << m0 << " " << m1 << " " << m2 << " " << m3 << " |";
#endif

	}

#if DEBUG_MODE

	printf("%.3f | ", elapsed_time_in_s());
	printf("%d | ", max_fifo_count);

	for (int i = 0; i < 3; i++)
		printf("%3.3f ", actual_ypr[i]);

	for (int i = 0; i < 3; i++)
		printf("%3.3f ", desired_ypr[i]);

	printf("%3.1f \"%s\" ", throttle, control_string);


#if DEBUG_MODE_WITH_PID

	for (int i = 0; i < 3; i++) {
                printf("%1.2f ", 4, stab_pids_ypr[i]->Kp);
                printf("%1.2f ", 4, stab_pids_ypr[i]->Ki);
                printf("%1.2f ", 4, stab_pids_ypr[i]->Kd);
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
