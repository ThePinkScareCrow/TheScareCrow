#include <math.h>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include <stdint.h>
#include "motor.hpp"
#include "BlackLib/BlackPWM.h"
#include <RF24/RF24.h>

using namespace std;

#include "MPU6050_6Axis_MotionApps20.h"
#include "PID.h"

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t g[3];
float gyro[3];
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define ToDeg(x) ((x)*57.2957795131) // *180/pi
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

float pid_tunings[6][3] = {
	{0.5, 0.003, 0.09},
	{0.5, 0.003, 0.09},
	{6.5, 0.1, 1.2},
	{6.5, 0.1, 1.2},
	{0, 0, 0},
	{0, 0, 0}
};

float throttle = 0;
float desired_ypr[3] = {0, 0, 0};

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
				if (command[1] == 'r')
					ypr_update_index = PID_YAW_RATE;
				else
					ypr_update_index = PID_YAW_STAB;
				break;
			case 'p':
				if (command[1] == 'r')
					ypr_update_index = PID_PITCH_RATE;
				else
					ypr_update_index = PID_PITCH_STAB;
				break;
			case 'r':
				if (command[1] == 'r')
					ypr_update_index = PID_ROLL_RATE;
				else
					ypr_update_index = PID_ROLL_STAB;
				break;
			}
			switch(command[2]) {
			case 'p':
				pid_tunings[ypr_update_index][0] = numeric_value;
				break;
			case 'i':
				pid_tunings[ypr_update_index][1] = numeric_value;
				break;
			case 'd':
				pid_tunings[ypr_update_index][2] = numeric_value;
				break;
			default:
				fprintf(stderr, "Bad input");
			}
			pids[ypr_update_index]->setKp(pid_tunings[ypr_update_index][0],
						      pid_tunings[ypr_update_index][1],
						      pid_tunings[ypr_update_index][2]);
		}
	}
}

void setup()
{

	pids[PID_PITCH_RATE].set_Kpid(pid_tunings[PID_PITCH_RATE][0],
				      pid_tunings[PID_PITCH_RATE][1],
				      pid_tunings[PID_PITCH_RATE][2]);
	pids[PID_ROLL_RATE].set_Kpid(pid_tunings[PID_ROLL_RATE][0],
				     pid_tunings[PID_ROLL_RATE][1],
				     pid_tunings[PID_ROLL_RATE][2]);
	pids[PID_PITCH_STAB].set_Kpid(pid_tunings[PID_PITCH_STAB][0],
				      pid_tunings[PID_PITCH_STAB][1],
				      pid_tunings[PID_PITCH_STAB][2]);
	pids[PID_ROLL_STAB].set_Kpid(pid_tunings[PID_ROLL_STAB][0],
				     pid_tunings[PID_ROLL_STAB][1],
				     pid_tunings[PID_ROLL_STAB][2]);

	// initialize device
	printf("Initializing I2C devices...\n");
	mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	// load and configure the DMP
	printf("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);
		mpuIntStatus = mpu.getIntStatus();
		printf("DMP ready!\n");
		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		printf("DMP Initialization failed (code %d)\n", devStatus);
	}

	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setChannel(RADIO_CHANNEL);
	radio.setRetries(TIME_BEFORE_RESENDING,TRIES_BEFORE_QUITTING);
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);
	radio.enableDynamicPayloads();
	radio.startListening();

	for(int i = 0; i < 4; i++)
		motors[i] = new Motor(motor_pins[i]);
}

void loop ()
{

	long yaw, pitch, roll, gyroYaw, gyroPitch, gyroRoll;
	static float yaw_target = 0;


	if (!dmpReady)
		return;

	do {
		fifoCount = mpu.getFIFOCount();
	} while (fifoCount < 42);

	if (fifoCount == 1024) {
		mpu.resetFIFO();
		printf("FIFO overflow!\n");
	} else  {
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		for (int i = 0; i < 3; i++){
			ypr[i] *= 180 / M_PI;
		}

		yaw = ypr[0];
		pitch = ypr[1];
		roll = ypr[2];

		mpu.dmpGetGyro(g, fifoBuffer);

		//0=gyroX, 1=gyroY, 2=gyroZ
		//swapped to match Yaw,Pitch,Roll
		//Scaled from deg/s to get tr/s
		for (int i = 0; i < 3; i++) {
			gyro[i] = ToDeg(g[3-i-1]);
		}
		gyroYaw =- gyro[0];
		gyroPitch =- gyro[1];
		gyroRoll = gyro[2];
	}

	if(throttle > 2) {  // Throttle raised, turn on stablisation.
		// Stablise PIDS
		float pitch_stab_output = constrain(pids[PID_PITCH_STAB].update_pid_std((float)desired_ypr[1], pitch, 1), -250, 250);
		float roll_stab_output = constrain(pids[PID_ROLL_STAB].update_pid_std((float)desired_ypr[2], roll, 1), -250, 250);
		float yaw_stab_output = constrain(pids[PID_YAW_STAB].update_pid_std(wrap_180(yaw_target), wrap_180(desired_ypr[0]), 1), -360, 360);

		// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
		if(abs(desired_ypr[0]) > 5) {
			yaw_stab_output = desired_ypr[0];
			yaw_target = yaw;   // remember this yaw for when pilot stops
		}

		// rate PIDS
		long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].update_pid_std(pitch_stab_output, gyroPitch, 1), - 500, 500);
		long roll_output =  (long) constrain(pids[PID_ROLL_RATE].update_pid_std(roll_stab_output, gyroRoll, 1), -500, 500);
		long yaw_output =  (long) constrain(pids[PID_YAW_RATE].update_pid_std(yaw_stab_output, gyroYaw, 1), -500, 500);

		float m0, m1, m2, m3;

		m0 = throttle - pids_ypr[0]->output + pids_ypr[1]->output - pids_ypr[2]->output;
		m1 = throttle + pids_ypr[0]->output - pids_ypr[1]->output - pids_ypr[2]->output;
		m2 = throttle - pids_ypr[0]->output - pids_ypr[1]->output + pids_ypr[2]->output;
		m3 = throttle + pids_ypr[0]->output + pids_ypr[1]->output + pids_ypr[2]->output;

		motors[0]->set_power(m0);
		motors[1]->set_power(m1);
		motors[2]->set_power(m2);
		motors[3]->set_power(m3);

	} else {
		// motors off

		for (int i = 0; i < 4; i++)
			motors[i]->set_power(0);

		// reset yaw target so we maintain this on takeoff
		yaw_target = yaw;
	}
}


int main(int argc, char ** args)
{
	setup();

	while(1)
	{
		loop();
		usleep(1);
	}
	return 0;
}
