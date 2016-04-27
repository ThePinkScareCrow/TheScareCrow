#include <iostream>
#include <stdlib.h>
#include <time.h>

struct timespec time_struct;

using namespace std;

long int millis()
{
	clock_gettime(CLOCK_MONOTONIC, &time_struct);
	return time_struct.tv_nsec / 1000000;
}

int main() {
	cout << millis();
	return 0;
}
