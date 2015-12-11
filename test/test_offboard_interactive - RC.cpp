#include <iostream>

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <cstdlib> //std::system
#include <cstddef>
#include <cassert>
#include <utility>

#include <chrono>
#include <thread>

#include "bjos/bjos.h"
#include "bjos/helpers/process.h"

#include "controllers/FlightController.h"

#include <wiringPi.h> // Include WiringPi library!
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

/*
 *  Test offboard
 */

using namespace bjos;

float VEL = 1.5;
float TAKEOFF = 2.0;
float ZVEL = 0.5;
bool running = true;

const int ch1 = 5; // RC input
const int ch2 = 11; // RC input
const int ch3 = 27; // RC input


const int ch7 = 3; // RC input
const int ch8 = 4; // RC input
#define HIGH 1
#define LOW 0


int pulseIn(int pin, int level)
{
	int timeout = 10000;
	struct timeval tn, t0, t1;
	long micros;

	gettimeofday(&t0, NULL);

	micros = 0;

	while (digitalRead(pin) != level)
	{
		gettimeofday(&tn, NULL);

		if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
		micros += (tn.tv_usec - t0.tv_usec);

		if (micros > timeout) return 0;
	}

	gettimeofday(&t1, NULL);

	while (digitalRead(pin) == level)
	{
		gettimeofday(&tn, NULL);

		if (tn.tv_sec > t0.tv_sec) micros = 1000000L; else micros = 0;
		micros = micros + (tn.tv_usec - t0.tv_usec);

		if (micros > timeout) return 0;
	}

	if (tn.tv_sec > t1.tv_sec) micros = 1000000L; else micros = 0;
	micros = micros + (tn.tv_usec - t1.tv_usec);

	return micros;
}

Heading check_RC()
{
	static Heading setp_old;
	Heading setp = Heading();
	delay(100);
	int RC1 = pulseIn(ch1, HIGH);
	delay(100);
	int RC2 = pulseIn(ch2, HIGH);
	delay(100);
	int RC3 = pulseIn(ch3, HIGH);
	delay(100);
	int RC7 = pulseIn(ch7, HIGH);
	delay(100);
	int RC8 = pulseIn(ch8, HIGH);
	printf("RC1: %i, RC2: %i, RC3: %i, RC7: %i, RC8: %i\n", RC1, RC2, RC3, RC7, RC8);
	if (RC1 < 1300)
	{
		setp.velocity.vx = VEL;
	}
	else if (RC1>1600)
	{
		setp.velocity.vx = VEL;
	}
	else {
		setp.velocity.vx = 0;
	}
		
	if (RC2 < 1300)
	{
		setp.velocity.vy = VEL;
	}
	else if (RC2>1600)
	{
		setp.velocity.vy = VEL;
	}
	else {
		setp.velocity.vy = 0;
	}

	if (RC3 < 1300)
	{
		setp.velocity.vz = ZVEL;
	}
	else if (RC3>1600)
	{
		setp.velocity.vz = ZVEL;
	}
	else {
		setp.velocity.vz = 0;
	}

	if (RC7 >1500)
	{
		std::cout << "Takeoff! :)" << std::endl;
		setp.velocity.vz = TAKEOFF;
		setp.velocity.vx = setp.velocity.vy = 0;
	}
	else
	{
		std::cout << "Landing... :(" << std::endl;
		setp.velocity.vz = -ZVEL;
		setp.velocity.vx = setp.velocity.vy = 0;
	}

	if (RC8 >1500)
	{
		std::cout << "Quitting... :(" << std::endl;
		setp.velocity.vz = -ZVEL;
		setp.velocity.vx = setp.velocity.vy = 0;
		Running = false;
	}
	setp_old = setp;
}





int main(){
    Process::installSignalHandler();
    //BJOS::init();
    BJOS *bjos = BJOS::getOS();
    if(bjos == nullptr){
        std::cout << "BJOS is not available..." << std::endl;
        return 0;
    }
    
    FlightController flight;
    bjos->getController("flight", &flight);
    if(!flight.isAvailable()){
        std::cout << "Failed to retrieve flight controller" << std::endl;
        return 0;
    }

	char ret;
	std::cout << "Interactive Offboard Tester!\n----------------------------" << std::endl;
	do {
		/*ret = std::cin.get();
		if (!Process::isActive()) ret = 'q';*/
		flight.setTargetCF(SET_TARGET_VELOCITY, Pose(), handle_input(check_RC()));
	} while (running);

	std::cout << "Byebye!";
}
