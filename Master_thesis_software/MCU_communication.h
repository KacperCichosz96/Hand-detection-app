#ifndef MCU_COM_H_
#define MCU_COM_H_

#include <iostream>
#include <Windows.h>

using namespace std;

//commands of Micro Maestro controler:
const uint8_t SET_TARGET = 0x84;			//set servos position
const uint8_t SET_LOCATION_MASK = 0x7F;		//mask to setting bits in location bytes

//servos positions = fingers position
const short POSITION_4 = 2000 * 4;		
const short POSITION_3 = 1750 * 4;
const short POSITION_2 = 1500 * 4;
const short POSITION_1 = 1250 * 4;
const short POSITION_0 = 1000 * 4;		

//stepper motor:
const uint8_t ANGLE_MASK = 0x7E;		//mask to setting number of steps to do

//motion mode - fingers movement or revolution in wrist:
const uint8_t  FINGERS_MASK = 0x80;			//MSB = 1 -> fingers movement
const uint8_t  ROTATION_MASK = 0x00;		//MSB = 0 -> revolution in wrist
const uint8_t END_COMMUNICATION = 0xEE;		//return to start position

enum move_mode
{
	FINGERS_MOVE,
	ANGLE_ROTATION
};

void send_data(const char* name_COM, const int* finger_position, const uint8_t rotation_val, move_mode mode);


#endif
