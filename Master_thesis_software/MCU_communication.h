#ifndef MCU_COM_H_
#define MCU_COM_H_

#include <iostream>
#include <Windows.h>


using namespace std;

//komendy Micro Maestro:
const uint8_t SET_TARGET = 0x84;			//ustaw po³o¿enie serwa, tj. wyœlij odpowiedni sygna³ PWM
const uint8_t SET_LOCATION_MASK = 0x7F;	//maska do ustawiania bitów w bajtach po³o¿enia

//pozycje serwa = pozycje palców:
const short POSITION_4 = 2000 * 4;		//serwo max. u góry - palce max. z³o¿ony
const short POSITION_3 = 1750 * 4;
const short POSITION_2 = 1500 * 4;
const short POSITION_1 = 1250 * 4;
const short POSITION_0 = 1000 * 4;		//serwo max. u do³u - palec schowany

//silnik krokowy:
const uint8_t ANGLE_MASK = 0x7E;		//maska ustawiania wartoœci kroków obrotu

//tryby ruchu - ruch palców czy obrót w nadgarstku
const uint8_t  FINGERS_MASK = 0x80;			//MSB = 1 -> ruch palców
const uint8_t  ROTATION_MASK = 0x00;		//MSB = 0 -> obrót w nadgarstku
const uint8_t END_COMMUNICATION = 0xEE;		//powrót do pozycji pocz¹tkowej

enum move_mode
{
	FINGERS_MOVE,
	ANGLE_ROTATION
};

void send_data(const char* name_COM, const int* finger_position, const uint8_t rotation_val, move_mode mode);


#endif
