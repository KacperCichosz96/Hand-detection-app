#ifndef MCU_COM_H_
#define MCU_COM_H_

#include <iostream>
#include <Windows.h>


using namespace std;

//komendy Micro Maestro:
const uint8_t SET_TARGET = 0x84;			//ustaw po�o�enie serwa, tj. wy�lij odpowiedni sygna� PWM
const uint8_t SET_LOCATION_MASK = 0x7F;	//maska do ustawiania bit�w w bajtach po�o�enia

//pozycje serwa = pozycje palc�w:
const short POSITION_4 = 2000 * 4;		//serwo max. u g�ry - palce max. z�o�ony
const short POSITION_3 = 1750 * 4;
const short POSITION_2 = 1500 * 4;
const short POSITION_1 = 1250 * 4;
const short POSITION_0 = 1000 * 4;		//serwo max. u do�u - palec schowany

//silnik krokowy:
const uint8_t ANGLE_MASK = 0x7E;		//maska ustawiania warto�ci krok�w obrotu

//tryby ruchu - ruch palc�w czy obr�t w nadgarstku
const uint8_t  FINGERS_MASK = 0x80;			//MSB = 1 -> ruch palc�w
const uint8_t  ROTATION_MASK = 0x00;		//MSB = 0 -> obr�t w nadgarstku
const uint8_t END_COMMUNICATION = 0xEE;		//powr�t do pozycji pocz�tkowej

enum move_mode
{
	FINGERS_MOVE,
	ANGLE_ROTATION
};

void send_data(const char* name_COM, const int* finger_position, const uint8_t rotation_val, move_mode mode);


#endif
