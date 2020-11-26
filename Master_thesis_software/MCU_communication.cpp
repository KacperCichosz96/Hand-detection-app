#include "MCU_communication.h"

void send_data(const char* name_COM, const int* finger_position, const uint8_t rotation_val, move_mode mode)
{
	HANDLE h_COM;	
	h_COM = CreateFile(name_COM, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (h_COM == INVALID_HANDLE_VALUE)
	{
		cout << "Failed to open COM port \a" << name_COM << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}
	else
		cout << "COM port (\" " << name_COM << "\") opened successfully" << endl << endl;

	DCB COM_parameters = { 0 };		

	GetCommState(h_COM, &COM_parameters);	

	COM_parameters.BaudRate = CBR_9600;			
	COM_parameters.ByteSize = 8;
	COM_parameters.Parity = NOPARITY;
	COM_parameters.StopBits = ONESTOPBIT;

	SetCommState(h_COM, &COM_parameters);		

	cout << "*************************************************************" << endl;
	cout << "Communication parameters:" << endl;
	cout << "Size of DCB structure: " << COM_parameters.DCBlength << endl;
	cout << "Baud rate: " << COM_parameters.BaudRate << endl;
	cout << "Data bits number: " << int(COM_parameters.ByteSize) << endl;
	cout << "Parity bit: " << int(COM_parameters.Parity) << endl;
	cout << "Stop bits: " << int(COM_parameters.StopBits) << endl << endl;
	cout << "*************************************************************" << endl;

	//stepper motor data:
	uint8_t stepper_data = 0;
	DWORD written_bytes = 0;
	DWORD total_written_bytes = 0;

	if(mode == FINGERS_MOVE)
		stepper_data = FINGERS_MASK | (rotation_val & ANGLE_MASK);
	else if (mode == ANGLE_ROTATION)
		stepper_data = ROTATION_MASK | (rotation_val & ANGLE_MASK);
	else
	{
		cout << "Incorrect data for stepper motor\a" << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}

	WriteFile(h_COM, &stepper_data, 1, &written_bytes, NULL);
	total_written_bytes += written_bytes;

	//servos data:
	const DWORD bytes_to_write = 4;		//for each servo 4 bytes are sent:
										//1st - command byte
										//2nd - servo number byte
										//3rd and 4th - location value bytes
	char servo_bytes[4] = { 0 };		

	servo_bytes[0] = SET_TARGET;		
	short destination = 0;				

	for (char servo_nr = 1; servo_nr <= 5; servo_nr++)	//send data to Micro Maestro controller channels number 2 to 6
	{
		servo_bytes[1] = servo_nr;

		switch (finger_position[servo_nr - 1])
		{
		case 0:
			destination = POSITION_0;
			break;
		case 1:
			destination = POSITION_1;
			break;
		case 2:
			destination = POSITION_2;
			break;
		case 3:
			destination = POSITION_3;
			break;
		case 4:
			destination = POSITION_4;
			break;
		}

		servo_bytes[2] = destination & SET_LOCATION_MASK;
		servo_bytes[3] = (destination >> 7) & SET_LOCATION_MASK;

		WriteFile(h_COM, servo_bytes, bytes_to_write, &written_bytes, NULL);
		cout << "Finger number " << (int)servo_nr << " , sent data about location number: " << destination << endl;
		total_written_bytes += written_bytes;
	}

	//cout << "All sent bytes: " << total_written_bytes << endl << endl;	//just for diagnostic purpose

	cout << "Press \"T\" in order to the hand returns to start position" << endl;
	char finish = 0;
	finish = cin.get();

	while (finish != 'T' && finish != 't')
		finish = cin.get();

	//return to start position:
	stepper_data = END_COMMUNICATION;
	WriteFile(h_COM, &stepper_data, 1, &written_bytes, NULL);
	total_written_bytes += written_bytes;

	destination = POSITION_0;
	servo_bytes[2] = destination & SET_LOCATION_MASK;
	servo_bytes[3] = (destination >> 7) & SET_LOCATION_MASK;

	for (char servo_nr = 1; servo_nr <= 5; servo_nr++)
	{
		servo_bytes[1] = servo_nr;
		WriteFile(h_COM, servo_bytes, bytes_to_write, &written_bytes, NULL);
		total_written_bytes += written_bytes;
	}

	//cout << "All sent bytes: " << total_written_bytes << endl << endl;	//just for diagnostic purpose

	CloseHandle(h_COM);
}