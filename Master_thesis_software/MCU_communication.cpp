#include "MCU_communication.h"

void send_data(const char* name_COM, const int* finger_position, const uint8_t rotation_val, move_mode mode)
{
	HANDLE h_COM;	//uchwyt do pliku (portu COM)

	h_COM = CreateFile(name_COM, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if (h_COM == INVALID_HANDLE_VALUE)
	{
		cout << "Nie uda�o si� otworzy� portu \a" << name_COM << endl;
		Sleep(2000);
		exit(EXIT_FAILURE);
	}
	else
		cout << "Port " << name_COM << " otwarty prwid�owo" << endl << endl;

	DCB COM_parameters = { 0 };		//DCB to struktura przechowuj�ca paramtery komunikacji z portem COM

	GetCommState(h_COM, &COM_parameters);		//przypisanie warto�ci parametr�w z portu o uchwycie h_COM do struktury parametr�w komunikacji
												//przekazanej jako drugi argument (wska�nik)

	COM_parameters.BaudRate = CBR_9600;			
	COM_parameters.ByteSize = 8;
	COM_parameters.Parity = NOPARITY;
	COM_parameters.StopBits = ONESTOPBIT;

	SetCommState(h_COM, &COM_parameters);		//przypisanie paramater�w ze struktury parametr�w komunikacji do uchwytu do portu COM

	cout << "*************************************************************" << endl;
	cout << "Ustawione paramtery komunikacji:" << endl;
	cout << "Rozmiar struktury DCB: " << COM_parameters.DCBlength << endl;
	cout << "Baud rate: " << COM_parameters.BaudRate << endl;
	cout << "Liczba bit�w danych: " << int(COM_parameters.ByteSize) << endl;
	cout << "Bit parzysto�ci: " << int(COM_parameters.Parity) << endl;
	cout << "Liczba bit�w STOP: " << int(COM_parameters.StopBits) << endl << endl;
	cout << "*************************************************************" << endl;

	//dane da silnika krokowego:
	uint8_t stepper_data = 0;
	DWORD written_bytes = 0;
	DWORD total_written_bytes = 0;

	if(mode == FINGERS_MOVE)
		stepper_data = FINGERS_MASK | (rotation_val & ANGLE_MASK);
	else if (mode == ANGLE_ROTATION)
		stepper_data = ROTATION_MASK | (rotation_val & ANGLE_MASK);
	else
	{
		cout << "B��DNE DANE !!!\a" << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}

	WriteFile(h_COM, &stepper_data, 1, &written_bytes, NULL);
	total_written_bytes += written_bytes;

	//dane dla serw:
	const DWORD bytes_to_write = 4;		//dla ka�dego serwa wysy�am 4 bajty: 1 - komenda (ustaw po�o�enie); 2 - nr serwa; 3 i 4 - odpowiednio dolny 
										//i g�rny bajt po�o�enia
	char servo_bytes[4] = { 0 };		//talica danych dla ka�dego serwa

	servo_bytes[0] = SET_TARGET;		//bajt komendy
	short destination = 0;				//2 bajty (typ short ma 2 bajty) po�o�enia serwa

	for (char servo_nr = 1; servo_nr <= 5; servo_nr++)		//wysy�amy dane na kana�y od 2 do 6 (indeksy 1-5)
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
		cout << "Palec nr " << (int)servo_nr << " , wys�ano sygna� o po�o�eniu nr " << destination << endl;
		total_written_bytes += written_bytes;
	}

	cout << "Wszytkich wys�anych bajt�w: " << total_written_bytes << endl << endl;

	cout << "Wci�nij \"T\", aby r�ka wr�ci�a do pozycji pocz�tkowej" << endl;
	char finish = 0;
	finish = cin.get();

	while (finish != 'T' && finish != 't')
		finish = cin.get();

	//powr�t do pozycji pocz�tkowej - pierwszy wys�any bajt ma warto�� 0xEE
	stepper_data = END_COMMUNICATION;
	WriteFile(h_COM, &stepper_data, 1, &written_bytes, NULL);
	total_written_bytes += written_bytes;

	destination = POSITION_0;
	servo_bytes[2] = destination & SET_LOCATION_MASK;
	servo_bytes[3] = (destination >> 7) & SET_LOCATION_MASK;

	for (char servo_nr = 1; servo_nr <= 5; servo_nr++)		//tu ju� bez znaczenia co wy�le, gdy� MCU zignoruje te dane, musz� je jednak wys�a�,
															//gdy� przerwanie od UART jest wywo�ywane po stalej liczbie 21 bajt�w (1 bajt krokowca
															//oraz 5x4=20 bajt�w serw)
	{
		servo_bytes[1] = servo_nr;
		WriteFile(h_COM, servo_bytes, bytes_to_write, &written_bytes, NULL);
		total_written_bytes += written_bytes;
	}

	cout << "Wszytkich wys�anych bajt�w: " << total_written_bytes << endl << endl;

	CloseHandle(h_COM);		//zamkni�cie portu COM
}