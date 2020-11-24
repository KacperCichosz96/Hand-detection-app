#include "headers.h"
#include "finger.h"
#include "MCU_communication.h"


int main()
{
	setlocale(LC_ALL, "");

	//nazwy plików ze zdjeciami:
	string fin_img_name("4.jpg");			//nazwa pliku do rozpoznania po³o¿enia palców, wraz z rozszerzeniem pliku
	string calib_side_name("R_side.jpg");	//nazwa pliku do kalibracji z boku, wraz z rozszerzeniem pliku
	string rot_detect_name("R1.jpg");		//nazwa pliku do detekcji obrotu, wraz z rozszerzeniem pliku
	
	//dane do wys³ania do MCU
	int fin_pos[5] = { 0 };				//dane dotycz¹ce po³o¿enia palców 
	uint8_t rotation = 0;				//dane dotycz¹ce obrotu
	move_mode fin_or_rot;

	//kalibracja - front:
	Mat calib_FR;				//nazwa pliku ze zdjeciem do kalibracji od frontu, wraz z rozszerzeniem pliku

	calib_FR = imread("1.jpg");

	if (calib_FR.data == NULL)		//je¿eli nie uda³o sie wczytaæ pliku ze zdjêciem to obiekt Mat jest pusty, tj. ma wartoœæ NULL
	{
		cout << "Nie uda³o siê wczytaæ pliku do kalibracji od frontu (obiekt \'calib_FR\") !!!\a" << endl;
		Sleep(4000);
		exit(EXIT_FAILURE);
	}

	string calib_FR_name("Calibration ROTATION front");
	Rect calib_FR_rect;
	vector<Point> calib_FR_contour;
	Point calib_FR_wrist_right;
	Point calib_FR_wrist_left;
	int calib_FR_wrist_width;
	Finger cal_fin[5] = { Finger(calib_FR),Finger(calib_FR),Finger(calib_FR),Finger(calib_FR),Finger(calib_FR) };
	
	char detection_type = 0;

	//podstawowa detekcja dokonywana zarówno przy rozpoznawaniu palców jak i obrotu d³oni:
	basic_detection(calib_FR, calib_FR_name, calib_FR_rect, calib_FR_contour, calib_FR_wrist_right, calib_FR_wrist_left, calib_FR_wrist_width);

	cout << endl << "Wybierz rodzaj rozpoznawania d³oni, wpisz:\a" << endl;
	cout << "\"P\" - ¿eby rozpoznaæ po³o¿enia palców" << endl;
	cout << "\"O\" - ¿eby rozpoznac obrót d³oni" << endl;

	detection_type = cin.get();
	while (detection_type != 'P' && detection_type != 'p' && detection_type != 'O' && detection_type != 'o')			//tak zmieniæ wszêdzie
		detection_type = cin.get();
	
	switch (detection_type)
	{
	case 'P':
	case'p':
		fin_or_rot = FINGERS_MOVE;
		//dalsza czêœæ kalibracji od frontu, która jest potrzebna tylko przy detekcji po³o¿enia palców:
		calibration_front_function(calib_FR, calib_FR_name, calib_FR_rect, calib_FR_contour, calib_FR_wrist_right, calib_FR_wrist_left, cal_fin);
		//detekcja po³o¿enia palców
		fingers_detection_function(fin_img_name, calib_FR_wrist_width, cal_fin, fin_pos);
		break;
	case 'O':
	case'o':
		fin_or_rot = ANGLE_ROTATION;
		rotation = rotation_detection_function(calib_side_name, rot_detect_name, calib_FR_rect);
		break;
	}

	bool is_OK = false;
	char ready_to_send = 0;

	cout << "Czy detekcja prebieg³a poprawnie i uruchomiæ rêkê?\t 'T' - tak; 'N' - nie" << endl;
	ready_to_send = cin.get();
	while (ready_to_send != 'T' && ready_to_send != 't' && ready_to_send != 'N' && ready_to_send != 'n')
		ready_to_send = cin.get();

	if (ready_to_send == 'N' || ready_to_send == 'n')
	{
		cout << "Wybrano zakoñczenie programu\a" << endl << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}

	send_data("COM6", fin_pos, rotation, fin_or_rot);
	
	system("pause");
	return 0;
}
