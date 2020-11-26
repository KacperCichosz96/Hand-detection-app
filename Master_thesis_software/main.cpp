#include "headers.h"
#include "finger.h"
#include "MCU_communication.h"


//This application is a part of my master thesis project called "Project of a hand controlled using a vision system".
//The app can work in 2 modes:
//								1 - fingers positions detection (user types "F" in console)
//								2 - calculating of revolution angle in wrist (user types "R" in console)
//
//In the 1st mode app needs just one image to calibration ("ToCalib_fingers.jpg" from example images). 
//In the 2nd mode app needs two images to calibration - one in front view ("ToCalib_front.jpg" from example images) and one
//in side view ("ToCalib_side.jpg" from example images). To see how app works with example images user can use as fin_img_name:
//
//								1st mode - "ToDetect_1.jpg", "ToDetect_2.jpg" or "ToDetect_3.jpg"
//								2nd mode - "Rev2.jpg", "Rev3.jpg", "Rev4.jpg" or "Rev5.jpg"
//
//In the project, for proper working of this app, I made numerous assumptions:
//
//		1)	The hand should be placed as straight as possible in the central part of the photo, the rotation of the wrist should only take place
//			in relation to the vertical axis, the forearm should be directed as vertically as possible.
//		2)	The hand should be photographed under strong lighting conditions, on a uniform background, far enough away from the surface on the
//			back that any shadows are as little visible as possible, and preferably not visible.
//		3)	In the calibration photos, the fingers should be straightened and separated, the same in the case of photos for which the angle of
//			rotation is determined.
//		4)	The hand is assumed to be in the plane of the body at shoulder height. In such a situation, the wrist may rotate freely about the
//			vertical axis in an angular range from 0 to about 90 degrees.
//		5)	It is possible to move the hand along the direction of the lens axis, but it should not be too large to get the most accurate result
//			of the gesture recognition.
//
//In the real project this app communicate with microcontroller unit (MCU) and sent calculated data. In the project used STM32F411VET
//and additional servos controller (https://www.pololu.com/product/1350). 
//To can use OpenCV library it is necessary to change some project configuration. How to do that in MS Visual Studio see:
//https://www.youtube.com/watch?v=M-VHaLHC4XI


int main()
{
	setlocale(LC_ALL, "");

	//names of images:
	string calib_front_name("ToCalib_fingers.jpg");	//name of image to calibration in BOTH MODE in front view
	string fin_img_name("ToDetect_3.jpg");			//name of image to detect fingers positions in FINGER DETECT MODE
	string calib_side_name("ToCalib_side.jpg");		//name of image to calibration in REVOLUTION DETECT MODE in side view
	string rot_detect_name("Rev2.jpg");				//name of image to calculate an angle of revolution in wrist in REVOLUTION DETECT MODE
	
	//data to send to MCU:
	int fin_pos[5] = { 0 };				//finger position data 
	uint8_t rotation = 0;				//value of revolution angle
	move_mode fin_or_rot;

	//Calibration - front (both modes):
	Mat calib_FR;				
	calib_FR = imread(calib_front_name);

	if (calib_FR.data == NULL)		
	{
		cout << "Failed to load an image to calibration in front view (image name:\"" << calib_front_name << "\")\a" << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}

	string calib_FR_name("Calib. front view");		
	Rect calib_FR_rect;
	vector<Point> calib_FR_contour;
	Point calib_FR_wrist_right;
	Point calib_FR_wrist_left;
	int calib_FR_wrist_width;
	Finger cal_fin[5] = { Finger(calib_FR),Finger(calib_FR),Finger(calib_FR),Finger(calib_FR),Finger(calib_FR) };
	
	char detection_type = 0;

	//basic part of calibration in front mode (done in both modes):
	basic_detection(calib_FR, calib_FR_name, calib_FR_rect, calib_FR_contour, calib_FR_wrist_right, calib_FR_wrist_left, calib_FR_wrist_width);

	cout << endl << "Choose a mode of hand detection, write:\a" << endl;
	cout << "\"F\" - to find fingers positions" << endl;
	cout << "\"R\" - to calculate revolution angle in wrist" << endl;

	detection_type = cin.get();
	while (detection_type != 'F' && detection_type != 'f' && detection_type != 'R' && detection_type != 'r')
		detection_type = cin.get();
	while (cin.get() != '\n')
		continue;

	switch (detection_type)
	{
	case 'F':
	case'f':
		fin_or_rot = FINGERS_MOVE;
		//the next part of calibration in front view mode (necessary just to detect fingers positions):
		calibration_front_function(calib_FR, calib_FR_name, calib_FR_rect, calib_FR_contour, calib_FR_wrist_right, calib_FR_wrist_left, cal_fin);
		//detection fingers positions:
		fingers_detection_function(fin_img_name, calib_FR_wrist_width, cal_fin, fin_pos);
		break;
	case 'R':
	case'r':
		fin_or_rot = ANGLE_ROTATION;
		rotation = rotation_detection_function(calib_side_name, rot_detect_name, calib_FR_rect);
		break;
	}

	bool is_OK = false;
	char ready_to_send = 0;

	cout << "Is the detection successfull and activate the arm?\t 'Y' - yes; 'N' - no" << endl;
	ready_to_send = cin.get();
	while (ready_to_send != 'Y' && ready_to_send != 'y' && ready_to_send != 'N' && ready_to_send != 'n')
	{
		while (cin.get() != '\n')
			continue;
		ready_to_send = cin.get();
	}
	while (cin.get() != '\n')
		continue;

	if (ready_to_send == 'N' || ready_to_send == 'n')
	{
		cout << "You chose to terminate the application\a" << endl << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}

	send_data("COM6", fin_pos, rotation, fin_or_rot);	//you have to check and choose the proper COM port name, the one where you connected MCU
	
	system("pause");
	return 0;
}
