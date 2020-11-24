#ifndef FINGER_H_
#define FINGER_H_

#include <iostream>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class Finger
{
private:
	Mat& img;
	Rect all_region;
	array<array<Point, 4>, 4> regions;
	Point apex;
	int finger_height;
	int region_size;
	int top_apex_dist;

public:
	enum f_mode { FINGER_MODE, THUMB_MODE };

	Finger(Mat& init_img, Point top_left = Point(0, 0), Point bottom_right = Point(0, 0), Point finger_top = Point(0, 0));	//konstruktor dla NIE kciuka
	Finger(Mat& init_img, Point top_left, Point bottom_right, f_mode mode);							//konstruktor dla kciuka
	Finger(Mat& init_img, Point start_point, Finger fin, double scale, f_mode mode);	//konstruktor dla palca "skalowanego" na w³aœciwym obrazie

	void draw_regions();
	int find_fin_location(Point f_top);
	Point give_point() { return regions[0][1]; };
	Finger& operator=(Finger copied_finger);


	~Finger();
};



#endif