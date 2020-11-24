#ifndef HEADERS_H_
#define HEADERS_H_

#include <iostream>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <math.h>
#include <limits>
#include "finger.h"


using namespace std;
using namespace cv;

enum index_mode
{
	DECREMENT_INDEX,
	INCREMENT_INDEX
};

enum apex_velley_mode
{
	VALLEY,
	APEX,
	CALIBRATION,
	DETECTION
};

enum wrist_side
{
	WRIST_RIGHT,
	WRIST_LEFT
};


void display_img(Mat& src, string name);	//wyœwietlanie zmniejszonego obrazu
void my_thresholding(Mat& src, Mat& img_org, double factor);
int hand_contours(Mat& src, vector<vector<Point>>& ctrs);
int biggest_countour(vector<vector<Point>>& ctrs);
int point_in_contour(vector<Point>& h_contour, Point RB_point);
bool forearm_horizontal(vector<Point>& h_contour, const int index, wrist_side side, int distance);
Point find_wrist(vector<Point>& h_contour, const int index, wrist_side side);
void find_apex_valley(const vector<Point>& h_contour, const int index, vector<Point>& apexes, vector<Point>& valleys, int init_dis, int max_dist, const apex_velley_mode C_or_D);
void find_thumb_apex(const vector<Point>& h_contour, const int index, Point& apex, int min_dist);
void leave_just_hand(Mat img_org, Mat& img_work, vector<vector<Point>>& ctrs, int hand_index, string name);

wrist_side basic_detection(Mat& img, string img_name, Rect& bound_rect, vector<Point>& hand_contour, Point& wrist_p_right, Point& wrist_p_left, int& wrist_width);
void calibration_front_function(Mat& img, string img_name, Rect& bounding_rect, vector<Point> contour, Point& wrist_right, Point& wrist_left, Finger* calibration_fingers);
void fingers_detection_function(string img_name, int calib_wrist_width, Finger* cal_fin, int* fin_pos);
uint8_t rotation_detection_function(string img_calib_name, string img_detect_name, const Rect & calib_front_rect);

#endif // !HEADERS_H_

