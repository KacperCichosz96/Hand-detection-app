#include "headers.h"


void display_img(Mat &src, const string name)
{
	Mat dst;
	resize(src, dst, Size(0, 0), 0.25, 0.25);
	namedWindow(name, WINDOW_AUTOSIZE);
	imshow(name, dst);
	waitKey(0);
	destroyWindow(name);
}

void my_thresholding(Mat& src, Mat& img_org, double factor)		//function calculating threshold value based on average value of hand pixels
{																//and background pixels
	Mat img_org_clone = img_org.clone();

	int rect_X = (src.cols / 2) - (src.cols / 20);
	int rect_Y = (src.rows * 7 / 10) - (src.rows / 10);
	int rect_size = src.cols / 10;
	Rect rect_in_hand(rect_X, rect_Y, rect_size, rect_size);

	rectangle(img_org_clone, rect_in_hand, Scalar(0, 0, 255), 10);	

	long hand_threshold_mean = 0;
	int hand_counter = 0;

	for (int i = rect_in_hand.x; i < rect_in_hand.x + rect_in_hand.width; i++)
		for (int j = rect_in_hand.y; j < rect_in_hand.y + rect_in_hand.height; j++)
		{
			hand_threshold_mean += src.at<uchar>(j, i);		//in Mat::at()<> insert coordinates in reverse order, that means (y,x)
			hand_counter++;
		}

	hand_threshold_mean /= hand_counter;
	cout << endl << "Average pixels value on the hand: " << hand_threshold_mean << endl;

	//background pixels:
	long back_threshold_mean = 0;			
	int back_counter = 0;

	Point back_p1(src.cols / 10, src.rows * 9 / 10);
	Point back_p2(src.cols / 10, src.rows / 10);
	Point back_p3(src.cols * 9 / 10, src.rows / 10);
	Point back_p4(src.cols * 9 / 10, src.rows * 9 / 10);

	LineIterator back_iter(src, back_p1, back_p2);
	for (int i = 0; i < back_iter.count; i++, back_iter++)
	{
		back_threshold_mean += (*(*back_iter));
		img_org_clone.at<Vec3b>(back_iter.pos().y, back_iter.pos().x) = Vec3b(255, 0.0);
		back_counter++;
	}

	back_iter = LineIterator(src, back_p2, back_p3);
	for (int i = 0; i < back_iter.count; i++, back_iter++)
	{
		back_threshold_mean += (*(*back_iter));
		img_org_clone.at<Vec3b>(back_iter.pos().y, back_iter.pos().x) = Vec3b(255, 0.0);	
		back_counter++;
	}

	back_iter = LineIterator(src, back_p3, back_p4);
	for (int i = 0; i < back_iter.count; i++, back_iter++)
	{
		back_threshold_mean += (*(*back_iter));
		img_org_clone.at<Vec3b>(back_iter.pos().y, back_iter.pos().x) = Vec3b(255, 0.0);	
		back_counter++;
	}

	back_threshold_mean /= back_counter;
	cout << endl << "Average pixels value on the background: " << back_threshold_mean << endl;

	int threshold_mode;
	int final_threshold;

	if (hand_threshold_mean > back_threshold_mean)
	{
		threshold_mode = THRESH_BINARY;
		final_threshold = hand_threshold_mean - factor * (hand_threshold_mean - back_threshold_mean);	
	}
	else
	{
		threshold_mode = THRESH_BINARY_INV;
		final_threshold = hand_threshold_mean + factor * (back_threshold_mean - hand_threshold_mean);
	}

	threshold(src, src, final_threshold, 255, threshold_mode);
	display_img(img_org_clone, "Areas of thresholding");
}

int hand_contours(Mat& src, vector<vector<Point>>& ctrs)	//function finding contours until find less than 100 contours
{
	int k_size = 3;
	Mat dst;
	Mat kernel;

	while (1)
	{
		kernel = getStructuringElement(MORPH_ELLIPSE, Size(k_size, k_size), Point(-1, -1));
		morphologyEx(src, dst, MORPH_OPEN, kernel, Point(-1, -1), 1, BORDER_REPLICATE);	
		findContours(dst, ctrs, RETR_LIST, CHAIN_APPROX_NONE);		

		if (ctrs.size() <= 100)
			break;
		else
		{
			ctrs.clear();
			k_size += 2;
		}
	}

	morphologyEx(src, src, MORPH_CLOSE, kernel, Point(-1, -1), 1, BORDER_REPLICATE);	//morphologyEx() changes source image
	return k_size;
}

int biggest_countour(vector<vector<Point>>& ctrs)
{
	int biggest_index = 0;

	for (int i = 1; i < ctrs.size(); i++)
		if (ctrs[i].size() > ctrs[biggest_index].size())
			biggest_index = i;

	return biggest_index;
}

int point_in_contour(vector<Point>& h_contour, Point RB_point)
{
	for (int i = 0; i < h_contour.size(); i++)
	{
		if (h_contour[i].x != RB_point.x)
			continue;
		else if (h_contour[i].y == RB_point.y)
			return i;
	}

	return -1;
}

bool forearm_vertical(vector<Point>& h_contour, const int index, wrist_side side, int distance)		//function checking if forearm is vertical
																									// or tilted to any side, that influences 
																									//on procedure of finding wrist points
{
	int size = h_contour.size();
	int temp_index;
	int neighbour;
	int counter = 0;
	index_mode mode = DECREMENT_INDEX;
	bool is_vertical = false;			

	if (index == (size - 1))
		temp_index = 0;
	else
		temp_index = index + 1;

	const int start_index = temp_index;		//pocz¹tkowy punkt przedramienia, tj. ten na samym dole obrazu

	//checking if decrementation or incrementation of index cause movement up in hand contour:
	if (h_contour[temp_index].y < h_contour[index].y)		//movement up is equal to state when Y coordinate value decreases
		mode = INCREMENT_INDEX;
	else
		mode = DECREMENT_INDEX;

	switch (mode)
	{
	case INCREMENT_INDEX:
		if (side == WRIST_RIGHT)
		{
			for (int i = 0; i < distance; i++)
			{
				(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
				if (h_contour[neighbour].x < h_contour[start_index].x)
					counter++;
				if (counter > 0.1*distance)		//if at the beginning we move constantly to the left, we can assume that forearm
												//is vertical on this side
				{
					is_vertical = true;
					break;
				}
				temp_index = neighbour;
			}
		}
		else if (side == WRIST_LEFT)
		{
			for (int i = 0; i < distance; i++)
			{
				(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
				if (h_contour[neighbour].x > h_contour[start_index].x)
					counter++;
				if (counter > 0.1*distance)		//if at the beginning we move constantly to the right, we can assume that forearm
												//is vertical on this side	
				{
					is_vertical = true;
					break;
				}
				temp_index = neighbour;
			}
		}
		break;
	case DECREMENT_INDEX:
		if (side == WRIST_RIGHT)
		{
			for (int i = 0; i < distance; i++)
			{
				(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
				if (h_contour[neighbour].x < h_contour[start_index].x)
					counter++;
				if (counter > 0.1*distance)		
				{
					is_vertical = true;
					break;
				}
				temp_index = neighbour;
			}
		}
		else if (side == WRIST_LEFT)
		{
			for (int i = 0; i < distance; i++)
			{
				(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
				if (h_contour[neighbour].x > h_contour[start_index].x)
					counter++;
				if (counter > 0.1*distance)	
				{
					is_vertical = true;
					break;
				}
				temp_index = neighbour;
			}
		}
		break;
	}
	return is_vertical;
}

Point find_wrist(vector<Point>& h_contour, const int index, wrist_side side)	//function finding first wrist point - start point in procedure
																				//of detecting fingers apexes and valleys
{
	int size = h_contour.size();
	int temp_index;
	int wrist_index;
	int neighbour;
	int last_neighbour;
	int check_distance = 200;
	index_mode mode = DECREMENT_INDEX;
	bool indeed_wrist = true;

	if (index == (size - 1))
		temp_index = 0;
	else
		temp_index = index + 1;

	//checking if decrementation or incrementation of index cause movement up in hand contour:
	if (h_contour[temp_index].y < h_contour[index].y)		//movement up is equal to state when Y coordinate value decreases
		mode = INCREMENT_INDEX;
	else
		mode = DECREMENT_INDEX;

	switch (mode)
{
	case INCREMENT_INDEX:
		if (side == WRIST_RIGHT)
		{
			while (1)
			{
				indeed_wrist = true;

				(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
				if (h_contour[neighbour].x > h_contour[temp_index].x)	//if the neighbour point has greater X coordinate value,
																		//it is located more to the right	
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;
					for (int i = 0; i < check_distance; i++)
					{
						temp_index = neighbour;
						(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
						if (h_contour[neighbour].x < h_contour[temp_index].x)	
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)
						{
							if (abs(h_contour[neighbour].x - h_contour[wrist_index].x) < (0.1*check_distance))	
							{
								indeed_wrist = false;
								temp_index = last_neighbour;
							}
						}
					}
					if (indeed_wrist)
						break;
				}
				temp_index = neighbour;
			}
		}
		else if (side == WRIST_LEFT)
		{
			while (1)
			{
				indeed_wrist = true;

				(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
				if (h_contour[neighbour].x < h_contour[temp_index].x)	//if the neighbour point has less X coordinate value,
																		//it is located more to the left
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;
					for (int i = 0; i < check_distance; i++)
					{
						temp_index = neighbour;
						(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
						if (h_contour[neighbour].x > h_contour[temp_index].x)	
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)	
						{
							if (abs(h_contour[wrist_index].x - h_contour[neighbour].x) < (0.1*check_distance))	
							{
								indeed_wrist = false;
								temp_index = last_neighbour;
							}
						}
					}
					if (indeed_wrist)
						break;
				}
				temp_index = neighbour;
			}
		}
		break;
	case DECREMENT_INDEX:
		if (side == WRIST_RIGHT)
		{
			while (1)
			{
				indeed_wrist = true;

				(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
				if (h_contour[neighbour].x > h_contour[temp_index].x)	//if the neighbour point has greater X coordinate value,
																		//it is located more to the right
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;

					for (int i = 0; i < check_distance; i++)	
					{
						temp_index = neighbour;
						(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
						if (h_contour[neighbour].x < h_contour[temp_index].x)	
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)
						{
							if (abs(h_contour[neighbour].x - h_contour[wrist_index].x) < (0.1*check_distance))	
							{
								indeed_wrist = false;
								temp_index = last_neighbour;
							}
						}
					}
					if (indeed_wrist)
						break;
				}
				temp_index = neighbour;
			}
		}
		else if (side == WRIST_LEFT)
		{
			while (1)
			{
				indeed_wrist = true;

				(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
				if (h_contour[neighbour].x < h_contour[temp_index].x)	//if the neighbour point has less X coordinate value,
																		//it is located more to the left
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;

					for (int i = 0; i < check_distance; i++)	
					{
						temp_index = neighbour;
						(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
						if (h_contour[neighbour].x > h_contour[temp_index].x)	
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)
						{
							if (abs(h_contour[wrist_index].x - h_contour[neighbour].x) < (0.1*check_distance))	
							{
								indeed_wrist = false;
								temp_index = last_neighbour;
							}
						}
					}
					if (indeed_wrist)
						break;
				}
				temp_index = neighbour;
			}
		}
		break;
	}
	return Point(h_contour[wrist_index]);
}

void find_apex_valley(const vector<Point>& h_contour, const int index, vector<Point>& apexes, vector<Point>& valleys, int init_dist, int max_dist, const apex_velley_mode C_or_D)
{
	int size = h_contour.size();
	int temp_index_init;
	int neighbour;
	int current_dist = 0;
	index_mode mode = INCREMENT_INDEX;
	apex_velley_mode A_or_V = APEX;

	if (index == (size - 1))
		temp_index_init = 0;
	else
		temp_index_init = index + 1;

	//checking if assumed INCREMENT_INDEX results as movement up:
	if (h_contour[temp_index_init].y >= h_contour[index].y)		//if it is not movement up, Y coordinate value increases or is the same	
	{
		mode = DECREMENT_INDEX;
		(index != 0) ? (temp_index_init = index - 1) : (temp_index_init = size - 1);
	}

	int temp_index = 0;

	while (init_dist <= max_dist)
	{
		temp_index = temp_index_init;
		current_dist = 0;
		A_or_V = APEX;

		while (temp_index != index)
		{
			if (mode == INCREMENT_INDEX)
				(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
			else
				(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);

			if (A_or_V == APEX)
			{
				if (h_contour[neighbour].y > h_contour[temp_index].y)
				{
					if (current_dist > init_dist)
					{
						apexes.push_back(h_contour[temp_index]);
						current_dist = 0;
					}
					else
						current_dist++;

					A_or_V = VALLEY;
				}
				else
					current_dist++;
			}
			else if (A_or_V == VALLEY)
			{
				if (h_contour[neighbour].y < h_contour[temp_index].y)	
				{
					if (current_dist > init_dist)
					{
					valleys.push_back(h_contour[temp_index]);
					current_dist = 0;
					}
					else
						current_dist++;

					A_or_V = APEX;
				}
				else
					current_dist++;
			}
			temp_index = neighbour;

			if ((C_or_D == CALIBRATION) && (apexes.size() == 4) && (valleys.size() == 4))	//during calibration we can assume that we get exactly 
				break;																		//4 apexes and 4 valleys points
			else if ((C_or_D == DETECTION) && (apexes.size() == 4))		//during fingers detection we can assume that we get exactly 4 apexes, 
				break;													//but 4 valleys are not necessary (e.g. when thumb is hidden)
		}

		if ((C_or_D == CALIBRATION) && (apexes.size() == 4) && (valleys.size() == 4))	
			break;																		
		else if ((C_or_D == DETECTION) && (apexes.size() == 4))		
			break;
		else
		{
			apexes.clear();
			valleys.clear();
		}

		init_dist++;
	}
}

void find_thumb_apex(const vector<Point>& h_contour, const int index, Point& apex, int min_dist)
{
	int size = h_contour.size();
	int temp_index;
	int neighbour;
	int current_dist = 0;
	index_mode mode = INCREMENT_INDEX;
	bool is_found = false;

	if (index == (size - 1))
		temp_index = 0;
	else
		temp_index = index + 1;	

	//checking if assumed INCREMENT_INDEX results as movement up:
	if (h_contour[temp_index].y >= h_contour[index].y)		//if it is not movement up, Y coordinate value increases or is the same
	{
		mode = DECREMENT_INDEX;
		(index != 0) ? (temp_index = index - 1) : (temp_index = size - 1);
	}

	//thumb apex is a point where the contour starts to go left (X coordinate value is getting less)
	while (!is_found)
	{
		if (mode == INCREMENT_INDEX)
			(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
		else
			(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);

		if (h_contour[neighbour].x < h_contour[temp_index].x)	
		{
			if (current_dist > min_dist)
			{
				apex = h_contour[temp_index];
				is_found = true;
			}
			else
				current_dist++;
		}
		else
			current_dist++;

		if(!is_found)
			temp_index = neighbour;
	}
}

void leave_just_hand(Mat img_org, Mat & img_work, vector<vector<Point>>& ctrs, int hand_index, string name)
{
	int is_inside = 0;

	for (int i = 0; i < ctrs.size(); i++)
	{
		if (i == hand_index)
			drawContours(img_org, ctrs, i, Scalar(255, 0, 0), 6);	//draw the hand contour to display
		else
		{
			is_inside = pointPolygonTest(ctrs[hand_index], ctrs[i].front(), false);

			if (is_inside >= 0)		//if contour is inside the hand contour
			{
				drawContours(img_org, ctrs, i, Scalar(0, 255, 0), 6);
				drawContours(img_work, ctrs, i, 255, FILLED);			//draw and fill as a part of hand
			}
			else	//je¿eli jest poza konturem d³oni
			{
				drawContours(img_org, ctrs, i, Scalar(0, 0, 255), 6);
				drawContours(img_work, ctrs, i, 0, FILLED);				//draw and fill as a part of background
			}
		}
	}

	//display_img(img_org, name + " - all contours");				//uncomment to display results
	//imwrite("RESULT " + name + " - all contours.jpg", img_org);	//uncomment to save results
}

wrist_side basic_detection(Mat & img, string img_name, Rect& bound_rect, vector<Point>& hand_contour, Point & wrist_p_R, Point & wrist_p_L, int & wrist_width)
{
	display_img(img, img_name);
	Mat work_img = img.clone();

	//1. choice of color space (HSV space):
	vector<Mat> HSV_channels;
	cvtColor(work_img, work_img, COLOR_BGR2HSV);

	split(work_img, HSV_channels);
	work_img = HSV_channels[1];		//choice of S channel in HSV color space, choice is made according to previous trials color spaces

	//display_img(work_img, (img_name + " - S channel"));				//uncomment to display result of operation
	imwrite(("RESULT " + img_name + " - S channel.jpg"), work_img);		//uncomment to save result of operation

	//2. blur:
	blur(work_img, work_img, Size((work_img.cols / 200), (work_img.cols / 200)), Point(-1, -1), BORDER_REPLICATE);
	//display_img(work_img, (img_name + " - blured"));				//uncomment to display result of operation
	//imwrite(("RESULT " +  img_name + " - blured.jpg"), work_img); //uncomment to save result of operation

	//3. thresholding:
	my_thresholding(work_img, img, 0.40);
	//display_img(work_img, (img_name + " - thresholded"));				//uncomment to display result of operation
	//imwrite(("RESULT " + img_name + " - thresholded.jpg"), work_img);	//uncomment to save result of operation

	//4. image improvement - morphological operations:
	vector<vector<Point>> contours;
	int kernel_size = 0;
	int hand_index = 0;

	kernel_size = hand_contours(work_img, contours);	//hand_contour() - making morphological open until find less than 100 contours
	hand_index = biggest_countour(contours);			//finding index of biggest contour, it should be a contour of the hand
	hand_contour = contours[hand_index];				

	vector<vector<Point>> HAND_contour;					//just contour of the hand
	HAND_contour.push_back(contours[hand_index]);

	cout << endl << "Found contours: " << contours.size() << endl << endl;

	leave_just_hand(img, work_img, contours, hand_index, img_name);
	//display_img(work_img, (img_name + " - just hand (filled)"));					//uncomment to display result of operation
	//imwrite(("RESULT " + img_name + " - just hand (filled).jpg"), work_img);		//uncomment to save result of operation

	//making black image with just a white contour of the hand:
	work_img = Mat::zeros(work_img.rows, work_img.cols, work_img.type());
	drawContours(work_img, HAND_contour, -1, 255, 1);
	//display_img(work_img, (img_name + " - just hand contour"));					//uncomment to display result of operation
	//imwrite(("RESULT " + img_name + " - just hand contour.jpg"), work_img);		//uncomment to save result of operation

	//5. finding of forearm bottom points:
	//bottom right point:
	LineIterator bottom_iter(work_img, Point(work_img.cols - 1, work_img.rows - 1), Point(0, work_img.rows - 1));

	for (int i = 0; i < bottom_iter.count; i++, bottom_iter++)
		if (*(*bottom_iter) == 255)
			break;

	Point right_bottom_point = Point(bottom_iter.pos().x, bottom_iter.pos().y);
	//bottom left point:
	bottom_iter = LineIterator(work_img, Point(0, work_img.rows - 1), Point(work_img.cols - 1, work_img.rows - 1));

	for (int i = 0; i < bottom_iter.count; i++, bottom_iter++)
		if (*(*bottom_iter) == 255)
			break;
	Point left_bottom_point = Point(bottom_iter.pos().x, bottom_iter.pos().y);

	//6. define index of found forearm points in contour of the hand:
	int right_point_index = -2;	
	int left_point_index = -2;

	right_point_index = point_in_contour(HAND_contour[0], right_bottom_point);
	left_point_index = point_in_contour(HAND_contour[0], left_bottom_point);
	circle(img, HAND_contour[0][right_point_index], 50, Scalar(255, 0, 0), FILLED);		//draw circle in right found point
	circle(img, HAND_contour[0][left_point_index], 50, Scalar(0, 255, 0), FILLED);		//draw circle in left found point

	//display_img(img, (img_name + " - forearm points"));					//uncomment to display result of operation
	//imwrite(("RESULT + " + img_name + " - forearm points.jpg"), img);		//uncomment to save result of operation

	//7. finding wrist point:
	wrist_side side;
	Point wrist_point_right;
	Point wrist_point_left;
	Point wrist_point;			//finally chosen wrist point (the one which is lower)

	//checking if forearm is vertical or tilted to the right or to the left:
	//right side:
	bool right_side_good = forearm_vertical(HAND_contour[0], right_point_index, WRIST_RIGHT, 100);

	if (right_side_good)	//if not tilted on the right side, we can search wrist_point on the right side of the hand
		wrist_point_right = find_wrist(HAND_contour[0], right_point_index, WRIST_RIGHT);
	else
		wrist_point_right = Point(100, 100);	//set point in the left high corner of the image, this point is not taken under consideration
	//left side:
	bool left_side_good = forearm_vertical(HAND_contour[0], left_point_index, WRIST_LEFT, 100);

	if (left_side_good)		//if not tilted on the left side, we can search wrist_point on the right side of the hand
		wrist_point_left = find_wrist(HAND_contour[0], left_point_index, WRIST_LEFT);
	else
		wrist_point_left = Point(50, 50);	//set point in the left high corner of the image, this point is not taken under consideration

	circle(img, wrist_point_right, 30, Scalar(0, 0, 127), FILLED);		//draw circle in right found point of the wrist
	circle(img, wrist_point_left, 30, Scalar(127, 0, 0), FILLED);		//draw circle in lef found point of the wrist

	//choose the one which is lower:
	if (wrist_point_left.y > wrist_point_right.y)
	{
		wrist_point = wrist_point_left;
		side = WRIST_LEFT;
	}
	else
	{
		wrist_point = wrist_point_right;
		side = WRIST_RIGHT;
	}

	//display_img(img, (img_name + " - wrists points"));		//uncomment to display result of operation
	//imwrite((img_name + " - wrists points.jpg"), img);		//uncomment to save result of operation

	//8. drawing wrist line:
	LineIterator wrist_iter(work_img, wrist_point, Point(0, 0));

	(side == WRIST_RIGHT) ? (wrist_iter = LineIterator(work_img, wrist_point, Point(0, wrist_point.y))) :
							(wrist_iter = LineIterator(work_img, wrist_point, Point(work_img.cols - 1, wrist_point.y)));
	wrist_iter++;
	wrist_width = 1;

	while (wrist_width < wrist_iter.count)	//drawing line until find an opposite side hand contour
	{
		if (*(*wrist_iter) != 255)
			work_img.at<uchar>(wrist_iter.pos().y, wrist_iter.pos().x) = 255;
		else break;

		wrist_iter++;
		wrist_width++;
	}

	cout << img_name << " - wrist width: " << wrist_width << endl;

	Point wrist_line_point;		//left wrist point - this is the start point in next procedure of
								//finding fingers apexes and valleys points of the hand
	Point wrist_line_point_thumb;	//right wrist point - will be useful to find apex of thumb
	
	if (side == WRIST_RIGHT)
	{
		wrist_line_point = Point(wrist_iter.pos().x, wrist_iter.pos().y);
		wrist_line_point_thumb = wrist_point;
	}
	else
	{
		wrist_line_point = wrist_point;
		wrist_line_point_thumb = Point(wrist_iter.pos().x, wrist_iter.pos().y);
	}

	wrist_p_L = wrist_line_point;		
	wrist_p_R = wrist_line_point_thumb;			

	//9. deleting the part of the contour under the wrist line:
	for (int i = wrist_line_point.y + 1; i < work_img.rows; i++)	
		for (int j = 0; j < work_img.cols; j++)						
			work_img.at<uchar>(i, j) = 0;

	//display_img(work_img, (img_name + " - just hand"));					//uncomment to display result of operation
	//imwrite(("RESULT " + img_name + " - just hand.jpg"), work_img);		//uncomment to save result of operation

	//10. draw a rectangle around the hand contour:
	HAND_contour.clear();
	findContours(work_img.clone(), HAND_contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	bound_rect = boundingRect(HAND_contour[0]);			
	rectangle(img, bound_rect, Scalar(0, 0, 0), 4);		

	cout << img_name << " - hand height: " << bound_rect.height << endl;
	cout << img_name << " - hand width: " << bound_rect.width << endl;

	display_img(img, (img_name + " - hand in rectangle"));
	imwrite(("RESULT " + img_name + " - hand in rectangle.jpg"), img);

	return side;
}

void calibration_front_function(Mat& img, string img_name, Rect& bounding_rect, vector<Point> contour, Point& wrist_right, Point& wrist_left, Finger* calibration_fingers)
{
	//1. finding fingers apexes and valleys:
	int wrist_start_index = -2;

	wrist_start_index = point_in_contour(contour, wrist_left);
	circle(img, contour[wrist_start_index], 50, Scalar(0, 0, 0), FILLED);

	//finding apexes and valleys of 4 fingers (except thumb):
	vector<Point> finger_apex;
	vector<Point> finger_valley;

	find_apex_valley(contour, wrist_start_index, finger_apex, finger_valley, (bounding_rect.height*0.20), (bounding_rect.height / 2), CALIBRATION); //

	//finding apex of the thumb:
	int wrist_point_index_thumb = point_in_contour(contour, wrist_right);
	Point thumb_apex;

	find_thumb_apex(contour, wrist_point_index_thumb, thumb_apex, (bounding_rect.height*0.3));
	finger_apex.push_back(thumb_apex);

	for (int i = 0; i < finger_apex.size(); i++)
		circle(img, finger_apex[i], 30, Scalar(255, 0, 255), FILLED);		//drawing found apex points

	for (int i = 0; i < finger_valley.size(); i++)
		circle(img, finger_valley[i], 30, Scalar(0, 255, 255), FILLED);		//drawing found valley points

	display_img(img, (img_name + " - apexes and valleys"));
	imwrite(("RESULT " + img_name + " - apexes and valleys.jpg"), img);

	//2. defining suspected areas to find each finger:
	//little finger:
	calibration_fingers[0] = Finger(img, Point(bounding_rect.x, 0), finger_valley[0], finger_apex[0]);	
	calibration_fingers[0].draw_regions();
	//ring finger:
	calibration_fingers[1] = Finger(img, calibration_fingers[0].give_point(), finger_valley[1], finger_apex[1]);
	calibration_fingers[1].draw_regions();
	//middle finger:
	calibration_fingers[2] = Finger(img, calibration_fingers[1].give_point(), finger_valley[2], finger_apex[2]);
	calibration_fingers[2].draw_regions();
	//index finger:
	Point fin_4_bottom_point(bounding_rect.x + bounding_rect.width, finger_valley[2].y);
	calibration_fingers[3] = Finger(img, calibration_fingers[2].give_point(), fin_4_bottom_point, finger_apex[3]);
	calibration_fingers[3].draw_regions();
	//thumb:
	Point thumb_top_left(finger_valley[3].x, finger_valley[2].y);
	Point thumb_bottom_right(bounding_rect.x + 1.005*bounding_rect.width, bounding_rect.y + bounding_rect.height);	

	calibration_fingers[4] = Finger(img, thumb_top_left, thumb_bottom_right, Finger::THUMB_MODE);
	calibration_fingers[4].draw_regions();
	display_img(img, (img_name + " - fingers regions"));
	imwrite(("RESULT " + img_name + " - fingers regions.jpg"), img);

	cout << "\n********************************************\n\n\a";
}

void fingers_detection_function(string img_name, int calib_wrist_width, Finger* cal_fin, int* fin_pos)
{
	Mat proper_IMG;
	proper_IMG = imread(img_name);

	if (proper_IMG.data == NULL)
	{
		cout << "Failed to load an image to calibration in front view (image name:\"" << img_name << "\")\a" << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}
	
	string proper_IMG_name("Fingers detection");
	Rect proper_IMG_rect;
	vector<Point> proper_IMG_contour;
	Point proper_IMG_wrist_right;
	Point proper_IMG_wrist_left;
	int proper_IMG_wrist_width;
	wrist_side proper_IMG_side;

	proper_IMG_side =
		basic_detection(proper_IMG, proper_IMG_name, proper_IMG_rect, proper_IMG_contour, proper_IMG_wrist_right, proper_IMG_wrist_left, proper_IMG_wrist_width);

	//calculating scaling factor:
	double scale_factor = double(proper_IMG_wrist_width) / double(calib_wrist_width);
	cout.precision(4);
	cout << "Scaling factor: " << scale_factor << endl;

	//scaling surrounding rectangle:
	proper_IMG_rect.width *= scale_factor;
	proper_IMG_rect.height *= scale_factor;
	(proper_IMG_side == WRIST_RIGHT) ?
		(proper_IMG_rect.y = proper_IMG_wrist_right.y - proper_IMG_rect.height) :
		(proper_IMG_rect.y = proper_IMG_wrist_left.y - proper_IMG_rect.height);

	rectangle(proper_IMG, proper_IMG_rect, Scalar(0, 0, 255), 4);
	display_img(proper_IMG, (proper_IMG_name + " - hand in scaled rectangle"));
	imwrite(("RESULT " + proper_IMG_name + " - hand in scaled rectangle.jpg"), proper_IMG);

	//1. defining regions suspected to find finger:
	Point fingers_START(proper_IMG_rect.x, 0);	

	Finger prop_fin_1(proper_IMG, fingers_START, cal_fin[0], scale_factor, Finger::FINGER_MODE);
	prop_fin_1.draw_regions();
	Finger prop_fin_2(proper_IMG, prop_fin_1.give_point(), cal_fin[1], scale_factor, Finger::FINGER_MODE);
	prop_fin_2.draw_regions();
	Finger prop_fin_3(proper_IMG, prop_fin_2.give_point(), cal_fin[2], scale_factor, Finger::FINGER_MODE);
	prop_fin_3.draw_regions();
	Finger prop_fin_4(proper_IMG, prop_fin_3.give_point(), cal_fin[3], scale_factor, Finger::FINGER_MODE);
	prop_fin_4.draw_regions();
	//thumb:
	Point prop_thumb_point(proper_IMG.cols - 1, proper_IMG_rect.y + proper_IMG_rect.height);
	Finger prop_thumb(proper_IMG, prop_thumb_point, cal_fin[4], scale_factor, Finger::THUMB_MODE);
	prop_thumb.draw_regions();

	//display_img(proper_IMG, (proper_IMG_name + " - fingers regions"));				//uncomment to display results
	//imwrite(("RESULT " + proper_IMG_name + " - fingers regions.jpg"), proper_IMG);	//uncomment to save results

	//2. finding fingers apexes and valleys:
	int PROPER_wrist_start_index = -2;		

	PROPER_wrist_start_index = point_in_contour(proper_IMG_contour, proper_IMG_wrist_left);
	//circle(proper_IMG, proper_IMG_contour[PROPER_wrist_start_index], 50, Scalar(0, 0, 0), FILLED);	//draw circle to check start point of
																										//apexes and vallesy finding:
	vector<Point> PROPER_finger_apex;
	vector<Point> PROPER_finger_valley;

	find_apex_valley(proper_IMG_contour, PROPER_wrist_start_index, PROPER_finger_apex,
		PROPER_finger_valley, 0, (proper_IMG_rect.height / 2), DETECTION);

	//finding thumb apex:
	int PROPER_wrist_point_index_thumb = point_in_contour(proper_IMG_contour, proper_IMG_wrist_right);
	Point PROPER_thumb_apex;

	find_thumb_apex(proper_IMG_contour, PROPER_wrist_point_index_thumb, PROPER_thumb_apex, (proper_IMG_rect.height*0.35));
	PROPER_finger_apex.push_back(PROPER_thumb_apex);

	for (int i = 0; i < PROPER_finger_apex.size(); i++)
		circle(proper_IMG, PROPER_finger_apex[i], 30, Scalar(255, 0, 255), FILLED);

	for (int i = 0; i < PROPER_finger_valley.size(); i++)
		circle(proper_IMG, PROPER_finger_valley[i], 30, Scalar(0, 255, 255), FILLED);

	display_img(proper_IMG, (proper_IMG_name + " - Appexes and valleys"));
	imwrite(("RESULT " + proper_IMG_name + " - Appexes and valleys.jpg"), proper_IMG);

	//3. assigning number of location region for each finger:
	//1) little finger:
	fin_pos[0] = prop_fin_1.find_fin_location(PROPER_finger_apex[0]);
	if (fin_pos[0] == 4)		//sprawdzeie rejonu palca serdecznego
		fin_pos[0] = prop_fin_2.find_fin_location(PROPER_finger_apex[0]);
	//2) ring finger:
	fin_pos[1] = prop_fin_2.find_fin_location(PROPER_finger_apex[1]);
	if (fin_pos[1] == 4)		//sprawdzeie rejonu palca ma³ego
		fin_pos[1] = prop_fin_1.find_fin_location(PROPER_finger_apex[1]);
	if (fin_pos[1] == 4)		//sprawdzeie rejonu palca œrodkowego
		fin_pos[1] = prop_fin_3.find_fin_location(PROPER_finger_apex[1]);
	//3) middle finger:
	fin_pos[2] = prop_fin_3.find_fin_location(PROPER_finger_apex[2]);
	if (fin_pos[2] == 4)		//sprawdzeie rejonu palca serdecznego
		fin_pos[2] = prop_fin_2.find_fin_location(PROPER_finger_apex[2]);
	if (fin_pos[2] == 4)		//sprawdzeie rejonu palca wskazujacego
		fin_pos[2] = prop_fin_4.find_fin_location(PROPER_finger_apex[2]);
	//4) index finger:
	fin_pos[3] = prop_fin_4.find_fin_location(PROPER_finger_apex[3]);
	if (fin_pos[3] == 4)		//sprawdzeie rejonu palca œrodkowego
		fin_pos[3] = prop_fin_3.find_fin_location(PROPER_finger_apex[3]);
	//5) thumb:
	fin_pos[4] = prop_thumb.find_fin_location(PROPER_finger_apex[4]);

	cout << endl << "Fingers found in regions (region number):" << endl;
	cout << "1 - little finger: " << fin_pos[0] << endl;
	cout << "2 - ring finger: " << fin_pos[1] << endl;
	cout << "3 - middle finger: " << fin_pos[2] << endl;
	cout << "4 - index finger: " << fin_pos[3] << endl;
	cout << "5 - thumb: " << fin_pos[4] << endl << endl;
	
	display_img(proper_IMG, (proper_IMG_name + " - detected fingers"));
	imwrite(("RESULT " + proper_IMG_name + " - detected fingers.jpg"), proper_IMG);
}

uint8_t rotation_detection_function(string img_calib_name, string img_detect_name, const Rect & calib_front_rect)
{
	//next part of the calibration - calibration in side view:
	Mat calib_SD;
	calib_SD = imread(img_calib_name);

	if (calib_SD.data == NULL)
	{
		cout << "Failed to load an image to calibration in front view (image name:\"" << img_calib_name << "\")\a" << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}
	
	string calib_SD_name("Calib. side view");
	Rect calib_SD_rect;
	vector<Point> calib_SD_contour;
	Point calib_SD_wrist_right;
	Point calib_SD_wrist_left;
	int calib_SD_wrist_width;

	basic_detection(calib_SD, calib_SD_name, calib_SD_rect, calib_SD_contour, calib_SD_wrist_right, calib_SD_wrist_left, calib_SD_wrist_width);

	int MAX_hand_width = calib_front_rect.width;
	int MIN_hand_width = calib_SD_rect.width;
	int hand_width_RANGE = MAX_hand_width - MIN_hand_width;
	int mean_height = (calib_front_rect.height + calib_SD_rect.height) / 2;

	//proper part of detecting revolution angle in wrist:
	Mat rot_img;
	rot_img = imread(img_detect_name);

	if (rot_img.data == NULL)
	{
		cout << "Failed to load an image to calibration in front view (image name:\"" << img_detect_name << "\")\a" << endl;
		Sleep(3000);
		exit(EXIT_FAILURE);
	}

	string rot_img_name("Detection ROTATION");
	Rect rot_img_rect;
	vector<Point> rot_img_contour;
	Point rot_img_wrist_right;
	Point rot_img_wrist_left;
	int rot_img_wrist_width;

	basic_detection(rot_img, rot_img_name, rot_img_rect, rot_img_contour, rot_img_wrist_right, rot_img_wrist_left, rot_img_wrist_width);

	int rot_hand_height = rot_img_rect.height;
	double scale_factor = (double)rot_hand_height / double(mean_height);
	int rot_hand_width = rot_img_rect.width*scale_factor;

	double cosine_angle = double(rot_hand_width - MIN_hand_width) / double(hand_width_RANGE);

	if (cosine_angle <= 0)		//if cosinus indicate that hand is narrower than during the calibration	
		cosine_angle = 0.0;	//set 0.0 (max. revolution)
	else if (cosine_angle > 1)	//if cosinus indicate that hand is wider than during the calibration	
		cosine_angle = 1.0;	//set 1.0 (no revolution)

	cout.precision(4);
	cout << endl << "Cosine of the revolution angle = " << cosine_angle << endl;

	const double RAD_to_DEGREES = 180.0 / 3.1416;
	double hand_rotation_angle = acos(cosine_angle);
	hand_rotation_angle *= RAD_to_DEGREES;

	const double ANGLE_to_STEPS = 50.0 / 90.0;		//90 degrees are 50 changes (LOW->HIGH->LOW) for stepper	motor (that is set in MCU)
	uint8_t hand_rotation_MCU = hand_rotation_angle * ANGLE_to_STEPS;
	hand_rotation_MCU *= 2;												

	cout << "Revolution angle (degrees): " << hand_rotation_angle << endl;
	cout << "Revolution angle (changes LOW->HIGH): " << (int)hand_rotation_MCU << endl << endl;	
	cout << "\n********************************************\n\n\a";							

	return hand_rotation_MCU;
}
