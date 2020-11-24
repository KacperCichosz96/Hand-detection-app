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

void my_thresholding(Mat& src, Mat& img_org, double factor)
{
	Mat img_org_clone = img_org.clone();
	//jasno�� pikseli na r�ce:
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
			hand_threshold_mean += src.at<uchar>(j, i);		//metoda Mat::at()<> przyjmuje wsp�rz�dne w zmienionej kolejno�ci, tj. (y,x)
			hand_counter++;
		}

	hand_threshold_mean /= hand_counter;
	cout << endl << "Warto�� �rednia na r�ce: " << hand_threshold_mean << endl;

	//jasno�� pikseli t�a:
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
		img_org_clone.at<Vec3b>(back_iter.pos().y, back_iter.pos().x) = Vec3b(255, 0.0);	//maluj na niebiesko
		back_counter++;
	}

	back_iter = LineIterator(src, back_p2, back_p3);
	for (int i = 0; i < back_iter.count; i++, back_iter++)
	{
		back_threshold_mean += (*(*back_iter));
		img_org_clone.at<Vec3b>(back_iter.pos().y, back_iter.pos().x) = Vec3b(255, 0.0);	//maluj na niebiesko
		back_counter++;
	}

	back_iter = LineIterator(src, back_p3, back_p4);
	for (int i = 0; i < back_iter.count; i++, back_iter++)
	{
		back_threshold_mean += (*(*back_iter));
		img_org_clone.at<Vec3b>(back_iter.pos().y, back_iter.pos().x) = Vec3b(255, 0.0);	//maluj na niebiesko
		back_counter++;
	}

	back_threshold_mean /= back_counter;
	cout << "Warto�� �rednia t�a: " << back_threshold_mean << endl;

	int threshold_mode;
	int final_threshold;

	if (hand_threshold_mean > back_threshold_mean)
	{
		threshold_mode = THRESH_BINARY;
		final_threshold = hand_threshold_mean - factor * (hand_threshold_mean - back_threshold_mean);	//0.2 dla kalibracji; 
																										//0.45-0.50 dla w�a�ciwego obrazu
	}
	else
	{
		threshold_mode = THRESH_BINARY_INV;
		final_threshold = hand_threshold_mean + factor * (back_threshold_mean - hand_threshold_mean);
	}

	threshold(src, src, final_threshold, 255, threshold_mode);
	display_img(img_org_clone, "Areas of thresholding");
	imwrite("Areas of thresholding.jpg", img_org_clone);
}

int hand_contours(Mat& src, vector<vector<Point>>& ctrs)
{
	int k_size = 3;
	Mat dst;
	Mat kernel;

	while (1)
	{

		kernel = getStructuringElement(MORPH_ELLIPSE, Size(k_size, k_size), Point(-1, -1));
		morphologyEx(src, dst, MORPH_OPEN, kernel, Point(-1, -1), 1, BORDER_REPLICATE);	//tu nie zmieniam obrazu �r�d�owego
		findContours(dst, ctrs, RETR_LIST, CHAIN_APPROX_NONE);		//findContours() modyfikuje obraz �r�d�owy

		if (ctrs.size() <= 100)
			break;
		else
		{
			ctrs.clear();
			k_size += 2;
		}
	}

	morphologyEx(src, src, MORPH_CLOSE, kernel, Point(-1, -1), 1, BORDER_REPLICATE);		//tu zmieniam obraz �r�d�owy
	return k_size;
}

int biggest_countour(vector<vector<Point>>& ctrs)
{
	int biggest_index = 0;

	for (int i = 1; i < ctrs.size(); i++)
	{
		if (ctrs[i].size() > ctrs[biggest_index].size())
			biggest_index = i;
	}

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

bool forearm_horizontal(vector<Point>& h_contour, const int index, wrist_side side, int distance)
{
	int size = h_contour.size();
	int temp_index;
	int neighbour;
	int counter = 0;
	index_mode mode = DECREMENT_INDEX;
	bool is_horizontal = false;			//na pocz�tku zak�adamy, �e przedrami� NIE jest pionowo ustawione

	//sprawdzenie czy nie jeste�my na ko�cu wektora punkt�w:
	if (index == (size - 1))
		temp_index = 0;
	else
		temp_index = index + 1;

	const int start_index = temp_index;		//pocz�tkowy punkt przedramienia, tj. ten na samym dole obrazu

	//sprawdzenie czy dekrementacja czy inkrementacja indeksu powoduje ruch do g�ry w konturze:
	if (h_contour[temp_index].y < h_contour[index].y)		//ruch do g�ry na obrazie to zmniejszanie sie warto�� wsp�rz�dnej y
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
				if (counter > 0.1*distance)		//je�li na pocz�tku par� razy skrecamy w lewo, tzn. �e po prawej stronie nadgarstka mo�emy szuka� punktu nadgarstka
				{
					is_horizontal = true;
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
				if (counter > 0.1*distance)		//je�li na pocz�tku par� razy skrecamy w prawo, tzn. �e po lewej stronie nadgarstka mo�emy szuka� punktu nadgarstka
				{
					is_horizontal = true;
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
				if (counter > 0.1*distance)		//je�li na pocz�tku par� razy skrecamy w lewo, tzn. �e po prawej stronie nadgarstka mo�emy szuka� punktu nadgarstka
				{
					is_horizontal = true;
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
				if (counter > 0.1*distance)		//je�li na pocz�tku par� razy skrecamy w prawo, tzn. �e po lewej stronie nadgarstka mo�emy szuka� punktu nadgarstka
				{
					is_horizontal = true;
					break;
				}
				temp_index = neighbour;
			}
		}
		break;
	}
	return is_horizontal;
}

Point find_wrist(vector<Point>& h_contour, const int index, wrist_side side)
{
	int size = h_contour.size();
	int temp_index;
	int wrist_index;
	int neighbour;
	int last_neighbour;
	int check_distance = 200;
	index_mode mode = DECREMENT_INDEX;
	bool indeed_wrist = true;

	//sprawdzenie czy nie jeste�my na ko�cu wektora punkt�w:
	if (index == (size - 1))
		temp_index = 0;
	else
		temp_index = index + 1;

	//sprawdzenie czy dekrementacja czy inkrementacja indeksu powoduje ruch do g�ry w konturze:
	if (h_contour[temp_index].y < h_contour[index].y)		//ruch do g�ry na obrazie to zmniejszanie sie warto�� wsp�rz�dnej y
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
				if (h_contour[neighbour].x > h_contour[temp_index].x)	//je�eli s�siedni punkt jest bardziej na prawo, tj. ma wi�ksza warto�� x
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;
					for (int i = 0; i < check_distance; i++)	//je�eli kt�ry� z  10 kolejnych s�siednich punkt�w nie jest na lewo, tj. ma mniejsz� warto�� x
					{
						temp_index = neighbour;
						(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
						if (h_contour[neighbour].x < h_contour[temp_index].x)	//czy nie jest na lewo LUB na tej samej szeroko�ci
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)	//sprawdzamy ostatni punkt z seri punkt�w od podejrzanego punktu nadgrastka
						{
							if (abs(h_contour[neighbour].x - h_contour[wrist_index].x) < (0.1*check_distance))	//je�eli to jest prawd� to mamy do czynienia z d�ug�
																												//prost� lini�
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
				if (h_contour[neighbour].x < h_contour[temp_index].x)	//je�eli s�siedni punkt jest bardziej na lewo, tj. ma mniejsz� warto�� x
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;
					for (int i = 0; i < check_distance; i++)	//je�eli kt�ry� z  10 kolejnych s�siednich punkt�w nie jest na prawo, tj. ma wi�ksz� warto�� x
					{
						temp_index = neighbour;
						(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
						if (h_contour[neighbour].x > h_contour[temp_index].x)	//czy nie jest na prawo LUB na tej samej szeroko�ci
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)	//sprawdzamy ostatni punkt z seri punkt�w od podejrzanego punktu nadgrastka
						{
							if (abs(h_contour[wrist_index].x - h_contour[neighbour].x) < (0.1*check_distance))	//je�eli to jest prawd� to mamy do czynienia z d�ug�
																												//prost� lini�
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
				if (h_contour[neighbour].x > h_contour[temp_index].x)	//je�eli s�siedni punkt jest bardziej na prawo, tj. ma wi�ksza warto�� x
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;

					for (int i = 0; i < check_distance; i++)	//je�eli kt�ry� z  10 kolejnych s�siednich punkt�w nie jest na lweo, tj. ma mniejsz� warto�� x
					{
						temp_index = neighbour;
						(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
						if (h_contour[neighbour].x < h_contour[temp_index].x)	//czy nie jest na lewo 
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)	//sprawdzamy ostatni punkt z seri punkt�w od podejrzanego punktu nadgrastka
						{
							if (abs(h_contour[neighbour].x - h_contour[wrist_index].x) < (0.1*check_distance))	//je�eli to jest prawd� to mamy do czynienia z d�ug�
																												//prost� lini�
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
				if (h_contour[neighbour].x < h_contour[temp_index].x)	//je�eli s�siedni punkt jest bardziej na lewo, tj. ma mniejsz� warto�� x
				{
					wrist_index = temp_index;
					last_neighbour = neighbour;

					for (int i = 0; i < check_distance; i++)	//je�eli kt�ry� z 20 kolejnych s�siednich punkt�w nie jest na prawo, tj. ma wi�ksz� warto�� x
					{
						temp_index = neighbour;
						(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);
						if (h_contour[neighbour].x > h_contour[temp_index].x)	//czy nie jest na prawo LUB na tej samej szeroko�ci
						{
							indeed_wrist = false;
							temp_index = last_neighbour;
							break;
						}
						if (i == check_distance-1)	//sprawdzamy ostatni punkt z seri punkt�w od podejrzanego punktu nadgrastka
						{
							if (abs(h_contour[wrist_index].x - h_contour[neighbour].x) < (0.1*check_distance))	//je�eli to jest prawd� to mamy do czynienia z d�ug�
																												//prost� lini�
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

	//sprawdzenie czy nie jeste�my na ko�cu wektora punkt�w:
	if (index == (size - 1))
		temp_index_init = 0;
	else
		temp_index_init = index + 1;	//pocz�tkowo zak��damy tryb inkrementacji indesu punkt�w w konturze

	//sprawdzenie czy za�o�ona inkrementacja indeksu daje ruch do g�ry:
	if (h_contour[temp_index_init].y >= h_contour[index].y)		//je�eli nie jest to ruch do g�ry to wsp�rz�dna y ro�nie lub pozostaje sta�a
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
				if (h_contour[neighbour].y > h_contour[temp_index].y)	//je�eli s�siedni punkt jest ni�ej, tj. ma wi�ksz� warto�� y
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
				if (h_contour[neighbour].y < h_contour[temp_index].y)	//je�eli s�siedni punkt jest wy�ej, tj. ma mniejsz� warto�� y
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

			if ((C_or_D == CALIBRATION) && (apexes.size() == 4) && (valleys.size() == 4))	//przy kalibracji musimy otzryma� dok�adnie 4 wierzcho�ki
				break;																		//palc�w i 4 "doliny"
			else if ((C_or_D == DETECTION) && (apexes.size() == 4))		//przy detekcji r�ki potrzebujemy tylko rozpoznania 4 wierzcho�k�w, gdy� nie
				break;													//zawsze wykryjemy 4 doliny, np. przy schowanym kciuku
		}

		if ((C_or_D == CALIBRATION) && (apexes.size() == 4) && (valleys.size() == 4))	//przy kalibracji musimy otzryma� dok�adnie 4 wierzcho�ki
			break;																		//palc�w i 4 "doliny"
		else if ((C_or_D == DETECTION) && (apexes.size() == 4))		//przy detekcji r�ki potrzebujemy tylko rozpoznania 4 wierzcho�k�w, gdy� nie
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

	//sprawdzenie czy nie jeste�my na ko�cu wektora punkt�w:
	if (index == (size - 1))
		temp_index = 0;
	else
		temp_index = index + 1;	//pocz�tkowo zak�adamy tryb inkrementacji indesu punkt�w w konturze

	//sprawdzenie czy za�o�ona inkrementacja indeksu daje ruch do g�ry:
	if (h_contour[temp_index].y >= h_contour[index].y)		//je�eli nie jest to ruch do g�ry to wsp�rz�dna y ro�nie lub pozostaje sta�a
	{
		mode = DECREMENT_INDEX;
		(index != 0) ? (temp_index = index - 1) : (temp_index = size - 1);
	}

	//wyznaczamy wierzcho�ek kciuka, jako punkt, od kt�rego kontur zaczyna przebiega� w lewo, tj. kolejne punkty maj� mniejsz� warto�� X
	while (!is_found)
	{
		if (mode == INCREMENT_INDEX)
			(temp_index == (size - 1)) ? (neighbour = 0) : (neighbour = temp_index + 1);
		else
			(temp_index == 0) ? (neighbour = (size - 1)) : (neighbour = temp_index - 1);

		if (h_contour[neighbour].x < h_contour[temp_index].x)	//je�eli s�siedni punkt jest bardziej na lewo, tj. ma mniejsz� warto�� x
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
			drawContours(img_org, ctrs, i, Scalar(255, 0, 0), 6);
		else
		{
			is_inside = pointPolygonTest(ctrs[hand_index], ctrs[i].front(), false);

			if (is_inside >= 0)		//je�eli jest w �rodku lub na kraw�dzi
			{
				drawContours(img_org, ctrs, i, Scalar(0, 255, 0), 6);
				drawContours(img_work, ctrs, i, 255, FILLED);			//zamaluj na bia�o
			}
			else	//je�eli jest poza konturem d�oni
			{
				drawContours(img_org, ctrs, i, Scalar(0, 0, 255), 6);
				drawContours(img_work, ctrs, i, 0, FILLED);				//zamaluj na czarno
			}
		}
	}

	display_img(img_org, name + " - all contours");
	imwrite(name + " - all contours.jpg", img_org);
}

wrist_side basic_detection(Mat & img, string img_name, Rect& bound_rect, vector<Point>& hand_contour, Point & wrist_p_R, Point & wrist_p_L, int & wrist_width)
{
	display_img(img, img_name);
	Mat work_img = img.clone();

	//1. wyb�r przestrzeni barw - przestrze� HSV:
	vector<Mat> HSV_channels;
	cvtColor(work_img, work_img, COLOR_BGR2HSV);	//zmiana przestrzeni barw z BGR na HSV

	split(work_img, HSV_channels);
	work_img = HSV_channels[1];						//wyb�r kana�u S, jako obrazu do dalszego przetwarzania
	display_img(work_img, (img_name + " - S channel"));
	imwrite((img_name + " - S channel.jpg"), work_img);

	//2. rozmycie:
	blur(work_img, work_img, Size((work_img.cols / 200), (work_img.cols / 200)), Point(-1, -1), BORDER_REPLICATE);
	//display_img(work_img, (img_name + " - Blured"));
	imwrite((img_name + " - Blured.jpg"), work_img);

	//3. progowanie:
	my_thresholding(work_img, img, 0.40);
	display_img(work_img, (img_name + " - Thresholded"));
	imwrite((img_name + " - Thresholded.jpg"), work_img);

	//4. poprawa obrazu - operacje morfologiczne:
	vector<vector<Point>> contours;
	int kernel_size = 0;
	int hand_index = 0;

	kernel_size = hand_contours(work_img, contours);		//hand_contour() - realizuje morfologiczne otwarcie, a� kontur�w jest mniej ni� 100
	hand_index = biggest_countour(contours);

	hand_contour = contours[hand_index];				//ARGUMENT 4 - przypisanie warto�ci(kontur samej d�oni)

	vector<vector<Point>> HAND_contour;					//kontur samej d�oni do dalszej pracy
	HAND_contour.push_back(contours[hand_index]);

	cout << endl << "Wykrytych kontur�w: " << contours.size() << endl << endl;

	leave_just_hand(img, work_img, contours, hand_index, img_name);
	display_img(work_img, (img_name + " - just hand filled"));
	imwrite((img_name + " - just hand filled.jpg"), work_img);

	//zast�powanie wype�nionego obrazu samym konturem:
	work_img = Mat::zeros(work_img.rows, work_img.cols, work_img.type());	//zaczernianie obrazu
	drawContours(work_img, HAND_contour, -1, 255, 1);
	display_img(work_img, (img_name + " - just hand contour"));
	imwrite((img_name + " - just hand contour.jpg"), work_img);

	//5. znalezienie dolnych punkt�w konturu d�oni:
	//dolny prawy punkt:
	LineIterator bottom_iter(work_img, Point(work_img.cols - 1, work_img.rows - 1), Point(0, work_img.rows - 1));

	for (int i = 0; i < bottom_iter.count; i++, bottom_iter++)
	{
		if (*(*bottom_iter) == 255)
			break;
	}
	Point right_bottom_point = Point(bottom_iter.pos().x, bottom_iter.pos().y);

	//dolny lewy punkt:
	bottom_iter = LineIterator(work_img, Point(0, work_img.rows - 1), Point(work_img.cols - 1, work_img.rows - 1));

	for (int i = 0; i < bottom_iter.count; i++, bottom_iter++)
	{
		if (*(*bottom_iter) == 255)
			break;
	}
	Point left_bottom_point = Point(bottom_iter.pos().x, bottom_iter.pos().y);

	//6. okre�lenie indeksu odnalezionego punktu w konturze d�oni:
	int right_point_index = -2;		//indeks dolnego prawego punktu d�oni PRZED wycieciem przedramienia
	int left_point_index = -2;

	right_point_index = point_in_contour(HAND_contour[0], right_bottom_point);
	left_point_index = point_in_contour(HAND_contour[0], left_bottom_point);
	circle(img, HAND_contour[0][right_point_index], 50, Scalar(255, 0, 0), FILLED);
	circle(img, HAND_contour[0][left_point_index], 50, Scalar(0, 255, 0), FILLED);
	//display_img(img, (img_name + " - wrists starts"));
	imwrite((img_name + " - wrists starts.jpg"), img);

	//7. znalezienie punktu nadgarstka:
	wrist_side side;
	Point wrist_point_right;
	Point wrist_point_left;
	Point wrist_point;			//dolny prawy punkt juz SAMEJ d�oni (ten punkt od strony kciuka)

	//sprawdzenie czy przedrami� jest pionowo, tj. czy nie zaczynamy od skr�tu w prawo po prawj stronie nadgarstka lub skr�tu w lewo po lewje stronie
	bool right_side_good = forearm_horizontal(HAND_contour[0], right_point_index, WRIST_RIGHT, 100);

	if (right_side_good)	//je�eli prawda, tzn. po prawej stronie nadgarstka mo�na szuka� punktu nadgarstka
		wrist_point_right = find_wrist(HAND_contour[0], right_point_index, WRIST_RIGHT);
	else
		wrist_point_right = Point(100, 100);	//ustawiam punkt bardzo wysoko, �eby tu nie ustali�o nadgarstka

	bool left_side_good = forearm_horizontal(HAND_contour[0], left_point_index, WRIST_LEFT, 100);

	if (left_side_good)	//je�eli prawda, tzn. po lewej stronie nadgarstka mo�na szuka� punktu nadgarstka
		wrist_point_left = find_wrist(HAND_contour[0], left_point_index, WRIST_LEFT);
	else
		wrist_point_left = Point(50, 50);	//ustawiam punkt bardzo wysoko, �eby tu nie ustali�o nadgarstka

	circle(img, wrist_point_right, 30, Scalar(0, 0, 127), FILLED);
	circle(img, wrist_point_left, 30, Scalar(127, 0, 0), FILLED);

	//wybieram punkt ni�szy:
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

	display_img(img, (img_name + " - wrists points"));
	imwrite((img_name + " - wrists points.jpg"), img);

	//8. rysowanie linii nadgarstka:
	LineIterator wrist_iter(work_img, wrist_point, Point(0, 0));		//pocz�tkowo musz� czym� zainicjalizaowa� obiekt, gdy� kalsa nie ma konstruktora 
																	//domy�lnego
	(side == WRIST_RIGHT) ?
		(wrist_iter = LineIterator(work_img, wrist_point, Point(0, wrist_point.y))) :
		(wrist_iter = LineIterator(work_img, wrist_point, Point(work_img.cols - 1, wrist_point.y)));

	wrist_iter++;
	wrist_width = 1;							//ARGUMENT 7 - przekazanaszeroko�� nadgarstka  

	while (wrist_width < wrist_iter.count)
	{
		if (*(*wrist_iter) != 255)
			work_img.at<uchar>(wrist_iter.pos().y, wrist_iter.pos().x) = 255;	//maluj na bia�o
		else break;

		wrist_iter++;
		wrist_width++;
	}

	cout << img_name << " - szeroko�� nadgarstka: " << wrist_width << endl;

	Point wrist_line_point;			//lewy punkty na lini nadgarstka - do detekcji wierzcho�k�w i dolin 
	Point wrist_line_point_thumb;	//prawy punkty na lini nadgarstka - do detekcji wierzcho�ka kciuka

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

	wrist_p_L = wrist_line_point;				//ARGUMENT 6
	wrist_p_R = wrist_line_point_thumb;			//ARGUMENT 5

	//9. usuwanie konturu poni�ej lini nadarstka:
	for (int i = wrist_line_point.y + 1; i < work_img.rows; i++)	//wsp�rz�dna y
		for (int j = 0; j < work_img.cols; j++)						//wsp�rz�dna x
			work_img.at<uchar>(i, j) = 0;

	//display_img(work_img, (img_name + " - just hand"));
	imwrite((img_name + " - just hand.jpg"), work_img);

	//10. opisanie d�oni prostok�tem:
	HAND_contour.clear();
	findContours(work_img.clone(), HAND_contour, RETR_EXTERNAL, CHAIN_APPROX_NONE);		//findContours() modyfikuje obraz �r�d�owy
																						//tu dzia�amy ju� na obrazie z samym konturem, dlatego
																						//dla jednego konturu znajduje si� 2 nowe, tj. jego
																						//zewn�trzn� i wewn�trzn� granic�; aby znale�� tylko
																						//zewn�trzny okre�lamy 3. argument:	RETR_EXTERNAL

	//int wrist_point_index = point_in_contour(HAND_contour[0], wrist_line_point);	//dopiero tu, na NOWYM konturze samej d�oni, mog� szuka� indeksu
	//																				//punktu nadgarstka

	//int wrist_point_index_thumb = point_in_contour(HAND_contour[0], wrist_line_point_thumb);

	bound_rect = boundingRect(HAND_contour[0]);			//ARGUMENT 3
	rectangle(img, bound_rect, Scalar(0, 0, 0), 4);		//prostok�t obrysowuj�cy r�k�

	cout << img_name << " - wysoko�� r�ki: " << bound_rect.height << endl;
	cout << img_name << " - szeroko�� r�ki: " << bound_rect.width << endl;

	display_img(img, (img_name + " - Hand in rectangle"));
	imwrite((img_name + " - Hand in rectangle.jpg"), img);

	return side;
}

void calibration_front_function(Mat& img, string img_name, Rect& bounding_rect, vector<Point> contour, Point& wrist_right, Point& wrist_left, Finger* calibration_fingers)
{
	//1. znajdowanie wierzcho�k�w i "dolin" palc�w:
	int wrist_start_index = -2;		//inicjalizauj�ca warto�� ujemna sygnalizuj�ca b��d (zostanie rzucony wyj�tek przy obs�udze tablicy)

	wrist_start_index = point_in_contour(contour, wrist_left);
	circle(img, contour[wrist_start_index], 50, Scalar(0, 0, 0), FILLED);

	//znajdowanie wierzcho�k�w i dolin 4 palc�w:
	vector<Point> finger_apex;
	vector<Point> finger_valley;

	find_apex_valley(contour, wrist_start_index, finger_apex, finger_valley, (bounding_rect.height*0.20), (bounding_rect.height / 2), CALIBRATION); //

	//znajdowanie wierzcho�ka kciuka:
	int wrist_point_index_thumb = point_in_contour(contour, wrist_right);
	Point thumb_apex;

	find_thumb_apex(contour, wrist_point_index_thumb, thumb_apex, (bounding_rect.height*0.3));
	finger_apex.push_back(thumb_apex);

	for (int i = 0; i < finger_apex.size(); i++)
		circle(img, finger_apex[i], 30, Scalar(255, 0, 255), FILLED);

	for (int i = 0; i < finger_valley.size(); i++)
		circle(img, finger_valley[i], 30, Scalar(0, 255, 255), FILLED);

	display_img(img, (img_name + " - Appexes and valleys"));
	imwrite((img_name + " - Appexes and valleys.jpg"), img);

	//2. wyznaczenie obszar�w poszukiwa� kolejnych palc�w:
	//ma�y palec:
	calibration_fingers[0] = Finger(img, Point(bounding_rect.x, 0), finger_valley[0], finger_apex[0]);		//punkt, wsp. y = 0 , �eby uzwgl�dnia� obszar do samej g�ry obrazu
	calibration_fingers[0].draw_regions();
	//palec serdeczny:
	calibration_fingers[1] = Finger(img, calibration_fingers[0].give_point(), finger_valley[1], finger_apex[1]);
	calibration_fingers[1].draw_regions();
	//palec �rodkowy:
	calibration_fingers[2] = Finger(img, calibration_fingers[1].give_point(), finger_valley[2], finger_apex[2]);
	calibration_fingers[2].draw_regions();
	//dla 4-go palca, tj. wskazuj�cego, mamy inny dolny prawy r�g regionu:
	Point fin_4_bottom_point(bounding_rect.x + bounding_rect.width, finger_valley[2].y);
	calibration_fingers[3] = Finger(img, calibration_fingers[2].give_point(), fin_4_bottom_point, finger_apex[3]);
	calibration_fingers[3].draw_regions();
	//kciuk:
	Point thumb_top_left(finger_valley[3].x, finger_valley[2].y);
	Point thumb_bottom_right(bounding_rect.x + 1.005*bounding_rect.width, bounding_rect.y + bounding_rect.height);	//1.005*calib_FR_rect.width - nieco
																													//zwi�ksza obszar dla pweno�ci detekji
	calibration_fingers[4] = Finger(img, thumb_top_left, thumb_bottom_right, Finger::THUMB_MODE);
	calibration_fingers[4].draw_regions();
	display_img(img, (img_name + " - finger's regions"));
	imwrite((img_name + " - finger's regions.jpg"), img);

	cout << "\n********************************************\n\n\a";
}

void fingers_detection_function(string img_name, int calib_wrist_width, Finger* cal_fin, int* fin_pos)
{
	//przygotowanie obrazu do detekcji:
	Mat proper_IMG;

	proper_IMG = imread(img_name);

	if (proper_IMG.data == NULL)		//je�eli nie uda�o sie wczyta� pliku ze zdj�ciem to obiekt Mat jest pusty, tj. ma warto�� NULL
	{
		cout << "Nie uda�o si� wczyta� pliku do po�o�enia palc�w (obiekt \"proper_IMG\") !!!\a" << endl;
		Sleep(4000);
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

	//okre�lenie czynnika skalujacego:
	double scale_factor = double(proper_IMG_wrist_width) / double(calib_wrist_width);	//trzeba zrobi� zdjecia z pionowo ustawionym przedramieniem
	cout.precision(4);
	cout << "Czynnik skaluj�cy: " << scale_factor << endl;

	//przeskalowanie prostok�ta opisujacego:
	proper_IMG_rect.width = scale_factor * proper_IMG_rect.width;
	proper_IMG_rect.height = scale_factor * proper_IMG_rect.height;
	(proper_IMG_side == WRIST_RIGHT) ?
		(proper_IMG_rect.y = proper_IMG_wrist_right.y - proper_IMG_rect.height) : (proper_IMG_rect.y = proper_IMG_wrist_left.y - proper_IMG_rect.height);

	rectangle(proper_IMG, proper_IMG_rect, Scalar(0, 0, 255), 4);
	display_img(proper_IMG, (proper_IMG_name + " - Hand in scaled rectangle"));
	imwrite((proper_IMG_name + " - Hand in scaled rectangle.jpg"), proper_IMG);

	//1. wyznaczanie przewidywanych region�w kolejnych palc�w:
	Point fingers_START(proper_IMG_rect.x, 0);		//punkt startowy do wyznaczenia rejon�w pierwszeg palca, a potem kolejnych; y=0 �eby bra� obszar
													//a� do samej g�ry obrazu

	Finger prop_fin_1(proper_IMG, fingers_START, cal_fin[0], scale_factor, Finger::FINGER_MODE);
	prop_fin_1.draw_regions();
	Finger prop_fin_2(proper_IMG, prop_fin_1.give_point(), cal_fin[1], scale_factor, Finger::FINGER_MODE);
	prop_fin_2.draw_regions();
	Finger prop_fin_3(proper_IMG, prop_fin_2.give_point(), cal_fin[2], scale_factor, Finger::FINGER_MODE);
	prop_fin_3.draw_regions();
	//dla 4-go palca, tj. wskazuj�cego, mamy inny dolny prawy r�g regionu - ale tu nie musz� si� tym martwi�, gdy� skaluje regiony 4-go palca
	//z kalibracji:
	Finger prop_fin_4(proper_IMG, prop_fin_3.give_point(), cal_fin[3], scale_factor, Finger::FINGER_MODE);
	prop_fin_4.draw_regions();
	//kciuk - potrzebny dolny prawy punkt przeskalowanego prostok�ta opisuj�cego d�o�:
	Point prop_thumb_point(proper_IMG.cols - 1, proper_IMG_rect.y + proper_IMG_rect.height);
	Finger prop_thumb(proper_IMG, prop_thumb_point, cal_fin[4], scale_factor, Finger::THUMB_MODE);
	prop_thumb.draw_regions();

	display_img(proper_IMG, (proper_IMG_name + " - finger's regions"));
	imwrite((proper_IMG_name + " - finger's regions.jpg"), proper_IMG);

	//2. znajdowanie wierzcho�k�w i "dolin" palc�w:
	int PROPER_wrist_start_index = -2;		//inicjalizauj�ca warto�� ujemna sygnalizuj�ca b��d

	PROPER_wrist_start_index = point_in_contour(proper_IMG_contour, proper_IMG_wrist_left);
	circle(proper_IMG, proper_IMG_contour[PROPER_wrist_start_index], 50, Scalar(0, 0, 0), FILLED);

	//znajdowanie wierzcho�k�w 4 palc�w i 3 dolin (4. dolina niekoniecznie musi by�, np. gdy kciuk jest schowany)
	vector<Point> PROPER_finger_apex;
	vector<Point> PROPER_finger_valley;

	find_apex_valley(proper_IMG_contour, PROPER_wrist_start_index, PROPER_finger_apex,
		PROPER_finger_valley, 0, (proper_IMG_rect.height / 2), DETECTION);

	//znajdowanie wierzcho�ka kciuka:
	int PROPER_wrist_point_index_thumb = point_in_contour(proper_IMG_contour, proper_IMG_wrist_right);
	Point PROPER_thumb_apex;

	find_thumb_apex(proper_IMG_contour, PROPER_wrist_point_index_thumb, PROPER_thumb_apex, (proper_IMG_rect.height*0.35));
	PROPER_finger_apex.push_back(PROPER_thumb_apex);

	for (int i = 0; i < PROPER_finger_apex.size(); i++)
		circle(proper_IMG, PROPER_finger_apex[i], 30, Scalar(255, 0, 255), FILLED);

	for (int i = 0; i < PROPER_finger_valley.size(); i++)
		circle(proper_IMG, PROPER_finger_valley[i], 30, Scalar(0, 255, 255), FILLED);

	display_img(proper_IMG, (proper_IMG_name + " - Appexes and valleys"));
	imwrite((proper_IMG_name + " - Appexes and valleys.jpg"), proper_IMG);

	//3.okre�lanie po�o�enia ka�dego z palc�w:
	//1) ma�y palec:
	fin_pos[0] = prop_fin_1.find_fin_location(PROPER_finger_apex[0]);
	if (fin_pos[0] == 4)		//sprawdzeie rejonu palca serdecznego
		fin_pos[0] = prop_fin_2.find_fin_location(PROPER_finger_apex[0]);
	//2) palec serdeczny:
	fin_pos[1] = prop_fin_2.find_fin_location(PROPER_finger_apex[1]);
	if (fin_pos[1] == 4)		//sprawdzeie rejonu palca ma�ego
		fin_pos[1] = prop_fin_1.find_fin_location(PROPER_finger_apex[1]);
	if (fin_pos[1] == 4)		//sprawdzeie rejonu palca �rodkowego
		fin_pos[1] = prop_fin_3.find_fin_location(PROPER_finger_apex[1]);
	//3) palec �rodkowy:
	fin_pos[2] = prop_fin_3.find_fin_location(PROPER_finger_apex[2]);
	if (fin_pos[2] == 4)		//sprawdzeie rejonu palca serdecznego
		fin_pos[2] = prop_fin_2.find_fin_location(PROPER_finger_apex[2]);
	if (fin_pos[2] == 4)		//sprawdzeie rejonu palca wskazujacego
		fin_pos[2] = prop_fin_4.find_fin_location(PROPER_finger_apex[2]);
	//4) palec wskazuj�cy:
	fin_pos[3] = prop_fin_4.find_fin_location(PROPER_finger_apex[3]);
	if (fin_pos[3] == 4)		//sprawdzeie rejonu palca �rodkowego
		fin_pos[3] = prop_fin_3.find_fin_location(PROPER_finger_apex[3]);
	//5) kciuk:
	fin_pos[4] = prop_thumb.find_fin_location(PROPER_finger_apex[4]);

	cout << endl << "Palce znalezione w rejonach nr:" << endl;
	cout << "1 - palec ma�y: " << fin_pos[0] << endl;
	cout << "2 - palec serdeczny: " << fin_pos[1] << endl;
	cout << "3 - palec �rodkowy: " << fin_pos[2] << endl;
	cout << "4 - palec wskazuj�cy: " << fin_pos[3] << endl;
	cout << "5 - kciuk: " << fin_pos[4] << endl << endl;
	
	display_img(proper_IMG, (proper_IMG_name + " - detected fingers"));
	imwrite((proper_IMG_name + " - detected fingers.jpg"), proper_IMG);
}

uint8_t rotation_detection_function(string img_calib_name, string img_detect_name, const Rect & calib_front_rect)
{
	//dalsza cz�� kalibracji - widok z boku:
	Mat calib_SD;
	
	calib_SD = imread(img_calib_name);

	if (calib_SD.data == NULL)		//je�eli nie uda�o sie wczyta� pliku ze zdj�ciem to obiekt Mat jest pusty, tj. ma warto�� NULL
	{
		cout << "Nie uda�o si� wczyta� pliku do kalibracji od boku (obiekt \'calib_SD\") !!!\a" << endl;
		Sleep(4000);
		exit(EXIT_FAILURE);
	}
	
	string calib_SD_name("Calibration ROTATION side");
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

	//detekcja obrotu - w�a�ciwy obraz:
	Mat rot_img;

	rot_img = imread(img_detect_name);

	if (rot_img.data == NULL)		//je�eli nie uda�o sie wczyta� pliku ze zdj�ciem to obiekt Mat jest pusty, tj. ma warto�� NULL
	{
		cout << "Nie uda�o si� wczyta� pliku do detekcji k�ta obrotu (obiekt \'rot_img\") !!!\a" << endl;
		Sleep(4000);
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

	double cosinus_angle = double(rot_hand_width - MIN_hand_width) / double(hand_width_RANGE);

	if (cosinus_angle <= 0)		//co je�li wskazuje, �e r�ka jest w�sza ni� przy widoku, jak na kalibracji z boku
		cosinus_angle = 0.0;	//usatw 0 czyli max. obr�t
	else if (cosinus_angle > 1)	//co je�li wskazuje, �e r�ka jest szersza ni� przy widoku, jak na kalibracji od frontu
		cosinus_angle = 1.0;	//usatw 1.0 czyli brak obrotu

	cout.precision(4);
	cout << endl << "Cosinus k�ta obrotu = " << cosinus_angle << endl;

	const double RAD_to_DEGREES = 180.0 / 3.1416;
	double hand_rotation_angle = acos(cosinus_angle);
	hand_rotation_angle *= RAD_to_DEGREES;

	const double ANGLE_to_STEPS = 50.0 / 90.0;							//90 stopni to 50 zmian stanu LOW->HIGH dla silnika krokowego
	uint8_t hand_rotation_MCU = hand_rotation_angle * ANGLE_to_STEPS;
	hand_rotation_MCU *= 2;												//50 krok�w to 90 stopni, ale 50 krok�w to 100 zmian stanu mi�dzy LOW a HIGH

	cout << "K�t obrotu (stopnie): " << hand_rotation_angle << endl;
	cout << "K�t obrotu (zmiany LOW->HIGH): " << (int)hand_rotation_MCU << endl << endl;	//uint8_t jest aliasem typu char, dlatego operator << 
	cout << "\n********************************************\n\n\a";							//klasy ostream traktuje go jak ten typ i wy�wietla
																							//znak, dlaetgo potrzebne jest rzutowanie: (int)...

	return hand_rotation_MCU;
}
