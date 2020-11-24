#include "finger.h"


Finger::Finger(Mat & init_img, Point top_left, Point bottom_right, Point finger_top) : img(init_img)
{
	all_region = Rect(top_left, bottom_right);
	apex = finger_top;
	finger_height = bottom_right.y - apex.y;
	region_size = finger_height / 4;
	top_apex_dist = apex.y - top_left.y;
	
	//towrzenie regionów:
	regions[0][0] = top_left;
	regions[0][1] = regions[0][0] + Point(all_region.width, 0);
	regions[0][2] = Point(all_region.x + all_region.width, apex.y + region_size);
	regions[0][3] = Point(all_region.x, apex.y + region_size);

	for (int i = 1; i < 4; i++)
	{
		regions[i][0] = regions[i - 1][3];
		regions[i][1] = regions[i - 1][2];
		regions[i][2] = regions[i][1] + Point(0, region_size);
		regions[i][3] = regions[i][0] + Point(0, region_size);
	}
}

Finger::Finger(Mat & init_img, Point top_left, Point bottom_right, f_mode mode) : img(init_img)
{
	all_region = Rect(top_left, bottom_right);
	region_size = (bottom_right.x - top_left.x)/4;
	finger_height = 0;

	regions[0][0] = Point(bottom_right.x, top_left.y);
	regions[0][1] = regions[0][0] + Point(0, all_region.height);
	regions[0][2] = regions[0][1] - Point(region_size, 0);
	regions[0][3] = regions[0][0] - Point(region_size, 0);

	for (int i = 1; i < 4; i++)
	{
		regions[i][0] = regions[i - 1][3];
		regions[i][1] = regions[i - 1][2];
		regions[i][2] = regions[i][1] - Point(region_size, 0);
		regions[i][3] = regions[i][0] - Point(region_size, 0);
	}
}

Finger::Finger(Mat& init_img, Point start_point, Finger fin, double scale, f_mode mode) : img(init_img)
{
	Point bottom_right;
	Point top_left;

	switch (mode)
	{
	case FINGER_MODE:
		bottom_right = Point(start_point.x + scale * fin.all_region.width, start_point.y + scale * fin.all_region.height);
		all_region = Rect(start_point, bottom_right);
		apex = Point(0, 0);		//ten punkt jest nam niepotrzebny przy detekcji palców
		finger_height = fin.finger_height*scale;
		region_size = finger_height / 4;
		top_apex_dist = fin.top_apex_dist*scale;

		//towrzenie regionów:
		regions[0][0] = start_point;
		regions[0][1] = regions[0][0] + Point(all_region.width, 0);
		regions[0][2] = Point(start_point.x + all_region.width, start_point.y + top_apex_dist + region_size);
		regions[0][3] = Point(start_point.x, start_point.y + top_apex_dist + region_size);

		for (int i = 1; i < 4; i++)
		{
			regions[i][0] = regions[i - 1][3];
			regions[i][1] = regions[i - 1][2];
			regions[i][2] = regions[i][1] + Point(0, region_size);
			regions[i][3] = regions[i][0] + Point(0, region_size);
		}
		break;
	case THUMB_MODE:
		top_left = Point(start_point.x - scale * fin.all_region.width, start_point.y - scale * fin.all_region.height);
		all_region = Rect(top_left, start_point);
		apex = Point(0, 0);		//ten punkt jest nam niepotrzebny przy detekcji palców
		finger_height = 0;		//ten wymiar jest niepotrzebny przy kciuku
		top_apex_dist = 0;		//ten wymiar jest niepotrzebny przy kciuku
		region_size = fin.region_size*scale;

		//towrzenie regionów:
		regions[0][0] = Point(start_point.x, top_left.y);
		regions[0][1] = start_point;
		regions[0][2] = regions[0][1] - Point(region_size, 0);
		regions[0][3] = regions[0][0] - Point(region_size, 0);

		for (int i = 1; i < 4; i++)
		{
			regions[i][0] = regions[i - 1][3];
			regions[i][1] = regions[i - 1][2];
			regions[i][2] = regions[i][1] - Point(region_size, 0);
			regions[i][3] = regions[i][0] - Point(region_size, 0);
		}
		break;
	}
}

void Finger::draw_regions()
{
	rectangle(img, regions[0][0], regions[0][2], Scalar(0, 0, 0), 10);
	rectangle(img, regions[1][0], regions[1][2], Scalar(255, 0, 0), 10);
	rectangle(img, regions[2][0], regions[2][2], Scalar(0, 255, 255), 10);
	rectangle(img, regions[3][0], regions[3][2], Scalar(0, 0, 255), 10);
}

int Finger::find_fin_location(Point f_top)
{
	int location_region = 0;	//domyœlnie palec jest z³o¿ony, tzn. znajduje siê poza wyznaczonymi regionami, czyli ma indeks równy regions.size()
	bool found = false;

	while (location_region < 4)
	{
		if (pointPolygonTest(regions[location_region], f_top, false) >= 0)
		{
			found = true;
			break;
		}
		location_region++;
	}

	if (!found)
		location_region = 4;

	//rysowanie obszaru, w którym wykryto punkt, je¿el palec nie jest schowany, czyli dla regionu o idndeksie 4
	if(location_region != 4)
		rectangle(img, regions[location_region][0], regions[location_region][2], Scalar(0, 255, 0), 20);	
	
	return location_region;
}

Finger & Finger::operator=(Finger copied_finger)
{
	this->all_region = copied_finger.all_region;
	this->regions = copied_finger.regions;
	this->apex = copied_finger.apex;
	this->finger_height = copied_finger.finger_height;
	this->region_size = copied_finger.region_size;
	this->top_apex_dist = copied_finger.top_apex_dist;

	return *this;
}

Finger::~Finger()
{
}
