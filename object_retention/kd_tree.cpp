#include <opencv2/opencv.hpp>
#include <iostream>
#include <array>
#include <vector>
#include"kd_tree.h"
class Contour_Point : public std::array<double, 2>//结点
{
public:
	// dimension of space (or "k" of k-d tree)
	// KDTree class accesses this member
	static const int DIM = 2;
	Contour_Point(const cv::Point& point, const int & BigThing_Box_ID);
	// the constructors
	MyPoint() {}
	MyPoint(double x, double y)
	{
		(*this)[0] = x;
		(*this)[1] = y;
	}
	Contour_Point() {}
	double X;
	double Y;
	bool right_flag;//right derecha
	bool left_flag;//left
	Contour_Point *right_point;
	Contour_Point *left_point;
	int box_id;//对应大件物品thing_boxes的id号
	bool operator==(const Contour_Point &point)const {
		return (this->X == point.X&&this->Y == point.Y);
	}
	bool operator!=(const Contour_Point &point)const {
		return (this->X != point.X || this->Y != point.Y);
	}
};