#pragma once
#include"KalmanTracker.h"
/*����Ϊ�����Ʒ��⼰���ٵĸ�����*/
class Contour_Point : public std::array<double, 2>//������
{
public:
	// dimension of space (or "k" of k-d tree)
	// KDTree class accesses this member
	static const int DIM = 2;
	// the constructors
	Contour_Point() {}
	Contour_Point(double x, double y)
	{
		(*this)[0] = x;
		(*this)[1] = y;
	}
	Contour_Point(double x, double y, int box_id)
	{
		(*this)[0] = x;
		(*this)[1] = y;
		this->box_id = box_id;
	}
	void set_coord(double x, double y)
	{
		(*this)[0] = x;
		(*this)[1] = y;
	}
	int box_id;//��Ӧ�����Ʒthing_boxes��id��
			   // ת�� OpenCV Point2d
	operator cv::Point2d() const { return cv::Point2d((*this)[0], (*this)[1]); }
};
class MyObejctBox
{
	/*@�������߿���*/
public:
	MyObejctBox(const std::vector<cv::Point>&points);
	MyObejctBox() {};
	~MyObejctBox() {};
	int x1;//��Ӿ������Ͻ�
	int y1;
	int x3;//��Ӿ������½�
	int y3;
	cv::Rect rect;
	void Update_Box(const std::vector<cv::Point>&points);
	void Update_coordinates(){
		this->x1 = this->rect.x;
		this->y1 = this->rect.y;
		this->x3 = this->rect.x+ this->rect.width-1;
		this->y3 = this->rect.y + this->rect.height - 1;
	}
};
class MyBigObejct
{
	/*@�������߿���*/
public:
	vector<Point>contour_points;
	MyObejctBox box;
	MyBigObejct(const std::vector<cv::Point>&points)
	{ 
		this->box = MyObejctBox(points);
		this->contour_points = points;
	};
	~MyBigObejct() {};
	
};
class ThingKalmanTracker :public KalmanTracker
{
private:
	float big_area_threshold;//�����������ֵ�����������ֵ�ı��ж�Ϊ�������
	const float big_area_distance=30.0;//�����������ֵ����С�������ֵ�Ĵ�������кϲ�
	float small_area_threshold;//С���������ֵ��С�������ֵ�ĵ��������˳�
	float distance_threshold;//������ֵ�����С�����������������С����С�����ֵ����С����鵽��������
	const double intersection_small_rate = 0.4;//���Դ�����ϲ�����ֵ
	const double aspect_ratio_threshold = 4.0;//�������ϳ����Ҫ��ļ����޳�
public:
	std::vector<MyBigObejct>big_obejcts;
	ThingKalmanTracker(const float &area_threshold = 3000.0, const float& small_area_threshold = 1000.0, const float& distance_threshold = 70.0);
	~ThingKalmanTracker() {}
	void ThingsDetector(const Mat &ForemaskImage);//����ǰ��ͼ�������������
	void ThingBox_Filter();//����һЩ�Զ���Ĺ����糤��ȣ������ȵȣ��Լ�������ǰ�������һЩ�ϲ���ɾ��
	void Filter_People(const vector<Rect>&people_boxes) {}//����iou����ͷ�˳���
};

