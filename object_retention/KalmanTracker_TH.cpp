#include"KalmanTracker_TH.h"
#include"kd_tree.h"
/*下面为大件物体的代码*/
bool MyCompare_x(const cv::Point point1, const cv::Point point2) { return point1.x < point2.x; }
bool MyCompare_y(const cv::Point point1, const cv::Point point2) { return point1.y < point2.y; }
float cal_points_distance(const vector<cv::Point>points1_small, const vector<cv::Point>points2_big)
{
	//计算两个点集之间的最小距离 使用voronoi图和kd树的技巧，使得时间复杂度下降
	float min_distance = std::numeric_limits<float>::max();
	return min_distance;
}
MyObejctBox::MyObejctBox(const std::vector<cv::Point>&points)
{
	this->x1 = (*std::min_element(points.begin(), points.end(), MyCompare_x)).x;
	this->x3 = (*std::max_element(points.begin(), points.end(), MyCompare_x)).x;
	this->y1 = (*std::min_element(points.begin(), points.end(), MyCompare_y)).y;
	this->y3 = (*std::max_element(points.begin(), points.end(), MyCompare_y)).y;
	this->rect = Rect(this->x1, this->y1, (this->x3) - (this->x1) + 1, (this->y3) - (this->y1) + 1);
}
void MyObejctBox::Update_Box(const std::vector<cv::Point>&points)
{
	/*用小的连通域来更新大的连通域*/
	int new_x1 = (*std::min_element(points.begin(), points.end(), MyCompare_x)).x;
	int new_x3 = (*std::max_element(points.begin(), points.end(), MyCompare_x)).x;
	int new_y1 = (*std::min_element(points.begin(), points.end(), MyCompare_y)).y;
	int new_y3 = (*std::max_element(points.begin(), points.end(), MyCompare_y)).y;
	if (new_x1 < this->x1) {
		this->x1 = new_x1;
		this->rect = Rect(this->x1, this->y1, (this->x3) - (this->x1) + 1, (this->y3) - (this->y1) + 1);
	}
	if (new_x3 > this->x3) {
		this->x3 = new_x3;
		this->rect = Rect(this->x1, this->y1, (this->x3) - (this->x1) + 1, (this->y3) - (this->y1) + 1);
	}
	if (new_y1 < this->y1) {
		this->y1 = new_y1;
		this->rect = Rect(this->x1, this->y1, (this->x3) - (this->x1) + 1, (this->y3) - (this->y1) + 1);
	}
	if (new_y3 > this->y3) {
		this->y3 = new_y3;
		this->rect = Rect(this->x1, this->y1, (this->x3) - (this->x1) + 1, (this->y3) - (this->y1) + 1);
	}
}
ThingKalmanTracker::ThingKalmanTracker(const float& big_area_threshold, const float& small_area_threshold, const float& distance_threshold)
{
	this->big_area_threshold = big_area_threshold;
	this->small_area_threshold = small_area_threshold;
	this->distance_threshold = distance_threshold;
}
void ThingKalmanTracker::ThingsDetector(const Mat& ForemaskImage)
{
	//首先初始化MyBigObejcts
	this->big_obejcts.clear();
	clock_t start_time;
	start_time = clock();
	//阈值化，将非纯白色（244~255）的所有像素设为0 将影子去掉
	Mat final_foremask;
	cv::threshold(ForemaskImage, final_foremask, 244, 255, cv::THRESH_BINARY);

	// 通过形态学操作对二值化的图像进行预处理
	Mat kernel = getStructuringElement(MORPH_ERODE, Size(3, 3));
	cv::morphologyEx(final_foremask, final_foremask, cv::MORPH_CLOSE, kernel, Point(-1, -1), 5);

	//寻找轮廓
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(final_foremask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	//计算轮廓面积 面积大的当作被检测出来的物体 
	vector<int>small_contour_ids;	//小轮廓的下标
	size_t big_thing_contourpoint_num = 0;
	for (int i = 0; i < contours.size(); i++)
	{
		const vector<Point>& contour = contours[i];
		double area = contourArea(Mat(contour));
		if (area > this->big_area_threshold)//大物体
		{
			big_obejcts.push_back(MyBigObejct(contour));
			big_thing_contourpoint_num += contour.size();
		}
		else if (area>this->small_area_threshold)//小物体
		{
			small_contour_ids.push_back(i);
		}
	}
	/*先对面积大的物体进行合并*/
	//先根据轮廓点进行排序，轮廓点多的在前面
	sort(big_obejcts.begin(), big_obejcts.end(),
		[](const MyBigObejct &A, const MyBigObejct &B) {return A.contour_points.size() > B.contour_points.size(); });
	if (big_obejcts.size() >= 2)
	{
		for (int i = big_obejcts.size() - 1; i >= 0; --i)
		{
			for (int j = i-1; j >= 0; --j)
			{
				int x_1 = max(big_obejcts[i].box.x1, big_obejcts[j].box.x1);//比x_2大则在x方向无交集
				int x_2 = min(big_obejcts[i].box.x3, big_obejcts[j].box.x3);
				int y_1 = max(big_obejcts[i].box.y1, big_obejcts[j].box.y1);//比y_2大则在y方向无交集
				int y_2 = min(big_obejcts[i].box.y3, big_obejcts[j].box.y3);
				if ((x_1 - x_2) < int(this->big_area_distance) && (y_1 - y_2) < int(this->big_area_distance))
				{
					const vector<Point> &contour = big_obejcts[j].contour_points;
					vector<Contour_Point>contour_points(contour.size());
					int point_count = 0;
					for (const auto& point : contour)
					{
						contour_points[point_count++] = Contour_Point(double(point.x), double(point.y), 0);//这里id无关紧要，id都给0
					}
					kdt::KDTree<Contour_Point> kdtree(contour_points);//建立kd树
					Contour_Point query;
					double min_distance = std::numeric_limits<double>::max();
					for (const auto&point : big_obejcts[i].contour_points)//查询
					{
						query.set_coord(point.x, point.y);
						double distance;
						const int Point_ID = kdtree.nnSearch(query, &distance);
						if (distance < min_distance)
						{
							min_distance = distance;
						}
					}
					n
					if (min_distance < this->big_area_distance) {
						//cout << "min_distance:" << min_distance << endl;
						big_obejcts[j].contour_points.reserve(big_obejcts[j].contour_points.size()+ big_obejcts[i].contour_points.size());//将最后一个点集并到前面
						big_obejcts[j].contour_points.insert(big_obejcts[j].contour_points.end(), big_obejcts[i].contour_points.begin(), big_obejcts[i].contour_points.end());
						big_obejcts[j].box.rect |= big_obejcts[i].box.rect;//更新矩形框
						big_obejcts[j].box.Update_coordinates();
						big_obejcts.erase(big_obejcts.begin() + i);//删除最后一个点集
						sort(big_obejcts.begin(), big_obejcts.end(),
							[](const MyBigObejct &A, const MyBigObejct &B) {return A.contour_points.size() > B.contour_points.size(); });//重新根据点集数目排序
					}
				}		
			}
		}
	}


	//面积小的物体归到最近的面积大的物体中
	if (small_contour_ids.size() > 0 && big_obejcts.size() > 0)
	{
		/*根据大物体的轮廓建立kd树*/
		//首先将大物体轮廓点放到数组中
		vector<Contour_Point>BigTH_contour_points(big_thing_contourpoint_num);
		{
			int i = 0;
			for (int j=0;j<big_obejcts.size();++j)
			{
				const vector<Point> &contour = big_obejcts[j].contour_points;
				for (const auto& point : contour)
				{
					BigTH_contour_points[i++] = Contour_Point(double(point.x), double(point.y), j);//iter.second :boxe_id
				}
			}
		}

		kdt::KDTree<Contour_Point> kdtree(BigTH_contour_points);//建立kd树

																//对小物体的轮廓进行KD树查询，把小物体轮廓合并到距离最小的大物体
		Contour_Point query;
		for (const auto&i : small_contour_ids)
		{
			const vector<Point>& small_contour = contours[i];
			double min_distance = std::numeric_limits<double>::max();
			int final_BoxID;
			for (const auto&point : small_contour)
			{
				query.set_coord(point.x, point.y);
				double distance;
				const int Point_ID = kdtree.nnSearch(query, &distance);
				if (distance < min_distance)
				{
					min_distance = distance;
					final_BoxID = BigTH_contour_points[Point_ID].box_id;
				}
			}
			if (min_distance < this->distance_threshold) {
				big_obejcts[final_BoxID].box.Update_Box(small_contour);
			}

		}
		BigTH_contour_points.clear();
	}
	//cout << "before filter:" << this->big_obejcts.size() << endl;
	this->ThingBox_Filter();//根据一些自定义的规则，如长宽比，交并比等，对检测出来的前景框进行一些合并及删除
	//cout << "after filter:" << this->big_obejcts.size() << endl;
	/*for (const auto &box : this->thing_boxes)
		cout << "box area:" << box.rect.area() << endl;*/
	//将轮廓以及大物体画出来
	RNG g_rng(12345);
	Mat dst = Mat::zeros(ForemaskImage.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		//cout << "轮廓点数: " << contours[i].size() << endl;
		Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));//任意值
		const vector<Point>& c = contours[i];
		double area = contourArea(Mat(c));
		if (area>this->small_area_threshold)
			drawContours(dst, contours, i, color, cv::FILLED, 8, hierarchy);
	}
	for (int i = 0; i < big_obejcts.size(); ++i)
	{
		int x1 = big_obejcts[i].box.x1;
		int x3 = big_obejcts[i].box.x3;
		int y1 = big_obejcts[i].box.y1;
		int y3 = big_obejcts[i].box.y3;
		cv::rectangle(dst, Point(x1, y1),
			Point(x3, y3),
			cv::Scalar(255, 255, 255));
	}
	cout << "spend time:" << double(clock() - start_time) / CLOCKS_PER_SEC << endl;
	//namedWindow("couters", 0);
	//imshow("couters", dst);
	//cv::waitKey(1);
}
void ThingKalmanTracker::ThingBox_Filter()
{
	//根据一些自定义的规则，如长宽比，交并比等，对检测出来的前景框进行一些合并及删除
	//首先将矩形按照面积从大到小进行排序
	sort(this->big_obejcts.begin(), this->big_obejcts.end(),
		[](const MyBigObejct &A, const MyBigObejct &B) {return A.box.rect.area() > B.box.rect.area(); });
	if (this->big_obejcts.size() >= 2)//如果检测结果超过2个矩形框，首先根据交叠矩形：较小面积矩形 的值来合并面积较小的矩形
	{
		for (int i = this->big_obejcts.size() - 1; i >= 0; --i)
		{
			double max_inter_small_rate = 0;
			int big_id = i - 1;
			for (int j = i - 1; j >= 0; --j){
				double inter_small_rate = double((this->big_obejcts[i].box.rect& this->big_obejcts[j].box.rect).area())
					/ double(big_obejcts[i].box.rect.area());
				if (inter_small_rate > max_inter_small_rate){
					max_inter_small_rate = inter_small_rate;
					big_id = j;
				}		
			}
			if (max_inter_small_rate > intersection_small_rate){//将面积小的矩形进行合并
				this->big_obejcts[big_id].box.rect |= this->big_obejcts[i].box.rect;
				this->big_obejcts[big_id].box.Update_coordinates();
				this->big_obejcts.erase(big_obejcts.begin() + i);//删除这个面积小的检测框
			}		
		}
	}
	//删除长宽比奇怪的检测框
	for (int i = this->big_obejcts.size() - 1; i >= 0; --i){
		const double aspect_ratio = double(this->big_obejcts[i].box.rect.width) / double(this->big_obejcts[i].box.rect.height);
		//cout << "aspect_ratio:" << aspect_ratio << endl;
		if (aspect_ratio > aspect_ratio_threshold || aspect_ratio < (1.0 / aspect_ratio_threshold)){
			this->big_obejcts.erase(big_obejcts.begin() + i);
		}
	}
}