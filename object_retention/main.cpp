#include<iostream>
#include<opencv2/opencv.hpp>
#include <string>
#include <vector>
#include<ctime>
#include<cmath>
#include<ctime>
#include <mutex>
#include"Fore_extraction.h"
#include"ThingDetect.h"
#include"ThingTrack.h"
using namespace std;
using namespace cv;
using namespace hlg;
//内存泄漏检测
#include <vld.h> 
//内存泄漏检测end

int main(int argc, const char** argv)
{
	string video_path = "E:\\数据集\\扶梯\\地铁\\loucengban\\clip_video\\ch03_20170315060019_31.avi";
	VideoCapture cap;
	if(!cap.open(video_path))
        printf("video/camera open failed!!!!!\n");
    ForeExtraction fore_extracter(cap.get(CAP_PROP_FRAME_HEIGHT), cap.get(CAP_PROP_FRAME_WIDTH),false);
	Mat inputFrame, foregroundMask;
    ThingDetector thingdetector;
    ThingTracker thingtracker;
    Rect setROI = Rect(0, 0, int(cap.get(CAP_PROP_FRAME_WIDTH)), int(cap.get(CAP_PROP_FRAME_WIDTH)));
    //clock_t total_time=0;
	for (int i = 0;; i++)
	{
		cap >> inputFrame;//原始大小
		if (inputFrame.empty())
		{
			cout << "Finished reading: empty frame" << endl;
			break;
		}
        clock_t start_time = clock();
        if (fore_extracter.extract(inputFrame, foregroundMask))//提取前景
        {                        
            thingdetector.ThingsDetect(foregroundMask);//从前景中提取出大件物体           
            static once_flag SetOutputCoordScale_flag;//由于在提取的时候，尺寸是固定的，因此输出的坐标需要进行尺度转换
            auto SetOutputCoordScale_fun= std::bind(&ThingDetector::SetOutputCoordScale, std::ref(thingdetector), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);//std::ref：为了只析构1次
            std::call_once(SetOutputCoordScale_flag, SetOutputCoordScale_fun, cap.get(CAP_PROP_FRAME_HEIGHT), cap.get(CAP_PROP_FRAME_WIDTH), fore_extracter.scaledSize);           
            static vector<Rect>Thing_Detected;
            static vector<Rect>people_boxes;//由于是测试，所以没有行人检测结果，设为空即可
            thingdetector.Get_Thing_Result(Thing_Detected, people_boxes);//滤除行人，并进行尺度变换
            thingtracker.track(Thing_Detected, setROI);
            cout << "main time:" << double(clock() - start_time) / CLOCKS_PER_SEC << endl;
            //total_time += clock() - start_time;
            const int retention_frame_threshold = 40;//滞留帧数阈值
            for (int i = 0; i < thingtracker.tracking_things.size(); ++i)
            {
                const int & x1 = thingtracker.tracking_things[i].box.x;
                const int & y1 = thingtracker.tracking_things[i].box.y;
                const int x3 = x1 + thingtracker.tracking_things[i].box.width - 1;
                const int y3 = y1 + thingtracker.tracking_things[i].box.height - 1;
                if (thingtracker.tracking_things[i].track_frame > retention_frame_threshold)
                {//达到滞留帧数阈值，用红色画框
                    cv::rectangle(inputFrame, Point(x1, y1),
                        Point(x3, y3),
                        cv::Scalar(0, 0, 255));
                }
                else
                {
                    cv::rectangle(inputFrame, Point(x1, y1),
                        Point(x3, y3),
                        cv::Scalar(255, 255, 255));
                }
                   
            }    
            namedWindow("result_image", 0);
            imshow("result_image", inputFrame);
            // interact with user
            const char key = (char)waitKey(0);
        }	
	}
    //cout << "total time:" << double(total_time) / CLOCKS_PER_SEC << endl;
	return 0;
}
