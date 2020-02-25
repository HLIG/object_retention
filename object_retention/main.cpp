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
//�ڴ�й©���
#include <vld.h> 
//�ڴ�й©���end

int main(int argc, const char** argv)
{
	string video_path = "E:\\���ݼ�\\����\\����\\loucengban\\clip_video\\ch03_20170315060019_31.avi";
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
		cap >> inputFrame;//ԭʼ��С
		if (inputFrame.empty())
		{
			cout << "Finished reading: empty frame" << endl;
			break;
		}
        clock_t start_time = clock();
        if (fore_extracter.extract(inputFrame, foregroundMask))//��ȡǰ��
        {                        
            thingdetector.ThingsDetect(foregroundMask);//��ǰ������ȡ���������           
            static once_flag SetOutputCoordScale_flag;//��������ȡ��ʱ�򣬳ߴ��ǹ̶��ģ���������������Ҫ���г߶�ת��
            auto SetOutputCoordScale_fun= std::bind(&ThingDetector::SetOutputCoordScale, std::ref(thingdetector), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);//std::ref��Ϊ��ֻ����1��
            std::call_once(SetOutputCoordScale_flag, SetOutputCoordScale_fun, cap.get(CAP_PROP_FRAME_HEIGHT), cap.get(CAP_PROP_FRAME_WIDTH), fore_extracter.scaledSize);           
            static vector<Rect>Thing_Detected;
            static vector<Rect>people_boxes;//�����ǲ��ԣ�����û�����˼��������Ϊ�ռ���
            thingdetector.Get_Thing_Result(Thing_Detected, people_boxes);//�˳����ˣ������г߶ȱ任
            thingtracker.track(Thing_Detected, setROI);
            cout << "main time:" << double(clock() - start_time) / CLOCKS_PER_SEC << endl;
            //total_time += clock() - start_time;
            const int retention_frame_threshold = 40;//����֡����ֵ
            for (int i = 0; i < thingtracker.tracking_things.size(); ++i)
            {
                const int & x1 = thingtracker.tracking_things[i].box.x;
                const int & y1 = thingtracker.tracking_things[i].box.y;
                const int x3 = x1 + thingtracker.tracking_things[i].box.width - 1;
                const int y3 = y1 + thingtracker.tracking_things[i].box.height - 1;
                if (thingtracker.tracking_things[i].track_frame > retention_frame_threshold)
                {//�ﵽ����֡����ֵ���ú�ɫ����
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
