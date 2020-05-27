#include<iostream>
#include<opencv2/opencv.hpp>
#include <string>
#include <vector>
#include<ctime>
#include<cmath>
#include<ctime>
#include <mutex>
#include"ThingInterface.h"

using namespace std;
using namespace cv;
using namespace hlg;
//�ڴ�й©���
#include <vld.h> 
//�ڴ�й©���end

int main(int argc, const char** argv)
{
    string video_path="D:\\opencv4.2\\opencv\\sources\\samples\\python\\tutorial_code\\video\\background_subtraction\\test2.avi";
	//string video_path = "E:\\���ݼ�\\����\\����\\loucengban\\clip_video\\ch03_20170315060019_31.avi";
	VideoCapture cap;
	if(!cap.open(video_path))//video_path
        printf("video/camera open failed!!!!!\n");
    //cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    //cap.set(CAP_PROP_FRAME_WIDTH, 640);
    bool show_flag = true;
    ThingInterface thing_interface;
    thing_interface.create_ForeExtraction(cap.get(CAP_PROP_FRAME_HEIGHT), cap.get(CAP_PROP_FRAME_WIDTH), show_flag);
    thing_interface.create_Thingdetector( 3000.0,  1000.0, 70.0, 30.0,  show_flag);
    thing_interface.create_Thingtracker();
    //ForeExtraction fore_extracter();
	Mat inputFrame, foregroundMask;
    //���ø���Ȥ����ROI
    Rect setROI = Rect(0, 0, int(cap.get(CAP_PROP_FRAME_WIDTH)), int(cap.get(CAP_PROP_FRAME_WIDTH)));
    /*Rect setROI = Rect(100, 50, int(0.6*int(cap.get(CAP_PROP_FRAME_WIDTH))), int(0.7*int(cap.get(CAP_PROP_FRAME_WIDTH))));*/
    //clock_t total_time=0;
	for (int i = 0;; i++)
	{
		cap >> inputFrame;//ԭʼ��С
		if (inputFrame.empty())
		{
			cout << "Finished reading: empty frame" << endl;
            cv::waitKey(0);
			break;
		}
        clock_t start_time = clock();
        
        if (thing_interface.fore_ExtractFore(inputFrame, foregroundMask))//��ȡǰ��
        {                        
            thing_interface.detect_ThingsDetect(foregroundMask);//��ǰ������ȡ���������           
            static once_flag SetOutputCoordScale_flag;//��������ȡ��ʱ�򣬳ߴ��ǹ̶��ģ���������������Ҫ���г߶�ת��
            auto SetOutputCoordScale_fun= std::bind(&ThingInterface::detect_SetOutputCoordScale, std::ref(thing_interface), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);//std::ref��Ϊ��ֻ����1��
            std::call_once(SetOutputCoordScale_flag, SetOutputCoordScale_fun, cap.get(CAP_PROP_FRAME_HEIGHT), cap.get(CAP_PROP_FRAME_WIDTH), thing_interface.fore_getScaledSize());
            static vector<Rect>Thing_Detected;
            static vector<Rect>people_boxes;//�����ǲ��ԣ�����û�����˼��������Ϊ�ռ���
            thing_interface.detect_Get_Thing_Result(Thing_Detected, people_boxes, setROI);//�˳����ˣ������г߶ȱ任
            thing_interface.track(Thing_Detected);
            cout << "main time:" << double(clock() - start_time) / CLOCKS_PER_SEC << endl;
            //total_time += clock() - start_time;
            const int retention_frame_threshold = 40;//����֡����ֵ
            const vector<vector<int>>&tracking_result = thing_interface.track_GetThingsInfo();
            for (int i = 0; i < tracking_result.size(); ++i)
            {
                const int  x1 = max(tracking_result[i][0], setROI.x);
                const int  y1 = max(tracking_result[i][1], setROI.y);
                const int x3 = min(tracking_result[i][2], setROI.x+setROI.width-1);
                const int y3 = min(tracking_result[i][3], setROI.y + setROI.height - 1);

                const int &track_frame = tracking_result[i][4];
                if (track_frame > retention_frame_threshold)
                {//�ﵽ����֡����ֵ���ú�ɫ����
                    cv::rectangle(inputFrame, Point(x1, y1),
                        Point(x3, y3),
                        cv::Scalar(0, 0, 255));
                }
                else
                {
                    cv::rectangle(inputFrame, Point(x1, y1),
                        Point(x3, y3),
                        cv::Scalar(0, 255, 255));
                }                   
            } 
            cv::rectangle(inputFrame, setROI, (255, 255, 255), 1);
            namedWindow("result_image", 0);
            imshow("result_image", inputFrame);
            // interact with user
            const char key = (char)waitKey(1);
        }	
	}
    //cout << "total time:" << double(total_time) / CLOCKS_PER_SEC << endl;
	return 0;
}
