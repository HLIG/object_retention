#include<iostream>
#include<opencv2/opencv.hpp>
#include <string>
#include <vector>
#include<ctime>
#include<cmath>
#include<ctime>
#include"fore_extraction.h"
#include"KalmanTracker_TH.h"
using namespace std;
using namespace cv;
using namespace hlg;
//内存泄漏检测
#include <vld.h> 
//内存泄漏检测end
RNG g_rng(12345);
const float min_area = 10.0; 


int main(int argc, const char** argv)
{

	string video_path = "E:\\数据集\\扶梯\\地铁\\loucengban\\clip_video\\ch03_20170315060019_31.avi";

	VideoCapture cap;
	if(!cap.open(video_path))
        printf("video/camera open failed!!!!!\n");

    ForeExtraction fore_extracter(cap.get(CAP_PROP_FRAME_HEIGHT), cap.get(CAP_PROP_FRAME_WIDTH),false);
	Mat inputFrame, foregroundMask;
	ThingKalmanTracker thingtracker;
	int frame_id = 0;

	for (int i = 0;; i++)
	{
		cap >> inputFrame;//原始大小
        //cout <<"inputFrame.h:"<< inputFrame.rows << " " << "inputFrame.w:" << inputFrame.cols << endl;
        //cout << "cap.get(CAP_PROP_FRAME_HEIGHT):" << cap.get(CAP_PROP_FRAME_HEIGHT) << " " << "cap.get(CAP_PROP_FRAME_WIDTH):" << cap.get(CAP_PROP_FRAME_WIDTH) << endl;
        //cout << "inputFrame width" << inputFrame.cols << "inputFrame height" << inputFrame.rows << endl;
		if (inputFrame.empty())
		{
			cout << "Finished reading: empty frame" << endl;
			break;
		}
        clock_t start_time = clock();
        if (fore_extracter.extract(inputFrame, foregroundMask))
        {
            cout << "main time:" << double(clock() - start_time) / CLOCKS_PER_SEC << endl;
            thingtracker.ThingsDetector(foregroundMask);
            //cout << "foregroundMask width" << foregroundMask.cols << "foregroundMask height" << foregroundMask.rows << endl;
            //将坐标转到
            Mat image_resize;
            resize(inputFrame, image_resize, fore_extracter.scaledSize);
            for (int i = 0; i < thingtracker.big_obejcts.size(); ++i)
            {
                int x1 = thingtracker.big_obejcts[i].box.x1;
                int x3 = thingtracker.big_obejcts[i].box.x3;
                int y1 = thingtracker.big_obejcts[i].box.y1;
                int y3 = thingtracker.big_obejcts[i].box.y3;
                //printf("x1=%d ,y1=%d,x3=%d,y3=%d\n", x1, y1, x3, y3);
                cv::rectangle(image_resize, Point(x1, y1),
                    Point(x3, y3),
                    cv::Scalar(255, 255, 255));
            }
            namedWindow("result_image", 0);
            imshow("result_image", image_resize);

            // interact with user
            const char key = (char)waitKey(1);
            if (key == 27 || key == 'q') // ESC
            {
                cout << "Exit requested" << endl;
                break;
            }
        }
		
	}
 
	return 0;
}
