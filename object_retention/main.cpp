#include<iostream>
#include<opencv2/opencv.hpp>
#include <string>
#include <vector>
#include<ctime>
#include"KalmanTracker_TH.h"
using namespace std;
using namespace cv;

RNG g_rng(12345);
const float min_area = 10.0;



int main(int argc, const char** argv)
{
	string video_path = "E:\\数据集\\扶梯\\地铁\\loucengban\\clip_video\\ch03_20170315060019_31.avi";
	const int init_background_length = 600;//开始的600帧不断循环，当作背景建模
	const int init_background_frame_length = 2;//从摄像头读取前面30帧当作背景
	vector<Mat>frame_buff(init_background_frame_length);
	VideoCapture cap;
	cap.open(video_path);
	Mat inputFrame;
	cap >> inputFrame;
	const Size scaledSize(640, 640 * inputFrame.rows / inputFrame.cols);
	/*knn参数*/
	const double learning_rate = 0.001;
	Ptr<BackgroundSubtractor> model = createBackgroundSubtractorKNN(20000, 2500, true);
	//createBackgroundSubtractorKNN(20000, 200, true);
	//createBackgroundSubtractorMOG2(20000, 100, true);
	for (int i = 0; i < init_background_frame_length; ++i)
	{
		resize(inputFrame, inputFrame, scaledSize, 0, 0, INTER_LINEAR);
		frame_buff[i] = inputFrame;
		cap >> inputFrame;
	}
	Mat  frame, foregroundMask, foreground, background, final_foremask;
	for (int i = 0; i < init_background_length; i++)
	{
		Mat temp = frame_buff[i%init_background_frame_length];
		model->apply(temp, foregroundMask, learning_rate);
	}

	bool doSmoothMask = true;
	ThingKalmanTracker thingtracker;
	int frame_id = 0;
	for (int i = 0;; i++)
	{

		// prepare input frame
		if (i>0)
			cap >> inputFrame;
		//change_to_hsv(inputFrame);
		if (inputFrame.empty())
		{
			cout << "Finished reading: empty frame" << endl;
			break;
		}
		const Size scaledSize(640, 640 * inputFrame.rows / inputFrame.cols);
		resize(inputFrame, frame, scaledSize, 0, 0, INTER_LINEAR);

		model->apply(frame, foregroundMask, learning_rate);


		// show processed frame
		

		//阈值化，将非纯白色（244~255）的所有像素设为0 将影子去掉
		cv::threshold(foregroundMask, final_foremask, 244, 255, cv::THRESH_BINARY);

		// show foreground image and mask (with optional smoothing)
		if (doSmoothMask)
		{
			Mat kernel = getStructuringElement(MORPH_ERODE, Size(3, 3));
			cv::morphologyEx(final_foremask, final_foremask, cv::MORPH_CLOSE, kernel, Point(-1, -1), 5);
			//GaussianBlur(foregroundMask, foregroundMask, Size(11, 11), 3.5, 3.5);
		}
		thingtracker.ThingsDetector(final_foremask);
		for (int i = 0; i < thingtracker.big_obejcts.size(); ++i)
		{
			int x1 = thingtracker.big_obejcts[i].box.x1;
			int x3 = thingtracker.big_obejcts[i].box.x3;
			int y1 = thingtracker.big_obejcts[i].box.y1;
			int y3 = thingtracker.big_obejcts[i].box.y3;
			cv::rectangle(frame, Point(x1, y1),
				Point(x3, y3),
				cv::Scalar(255, 255, 255));
		}
		namedWindow("image", 0);
		imshow("image", frame);
		if (foreground.empty())
			foreground.create(scaledSize, frame.type());
		foreground = Scalar::all(0);
		frame.copyTo(foreground, final_foremask);
		//imshow("foreground mask", final_foremask);
		//imshow("foreground image", foreground);

		// show background image
		model->getBackgroundImage(background);
		if (!background.empty());
			//imshow("mean background image", background);

		// interact with user
		const char key = (char)waitKey(0);
		if (key == 27 || key == 'q') // ESC
		{
			cout << "Exit requested" << endl;
			break;
		}

		else if (key == 's')
		{
			doSmoothMask = !doSmoothMask;
			cout << "Toggle foreground mask smoothing: " << (doSmoothMask ? "ON" : "OFF") << endl;
		}
	}
	return 0;
}
