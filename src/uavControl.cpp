#include "opencv2/core/version.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>

using namespace std;
using namespace cv;


int imageCounter = 0;
bool cameraType = false;
std::string folderName;
std::string objectName;
std::string fileName;
int saveData = 0;
cv::Mat frame;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat hsv_image;	// added: processing background
		cv::Mat mask;	// added: processing background
		cv::Mat res;	// added: processing background

		frame = cv_bridge::toCvShare(msg, "bgr8")->image;

		cv::cvtColor(frame, hsv_image, COLOR_BGR2HSV);	// added: processing background
		cv::inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);	// added: processing background
		bitwise_and(frame, frame, res, mask);	// added: processing background
		frame = res;

		cv::imshow("view", frame);
		cv::waitKey(30);

		if (saveData == 1)
		{
			imageCounter++;
			fileName.clear();
			std::stringstream strCounter;
			strCounter << imageCounter;
			fileName = folderName + "/" + objectName + "_" + strCounter.str() + ".jpg";
			cv::imwrite(fileName, frame);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void imageCallback(cv::VideoCapture cap)
{
	cv::Mat res;
	cv::Mat hsv_image;	// added: processing background
	cv::Mat mask;	// added: processing background

	cap >> frame;

	cv::cvtColor(frame, hsv_image, COLOR_BGR2HSV);	// added: processing background
	cv::inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);	// added: processing background
	bitwise_and(frame, frame, res, mask);	// added: processing background
	frame = res;

	cv::imshow("view", frame);
	cv::waitKey(30);
}


void cbfunc()
{
	printf("called");
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_control");
	ros::NodeHandle nh;

	std::cout << "Folder name: ";
	std::cin >> folderName;
	std::cout << "Object name: ";
	std::cin >> objectName;
	std::cout << "Save data [Y=1/N=0]?: ";
std:cin >> saveData;
	std::cout << "Webcam [0] or bebop camera [1]?: ";
	std::cin >> cameraType;

	cv::namedWindow("view");
	cv::startWindowThread();

	image_transport::ImageTransport it(nh);

	cv::VideoCapture cap(0); // open the default camera

	if (cameraType == true)
	{
		image_transport::Subscriber sub = it.subscribe("/bebop/image_raw", 1, imageCallback);

		ros::spin();
	}
	else
	{
		if (!cap.isOpened())  // check if we succeeded
			return -1;

		std::cout << "Webcam opened to read images" << std::endl;

	}

	image_transport::Publisher pub = it.advertise("/image_processed", 1);	// maybe this needs to be global
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();




	ros::Rate loop_rate(100);

	while (nh.ok())
	{
		if (cameraType == false)
		{
			imageCallback(cap);
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		}

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	cv::destroyWindow("view");
}
