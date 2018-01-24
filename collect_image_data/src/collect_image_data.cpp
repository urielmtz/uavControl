#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
bool isThereImage = false;
std::string folderName;
std::string objectName;
std::string fileName;
std::string fileNameProcessed;
std::string saveData;
cv::Mat res;	// added: processing background

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
	isThereImage = false;

	cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);

    cv::Mat frame;
	cv::Mat hsv_image;	// added: processing background
	cv::Mat mask;	// added: processing background
	frame = cv_bridge::toCvShare(msg, "bgr8")->image;

	cv::cvtColor(frame, hsv_image, COLOR_BGR2HSV);	// added: processing background
	cv::inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);	// added: processing background
	bitwise_and(frame, frame, res, mask);	// added: processing background
	cv::imshow("Processed image", res);	// added: processing background

	isThereImage = true;

	if( strcmp("Y", saveData) )
	{
		imageCounter++;
		fileName.clear();
		std::stringstream strCounter;
		strCounter << imageCounter;
		fileName = folderName + "/" + objectName + "_" + strCounter.str() + ".jpg";
		fileNameProcessed = folderName + "/" + objectName + "_" + strCounter.str() + "_processed.jpg";
		cv::imwrite(fileName, frame);
		cv::imwrite(fileNameProcessed, frame);
	}
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collect_image_data");
  ros::NodeHandle nh;

  std::cout << "Folder name: ";
  std::cin >> folderName;
  std::cout << "Object name: ";
  std::cin >> objectName;
  std::count << "Save data [Y/N]?: ";
  std:cin >> saveData;

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/bebop/image_raw", 1, imageCallback);
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/image_processed", 1);	// maybe this needs to be global
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res).toImageMsg();

  ros::Rate loop_rate(5);
  
  while(nh.ok())
  {
	  if(isThereImage == true)
	  {
		  pub.publish(msg);
		  ros::spinOnce();
		  loop_rate.sleep();
	  }
  }

  ros::spin();

  cv::destroyWindow("view");
}
