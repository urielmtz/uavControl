/********************************************************************
* filename: uavControl
* author(s): Uriel Martinez-Hernandez (1)
*            Adrian Rubio-Solis (2)
*
* Institutions:
* ---- 1 - Department of Electronic and Electrical Engineering
*          University of Bath
*
* ---- 2 - Department of Automatic Control and Systems Engineering
*          University of Sheffield
*
* date: 12-02-2018
********************************************************************/


#include "opencv2/core/version.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <curses.h>
#include <unistd.h>

#include <sys/select.h>
#include <termios.h>

#include <algorithm>
#include <math.h>



using namespace std;
using namespace cv;
using namespace saliency;

int imageCounter = 0;
bool cameraType = false;
std::string folderName;
std::string objectName;
std::string fileName;
int saveData = 0;
cv::Mat frameBebop;
cv::Mat frameWebcam;
cv::Mat frameSaliency;


int yCentre = 480 / 2;
int xCentre = 856 / 2;
double minimo, maximo;
cv::Point min_loc, max_loc;
bool isUavInTarget = true;

float forwardSpeed = 0.1;
float backwardSpeed = -0.1;
float rotateLeft = -0.1;
float rotateRight = 0.1;

float maxProbability = 0.0;
float probabilityThreshold = 0.9999;


/* Callback that reads input image from the Bebop camera */
void imageCallbackBebop(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat hsv_image;	// added: processing background
		cv::Mat mask;	// added: processing background
		cv::Mat res;	// added: processing background

		Ptr<Saliency> saliencyAlgorithm;

		Mat binaryMap;
		Mat image;

		frameBebop = cv_bridge::toCvShare(msg, "bgr8")->image;

		cv::cvtColor(frameBebop, hsv_image, COLOR_BGR2HSV);	// added: processing background
		cv::inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);	// added: processing background
		bitwise_and(frameBebop, frameBebop, res, mask);	// added: processing background
		frameBebop = res;

		frameBebop.copyTo(image);

		String saliency_algorithm = "SPECTRAL_RESIDUAL";

		if (saliency_algorithm.find("SPECTRAL_RESIDUAL") == 0)
		{
			Mat saliencyMap;
			saliencyAlgorithm = StaticSaliencyFineGrained::create();
			if (saliencyAlgorithm->computeSaliency(image, saliencyMap))
			{

				if (isUavInTarget == true)
				{
					minMaxLoc(saliencyMap, &minimo, &maximo, &min_loc, &max_loc);
					circle(saliencyMap, max_loc, 20, (0, 0, 255), -1);
					isUavInTarget = false;
				}

				imshow("Bebop - Saliency Map", saliencyMap);
				imshow("Bebop - Original Image", image);
				waitKey(50);
			}
		}

		cv::imshow("Bebop - view", frameBebop);
		cv::waitKey(30);


		if (saveData == 1)
		{
			imageCounter++;
			fileName.clear();
			std::stringstream strCounter;
			strCounter << imageCounter;
			fileName = folderName + "/" + objectName + "_" + strCounter.str() + ".jpg";
			cv::imwrite(fileName, frameBebop);
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

/* Function that reads input images from the webcam */
void imageCallbackWebcam(cv::VideoCapture cap)
{
	cv::Mat res;
	cv::Mat hsv_image;	// added: processing background
	cv::Mat mask;	// added: processing background


	Ptr<Saliency> saliencyAlgorithm;

  	Mat binaryMap;
  	Mat image;

	cap >> frameWebcam;

	cv::cvtColor(frameWebcam, hsv_image, COLOR_BGR2HSV);	// added: processing background
	cv::inRange(hsv_image, Scalar(0, 70, 50), Scalar(10, 255, 255), mask);	// added: processing background
	bitwise_and(frameWebcam, frameWebcam, res, mask);	// added: processing background
	frameWebcam = res;

  	frameWebcam.copyTo( image );

	String saliency_algorithm = "FINE_GRAINED";

	if( saliency_algorithm.find( "FINE_GRAINED" ) == 0 )
	{
		Mat saliencyMap;
		saliencyAlgorithm = StaticSaliencyFineGrained::create();
		if( saliencyAlgorithm->computeSaliency( image, saliencyMap ) )
		{
			cv::Size s = saliencyMap.size();
			int rows = saliencyMap.rows;   int cols = saliencyMap.cols;
			rows = s.height;     cols = s.width;
			minMaxLoc(saliencyMap, &minimo, &maximo, &min_loc, &max_loc);
			circle(saliencyMap, max_loc, 20, (0, 0, 255), -1);

			imshow( "Webcam - Saliency Map", saliencyMap );
			imshow( "Webcam - Original Image", image );
			waitKey( 50 );
		}
	}

	cv::imshow("Webcam - view", frameWebcam);
	cv::waitKey(30);
}

void cnnCallback(const std_msgs::Float32::ConstPtr& msg)
{
	maxProbability = msg->data;
}



/* Main */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "uav_control");
	ros::NodeHandle nh;

	std::cout << "Select input device Webcam [0] or Bebop camera [1]?: ";
	std::cin >> cameraType;

	cv::startWindowThread();

	ros::Publisher takeoffCommand_pub = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
	ros::Publisher landCommand_pub = nh.advertise<std_msgs::Empty>("/bebop/land", 1);
	ros::Publisher velCommand_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);

	image_transport::ImageTransport it(nh);

	cv::VideoCapture cap(0); // open the default camera

	image_transport::Subscriber sub = it.subscribe("/bebop/image_raw", 1, imageCallbackBebop); // received input image from Bebop robot
	image_transport::Publisher pub = it.advertise("/image_processed", 1);	// sends segmented image to CNN in Python for classification

	ros::Subscriber subCNN = nh.subscribe<std_msgs::Float32>("/cnn_probability", 1, cnnCallback); // receives output probability from CNN in Python

	if( cameraType == false )
	{
		if (!cap.isOpened())  // check if we succeeded
			return -1;

		std::cout << "Webcam opened to read images" << std::endl;

	}


	std_msgs::Empty msgLanding;
	geometry_msgs::Twist velMsg;

	ros::Rate loop_rate(100);
	ros::spinOnce();


	while (ros::ok())
	{
		if (cameraType == false)  // Webcam
		{
			imageCallbackWebcam(cap);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameWebcam).toImageMsg();
			pub.publish(msg);
		}
		else if(cameraType == true)  // Bebop
		{
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frameBebop).toImageMsg();
			pub.publish(msg);
		}
		else
		{
			std::cout << "Error, no video input selected or found" << std::endl;
		}


		/*First approach to control the Bebop position during exploration of salient regions.
		* This is a naive method based on calculation of small movements without position feedback.
		*/
		if (isUavInTarget == false)
		{
			int pixelStep = 10;
			int xTargetDifference = abs(max_loc.x - xCentre);
			int movementSteps = int(xTargetDifference / pixelStep);
			int maxNumberOfSteps = 10;


			if (movementSteps > maxNumberOfSteps)
				movementSteps = maxNumberOfSteps;

			std::cout << "Maximum number of steps: " << movementSteps << std::endl;

			for (int i = 0; i < movementSteps; i++)
			{
				velMsg.linear.x = 0;
				velMsg.linear.y = 0;
				velMsg.linear.z = 0;
				velMsg.angular.z = 0;

				if (max_loc.x < xCentre)
				{// left
					velMsg.linear.y = forwardSpeed;
					std::cout << "Step to left: " << i + 1 << std::endl;
				}
				else if (max_loc.x > xCentre)
				{// right
					velMsg.linear.y = backwardSpeed;
					std::cout << "Step to right: " << i + 1 << std::endl;
				}

				velCommand_pub.publish(velMsg);

				usleep(500000);
			}

			isUavInTarget = true;
		}

		ros::spinOnce();
		loop_rate.sleep();

		/* The object exploration task stops whenever the threshold is exceeded.
		* Then, the Bebop robot is landed.
		*/
		std::cout << "Current probability: " << maxProbability << std::endl;
		if (maxProbability > probabilityThreshold)
		{
			std::cout << "======= Probability threshold exceeded =======" << std::endl;
			std::cout << "END OF ACTIVE EXPLORATION PROCESS" << std::endl;
			break;
		}
	}

	landCommand_pub.publish(msgLanding);
}

