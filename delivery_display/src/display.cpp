#include <ros/ros.h>
#include <sstream>
#include <iostream>     // std::cout
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sstream>

#define SCANNING_REQUIRED -1
#define SCANNING_DONE 1
#define NO_SCANNING_NEEDED 0

int bin1 = 1, bin2 = 0, bin3 = 3;
int scanning_status = NO_SCANNING_NEEDED; 
bool system_blocked = false;
bool parts_ready = false;
std::string message = "";

void bins_required_callback (const std_msgs::Int8MultiArray::ConstPtr& msg){
	bin1 = msg->data[0];
	bin2 = msg->data[1];
	if (msg->data.size()>2){
		bin3 = msg->data[2];
	}
	else{
		bin3 = 0;
	}
}

void scanning_status_callback (const std_msgs::Int8::ConstPtr& msg){
	scanning_status = msg->data;
}

void system_blocked_callback (const std_msgs::Bool::ConstPtr& msg){
	system_blocked = msg->data;
}

void parts_ready_callback (const std_msgs::Bool::ConstPtr& msg){
	parts_ready = msg->data;
}

void message_callback (const std_msgs::String::ConstPtr& msg){
	message = msg->data;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "display_delivery");
	ros::WallDuration sleep_t(1.0);
	usleep(1000*1000);
	ros::NodeHandle nh_;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Subscriber bins_required_sub = nh_.subscribe("display/bins_required", 1, bins_required_callback);
	ros::Subscriber scanning_status_sub = nh_.subscribe("display/scanning_status", 1, scanning_status_callback);
	ros::Subscriber system_blocked_sub = nh_.subscribe("display/system_blocked", 1, system_blocked_callback);
	ros::Subscriber parts_ready_sub = nh_.subscribe("display/parts_ready", 1, parts_ready_callback);
	ros::Subscriber message_sub = nh_.subscribe("display/message", 1, message_callback);

	int w_width = 1920;
	int w_height = 1080;

	cv::Mat image = cv::Mat::zeros(w_height,w_width,CV_8UC3); 
	cv::namedWindow( "Display delivery status", 2 );

	cv::Scalar red = cv::Scalar(0,0,255);
	cv::Scalar green = cv::Scalar(0,255,0);
	cv::Scalar blue = cv::Scalar(255,0,0);
	cv::Scalar black = cv::Scalar(0,0,0);
	cv::Scalar white = cv::Scalar(255,255,255);
	cv::Scalar grey = cv::Scalar(51,51,51);
	cv::Scalar yellow = cv::Scalar(0,204,255);

	while(ros::ok()){
		// Parts needed rectangle //
		cv::rectangle( image,cv::Point(0,0),cv::Point(2*w_width/5, w_height/5), blue, -1, 8);
		cv::putText(image, "Parts needed :", cv::Point( 30, w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, yellow, 3);

		// Numbers rectangles //
		std::ostringstream int_result;
		if(bin1!=0){
			int_result << bin1;
			cv::rectangle( image,cv::Point(2*w_width/35,0.28*w_height),cv::Point(4*w_width/35, 0.44*w_height),white, -1, 8);
			cv::putText(image, int_result.str(), cv::Point(2.5*w_width/35,w_height/4+w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, black, 3);
		}
		else{
			cv::rectangle( image,cv::Point(2*w_width/35,0.28*w_height),cv::Point(4*w_width/35, 0.44*w_height),black, -1, 8);
		}
		if(bin2!=0){
			int_result.str("");
			int_result.clear();
			int_result << bin2;
			cv::rectangle( image,cv::Point(6*w_width/35,0.28*w_height),cv::Point(8*w_width/35, 0.44*w_height),white, -1, 8);
			cv::putText(image, int_result.str(), cv::Point(6.5*w_width/35,w_height/4+w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, black, 3);
		}
		else{
			cv::rectangle( image,cv::Point(6*w_width/35,0.28*w_height),cv::Point(8*w_width/35, 0.44*w_height),black, -1, 8);
		}
		if(bin3!=0){
			int_result.str("");
			int_result.clear();
			int_result << bin3;
			cv::rectangle( image,cv::Point(10*w_width/35,0.28*w_height),cv::Point(12*w_width/35, 0.44*w_height),white, -1, 8);
			cv::putText(image, int_result.str(), cv::Point(10.5*w_width/35,w_height/4+w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, black, 3);
		}
		else{
			cv::rectangle( image,cv::Point(10*w_width/35,0.28*w_height),cv::Point(12*w_width/35, 0.44*w_height),black, -1, 8);
		}

		// Parts ready rectangle //
		if (parts_ready){
			cv::rectangle( image,cv::Point(0,0.54*w_height),cv::Point(w_width/4, 0.68*w_height),green, -1, 8);
			cv::putText(image, "Parts ready !", cv::Point( 30, w_height/2+w_height/8), cv::FONT_HERSHEY_SIMPLEX, 1.5, white, 2);
		}
		else{
			cv::rectangle( image,cv::Point(0,0.54*w_height),cv::Point(w_width/4, 0.68*w_height),red, -1, 8);
			cv::putText(image, "Parts not ready !", cv::Point( 30, w_height/2+w_height/8), cv::FONT_HERSHEY_SIMPLEX, 1.5, white, 2);
		}

		// Scanning rectangles //
		switch(scanning_status){
		case SCANNING_REQUIRED:
			cv::rectangle( image,cv::Point(w_width/2,0),cv::Point(w_width/2+w_width/4, w_height/4),grey, -1, 8);	
			cv::rectangle( image,cv::Point(w_width/2+w_width/4,0),cv::Point(w_width, w_height/4),red, -1, 8);
			break;

		case SCANNING_DONE:
			cv::rectangle( image,cv::Point(w_width/2,0),cv::Point(w_width/2+w_width/4, w_height/4),green, -1, 8);	
			cv::rectangle( image,cv::Point(w_width/2+w_width/4,0),cv::Point(w_width, w_height/4),grey, -1, 8);
			break;

		case NO_SCANNING_NEEDED:
			cv::rectangle( image,cv::Point(w_width/2,0),cv::Point(w_width/2+w_width/4, w_height/4),grey, -1, 8);	
			cv::rectangle( image,cv::Point(w_width/2+w_width/4,0),cv::Point(w_width, w_height/4),grey, -1, 8);
			break;
		}
		cv::putText(image, "Scanning", cv::Point( 0.65*w_width, w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, white, 3);

		// System blocked rectangle //
		if (system_blocked){
			cv::rectangle( image,cv::Point(w_width/2,w_height/2),cv::Point(w_width,w_height/2+ w_height/4), red, -1, 8);
			cv::putText(image, "System blocked", cv::Point( 0.65*w_width, w_height/2+ w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, white, 3);
		}
		else{
			cv::rectangle( image,cv::Point(w_width/2,w_height/2),cv::Point(w_width,w_height/2+ w_height/4), black, -1, 8);
		}

		// Print last update message //
		cv::rectangle( image,cv::Point(0,3*w_height/4),cv::Point(w_width,w_height), black, -1, 8);
		cv::putText(image, message, cv::Point(30, 3*w_height/4+w_height/8), cv::FONT_HERSHEY_SIMPLEX, 2, white, 3);

		cv::namedWindow( "Display delivery status", cv::WINDOW_AUTOSIZE );
		cv::imshow( "Display delivery status", image );

		ros::spinOnce();

		/// Exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		if( cv::waitKey(1)%256==27 ) break;
	}
	//cv::waitKey(0);                                     
	ros::shutdown();
	return 0;
}
