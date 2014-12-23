#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include<sstream>
#include<iostream>
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include<sstream>
#include <opencv/cv.h>
#include "detectobs.cpp"
#include "Perspective_transform.cpp"
#include "mid_point.cpp"
#include <fstream>
#include <iostream>
#include "./MarkovModule/markov.cpp"
using namespace std;
int rc_old = 0;
int rc = 0;
int H = 150;
cv::Mat imageGray;
cv::Mat imageColor;

  int i=0;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
++i;	
  try  
   { 
   //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    //string ssave1 = "/home/iiith/Desktop/new/" + ss.str()+ ".jpg";
    imageGray = cv_bridge::toCvShare(msg, "mono8")->image;
   // cv::imshow("view", cv_bridge::toCvShare(msg, "mono8")->image);       
    imageColor = cv_bridge::toCvShare(msg, "bgr8")->image;
    //cv::imshow("imageColor", cv_bridge::toCvShare(msg, "bgr8")->image);    
    //cv::imwrite(ssave1,cv_bridge::toCvShare(msg, "bgr8")->image);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
   
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_subsc");
  cv::namedWindow("view");
  ros::NodeHandle nh;
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  stringstream ss;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::Int16>("midpoint", 10);
  image_transport::Subscriber sub = it.subscribe("/camera/right/image_raw", 10, imageCallback);
  std_msgs::Int16 msg; 
  int move  = 0;
  while(ros::ok()){
	ros::spinOnce();
 if(i>0){
static int start = 1;
    ss<<i;
    cv::Mat Mask(640,480,CV_8UC1,Scalar(255));
    MarkovRandomField mrf;
    rc = mrf.Mrf(imageGray, Mask, imageColor,H);

if(start){
if( rc < 100000)
{
H = (int)(H*0.75 + 0.25*(1-1.0*((1.0*rc)/(100000.0))));
cout << "rc" << rc << "THIS H: " << H << endl; 
}
else if( rc > 200000){
H = (int)(H*0.75 + 0.25*(1-1.0*((1.0*rc)/(200000.0))));
cout << "rc" << rc << "NOW H: " << H << endl;
}
else
start=0;
}
else{
    if(rc_old == 0)
    	   rc_old = rc;
    else
	   H = (int)(H*0.7 + H*((rc*1.0)/(1.0*rc_old))*0.3);
    if(H > 200)
	   H = 200;
    else if(H < 50)
	   H = 50;
           cout << " rc: " << rc << " H: " << H;
  /// string ssave1 = "/home/iiith/Desktop/new_result/" + ss.str()+ ".jpg";
  //  cv::imwrite(ssave1,imageColor);
    cv::imshow("view",imageColor);
    //cv::Mat pti = Perspective_transform(imageColor);
   // string ssave2 = "/home/iiith/Desktop/Perspective_transform/" + ss.str()+ ".jpg";
 //   cv::imwrite(ssave2,pti);
    move = mid_point(imageColor);
	msg.data = move;
        ROS_INFO("%d", msg.data);
        chatter_pub.publish(msg);
}
  }
  }
  //cv::destroyWindow("view");
}
