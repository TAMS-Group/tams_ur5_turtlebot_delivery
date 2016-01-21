#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ImageConverter
{

public:
//Konstruktor
  ImageConverter()
    : it_(nh_), dit_(dnh_),cloud()
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

  // Create a ROS subscriber for the input point cloud
    sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, &ImageConverter::cloud_cb, this);
    
    cv::namedWindow(OPENCV_WINDOW);

    search_dist[0] = 1.90;
    search_dist[1] = 1.80;

    arrCtr = 0;
    for (int i = 0; i <= ms; i++){
	medArrayX[i] = 0;
	medArrayY[i] = 0;
    }

  }


//Destruktor
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

//Callback RGB Image
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   //ACHTUNG: In Graustufen nur ein Wert pro Pixel statt drei!
   //Bild in Graustufen
   cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3, cv::Scalar::all(0));
   //handle Pointcloud
   if(cloud.size() != 0){
	int minY = 0;
	int maxY = 0;
	int minX = 0;
	int maxX = 0;

	for(int i = 0; i < cloud.height; i++)
	{
	    for(int j = 0; j < cloud.width; j++)
	    {
		if (cloud.at(j,i).z < search_dist[0] && cloud.at(j,i).z > search_dist[1]){
		    img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,0,255);
		}
	    }
	}


//***********************//
// Bounding box around table
//***********************//

	int largest_area=0;
 	int largest_contour_index=0;
 	cv::Rect bounding_rect;
 
 	cv::Mat thr(img.rows, img.cols, CV_8UC3); 
 	cv::Mat dst(img.rows, img.cols, CV_8UC3, cv::Scalar::all(0));
 	cv::cvtColor(img, thr, CV_BGR2GRAY); //Convert to gray
 	cv::threshold(thr, thr,25, 255,cv::THRESH_BINARY); //Threshold the gray
  
    	std::vector<std::vector<cv::Point> > contours; // Vector for storing contour
    	std::vector<cv::Vec4i> hierarchy;
 
    	cv::findContours( thr, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
   
	
     	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
      	{
       		double a = contourArea(contours[i],false);  //  Find the area of contour
       		if(a > largest_area){
       			largest_area = a;
       			largest_contour_index = i;                //Store the index of largest contour
       			bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
       		}
      	}
	 img = cv::Scalar::all(0);
 	cv::Scalar color( 255,255,255);
 	cv::drawContours( dst, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
 	cv::rectangle(img, bounding_rect,  cv::Scalar(0,255,0),1, 8,0);  

//***************************
// Bounding box around object
   	float h = 10; 

    // find highest point over table 
        for(int i = bounding_rect.y; i < bounding_rect.y + bounding_rect.height; i++)
	{
	    for(int j = bounding_rect.x; j < bounding_rect.x + bounding_rect.width; j++)
	    {
   		if (cloud.at(j,i).z < h){
		    h = cloud.at(j,i).z;
		}
	    }
	}
        
	cv::Mat img_o(img.rows, img.cols, CV_8UC3, cv::Scalar::all(0));

	for(int i = bounding_rect.y; i < bounding_rect.y + bounding_rect.height; i++)
	{
	    for(int j = bounding_rect.x; j < bounding_rect.x + bounding_rect.width; j++)
	    {
		if (cloud.at(j,i).z < h+0.03 && cloud.at(j,i).z > h-0.03){
		    img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,255,255);
		    img_o.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,255,255);
		}
		if (cloud.at(j,i).z < search_dist[0] && cloud.at(j,i).z > search_dist[1]){
		    img.at<cv::Vec3b>(cv::Point(j,i)) = cv::Vec3b(0,0,255);
		}
	    }	
	}

        cv::Rect bounding_rect_o;
        largest_area=0;
 	largest_contour_index=0;

 	cv::cvtColor(img_o, thr, CV_BGR2GRAY); //Convert to gray
 	cv::threshold(thr, thr,25, 255,cv::THRESH_BINARY); //Threshold the gray
  
 	cv::findContours( thr, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
   	
     	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
      	{
       		double a = contourArea(contours[i],false);  //  Find the area of contour
       		if(a > largest_area){
       			largest_area = a;
       			largest_contour_index = i;                //Store the index of largest contour
       			bounding_rect_o = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
       		}
      	}

 	cv::drawContours( dst, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
 	cv::rectangle(img, bounding_rect_o,  cv::Scalar(0,255,0),1, 8,0);

	object_transform(bounding_rect_o, h);
   }
  
   float alpha = 0.5;
   float beta = ( 1.0 - alpha );
   cv::addWeighted( cv_ptr->image, alpha, img, beta, 0.0, cv_ptr->image);
 
   // Output modified video stream
   cv::imshow(OPENCV_WINDOW, cv_ptr->image);
   cv::waitKey(3);

   image_pub_.publish(cv_ptr->toImageMsg());

}
 
//Callback Pointcloud
void cloud_cb(const PointCloud::ConstPtr& msg)
{
  cloud = *msg;
}

//get object position and send transform
void object_transform(cv::Rect bounding_rect, float h){
    int x = bounding_rect.x + (bounding_rect.width/2);
    int y = bounding_rect.y + (bounding_rect.height/2);
    
    medArrayX[ms] -= medArrayX[arrCtr];
   medArrayX[arrCtr] = x;
   medArrayX[ms] += medArrayX[arrCtr];
   medArrayY[ms] -= medArrayY[arrCtr];
   medArrayY[arrCtr] = y;
   medArrayY[ms] += medArrayY[arrCtr];
   
   
   if(arrCtr == ms-1){
   	arrCtr = 0;
   }
   x = medArrayX[ms]/ms;
   y = medArrayY[ms]/ms;

   float clX = cloud.at(x,y).x;
   float clY = cloud.at(x,y).y;
    
   if(clX != clX || clY != clY){}
   else{


    ros::Rate rate(10.0); 
    try {
    	listener.waitForTransform("/world", "/camera_link", ros::Time(0), ros::Duration(10.0) );
    	listener.lookupTransform("/world", "/camera_link", ros::Time(0), tmptransform);
    } catch (tf::TransformException ex) {
    	ROS_ERROR("%s",ex.what());
    }
    


    //if(x > 0 && y > 0){
	//transform.setOrigin(tf::Vector3(h, -cloud.at(x,y).x-0.03, -cloud.at(x,y).y));
	transform.setOrigin(tf::Vector3(clX+0.03, -clY, -h));
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );	
   // }

    //if(transform.getOrigin().getZ() != 0){
        transform *= tmptransform;
    	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_link", "/object"));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/object"));
	//ROS_INFO("X: %f Y: %f Z: %f", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    arrCtr++;
    }
}




protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::NodeHandle dnh_;
  image_transport::ImageTransport dit_;
  image_transport::Subscriber depth_image_sub_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  PointCloud cloud;
  bool dist[640][480];

  float search_dist[2];
  const static int ms = 10;
  int medArrayX[ms+1];
  int medArrayY[ms+1];
  int arrCtr;

  tf::StampedTransform tmptransform;
  tf::StampedTransform transform;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
return 0;
}
