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
#include <map>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>


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

    cloud_pub = nh3.advertise<sensor_msgs::PointCloud2>("alignedcloud", 1);
    sub = nh.subscribe("/camera/depth_registered/points", 1, &ImageConverter::cloud_cb, this);    

    search_dist[0] = 0.76;
    search_dist[1] = 0.67;

    ctrtmp = 0;

    arrCtr = 0;
    relevantArrayIndices = 0;
    for (int i = 0; i < ms; i++){
	medArrayX[i] = 0;
	medArrayY[i] = 0;
  	medArrayZ[i] = 0;
    }
  }


//Destruktor
  ~ImageConverter()
  {
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
        bounding_rect.y += 40;
	bounding_rect.height -= 40;
 	cv::rectangle(img, bounding_rect,  cv::Scalar(0,255,0),1, 8,0);  

    	 

//***************************
// Bounding box around object
   	float h = 0; 

    // find highest point over table 
        for(int i = bounding_rect.y; i < bounding_rect.y + bounding_rect.height; i++)
	{
	    for(int j = bounding_rect.x; j < bounding_rect.x + bounding_rect.width; j++)
	    {
   		if (cloud.at(j,i).z > h){
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

   image_pub_.publish(cv_ptr->toImageMsg());

}
 
//Callback Pointcloud
void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    sensor_msgs::PointCloud2 tmcloud;
    try {
        ros::Time tmptime = ros::Time::now();
  	cloudlistener.waitForTransform( "/world", "/camera_rgb_optical_frame",tmptime, ros::Duration(10.0) );
	cloudlistener.lookupTransform( "/world","/camera_rgb_optical_frame", tmptime, cloudtransform);
    } catch (tf::TransformException ex) {
    	ROS_ERROR("%s",ex.what());
    }

    geometry_msgs::TransformStamped geotransform;
    tf::transformStampedTFToMsg(cloudtransform, geotransform);

    tf2::doTransform (*msg, tmcloud, geotransform); 
  
    cloud_pub.publish(tmcloud);


    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(tmcloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new    pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    
    cloud = *temp_cloud;
}

//MODE calculator
int modeCalc(int array[], int cnt)
{
    std::map<int,int> IntMap;
    for( int a = 0; a <= cnt; ++a ) 
    {
	IntMap[array[a]]++;
    } 
    std::map<int,int>::iterator it = IntMap.begin();
    int max = it->first;
    for( ; it != IntMap.end(); ++it )
    {        
	if(IntMap[it->first] > IntMap[max] ) max = it->first;
    }
    return max;
}

//MODE calculator
float modeFCalc(float array[], int cnt)
{
    std::map<float,int> FMap;
    for( int a = 0; a <= cnt; ++a ) 
	{
		FMap[array[a]]++;
	} 
    std::map<float,int>::iterator it = FMap.begin();
    float max = it->first;
    for( ; it != FMap.end(); ++it )
	{        
	if(FMap[it->first] > FMap[max] ) max = it->first;
 	}
    return max;
}


//get object position and send transform
void object_transform(cv::Rect bounding_rect, float h){
    int x = bounding_rect.x + (bounding_rect.width/2);
    int y = bounding_rect.y + (bounding_rect.height/2);
/*    int h_int = (int)(h*100.0); 
   	
   medArrayX[arrCtr] = x;
   medArrayY[arrCtr] = y;
   medArrayZ[arrCtr] = h_int;
   	
   int mode_x = modeCalc(medArrayX, relevantArrayIndices);
   int mode_y = modeCalc(medArrayY, relevantArrayIndices);
   int mode_z = modeCalc(medArrayZ, relevantArrayIndices);

   if(relevantArrayIndices > ms-1){
	relevantArrayIndices = ms-1;
   }

   if(arrCtr == ms-1){
   	arrCtr = 0;
   }

   float z = (float)mode_z;
   
   float clX = cloud.at(mode_x,mode_y).x;
   float clY = cloud.at(mode_x,mode_y).y;
   float clH = z/100.0;

*/
   float clX = cloud.at(x,y).x;
   float clY = cloud.at(x,y).y;
   float clH = h;
    
   if(clX != clX || clY != clY){}
   else{
    
    transform.setOrigin(tf::Vector3(clX, clY, clH));
    
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );		
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/object"));
    
    std::cout << "X " << clX << " Y " << clY << " Z " << clH << std::endl;
     
    arrCtr++;
    relevantArrayIndices++;
    }
}




protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::NodeHandle nh3;
  ros::Publisher cloud_pub;

  ros::NodeHandle dnh_;
  image_transport::ImageTransport dit_;
  image_transport::Subscriber depth_image_sub_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  PointCloud cloud;
  bool dist[640][480];

  float search_dist[2];
  const static int ms = 5; //Mittel
  int medArrayX[ms];
  int medArrayY[ms];
  int medArrayZ[ms];

  int ctrtmp;

  int arrCtr;
  int relevantArrayIndices;

  tf::StampedTransform tmptransform;
  tf::StampedTransform transform;
  tf::StampedTransform cloudtransform;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::TransformListener cloudlistener;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
return 0;
}
