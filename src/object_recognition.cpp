#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
//#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

class ObjectRecognition{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        tf::TransformListener *tf_listener; 
        ros::Publisher tf_pub;

    public:
        ObjectRecognition(){
            sub = nh.subscribe<PointCloud>("camera/depth_registered/points", 1, &ObjectRecognition::pointCloudCb, this);
            tf_pub = nh.advertise<PointCloud> ("tf_points2", 1);
            tf_listener = new tf::TransformListener;
        }

        void pointCloudCb(const PointCloud::ConstPtr& cloud_in){
            PointCloud::Ptr cloud_tf (new PointCloud);
            PointCloud::Ptr cloud_filtered (new PointCloud);

            std_msgs::Header header= pcl_conversions::fromPCL(cloud_in->header);
            
            tf::StampedTransform transform;        
            try {
                tf_listener->waitForTransform("/world", header.frame_id, header.stamp, ros::Duration(5.0));
                tf_listener->lookupTransform ("/world", header.frame_id, header.stamp, transform);
            }
            catch(std::runtime_error &e){
                return;
            }

            pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
            cloud_tf->header.frame_id = "/world";
            
            pcl::CropBox<Point> box;
            box.setInputCloud(cloud_tf);
            //this is our region of interesst
            box.setMin(Eigen::Vector4f(0.25,-0.05,0.77,1.0));
            box.setMax(Eigen::Vector4f(1.25,0.5,1.2,1.0));
            box.filter (*cloud_filtered);

            tf_pub.publish(cloud_filtered);

        }

        ~ObjectRecognition(){}
};


int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
    ObjectRecognition objectRecognition;
    ros::spin();
    return 0;
}
