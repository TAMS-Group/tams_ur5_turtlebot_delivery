#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ObjectRecognition{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        tf::TransformListener *tf_listener; 
        ros::Publisher tf_pub;
        PointCloud pcl_out;

    public:
        ObjectRecognition(){
            sub = nh.subscribe<PointCloud>("camera/depth_registered/points", 1, &ObjectRecognition::pointCloudCb, this);
            tf_pub = nh.advertise<PointCloud> ("tf_points2", 1);
            tf_listener = new tf::TransformListener;
        }

        void pointCloudCb(const PointCloud::ConstPtr& cloud){
            std_msgs::Header header= pcl_conversions::fromPCL(cloud->header);
            tf_listener->waitForTransform("/world", header.frame_id, header.stamp, ros::Duration(5.0));
            pcl_ros::transformPointCloud("/world", *cloud, pcl_out, *tf_listener);
            tf_pub.publish(pcl_out);
        }

        ~ObjectRecognition(){}
};


int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
    ObjectRecognition objectRecognition;
    ros::spin();
    return 0;
}
