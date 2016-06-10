#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

#include <visualization_msgs/Marker.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

class ObjectRecognition{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        tf::TransformListener *tf_listener; 
        ros::Publisher cloud_pub;
        tf::TransformBroadcaster *tf_pub;
        ros::Publisher marker_pub;

    public:
        ObjectRecognition(){
            sub = nh.subscribe<PointCloud>("camera/depth_registered/points", 1, &ObjectRecognition::pointCloudCb, this);
            cloud_pub = nh.advertise<PointCloud> ("object_cloud", 1);
            tf_listener = new tf::TransformListener;
            tf_pub = new tf::TransformBroadcaster;
            marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        }

        void pointCloudCb(const PointCloud::ConstPtr& cloud_in){
            PointCloud::Ptr cloud_tf (new PointCloud);
            PointCloud::Ptr cloud_filtered (new PointCloud);

            std_msgs::Header header= pcl_conversions::fromPCL(cloud_in->header);
            
            tf::StampedTransform transform;        
            try {
                tf_listener->waitForTransform("/table_top", header.frame_id, header.stamp, ros::Duration(5.0));
                tf_listener->lookupTransform ("/table_top", header.frame_id, header.stamp, transform);
            }
            catch(std::runtime_error &e){
                return;
            }

            pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
            cloud_tf->header.frame_id = "/table_top";
            
            pcl::CropBox<Point> box;
            box.setInputCloud(cloud_tf);
            //this is our region of interest
            box.setMin(Eigen::Vector4f(-0.5,-0.4,0.03,1.0));
            box.setMax(Eigen::Vector4f(0.5,0.2,0.5,1.0));
            box.filter (*cloud_filtered);

            std::vector<pcl::PointIndices> indices;
            pcl::EuclideanClusterExtraction<Point> cluster;
            cluster.setClusterTolerance (0.02);
            cluster.setMinClusterSize (10);
            cluster.setInputCloud(cloud_filtered);
            cluster.extract(indices);

            if(indices.size() == 0){
                ROS_WARN_THROTTLE(10, "No object detected in ROI");
                return;
            }

            pcl::ExtractIndices<Point> extractor;
            pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices(indices[0]));
            extractor.setInputCloud(cloud_filtered);
            extractor.setIndices(objectIndices);
            PointCloud::Ptr objectCloud(new PointCloud);
            extractor.filter(*objectCloud);

            Eigen::Vector4f min_pt;
            Eigen::Vector4f max_pt;
            pcl::getMinMax3D (*objectCloud, min_pt, max_pt);

            Eigen::Vector4f center = (max_pt - min_pt)/2 + min_pt;
            center[2] = max_pt[2];

            transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
            transform.setRotation( tf::Quaternion(0, 0, 0, 1) );            
            tf_pub->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/table_top", "/object"));

            visualization_msgs::Marker cylinder;
            cylinder.header.frame_id = "/table_top";
            cylinder.header.stamp = ros::Time::now();
            cylinder.ns = "object";
            cylinder.action = visualization_msgs::Marker::ADD;
            cylinder.type = visualization_msgs::Marker::CYLINDER;
            cylinder.color.g = 1.0f;
            cylinder.color.a = 1.0;

            cylinder.pose.orientation.w = 1.0;
            cylinder.scale.x = 0.078;
            cylinder.scale.y = 0.078;
            cylinder.scale.z = center[2];
            cylinder.pose.position.x = center[0];
            cylinder.pose.position.y = center[1];
            cylinder.pose.position.z = center[2]/2;

            marker_pub.publish(cylinder);
            cloud_pub.publish(objectCloud);

        }

        ~ObjectRecognition(){}
};

int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
    ObjectRecognition objectRecognition;
    ros::spin();
    return 0;
}
