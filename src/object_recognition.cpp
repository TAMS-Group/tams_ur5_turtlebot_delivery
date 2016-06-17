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
        tf::TransformBroadcaster *tf_pub;
        ros::Publisher cloud_pub;
        ros::Publisher marker_pub;
        std::list<Eigen::Vector4f> center_window;

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
            std_msgs::Header header = pcl_conversions::fromPCL(cloud_in->header);

            visualization_msgs::Marker cylinder;
            cylinder.ns = "object";
            cylinder.header = header;
            cylinder.header.frame_id = "/table_top";
            cylinder.action = visualization_msgs::Marker::DELETE;
            cylinder.type = visualization_msgs::Marker::CYLINDER;
            cylinder.color.g = 1.0f;
            cylinder.color.a = 1.0;

            tf::StampedTransform transform;
            try{
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
            box.setMin(Eigen::Vector4f(-0.45,-0.35,0.05,1.0));
            box.setMax(Eigen::Vector4f(0.5,0.15,0.5,1.0));
            box.filter (*cloud_filtered);

            std::vector<pcl::PointIndices> indices;
            pcl::EuclideanClusterExtraction<Point> cluster;
            cluster.setClusterTolerance (0.02);
            cluster.setMinClusterSize (10);
            cluster.setInputCloud(cloud_filtered);
            cluster.extract(indices);
     
            if(indices.size() == 0){
                marker_pub.publish(cylinder);
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
            Eigen::Vector4f variance;

            lowPass(center, variance);

            if(variance.norm() > 0.001){
                marker_pub.publish(cylinder);
                return;
            }

            transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
            transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
            tf_pub->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/table_top", "/object"));

            cylinder.pose.orientation.w = 1.0;
            cylinder.scale.x = 0.078;
            cylinder.scale.y = 0.078;
            cylinder.scale.z = center[2];
            cylinder.pose.position.x = center[0];
            cylinder.pose.position.y = center[1];
            cylinder.pose.position.z = center[2]/2;
            cylinder.action = visualization_msgs::Marker::ADD;

            marker_pub.publish(cylinder);

            cloud_pub.publish(objectCloud);
        }

        void lowPass(Eigen::Vector4f& center, Eigen::Vector4f& variance){
            Eigen::Vector4f sum(0,0,0,0);
            Eigen::Vector4f sum_squared(0,0,0,0);

            center_window.push_back(center);
            for(std::list<Eigen::Vector4f>::iterator it = center_window.begin() ; it != center_window.end(); ++it){
                sum = sum + *it;
		sum_squared = sum_squared + (it->array() * (it->array())).matrix();
            }

	    center = sum/double(center_window.size());
            variance = sum_squared/center_window.size() - (center.array()*center.array()).matrix() ;

            variance[3] = 0;

            if(center_window.size() > 3){
                center_window.pop_front();
            }
        }

        ~ObjectRecognition(){}
};

int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
    ObjectRecognition objectRecognition;
    ros::spin();
    return 0;
}
