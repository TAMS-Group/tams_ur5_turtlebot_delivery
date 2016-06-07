#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

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

            std::vector<pcl::PointIndices> indices;
            pcl::EuclideanClusterExtraction<Point> cluster;
            cluster.setClusterTolerance (0.02);
            cluster.setMinClusterSize (10);
            cluster.setInputCloud(cloud_filtered);
            cluster.extract(indices);

            pcl::ExtractIndices<Point> extractor;
            pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices(indices[0]));
            extractor.setIndices(objectIndices);
            PointCloud::Ptr objectCloud;
            extractor.filter(*objectCloud);

            Eigen::Vector4f* min_pt;
            Eigen::Vector4f* max_pt;
            pcl::getMinMax3D (objectCloud, min_pt, max_pt);

            tf_pub.publish(objectCloud);

        }

        ~ObjectRecognition(){}
};


int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
    ObjectRecognition objectRecognition;
    ros::spin();
    return 0;
}
