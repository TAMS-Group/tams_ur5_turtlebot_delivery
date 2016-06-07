#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

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
            try {
                tf_listener->waitForTransform("/world", header.frame_id, header.stamp, ros::Duration(5.0));
                pcl_ros::transformPointCloud("/world", *cloud_in, *cloud_tf, *tf_listener);
            }
            catch(std::runtime_error &e){
                return;
            }

            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud (cloud_in);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.0, 1.0);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_filtered);

            tf_pub.publish(cloud_filtered);

        }

        void passThroughFilter(boost::shared_ptr<PointCloud>& cloud_in,boost::shared_ptr<PointCloud>& cloud_out, std::string field, float limit_min, float limit_max, bool limit_neg){
            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud (cloud_in);
            pass.setFilterFieldName (field);
            pass.setFilterLimits (limit_min, limit_max);
            pass.setFilterLimitsNegative (limit_neg);
            pass.filter (*cloud_out);
        }

        ~ObjectRecognition(){}
};


int main(int argc, char** argv){
    ros::init(argc, argv, "object_recognition");
    ObjectRecognition objectRecognition;
    ros::spin();
    return 0;
}
