#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


float minFilterLtd, maxFilterLtd;
int disTh;
std::string topic_name, fieldName, baselink;
std::string output_name;
ros::Publisher pub;


tf::StampedTransform lookup_transform(const std::string& target_frame, const std::string& source_frame) 
{
    tf::StampedTransform transform;
    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform (target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }
    return transform;
}

void transformPointCloud(const std::string &target_frame, const pcl::PointCloud<pcl::PointXYZ> &in, pcl::PointCloud<pcl::PointXYZ> &out)
{
    pcl_ros::transformPointCloud(in, out, lookup_transform(target_frame, in.header.frame_id));
} 


void callback(const sensor_msgs::PointCloud2ConstPtr &msg) {


pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);


pcl::fromROSMsg<pcl::PointXYZ>(*msg, *raw_cloud);
transformPointCloud(baselink, *raw_cloud, *transformed_cloud);


pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud (transformed_cloud);
pass.setFilterFieldName (fieldName);
pass.setFilterLimits (minFilterLtd, maxFilterLtd);

sensor_msgs::PointCloud2 output_msg;
pcl::toROSMsg(*transformed_cloud, output_msg);

output_msg.header.stamp = msg->header.stamp;
output_msg.header.frame_id = baselink;

pub.publish(output_msg);

}

int main(int argc, char **argv){

ros::init(argc, argv, "fetch_object_grabber");

ros::NodeHandle nh("~");



  nh.param<std::string>("topic_name", topic_name, "/head_camera/depth_downsample/points");
  nh.param<std::string>("output_name", output_name, "output");
  nh.param<std::string>("baselink", baselink, "base_link");

    
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topic_name, 1, callback);  
  pub = nh.advertise<sensor_msgs::PointCloud2>(output_name, 1);
   
   
    ros::Rate rate(10);
    while (ros::ok()) {

    nh.param<float>("minFilterLtd", minFilterLtd, 0.0);
    nh.param<float>("maxFilterLtd", maxFilterLtd, 1.0);
    nh.param<std::string>("fieldName", fieldName, "z");
    nh.param<int>("disTh", disTh, 10);


    ros::spinOnce(); 
    rate.sleep();
    }    

    // ros::spin();
    return 0;
  }