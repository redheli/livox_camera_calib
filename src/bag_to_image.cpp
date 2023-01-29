#include "include/CustomMsg.h"
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Image.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

string bag_file;
string rect_image_topic;
string output_dir;
bool is_custom_msg;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("bag_file", bag_file, "/home/max/ground_map/rosbag/home-clib/2023-01-29-21-35-40.bag");
  nh.param<string>("output_dir", output_dir, "/home/max/ground_map/rosbag/home-clib/images");
  // nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
  nh.param<string>("rect_image_topic", rect_image_topic, "/camera/left/image_rect");
  // nh.param<bool>("is_custom_msg", is_custom_msg, false);
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_) {
    std::string msg = "Loading the rosbag " + bag_file + " failue";
    ROS_ERROR_STREAM(msg.c_str());
    return -1;
  }
  ROS_INFO("Loading the rosbag %s", bag_file.c_str());
  ROS_INFO("Output folder %s", output_dir.c_str());
  ROS_INFO("Rectified image topic %s", rect_image_topic.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return -1;
  }
  std::vector<string> topic_vec;
  topic_vec.push_back(rect_image_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topic_vec));
  int count = 0;
  for (const rosbag::MessageInstance &m : view) {
      
      if(count % 100 == 0){
        sensor_msgs::Image msg;
        msg = *(m.instantiate<sensor_msgs::Image>()); // message
        double img_timestamp = msg.header.stamp.toSec();
        cv::Mat image_get = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 )->image.clone();
        std::string fn = output_dir + "/" + std::to_string(img_timestamp)+".png";
        cv::imwrite(fn,image_get);
        ROS_INFO("save image %d ts %f file %s",count,img_timestamp,fn.c_str());
      }
      count++;
  }
  
  // string msg = "Sucessfully save point cloud to pcd file: " + output_dir;
  // ROS_INFO_STREAM(msg.c_str());
  return 0;
}