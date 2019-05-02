#include <iostream>
#include <cstdlib>
#include <fstream>
#include <time.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <limits>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

//for image input
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <ros/package.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

// PCL specific includes
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include <darknet_dnn/BoundingBox.h>

#include <float.h>
#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))

class TF_Broadcaster{
private:
  ros::NodeHandle nh;
  //ros::Subscriber sub_ctrl;
  ros::Subscriber sub_bbox;
  ros::Subscriber sub_point_cloud;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;

  pcl::PointCloud<pcl::PointXYZ> cloud_local;

  std::string cloud_topic_name;
  std::string camera_frame_name;

  bool is_get_point_cloud = false;


public:
  TF_Broadcaster(){
    //nh.param("cloud_topic_name", this->cloud_topic_name);
    //nh.param("camera_frame_name", this->camera_frame_name);
    ros::param::get("cloud_topic_name", this->cloud_topic_name);
    ros::param::get("camera_frame_name", this->camera_frame_name);

    ROS_INFO("cloud_topic_name:[%s]", this->cloud_topic_name.c_str());
    ROS_INFO("camera_frame_name:[%s]", this->camera_frame_name.c_str());

    sub_bbox = nh.subscribe("/tracker_ros/goturn/result_bbox", 1, &TF_Broadcaster::bbox_CB, this);
    sub_point_cloud = nh.subscribe(cloud_topic_name, 1, &TF_Broadcaster::point_cloud_CB, this);

    ROS_INFO("tf_broadcaster initialize OK");
  }

  void bbox_CB(const darknet_dnn::BoundingBox& msg){
    if(this->is_get_point_cloud == false){
      ROS_WARN("Waiting PointCloud");
      return;
    }

    ////////////////////////////////////////
    // base_footprint基準の点群に座標変換する //
    ///////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ> cloud_transform;
    std::string base_frame_name = "base_footprint";

    //座標変換ができるかどうかを確認
    bool is_possible_transform = listener.canTransform(base_frame_name, camera_frame_name, ros::Time(0));
    if(is_possible_transform){
      pcl_ros::transformPointCloud(base_frame_name, ros::Time(0), cloud_local, camera_frame_name, cloud_transform, listener);
    }//座標変換成功

    else{
      ROS_WARN("Failure PointCloud Transform");
      cloud_transform = cloud_local; //元の座標系のままで処理を行う
      base_frame_name = camera_frame_name; //座標変換を行わないようにするためにフレーム名を変更する
    }//座標変換失敗

    pcl::PointXYZ object_pt;
    double shortest_distance = DBL_MAX;

    //BoundingBoxの中心付近のみの点群を用いるために、上下左右の端は無視する
    double point_thresh = 0.25;
    double point_lower_rate = 0.25;
    double point_upper_rate = 0.25;

    int shortest_distance_y = 0;

    for(int temp_y = msg.height*point_thresh; temp_y < msg.height*(1.0-point_thresh); temp_y++)
    {
      int object_y = msg.y + temp_y;
      for(int temp_x = 0; temp_x < msg.width; temp_x++)
      {
        int object_x = msg.width + temp_x;
        if(cloud_transform.points[cloud_transform.width*object_y+object_x].x < shortest_distance)
        {
          shortest_distance = cloud_transform.points[cloud_transform.width*object_y+object_x].x;
          object_pt = cloud_transform.points[cloud_transform.width*object_y+object_x];
          shortest_distance_y = cloud_transform.width*object_y;
        }
      }
    }

    //同じ高さの点の平均値を算出
    pcl::PointXYZ object_ave_pt;
    double temp_count = 0;
    for(int temp_x = msg.width*point_thresh; temp_x < msg.width*(1-point_thresh); temp_x++)
    {
      double object_x = msg.x + temp_x;
      if(std::isnan(cloud_transform.points[shortest_distance_y+object_x].x)
      || std::isnan(cloud_transform.points[shortest_distance_y+object_x].y)
      || std::isnan(cloud_transform.points[shortest_distance_y+object_x].z)) { continue; }
      object_ave_pt.x += cloud_transform.points[shortest_distance_y+object_x].x;
      object_ave_pt.y += cloud_transform.points[shortest_distance_y+object_x].y;
      object_ave_pt.z += cloud_transform.points[shortest_distance_y+object_x].z;
      temp_count += 1;
    }

    if(temp_count != 0){
      object_ave_pt.x /= temp_count;
      object_ave_pt.y /= temp_count;
      object_ave_pt.z /= temp_count;
    }

    if(std::isnan(object_ave_pt.x)==false && std::isnan(object_ave_pt.y)==false && std::isnan(object_ave_pt.z)==false){
      br.sendTransform(
        tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1),
        tf::Vector3(object_ave_pt.x,object_ave_pt.y,object_ave_pt.z)),
        ros::Time::now(),
        base_frame_name,
        "Target"));}

  }//bbox_CB

  void point_cloud_CB(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, cloud_local);

    if(cloud_local.points.size() == 0){
      ROS_ERROR("No PointCloud");
      this->is_get_point_cloud = false;
      return;
    }
    else{
      this->is_get_point_cloud = true;
      // /ROS_INFO("Get PointCloud");
    }
  }//point_cloud_CB
};//class TF_Broadcaster



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster_goturn");
  TF_Broadcaster tb;
  ros::spin();
  return 0;
}
