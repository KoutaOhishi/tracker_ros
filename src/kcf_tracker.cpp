#include "ros/ros.h"
#include "iostream"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include "opencv2/opencv_modules.hpp"
#include <image_transport/image_transport.h>
#include "darknet_dnn/BoundingBox.h"


class KCFTracker{
private:
  ros::NodeHandle nh;

  std::string sub_img_name;
  std::string target_boundingBox_name;

  ros::Subscriber sub_img;
  ros::Subscriber sub_bbox;
  ros::Publisher pub_bbox;
  ros::Publisher pub_result;

  image_transport::Publisher pub_img;

  double target_x = 0.0;
  double target_y = 0.0;
  double target_width = 0.0;
  double target_height = 0.0;

  bool tracking_ctrl = false;
  bool tracking_bbox_update = false;

  cv::Ptr<cv::Tracker> tracker = cv::TrackerKCF::create();
  cv::Rect2d bbox;

public:
  KCFTracker(){
    //nh.param("sub_img_name", this->sub_img_name, std::string("/usb_cam/image_raw"));
    //nh.param("target_boundingBox_name", this->target_boundingBox_name, std::string("/update_bbox"));
    ros::param::get("sub_img_name", this->sub_img_name);
    ros::param::get("target_boundingBox_name", this->target_boundingBox_name);

    ROS_INFO("sub_img_name:[%s]", this->sub_img_name.c_str());
    ROS_INFO("target_boundingBox:[%s]", this->target_boundingBox_name.c_str());

    this->sub_img = nh.subscribe(this->sub_img_name, 100, &KCFTracker::sub_img_CB, this);
    this->sub_bbox = nh.subscribe(this->target_boundingBox_name, 10, &KCFTracker::sub_bbox_CB, this);
    this->pub_bbox = nh.advertise<darknet_dnn::BoundingBox>("/tracker_ros/kcf/result_bbox", 1);
    this->pub_result = nh.advertise<std_msgs::Bool>("/tracker_ros/kcf/result_flag", 1);

    image_transport::ImageTransport it(this->nh);
    this->pub_img = it.advertise("/tracker_ros/kcf/result_img", 1);
  }

  void sub_bbox_CB(const darknet_dnn::BoundingBox& msg)
  {
    this->target_x = (double)msg.x;
    this->target_y = (double)msg.y;
    this->target_width = (double)msg.width;
    this->target_height = (double)msg.height;

    this->bbox.x = (double)msg.x;
    this->bbox.y = (double)msg.y;
    this->bbox.width = (double)msg.width;
    this->bbox.height = (double)msg.height;

    this->tracking_ctrl = true;
    this->tracking_bbox_update = true;

  }//sub_bbox_CB

  bool bbox_init = false;
  void sub_img_CB(const sensor_msgs::ImageConstPtr& msg)
  {
    if(this->tracking_ctrl == true){
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      if(bbox_init==false){
        bbox_init = true;

        cv::rectangle(image, this->bbox, cv::Scalar(255,0,0), 2, 1);
        cv::imshow("Tracker_ROS KCF Result", image);

        this->tracker->init(image, this->bbox);
        ROS_INFO("### Setup Complete ###");
      }//最初のbboxの読み込み完了

      bool tracking_result = false;

      if (this->tracking_bbox_update  == true){
        this->tracker = cv::TrackerKCF::create();
        this->tracker->init(image, this->bbox);
        this->tracking_bbox_update=false;
        ROS_INFO("### Target BoundingBox has been updated ###");
        tracking_result = this->tracker->update(image, this->bbox);
      }

      tracking_result = this->tracker->update(image, this->bbox);

      if (tracking_result==true){
        cv::rectangle(image, this->bbox, cv::Scalar(0,0,255), 2, 1);

        cv::line(image, cv::Point(this->bbox.x,this->bbox.y),cv::Point(this->bbox.x+this->bbox.width,this->bbox.y+this->bbox.height), cv::Scalar(0,0,255), 2, 1);
        cv::line(image, cv::Point(this->bbox.x,this->bbox.y+this->bbox.height),cv::Point(this->bbox.x+this->bbox.width,this->bbox.y), cv::Scalar(0,0,255), 2, 1);

        darknet_dnn::BoundingBox result_bbox;
        result_bbox.x = this->bbox.x;
        result_bbox.y = this->bbox.y;
        result_bbox.width = this->bbox.width;
        result_bbox.height = this->bbox.height;
        this->pub_bbox.publish(result_bbox);

        std_msgs::Bool msg;
        msg.data = true;
        this->pub_result.publish(msg);

        //ROS_INFO("### Tracking Success ###");

      }

      else{
        ROS_WARN("### Tracking Failure ... ###");
        std_msgs::Bool msg;
        msg.data = false;
        this->pub_result.publish(msg);
      }

      //cv::imshow("KCF_ROS Result", image);
      //cv::waitKey(1);

      sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      this->pub_img.publish(img_msg);

    }
  }//sub_img_CB
};//KCFTracker


int main(int argc, char** argv){
  ros::init(argc, argv, "kcf");
  ROS_INFO("### KCF_ROS Start ###");

  KCFTracker t;

  ros::spin();
}
