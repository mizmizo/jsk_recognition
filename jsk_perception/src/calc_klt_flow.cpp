//
// JSK calc_klt_flow
//
#include <ros/ros.h>
#include <ros/names.h>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <jsk_recognition_msgs/Flow2D.h>
#include <jsk_recognition_msgs/Flow2DArrayStamped.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdlib.h>

class calc_klt_flow_node {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub;
  ros::Publisher image_pub_;
  ros::Publisher result_pub_;
  std::string namespace_;
  cv::Mat prevImg;
  cv::Mat flow;
  cv::Point2f point;
  std::vector<cv::Point2f> points[2];
  int _maxCorners;
  double _qualityLevel;
  double _minDistance;
  int _blockSize;
  int _subPixWinSize;
  int _winSize;
  int _maxLevel;

public:
  calc_klt_flow_node () : nh_(), it_(nh_), flow(0, 0, CV_8UC3), prevImg(0, 0, CV_8UC1) {
    //flow = new cv::Mat(0, 0, CV_8UC1);
    namespace_ = nh_.resolveName("camera");
    camera_sub = it_.subscribeCamera(namespace_ + "/image_rect_color", 10, &calc_klt_flow_node::cameraCB, this);
    result_pub_ = nh_.advertise<jsk_recognition_msgs::Flow2DArrayStamped> ("flows_result", 1);
    image_pub_ = nh_.advertise<sensor_msgs::Image> ("flow_image", 1);
    nh_.param("maxCorners", _maxCorners, 200);
    nh_.param("qualityLevel", _qualityLevel, 0.05);
    nh_.param("minDistance", _minDistance, 5.0);
    nh_.param("blockSize", _blockSize, 3);
    nh_.param("subPixWinSize", _subPixWinSize, 15);
    nh_.param("winSize", _winSize, 20);
    nh_.param("maxLevel", _maxLevel, 5);
  }

  void cameraCB(const sensor_msgs::ImageConstPtr &img,
                const sensor_msgs::CameraInfoConstPtr &info) {
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr flow_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, "mono8");
    flow_ptr = cv_bridge::toCvCopy(img, "rgb8");
    bool prevImg_update_required = false;
    if((flow.cols != (int)img->width) ||
       (flow.rows != (int)img->height)) {
      JSK_ROS_INFO("make flow");
      flow_ptr->image.copyTo(flow);
      prevImg_update_required = true;
    }
    if(prevImg_update_required) {
      cv_ptr->image.copyTo(prevImg);
      goodFeaturesToTrack(prevImg, points[0], _maxCorners, _qualityLevel, _minDistance, cv::Mat());
      cv::cornerSubPix(prevImg, points[0], cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);
      prevImg_update_required = false;
      JSK_ROS_INFO("return");
      return;
    }
    //
    //JSK_ROS_INFO("subscribe image");
    //prevImg.
    //cv::Mat *nextImg = new cv::Mat(ipl_);
    cv::Mat nextImg(img->height, img->width, CV_8UC1);
    //memcpy(nextImg->data, ipl_->imageData, img->height*img->width)
    cv_ptr->image.copyTo(nextImg);
    flow_ptr->image.copyTo(flow);
    goodFeaturesToTrack(nextImg, points[1], _maxCorners, _qualityLevel, _minDistance, cv::Mat(), _blockSize);
    cv::cornerSubPix(nextImg, points[1], cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);
    std::vector<cv::Point2f> tmp_points = points[1];

    std::vector<uchar> features_found;
    //features_found.reserve(_maxCorners);
    std::vector<float> feature_errors;
    //feature_errors.reserve(_maxCorners);
    cv::calcOpticalFlowPyrLK(prevImg, nextImg, points[0], points[1], features_found, feature_errors, cv::Size(_winSize, _winSize),
                             _maxLevel, termcrit, 0, 0.001);
    //cv::OPTFLOW_USE_INITIAL_FLOW); 
    jsk_recognition_msgs::Flow2DArrayStamped flows_result_msg;
    flows_result_msg.header = img->header;

    
    
    size_t i;
    for(i = 0; i < points[1].size(); i++)
      {
        if(!features_found[i]) continue;
        cv::circle(flow, points[1][i], 5, cv::Scalar(255,0,0), 2, 8);
        cv::line(flow, points[1][i], points[0][i], cv::Scalar(255,0,0), 2, 8, 0);
        jsk_recognition_msgs::Flow2D flow_result;
        flow_result.point.x = points[1][i].x;
        flow_result.point.y = points[1][i].y;
        flow_result.velocity.x = points[1][i].x - points[0][i].x;
        flow_result.velocity.y = points[1][i].y - points[0][i].y ;
        flows_result_msg.flows.push_back(flow_result);
      }
    nextImg.copyTo(prevImg);
    std::swap(tmp_points, points[0]);
    flow.copyTo(flow_ptr->image);
    sensor_msgs::ImagePtr flow_image_msg = flow_ptr->toImageMsg();
    image_pub_.publish(flow_image_msg);
    result_pub_.publish(flows_result_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calc_klt_flow");
  //cv::namedWindow(std::string("window"), );
  calc_klt_flow_node klt_flow_node;

  ros::spin();
  return 0;
}
