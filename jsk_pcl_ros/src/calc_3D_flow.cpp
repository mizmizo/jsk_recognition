// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_pcl_ros/calc_3D_flow.h"
#include <ros/ros.h>
#include <ros/names.h>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/fill_image.h>
#include <jsk_recognition_msgs/Flow2D.h>
#include <jsk_recognition_msgs/Flow2DArrayStamped.h>
#include <jsk_recognition_msgs/Flow3D.h>
#include <jsk_recognition_msgs/Flow3DArrayStamped.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv.hpp>
#include <stdlib.h>
#include <math.h>


namespace jsk_pcl_ros
{
  void Calc3DFlow::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("maxCorners", _maxCorners, 100);
    pnh_->param("qualityLevel", _qualityLevel, 0.05);
    pnh_->param("minDistance", _minDistance, 5.0);
    pnh_->param("blockSize", _blockSize, 3);
    pnh_->param("subPixWinSize", _subPixWinSize, 15);
    pnh_->param("winSize", _winSize, 20);
    pnh_->param("maxLevel", _maxLevel, 5);
    pnh_->param("approximate_sync", approximate_sync_, true);
    result_pub_ = advertise<jsk_recognition_msgs::Flow3DArrayStamped>(
      *pnh_, "output/flow", 1);
    image_pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output/image", 1);

  }

  void Calc3DFlow::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_image_.subscribe(*pnh_, "input/image", 1);
    sub_info_.subscribe(*pnh_, "input/camera_info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_cloud_, sub_image_, sub_info_);
      async_->registerCallback(boost::bind(&Calc3DFlow::calc3Dflow,
                                           this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_cloud_, sub_image_, sub_info_);
      sync_->registerCallback(boost::bind(&Calc3DFlow::calc3Dflow,
                                          this, _1, _2, _3));
    }
  }

  void Calc3DFlow::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_info_.unsubscribe();
    sub_image_.unsubscribe();
  }
  
  pcl::PointXYZ Calc3DFlow::trimmedmean(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    cv::Point2f point)
  {
    std::vector<float> tmp_x,tmp_y,tmp_z;
    for(int i = -1; i < 2; i++)
      {
        for(int j = -1;j < 2; j++)
          {
            tmp_x.push_back(cloud->points[((int)point.y + i) * cloud->width + (int)point.x + j].x);
            tmp_y.push_back(cloud->points[((int)point.y + i) * cloud->width + (int)point.x + j].y);
            tmp_z.push_back(cloud->points[((int)point.y + i) * cloud->width + (int)point.x + j].z);
          }
      }
    std::sort(tmp_x.begin(), tmp_x.end());
    std::sort(tmp_y.begin(), tmp_y.end());
    std::sort(tmp_z.begin(), tmp_z.end());
    pcl::PointXYZ mean;
    mean.x = mean.y = mean.z = 0;
    for(int i = 2; i < 7; i++)
      {
        mean.x += tmp_x.at(i);
        mean.y += tmp_y.at(i);
        mean.z += tmp_z.at(i);
      }
    mean.x = mean.x / 5;
    mean.y = mean.y / 5;
    mean.z = mean.z / 5;
    return(mean);
  }

  void Calc3DFlow::calc3Dflow(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    if ((cloud_msg->header.frame_id != image_msg->header.frame_id) ||
        (cloud_msg->header.frame_id != info_msg->header.frame_id)) {
      JSK_NODELET_FATAL("frame_id is not collect: [%s, %s, %s",
                    cloud_msg->header.frame_id.c_str(),
                    image_msg->header.frame_id.c_str(),
                    info_msg->header.frame_id.c_str());
      return;
    }
    vital_checker_->poke();
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr flow_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, "mono8");
    flow_ptr = cv_bridge::toCvCopy(image_msg, "rgb8");
    bool prevImg_update_required = false;
    if((flow.cols != (int)image_msg->width) ||
       (flow.rows != (int)image_msg->height)) {
      JSK_ROS_INFO("make flow");
      flow_ptr->image.copyTo(flow);
      prevcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
      prevImg_update_required = true;
    }
    if(prevImg_update_required) {
      cv_ptr->image.copyTo(prevImg);
      goodFeaturesToTrack(prevImg, points[0], _maxCorners, _qualityLevel, _minDistance, cv::Mat());
      cv::cornerSubPix(prevImg, points[0], cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);
      pcl::fromROSMsg(*cloud_msg, *prevcloud);
      prevImg_update_required = false;
      JSK_ROS_INFO("return");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    cv::Mat nextImg(image_msg->height, image_msg->width, CV_8UC1);
    cv_ptr->image.copyTo(nextImg);
    flow_ptr->image.copyTo(flow);
    goodFeaturesToTrack(nextImg, points[1], _maxCorners, _qualityLevel, _minDistance, cv::Mat(), _blockSize);
    cv::cornerSubPix(nextImg, points[1], cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);
    std::vector<cv::Point2f> tmp_points = points[1];

    std::vector<uchar> features_found;
    std::vector<float> feature_errors;
    cv::calcOpticalFlowPyrLK(prevImg, nextImg, points[0], points[1], features_found, feature_errors, cv::Size(_winSize, _winSize),
                             _maxLevel, termcrit, 0, 0.001);
    //cv::OPTFLOW_USE_INITIAL_FLOW); 
    jsk_recognition_msgs::Flow3DArrayStamped flows_result_msg;
    flows_result_msg.header = image_msg->header;

    size_t i;
    for(i = 0; i < features_found.size(); i++)
      {
        if(!features_found[i]
           || points[0][i].x >= prevcloud->width
           || points[0][i].y >= prevcloud->height
           || points[1][i].x >= cloud->width
           || points[1][i].y >= cloud->height
           || points[0][i].x < 1
           || points[0][i].y < 1
           || points[1][i].x < 1
           || points[1][i].y < 1 )
          continue;
        // pcl::PointXYZ prevp = prevcloud->points[points[0][i].y * prevcloud->width + points[0][i].x];
        // pcl::PointXYZ nextp = cloud->points[points[1][i].y * cloud->width + points[1][i].x];
        pcl::PointXYZ prevp = trimmedmean(prevcloud, points[0][i]);
        pcl::PointXYZ nextp = trimmedmean(cloud, points[1][i]);
        if(isnan(nextp.x)
           || isnan(nextp.y)
           || isnan(nextp.z)
           || isnan(prevp.x)
           || isnan(prevp.y)
           || isnan(prevp.z)) continue;
        cv::circle(flow, points[1][i], 5, cv::Scalar(255,0,0), 2, 8);
        cv::line(flow, points[1][i], points[0][i], cv::Scalar(255,0,0), 2, 8, 0);
        jsk_recognition_msgs::Flow3D flow_result;

        flow_result.point.x = nextp.x;
        flow_result.point.y = nextp.y;
        flow_result.point.z = nextp.z;
        flow_result.velocity.x = nextp.x - prevp.x;
        flow_result.velocity.y = nextp.y - prevp.y;
        flow_result.velocity.z = nextp.z - prevp.z;
        flows_result_msg.flows.push_back(flow_result);
        std::cout << "2D flow : " << (int)points[1][i].x - (int)points[0][i].x << " " << (int)points[1][i].y - (int)points[0][i].y << std::endl;
        std::cout << "3D flow : " << flow_result.velocity.x << " " << flow_result.velocity.y << " " << flow_result.velocity.z << std::endl;
      }
    nextImg.copyTo(prevImg);
    std::swap(tmp_points, points[0]);
    pcl::copyPointCloud(*cloud, *prevcloud);
    flow.copyTo(flow_ptr->image);
    sensor_msgs::ImagePtr flow_image_msg = flow_ptr->toImageMsg();
    image_pub_.publish(flow_image_msg);
    result_pub_.publish(flows_result_msg);
    std::cout << std::endl;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::Calc3DFlow, nodelet::Nodelet);
