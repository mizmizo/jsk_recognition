// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *  POSSIBILITY OF SUCH ifndef.
 *********************************************************************/

#ifndef JSK_PCL_ROS_FLOW_TRACKING_H_
#define JSK_PCL_ROS_FLOW_TRACKING_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <cv_bridge/cv_bridge.h>
#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <tf/transform_broadcaster.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/Flow3DArrayStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "jsk_pcl_ros/pcl_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_topic_tools/diagnostic_nodelet.h"
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace jsk_pcl_ros
{
  class FlowTracking: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::Image
    >  SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::Image
    >  ApproximateSyncPolicy;
    FlowTracking(): DiagnosticNodelet("FlowTracking") { }
    virtual void onInit();
    virtual pcl::PointXYZ trimmedmean(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      cv::Point2i point);
    virtual bool comparebox(const jsk_recognition_msgs::BoundingBox& input_box,
                            uint* label);
    virtual std::vector<cv::Point3d> getVertices(const jsk_recognition_msgs::BoundingBox& box);
    virtual bool comparevertices(
      const cv::Point3d& vertices,
      const jsk_recognition_msgs::BoundingBox& compared_box);
    virtual Eigen::MatrixXf pseudoinverse(const Eigen::MatrixXf& m);
    virtual void box_extract(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &box);
    virtual void flow_extract(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::Image::ConstPtr& image_msg);
  protected:
    virtual void subscribe();
    virtual void unsubscribe();
    virtual bool initServiceCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >async_;
    ros::Subscriber sub_box_;
    boost::mutex mutex_;
    ros::Publisher box_pub_;
    ros::Publisher image_pub_;
    ros::Publisher result_pub_;
    ros::Publisher vis_pub_;
    ros::ServiceServer init_srv_;
    bool approximate_sync_;
    bool publish_marker_;
    bool tracking_mode_;
    bool check_deformation_;
    int _maxCorners;
    double _qualityLevel;
    double _minDistance;
    int _blockSize;
    int _subPixWinSize;
    int _winSize;
    int _maxLevel;
    cv::Mat prevImg;
    cv::Mat flow;
    bool need_to_flow_init;
    std::vector<cv::Point2f> points[2];
    pcl::PointCloud<pcl::PointXYZ>::Ptr prevcloud;
    std::vector<jsk_recognition_msgs::BoundingBox> labeled_boxes;
    std::vector<jsk_recognition_msgs::BoundingBox> copy_labeled_boxes;
    //std::vector<std::vector <Eigen::Quaternionf> > flow_positions;
    std::vector<Eigen::MatrixXf> flow_positions;
    std::vector<uint> flow_labels;
    bool need_to_label_init;
  };

}

#endif
