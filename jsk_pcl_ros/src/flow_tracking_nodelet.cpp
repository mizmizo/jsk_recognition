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
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "jsk_pcl_ros/flow_tracking.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <boost/format.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/irange.hpp>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_topic_tools/color_utils.h>
#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include "jsk_pcl_ros/pcl_util.h"

namespace jsk_pcl_ros
{
  void FlowTracking::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("maxCorners", _maxCorners, 100);
    pnh_->param("qualityLevel", _qualityLevel, 0.05);
    pnh_->param("minDistance", _minDistance, 5.0);
    pnh_->param("blockSize", _blockSize, 3);
    pnh_->param("subPixWinSize", _subPixWinSize, 15);
    pnh_->param("winSize", _winSize, 20);
    pnh_->param("maxLevel", _maxLevel, 5);
    pnh_->param("tracking_mode", tracking_mode_, true);
    pnh_->param("approximate_sync", approximate_sync_, true);
    pnh_->param("publish_marker", publish_marker_, true);
    box_pub_ = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "output/boxes", 1);
    result_pub_ = advertise<jsk_recognition_msgs::Flow3DArrayStamped>(
      *pnh_, "output/flow", 1);
    image_pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output/image", 1);
    if(publish_marker_)
      vis_pub_ = advertise<visualization_msgs::Marker>(*pnh_, "output/visualized_flow", 1);
    if(tracking_mode_){
      init_srv_ = pnh_->advertiseService("initialize", &FlowTracking::initServiceCallback, this);
    }
    need_to_label_init = true;
    onInitPostProcess();

  }
  
  void FlowTracking::subscribe()
  {
    sub_box_ = pnh_->subscribe("input/box", 1, &FlowTracking::box_extract, this);
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_image_.subscribe(*pnh_, "input/image", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_cloud_, sub_image_);
      async_->registerCallback(boost::bind(&FlowTracking::flow_extract,
                                           this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_cloud_, sub_image_);
      sync_->registerCallback(boost::bind(&FlowTracking::flow_extract,
                                          this, _1, _2));
    }
  }
  
  void FlowTracking::unsubscribe()
  {
    sub_box_.shutdown();
    sub_cloud_.unsubscribe();
    sub_image_.unsubscribe();
  }
  
  pcl::PointXYZ FlowTracking::trimmedmean(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          cv::Point2i point)
  {
    std::vector<float> tmp_x,tmp_y,tmp_z;
    for(int i = -1; i < 2; i++)
      {
        for(int j = -1;j < 2; j++)
          {
            tmp_x.push_back(cloud->points[(point.y + i) * cloud->width + point.x + j].x);
            tmp_y.push_back(cloud->points[(point.y + i) * cloud->width + point.x + j].y);
            tmp_z.push_back(cloud->points[(point.y + i) * cloud->width + point.x + j].z);
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

  bool FlowTracking::comparebox(const jsk_recognition_msgs::BoundingBox& input_box,
                                      uint* label)
  {
    size_t i,j;
    bool box_fit = false;
    std::vector<cv::Point3d> input_vertices = getVertices(input_box);
    for(i = 0; i < copy_labeled_boxes.size(); i++){
      std::vector<cv::Point3d> compared_vertices = getVertices(copy_labeled_boxes.at(i));
      bool fit_candidate(false);
      for(j = 0; j < 8; j++){
        if(comparevertices(input_vertices.at(j), copy_labeled_boxes.at(i))
           || comparevertices(compared_vertices.at(j), input_box)){
          fit_candidate = true;
          break;
        }
      }
      if(fit_candidate){
        if(!box_fit){
          *label = copy_labeled_boxes.at(i).label;
            box_fit = true;
        } else {
          box_fit = false;
          break;
        }
      }
    }
    if(box_fit){
      return true;
    } else {
      return false;
        }
  }

  std::vector<cv::Point3d> FlowTracking::getVertices(const jsk_recognition_msgs::BoundingBox& box)
  {
    Eigen::Affine3f pose;
    tf::poseMsgToEigen(box.pose, pose);
    Eigen::Vector3f local_a(box.dimensions.x / 2, box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_b(-box.dimensions.x / 2, box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_c(-box.dimensions.x / 2, -box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_d(box.dimensions.x / 2, -box.dimensions.y / 2, box.dimensions.z / 2);
    Eigen::Vector3f local_e(box.dimensions.x / 2, box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f local_f(-box.dimensions.x / 2, box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f local_g(-box.dimensions.x / 2, -box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f local_h(box.dimensions.x / 2, -box.dimensions.y / 2, -box.dimensions.z / 2);
    Eigen::Vector3f a = pose * local_a;
    Eigen::Vector3f b = pose * local_b;
    Eigen::Vector3f c = pose * local_c;
    Eigen::Vector3f d = pose * local_d;
    Eigen::Vector3f e = pose * local_e;
    Eigen::Vector3f f = pose * local_f;
    Eigen::Vector3f g = pose * local_g;
    Eigen::Vector3f h = pose * local_h;
    cv::Point3d cv_a(a[0], a[1], a[2]);
    cv::Point3d cv_b(b[0], b[1], b[2]);
    cv::Point3d cv_c(c[0], c[1], c[2]);
    cv::Point3d cv_d(d[0], d[1], d[2]);
    cv::Point3d cv_e(e[0], e[1], e[2]);
    cv::Point3d cv_f(f[0], f[1], f[2]);
    cv::Point3d cv_g(g[0], g[1], g[2]);
    cv::Point3d cv_h(h[0], h[1], h[2]);
    std::vector<cv::Point3d> ret;
    ret.push_back(cv_a);
    ret.push_back(cv_b);
    ret.push_back(cv_c);
    ret.push_back(cv_d);
    ret.push_back(cv_e);
    ret.push_back(cv_f);
    ret.push_back(cv_g);
    ret.push_back(cv_h);
    return ret;
  }

  bool FlowTracking::comparevertices(const cv::Point3d& vertice,
                                         const jsk_recognition_msgs::BoundingBox& compared_box)
  {
    std::vector<cv::Point3d> compared_vertices = getVertices(compared_box);
    Eigen::Vector3f v_target((vertice.x*100 - compared_vertices.at(0).x*100),
                             (vertice.y*100 - compared_vertices.at(0).y*100),
                             (vertice.z*100 - compared_vertices.at(0).z*100));
    Eigen::Vector3f v_x((compared_vertices.at(1).x*100 - compared_vertices.at(0).x*100),
                        (compared_vertices.at(1).y*100 - compared_vertices.at(0).y*100),
                        (compared_vertices.at(1).z*100 - compared_vertices.at(0).z*100));
    Eigen::Vector3f v_y((compared_vertices.at(3).x*100 - compared_vertices.at(0).x*100),
                        (compared_vertices.at(3).y*100 - compared_vertices.at(0).y*100),
                        (compared_vertices.at(3).z*100 - compared_vertices.at(0).z*100));
    Eigen::Vector3f v_z((compared_vertices.at(4).x*100 - compared_vertices.at(0).x*100),
                        (compared_vertices.at(4).y*100 - compared_vertices.at(0).y*100),
                        (compared_vertices.at(4).z*100 - compared_vertices.at(0).z*100));

      if((v_target.dot(v_x) / (v_x.norm() * v_x.norm())) > -0.1 &&
       (v_target.dot(v_x) / (v_x.norm() * v_x.norm())) < 1.1 &&
       (v_target.dot(v_y) / (v_y.norm() * v_y.norm())) > -0.1 &&
       (v_target.dot(v_y) / (v_y.norm() * v_y.norm())) < 1.1 &&
       (v_target.dot(v_z) / (v_z.norm() * v_z.norm())) > -0.1 &&
       (v_target.dot(v_z) / (v_z.norm() * v_z.norm())) < 1.1)
      {
        return true;
      } else {
      return false;
    }
  }

  Eigen::MatrixXf FlowTracking::pseudoinverse(const Eigen::MatrixXf& m)
  {
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXf sigma(svd.singularValues());
    Eigen::VectorXf sigma_inv;
    sigma_inv.resize(sigma.size());
    size_t i;
    double tolerance = 1.0e-6;
    for(long i=0; i<sigma.size(); i++)
      {
        if(sigma(i) > tolerance)
          sigma_inv(i)= 1.0/sigma(i);
        else
          sigma_inv(i)= 0.0;
      }
    return svd.matrixV() * sigma_inv.asDiagonal() * svd.matrixU().transpose();
  }

  bool FlowTracking::initServiceCallback(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    need_to_flow_init = true;
    need_to_label_init = true;
    return true;
  }

  void FlowTracking::box_extract(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& box)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    size_t i;
    if(box->boxes.size() > 0){
      if(labeled_boxes.size() > 0)
        {
          size_t j,k,l,m,count;
          j = k = l = 0;
          uint max_label = labeled_boxes.at(labeled_boxes.size() - 1).label;
          size_t boxes_size = labeled_boxes.size();
          std::vector<jsk_recognition_msgs::BoundingBox> tmp_boxes;
          tmp_boxes.resize(box->boxes.size() + boxes_size);
          bool is_translated = false;
          for(i = 0; i < labeled_boxes.size(); i++){
            Eigen::Quaternionf prev_q(copy_labeled_boxes.at(i).pose.orientation.w, copy_labeled_boxes.at(i).pose.orientation.x, copy_labeled_boxes.at(i).pose.orientation.y, copy_labeled_boxes.at(i).pose.orientation.z);
            Eigen::Quaternionf next_q(labeled_boxes.at(i).pose.orientation.w, labeled_boxes.at(i).pose.orientation.x, labeled_boxes.at(i).pose.orientation.y, labeled_boxes.at(i).pose.orientation.z);
            if(fabs(labeled_boxes.at(i).pose.position.x - copy_labeled_boxes.at(i).pose.position.x) > 0.005 ||
               fabs(labeled_boxes.at(i).pose.position.y - copy_labeled_boxes.at(i).pose.position.y) > 0.005 ||
               fabs(labeled_boxes.at(i).pose.position.z - copy_labeled_boxes.at(i).pose.position.z) > 0.005 ||
               fabs(prev_q.w() - next_q.w()) > 0.01 ||
               fabs(prev_q.w() - next_q.x()) > 0.01 ||
               fabs(prev_q.w() - next_q.y()) > 0.01 ||
               fabs(prev_q.w() - next_q.z()) > 0.01){
              is_translated = true;
              break;
            }
          }
          if(!is_translated){
            need_to_flow_init = true;
            need_to_label_init = true;
            for(i = 0; i < box->boxes.size(); i++){
              jsk_recognition_msgs::BoundingBox input_box;
              input_box = box->boxes[i];
              uint label = max_label + 1;

              if(comparebox(input_box, &label)){ //fit-box label
                input_box.label = label;

                if (l == 0){
                  tmp_boxes.at(l) = input_box;
                  l++;
                } else {
                  for(m = 0; m < l; m++){
                    if(tmp_boxes.at(m).label == label){ //label split
                      tmp_boxes.at(boxes_size + j) = input_box;
                      tmp_boxes.at(boxes_size + j).label = max_label + j + 1;
                      j++;
                      need_to_label_init = true;
                      break;
                    } else if(m == l - 1){ //update label
                      tmp_boxes.at(l) = input_box;
                      l++;
                      break;
                    }
                  }
                }
              } else if(label > max_label){
                tmp_boxes.at(boxes_size + j) = input_box; //new label
                tmp_boxes.at(boxes_size + j).label = max_label + j + 1;
                j++;
                need_to_label_init = true;
              }
            }
            
            for(i = 0; i < boxes_size; i++){ //update boundingbox
              for(m = 0; m < l; m++){
                if(tmp_boxes.at(m).label == labeled_boxes.at(i).label){
                  labeled_boxes.at(i) = tmp_boxes.at(m);
                  break;
                }
              }
            }
            labeled_boxes.resize(boxes_size + j);
            for(i = 0; i < j; i++){
              labeled_boxes.at(boxes_size + i) = tmp_boxes.at(boxes_size + i);
            }
          }
        } else {
        for(i = 0; i < box->boxes.size(); i++){
          jsk_recognition_msgs::BoundingBox tmp_box;
          tmp_box = box->boxes[i];
          tmp_box.label = i;
          labeled_boxes.push_back(tmp_box);
        }
        need_to_flow_init = true;
        need_to_label_init = true;
      }
    }
    
    copy_labeled_boxes = labeled_boxes;

    jsk_recognition_msgs::BoundingBoxArray box_msg;
    box_msg.header = box->header;
    for(i = 0; i < labeled_boxes.size(); i++){
      labeled_boxes.at(i).header = box->header;
      box_msg.boxes.push_back(labeled_boxes.at(i));
    }
    box_pub_.publish(box_msg);

  }

  void FlowTracking::flow_extract(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(labeled_boxes.size() == 0)return;

    /* === calc flow === */
    if (cloud_msg->header.frame_id != image_msg->header.frame_id){
      JSK_NODELET_FATAL("frame_id is not collect: [%s, %s",
                        cloud_msg->header.frame_id.c_str(),
                        image_msg->header.frame_id.c_str());
      return;
    }
    vital_checker_->poke();
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr flow_ptr;
    std::vector<cv::Point2f> new_points[2];
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
      pcl::fromROSMsg(*cloud_msg, *prevcloud);
      prevImg_update_required = false;
      need_to_flow_init = true;
      JSK_ROS_INFO("return");
      return;
    }
    if(!need_to_flow_init){
      if(points[0].size() < 4){
        need_to_flow_init = true;
        need_to_label_init = true;
      }
    }
    if(need_to_flow_init){
      goodFeaturesToTrack(prevImg, points[0], _maxCorners, _qualityLevel, _minDistance, cv::Mat());
      cv::cornerSubPix(prevImg, points[0], cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);
      size_t i,j,k;
      k = 0;
      for(i = 0; i < points[0].size(); i++){
        bool overlap = false;
        cv::Point2f point = points[0].at(i);

        for(j = 0; j < k; j++){
          if(fabs(point.x - points[0][j].x) < 2.0 &&
             fabs(point.y - points[0][j].y) < 2.0){
            overlap = true;
          }
        }
        if(!overlap){
          points[0][k++] = points[0][i];
        }
      }
      points[0].resize(k);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    cv::Mat nextImg(image_msg->height, image_msg->width, CV_8UC1);
    cv_ptr->image.copyTo(nextImg);
    flow_ptr->image.copyTo(flow);
    //for not tracking_mode
    std::vector<cv::Point2f> tmp_points;
    goodFeaturesToTrack(nextImg, tmp_points, _maxCorners, _qualityLevel, _minDistance, cv::Mat(), _blockSize);
    cv::cornerSubPix(nextImg, tmp_points, cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);

    std::vector<uchar> features_found;
    std::vector<float> feature_errors;
    cv::calcOpticalFlowPyrLK(prevImg, nextImg, points[0], points[1], features_found, feature_errors, cv::Size(_winSize, _winSize),
                             _maxLevel, termcrit, 0, 0.001);

    //calc back flow
    std::vector<uchar> back_features_found;
    std::vector<float> back_feature_errors;
    std::vector<cv::Point2f> back_points;
    cv::calcOpticalFlowPyrLK(nextImg, prevImg, points[1], back_points, back_features_found, back_feature_errors,
                             cv::Size(_winSize, _winSize), _maxLevel, termcrit, 0, 0.001);


    //calc new flow
    std::vector<uchar> new_features_found;
    std::vector<float> new_feature_errors;
    std::vector<uchar> new_back_features_found;
    std::vector<float> new_back_feature_errors;
    std::vector<cv::Point2f> new_back_points;
    if(!need_to_flow_init){
      goodFeaturesToTrack(prevImg, new_points[0], _maxCorners, _qualityLevel, _minDistance, cv::Mat());
      cv::cornerSubPix(prevImg, new_points[0], cv::Size(_subPixWinSize, _subPixWinSize), cv::Size(-1,-1), termcrit);
      cv::calcOpticalFlowPyrLK(prevImg, nextImg, new_points[0], new_points[1], new_features_found, new_feature_errors, cv::Size(_winSize, _winSize),
                               _maxLevel, termcrit, 0, 0.001);
      cv::calcOpticalFlowPyrLK(nextImg, prevImg, new_points[1], new_back_points, new_back_features_found, new_back_feature_errors,
                               cv::Size(_winSize, _winSize), _maxLevel, termcrit, 0, 0.001);
    }

    jsk_recognition_msgs::Flow3DArrayStamped flows_result_msg;
    flows_result_msg.header = image_msg->header;
    visualization_msgs::Marker marker;
    if(publish_marker_){
      marker.header = image_msg->header;
      marker.ns = "visualized_flow";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1.0;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.scale.x = 0.02;
    }
    //prepare flow labelling
    std::vector<std::vector<jsk_recognition_msgs::Flow3D> > checked_flows;
    std::vector<int> flow_label_count(labeled_boxes.size(),0);
    uint max_label = labeled_boxes.at(labeled_boxes.size() - 1).label;
    size_t i, j, k, l, m;
    uint flow_label; // not same as BoundingBox.label
    if(need_to_label_init){
      flow_labels.resize(features_found.size());
    }
    checked_flows.resize(labeled_boxes.size());
    for(i = 0; i < labeled_boxes.size(); i++){
      checked_flows.at(i).resize(features_found.size());
    }

    j = 0;
    for(i = 0; i < features_found.size(); i++)
      {
        cv::Point2i tmp_prevp = points[0][i];
        cv::Point2i tmp_nextp = points[1][i];
        if(!features_found[i]
           ||!back_features_found[i]
           || tmp_prevp.x >= prevcloud->width - 1
           || tmp_prevp.y >= prevcloud->height - 1
           || tmp_nextp.x >= cloud->width - 1
           || tmp_nextp.y >= cloud->height - 1
           || tmp_prevp.x < 2
           || tmp_prevp.y < 2
           || tmp_nextp.x < 2
           || tmp_nextp.y < 2 ){
          if(!need_to_label_init){
            //remove label
            flow_labels.erase(flow_labels.begin() + j);
          }
          continue;
        }
        //back flow check
        double theta = ((points[1][i].x - points[0][i].x) * (points[1][i].x - back_points[i].x)
                        + (points[1][i].y - points[0][i].y) * (points[1][i].y - back_points[i].y))
          / (sqrt((points[1][i].x - points[0][i].x) * (points[1][i].x - points[0][i].x)
                  + (points[1][i].y - points[0][i].y) * (points[1][i].y - points[0][i].y))
             * sqrt((points[1][i].x - back_points[i].x) * (points[1][i].x - back_points[i].x)
                    + (points[1][i].y - back_points[i].y) * (points[1][i].y - back_points[i].y)));
        if(theta < 0.7){
          if(!need_to_label_init){
            //remove label
            flow_labels.erase(flow_labels.begin() + j);
          }
          continue;
        }
        pcl::PointXYZ prevp = trimmedmean(prevcloud, tmp_prevp);
        pcl::PointXYZ nextp = trimmedmean(cloud, tmp_nextp);
        if(isnan(nextp.x)
           || isnan(nextp.y)
           || isnan(nextp.z)
           || isnan(prevp.x)
           || isnan(prevp.y)
           || isnan(prevp.z)){
          if(!need_to_label_init){
            //remove label
            flow_labels.erase(flow_labels.begin() + j);
          }
          continue;
        }
        jsk_recognition_msgs::Flow3D flow_result;
        flow_result.point.x = nextp.x;
        flow_result.point.y = nextp.y;
        flow_result.point.z = nextp.z;
        flow_result.velocity.x = nextp.x - prevp.x;
        flow_result.velocity.y = nextp.y - prevp.y;
        flow_result.velocity.z = nextp.z - prevp.z;

        if(fabs(flow_result.velocity.x) > 0.5
           ||fabs(flow_result.velocity.y) > 0.5
           ||fabs(flow_result.velocity.z) > 0.5){
          if(!need_to_label_init){
            //remove label
            flow_labels.erase(flow_labels.begin() + j);
          }
          continue;
        }
        if(need_to_label_init){
          //new labelling
          cv::Point3d point(flow_result.point.x - flow_result.velocity.x,
                            flow_result.point.y - flow_result.velocity.y,
                            flow_result.point.z - flow_result.velocity.z);
          k = 0;
            for(l = 0; l < labeled_boxes.size(); l++){
              if(comparevertices(point, labeled_boxes.at(l))){
                k++;
                flow_label = l;
              }
            }
            if(k == 1){
              checked_flows.at(flow_label).at(flow_label_count.at(flow_label)) = flow_result;
              flow_label_count.at(flow_label)++;
              flow_labels.at(j) = labeled_boxes.at(flow_label).label;
            } else {
              continue;
            }
        } else {
          //apply label
          if(flow_labels.at(j) <= max_label){
            for(l = 0; l < labeled_boxes.size(); l++){
              if(flow_labels.at(j) == labeled_boxes.at(l).label){
                flow_label = l;
                break;
              }
            }
            Eigen::Vector4f tmp_point(flow_result.point.x, flow_result.point.y, flow_result.point.z, 1.0);
            checked_flows.at(flow_label).at(flow_label_count.at(flow_label)) = flow_result;
            flow_label_count.at(flow_label)++;
          } else {
            //remove label
            flow_labels.erase(flow_labels.begin() + j);
            continue;
          }
        }
        if(flow_labels.at(j) % 3 == 0){
          cv::circle(flow, points[1][i], 5, cv::Scalar(255,0,0), 2, 8);
          cv::line(flow, points[1][i], points[0][i], cv::Scalar(255,0,0), 2, 8, 0);
        } else if(flow_labels.at(j) % 3 == 1){
          cv::circle(flow, points[1][i], 5, cv::Scalar(0,255,0), 2, 8);
          cv::line(flow, points[1][i], points[0][i], cv::Scalar(0,255,0), 2, 8, 0);
        } else if(flow_labels.at(j) % 3 == 2){
          cv::circle(flow, points[1][i], 5, cv::Scalar(0,0,255), 2, 8);
          cv::line(flow, points[1][i], points[0][i], cv::Scalar(0,0,255), 2, 8, 0);
        }
        points[1][j++] = points[1][i];
        flows_result_msg.flows.push_back(flow_result);

        if(publish_marker_){
          geometry_msgs::Point start_point;
          start_point.x = flow_result.point.x - flow_result.velocity.x * 5;
          start_point.y = flow_result.point.y - flow_result.velocity.y * 5;
          start_point.z = flow_result.point.z - flow_result.velocity.z * 5;
          marker.points.push_back(start_point);
          marker.points.push_back(flow_result.point);
        }
      }
    need_to_label_init = false;
    flow_labels.resize(j);
    for(i = 0; i < labeled_boxes.size(); i++){
      checked_flows.at(i).resize(flow_label_count.at(i));
    }


    //labelling new flow
    if(!need_to_flow_init){
      int old_count(j);
      flow_labels.resize(j + new_features_found.size());
      points[1].resize(j + new_features_found.size());
      for(i = 0; i < labeled_boxes.size(); i++){
        checked_flows.at(i).resize(flow_label_count.at(i) + new_features_found.size());
      }

      for(i = 0; i < new_features_found.size(); i++){
        cv::Point2i tmp_prevp = new_points[0][i];
        cv::Point2i tmp_nextp = new_points[1][i];
        if(!new_features_found[i]
           || !new_back_features_found[i]
           || tmp_prevp.x >= prevcloud->width - 1
           || tmp_prevp.y >= prevcloud->height - 1
           || tmp_nextp.x >= cloud->width - 1
           || tmp_nextp.y >= cloud->height - 1
           || tmp_prevp.x < 2
           || tmp_prevp.y < 2
           || tmp_nextp.x < 2
           || tmp_nextp.y < 2 )
          continue;
        bool overlap = false;
        for(l = 0; l < old_count; l++){
          if(fabs(new_points[1][i].x - points[1][l].x) < 2.0 &&
             fabs(new_points[1][i].y - points[1][l].y) < 2.0){
            overlap = true;
            break;
          }
        }
        if(overlap)continue;

        double theta = ((new_points[1][i].x - new_points[0][i].x) * (new_points[1][i].x - new_back_points[i].x)
                        + (new_points[1][i].y - new_points[0][i].y) * (new_points[1][i].y - new_back_points[i].y))
          / (sqrt((new_points[1][i].x - new_points[0][i].x) * (new_points[1][i].x - new_points[0][i].x)
                  + (new_points[1][i].y - new_points[0][i].y) * (new_points[1][i].y - new_points[0][i].y))
             * sqrt((new_points[1][i].x - new_back_points[i].x) * (new_points[1][i].x - new_back_points[i].x)
                    + (new_points[1][i].y - new_back_points[i].y) * (new_points[1][i].y - new_back_points[i].y)));
        if(theta < 0.7)
          continue;
        pcl::PointXYZ prevp = trimmedmean(prevcloud, tmp_prevp);
        pcl::PointXYZ nextp = trimmedmean(cloud, tmp_nextp);
        if(isnan(nextp.x)
           || isnan(nextp.y)
           || isnan(nextp.z)
           || isnan(prevp.x)
           || isnan(prevp.y)
           || isnan(prevp.z))
          continue;
        jsk_recognition_msgs::Flow3D flow_result;
        flow_result.point.x = nextp.x;
        flow_result.point.y = nextp.y;
        flow_result.point.z = nextp.z;
        flow_result.velocity.x = nextp.x - prevp.x;
        flow_result.velocity.y = nextp.y - prevp.y;
        flow_result.velocity.z = nextp.z - prevp.z;
        if(fabs(flow_result.velocity.x) > 0.5
           ||fabs(flow_result.velocity.y) > 0.5
           ||fabs(flow_result.velocity.z) > 0.5)
          continue;


        //labelling
        cv::Point3d point(flow_result.point.x - flow_result.velocity.x,
                          flow_result.point.y - flow_result.velocity.y,
                          flow_result.point.z - flow_result.velocity.z);
        k = 0;
        for(l = 0; l < labeled_boxes.size(); l++){
          if(comparevertices(point, labeled_boxes.at(l))){
            k++;
            flow_label = l;
          }
        }
        if(k == 1 && checked_flows.at(flow_label).size() > 3){
          checked_flows.at(flow_label).at(flow_label_count.at(flow_label)) = flow_result;
          flow_labels.at(j) = labeled_boxes.at(flow_label).label;
          flow_label_count.at(flow_label)++;
          points[1][j++] = new_points[1][i];
        }
      }
      flow_labels.resize(j);
      for(i = 0; i < labeled_boxes.size(); i++){
        checked_flows.at(i).resize(flow_label_count.at(i));
      }
    }
    need_to_flow_init = false;

    if(tracking_mode_){
      points[1].resize(j);
      std::swap(points[1],points[0]);
    } else {
      std::swap(tmp_points, points[0]);
    }
    nextImg.copyTo(prevImg);
    pcl::copyPointCloud(*cloud, *prevcloud);
    flow.copyTo(flow_ptr->image);
    sensor_msgs::ImagePtr flow_image_msg = flow_ptr->toImageMsg();

    /* === box update === */

    //calc translation and rotation
    for(i = 0; i < checked_flows.size(); i++){
      if(checked_flows.at(i).size() > 3){
        //RANSAC param
        int Rloop = 100;
        double Rthre = 0.007;
        int Rcnt = checked_flows.at(i).size() * 0.6;

        std::vector<Eigen::MatrixXf> good_matrixes;
        std::vector<Eigen::MatrixXf> good_gs;
        std::vector<double> good_errors;
        srand((unsigned int)time(NULL));

        for(j = 0; j < Rloop; j++){
          Eigen::MatrixXf picked_flow;
          Eigen::MatrixXf picked_pre_pos;
          Eigen::MatrixXf pre_g;
          picked_flow.resize(4, 4);
          picked_pre_pos.resize(4, 4);
          pre_g = Eigen::MatrixXf::Zero(4,1);
          for(l = 0; l < 4; l++){
            int itr = rand() % checked_flows.at(i).size();
            picked_flow(0, l) = checked_flows.at(i).at(itr).velocity.x;
            picked_flow(1, l) = checked_flows.at(i).at(itr).velocity.y;
            picked_flow(2, l) = checked_flows.at(i).at(itr).velocity.z;
            picked_flow(3, l) = 1.0;
            picked_pre_pos(0, l) = checked_flows.at(i).at(itr).point.x - checked_flows.at(i).at(itr).velocity.x;
            picked_pre_pos(1, l) = checked_flows.at(i).at(itr).point.y - checked_flows.at(i).at(itr).velocity.y;
            picked_pre_pos(2, l) = checked_flows.at(i).at(itr).point.z - checked_flows.at(i).at(itr).velocity.z;
            picked_pre_pos(3, l) = 1.0;
            pre_g(0, 0) += picked_pre_pos(0, l) / 4;
            pre_g(1, 0) += picked_pre_pos(1, l) / 4;
            pre_g(2, 0) += picked_pre_pos(2, l) / 4;
          }

          for(l = 0; l < 4; l++){
            picked_pre_pos.col(l) -= pre_g;
          }

          int inlier = 0;
          double error_sum = 0;
          Eigen::MatrixXf tmp_matrix = picked_flow * pseudoinverse(picked_pre_pos);
          if(fabs((tmp_matrix.block(0, 0, 3, 3) + Eigen::Matrix3f::Identity()).determinant() - 1.0) > 0.2)continue;
          for(l = 0; l < checked_flows.at(i).size(); l++){
            Eigen::Vector4f pre_pos(checked_flows.at(i).at(l).point.x - checked_flows.at(i).at(l).velocity.x - pre_g(0, 0),
                                    checked_flows.at(i).at(l).point.y - checked_flows.at(i).at(l).velocity.y - pre_g(0, 0),
                                    checked_flows.at(i).at(l).point.z - checked_flows.at(i).at(l).velocity.z - pre_g(0, 0),
                                    1.0);
            Eigen::Vector4f calced_vel = tmp_matrix * pre_pos;
            Eigen::Vector3f error_v(checked_flows.at(i).at(l).velocity.x - pre_pos(0),
                                    checked_flows.at(i).at(l).velocity.y - pre_pos(1),
                                    checked_flows.at(i).at(l).velocity.z - pre_pos(2));
            double error = error_v.norm();
            if(error < Rthre){
              inlier++;
              error_sum += error;
            }
          }
          if(inlier > Rcnt){
            good_matrixes.push_back(tmp_matrix);
            good_gs.push_back(pre_g);
            double error_ave = error_sum / inlier;
            good_errors.push_back(error_ave);
          }
        }

        Eigen::MatrixXf ht_matrix;
        Eigen::MatrixXf g_pos;
        if(good_matrixes.size() > 0){
          std::vector<double>::iterator min_itr = std::min_element(good_errors.begin(), good_errors.end());
          size_t min_index = std::distance(good_errors.begin(), min_itr);
          ht_matrix = good_matrixes.at(min_index);
          g_pos = good_gs.at(min_index);
        } else {
          Eigen::MatrixXf flow_mat;
          Eigen::MatrixXf pre_pos;
          flow_mat.resize(4, checked_flows.at(i).size());
          pre_pos.resize(4, checked_flows.at(i).size());
          g_pos = Eigen::MatrixXf::Zero(4,1);
          for(l = 0; l < checked_flows.at(i).size(); l++){
            int itr = rand() % checked_flows.at(i).size();
            flow_mat(0, l) = checked_flows.at(i).at(itr).velocity.x;
            flow_mat(1, l) = checked_flows.at(i).at(itr).velocity.y;
            flow_mat(2, l) = checked_flows.at(i).at(itr).velocity.z;
            flow_mat(3, l) = 1.0;
            pre_pos(0, l) = checked_flows.at(i).at(itr).point.x - checked_flows.at(i).at(itr).velocity.x;
            pre_pos(1, l) = checked_flows.at(i).at(itr).point.y - checked_flows.at(i).at(itr).velocity.y;
            pre_pos(2, l) = checked_flows.at(i).at(itr).point.z - checked_flows.at(i).at(itr).velocity.z;
            pre_pos(3, l) = 1.0;
            g_pos(0, 0) += pre_pos(0, l) / checked_flows.at(i).size();
            g_pos(1, 0) += pre_pos(1, l) / checked_flows.at(i).size();
            g_pos(2, 0) += pre_pos(2, l) / checked_flows.at(i).size();
          }

          for(l = 0; l < checked_flows.at(i).size(); l++){
            pre_pos(0, l) -= g_pos(0, 0);
            pre_pos(1, l) -= g_pos(1, 0);
            pre_pos(2, l) -= g_pos(2, 0);
          }
          ht_matrix = flow_mat * pseudoinverse(pre_pos);
        }

        //box_update

        Eigen::MatrixXf box_ref_g;
        box_ref_g.resize(4, 1);
        box_ref_g(0, 0) = labeled_boxes.at(i).pose.position.x - g_pos(0, 0);
        box_ref_g(1, 0) = labeled_boxes.at(i).pose.position.y - g_pos(1, 0);
        box_ref_g(2, 0) = labeled_boxes.at(i).pose.position.z - g_pos(2, 0);
        box_ref_g(3, 0) = 1.0;

        Eigen::MatrixXf box_translate(ht_matrix * box_ref_g);
        labeled_boxes.at(i).pose.position.x += box_translate(0, 0);
        labeled_boxes.at(i).pose.position.y += box_translate(1, 0);
        labeled_boxes.at(i).pose.position.z += box_translate(2, 0);

        ht_matrix.block(0, 0, 3, 3) += Eigen::Matrix3f::Identity();
        Eigen::Matrix3f rot_mat(ht_matrix.block(0, 0, 3, 3));
        Eigen::Quaternionf rot_q(rot_mat);
        Eigen::Quaternionf prev_q(labeled_boxes.at(i).pose.orientation.w,
                                  labeled_boxes.at(i).pose.orientation.x,
                                  labeled_boxes.at(i).pose.orientation.y,
                                  labeled_boxes.at(i).pose.orientation.z);
        Eigen::Quaternionf next_q = rot_q * prev_q;
        labeled_boxes.at(i).pose.orientation.w = next_q.w();
        labeled_boxes.at(i).pose.orientation.x = next_q.x();
        labeled_boxes.at(i).pose.orientation.y = next_q.y();
        labeled_boxes.at(i).pose.orientation.z = next_q.z();

        //update box.header
        labeled_boxes.at(i).header = image_msg->header;
      }
    }
    k = 0;
    for(i = 0; i < checked_flows.size(); i++){
      if(checked_flows.at(i).size() < 4){
        for(j = 0; j < flow_labels.size(); j++){
          if(flow_labels.at(j) == labeled_boxes.at(i - k).label){
            flow_labels.at(j) = max_label + 2;
          }
        }
        labeled_boxes.erase(labeled_boxes.begin() + i - k);
        copy_labeled_boxes.erase(copy_labeled_boxes.begin() + i - k);
        k++;
      }
    }
    jsk_recognition_msgs::BoundingBoxArray box_msg;
    box_msg.header = image_msg->header;
    for(i = 0; i < labeled_boxes.size(); i++){
      box_msg.boxes.push_back(labeled_boxes.at(i));
    }

    image_pub_.publish(flow_image_msg);
    result_pub_.publish(flows_result_msg);
    if(publish_marker_)
      vis_pub_.publish(marker);
    box_pub_.publish(box_msg);
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FlowTracking,nodelet::Nodelet);
