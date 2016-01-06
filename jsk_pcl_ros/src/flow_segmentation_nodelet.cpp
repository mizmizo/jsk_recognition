// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_pcl_ros/flow_segmentation.h"
#include <cv_bridge/cv_bridge.h>
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
#include <Eigen/Geometry> 

#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include "jsk_pcl_ros/pcl_util.h"

namespace jsk_pcl_ros
{
  void FlowSegmentation::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("publish_tf", publish_tf_, false);
    if (!pnh_->getParam("tf_prefix", tf_prefix_))
      {
        if (publish_tf_) {
          JSK_ROS_WARN("~tf_prefix is not specified, using %s", getName().c_str());
        }
        tf_prefix_ = getName();
      }
    box_pub_ = advertise<jsk_recognition_msgs::BoundingBoxArray>(*pnh_, "boxes", 1);
  }
  
  void FlowSegmentation::subscribe()
  {
    sub_box_ = pnh_->subscribe("box", 1, &FlowSegmentation::box_extract, this);
    sub_flow_ = pnh_->subscribe("flow", 5, &FlowSegmentation::flow_extract, this);
  }
  
  void FlowSegmentation::unsubscribe()
  {
    sub_box_.shutdown();
    sub_flow_.shutdown();
  }
  
  bool FlowSegmentation::comparebox(const jsk_recognition_msgs::BoundingBox& input_box,
                                      uint* label)
  {
    size_t i,j;
    bool box_fit = false;
    std::vector<cv::Point3d> input_vertices = getVertices(input_box);
    //    std::cout << "input_vertices " << input_vertices << std::endl;
    for(i = 0; i < labeled_boxes.size(); i++){
      std::vector<cv::Point3d> compared_vertices = getVertices(labeled_boxes.at(i));
      //    std::cout << "compared_vertices " << compared_vertices << std::endl;
      bool fit_candidate(false);
      for(j = 0; j < 8; j++){
        if(comparevertices(input_vertices.at(j), labeled_boxes.at(i))
           || comparevertices(compared_vertices.at(j), input_box)){
          std::cout << "fit_candidate" << std::endl;
          fit_candidate = true;
          break;
        }
      }
      if(fit_candidate){
        if(!box_fit){
          box_fit = true;
          *label = labeled_boxes.at(i).label;
        } else {
          box_fit = false;
          break;
        }
      }
    }
    std::cout << "box_fit " << box_fit << std::endl;
    if(box_fit){
      return true;
    } else {
      return false;
        }    
  }

  std::vector<cv::Point3d> FlowSegmentation::getVertices(const jsk_recognition_msgs::BoundingBox& box)
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

  bool FlowSegmentation::comparevertices(const cv::Point3d& vertice,
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
    /*    Eigen::Vector3f v_target(vertice.x - compared_vertices.at(0).x,
                             vertice.y - compared_vertices.at(0).y,
                             vertice.z - compared_vertices.at(0).z);
    Eigen::Vector3f v_x(compared_vertices.at(1).x - compared_vertices.at(0).x,
                        compared_vertices.at(1).y - compared_vertices.at(0).y,
                        compared_vertices.at(1).z - compared_vertices.at(0).z);
    Eigen::Vector3f v_y(compared_vertices.at(3).x - compared_vertices.at(0).x,
                        compared_vertices.at(3).y - compared_vertices.at(0).y,
                        compared_vertices.at(3).z - compared_vertices.at(0).z);
    Eigen::Vector3f v_z(compared_vertices.at(4).x - compared_vertices.at(0).x,
                        compared_vertices.at(4).y - compared_vertices.at(0).y,
                        compared_vertices.at(4).z - compared_vertices.at(0).z);*/
    
    //    float test = (v_target.dot(v_x) / (v_x.norm() * v_x.norm()));
    //    std::cout << "test_dot " << test << std::endl;
    if(v_x.norm() == 0)std::cout << "nan!!!" << std::endl;
      if((v_target.dot(v_x) / (v_x.norm() * v_x.norm())) > 0.0 &&
       (v_target.dot(v_x) / (v_x.norm() * v_x.norm())) < 1.0 &&
       (v_target.dot(v_y) / (v_y.norm() * v_y.norm())) > 0.0 &&
       (v_target.dot(v_y) / (v_y.norm() * v_y.norm())) < 1.0 &&
       (v_target.dot(v_z) / (v_z.norm() * v_z.norm())) > 0.0 &&
       (v_target.dot(v_z) / (v_z.norm() * v_z.norm())) < 1.0)
      {
        //    std::cout << "oclued true" << std::endl; 
        return true;
      } else {
        //std::cout << "oclued false" << std::endl;
      return false;
    }
  }
  
  void FlowSegmentation::box_extract(const jsk_recognition_msgs::BoundingBoxArrayConstPtr& box)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    std::cout << " " << std::endl;
    std::cout << "box_extract" << std::endl;
    if(labeled_boxes.size() > 0)
      {
        size_t i,j,k;
        j = k = 0;
        uint max_label = labeled_boxes.at(labeled_boxes.size() - 1).label;
        std::vector<jsk_recognition_msgs::BoundingBox> tmp_boxes;
        tmp_boxes.resize(box->boxes.size() + max_label);
        std::cout << "test2" << std::endl;
        for(i = 0; i < box->boxes.size(); i++){
            jsk_recognition_msgs::BoundingBox input_box;
            input_box = box->boxes[i];
            input_box.header = box->header;
            uint label;
            std::cout << "test3 i:" << i << std::endl;
            if(comparebox(input_box, &label)){ //fit-box label
              std::cout << "label " << label << std::endl;
              input_box.label = label;
              if(tmp_boxes.at(label).header.stamp != box->header.stamp){ //update label
                tmp_boxes.at(label) = input_box;
              } else {
                tmp_boxes.at(max_label + j) = input_box; //label split
                tmp_boxes.at(max_label + j).label = max_label + j;
                j++;
              }
            } else {
              tmp_boxes.at(max_label + j) = input_box; //new label
              tmp_boxes.at(max_label + j).label = max_label + j;
              j++;
            }
        }
        
        for(i = 0; i < labeled_boxes.size(); i++){ //update boundingbox
          if(tmp_boxes.at(labeled_boxes.at(i).label).header.stamp == box->header.stamp){
            labeled_boxes.at(k) = tmp_boxes.at(labeled_boxes.at(i).label);
            k++;
          }
        }
        labeled_boxes.resize(k + j);
        for(i = 0; i < j; i++){
          labeled_boxes.at(k + i) = tmp_boxes.at(max_label + i);
        }
      } else {
      size_t i;
      for(i = 0; i < box->boxes.size(); i++){
        jsk_recognition_msgs::BoundingBox tmp_box;
        tmp_box = box->boxes[i];
        tmp_box.label = i;
        labeled_boxes.push_back(tmp_box);
      }
    }
  }

  
  
  void FlowSegmentation::flow_extract(const jsk_recognition_msgs::Flow3DArrayStampedConstPtr& flow)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(!(labeled_boxes.size() > 0)) return;

    std::cout << " " << std::endl;
    std::cout << "flow_extract" << std::endl;

    //flow_lebeling
    std::vector<jsk_recognition_msgs::Flow3D> unchecked_flows(flow->flows);
    std::vector<std::vector<jsk_recognition_msgs::Flow3D> > checked_flows;
    std::vector<jsk_recognition_msgs::Flow3D> translation_flows;
    std::vector<int> flow_label_count(labeled_boxes.size(),0);
    size_t i,j;
    int k;
    uint flow_label; // not same as BoundingBox.label
    checked_flows.resize(labeled_boxes.size());
    for(i = 0; i < labeled_boxes.size(); i++){
      checked_flows.at(i).resize(unchecked_flows.size());
    }
    translation_flows.resize(labeled_boxes.size());
    std::cout << "flow test 1" << std::endl;
    for(i = 0; i < unchecked_flows.size(); i++){
      cv::Point3d point(unchecked_flows.at(i).point.x, unchecked_flows.at(i).point.y, unchecked_flows.at(i).point.z);
      k = 0;
      for(j = 0; j < labeled_boxes.size(); j++){
        if(comparevertices(point, labeled_boxes.at(j))){
          k++;
          flow_label = j;
        } 
      }
      if(k == 1){
        checked_flows.at(flow_label).push_back(unchecked_flows.at(i));
        flow_label_count.at(flow_label)++;
      }            
    }
    for(i = 0; i < labeled_boxes.size(); i++){
      checked_flows.at(i).resize(flow_label_count.at(i));
    }

    std::cout << "flow test 2" << std::endl;
    //calc translation_flows
    for(i = 0; i < checked_flows.size(); i++){
      for(j = 0; j < checked_flows.at(i).size(); j++){
        if(j == 0){
          std::cout << "flow test 2.0" << std::endl;
          translation_flows.at(i) = checked_flows.at(i).at(j);
          translation_flows.at(i).velocity.x = translation_flows.at(i).velocity.x / checked_flows.at(i).size();
          translation_flows.at(i).velocity.y = translation_flows.at(i).velocity.y / checked_flows.at(i).size();
          translation_flows.at(i).velocity.z = translation_flows.at(i).velocity.z / checked_flows.at(i).size();
        } else {
          std::cout << "flow test 2.1" << std::endl;
          float test = translation_flows.at(i).velocity.x;
          std::cout << "flow test 2.1.1" << std::endl;
          translation_flows.at(i).velocity.x += checked_flows.at(i).at(j).velocity.x / checked_flows.at(i).size();
          translation_flows.at(i).velocity.y += checked_flows.at(i).at(j).velocity.y / checked_flows.at(i).size();
          translation_flows.at(i).velocity.z += checked_flows.at(i).at(j).velocity.z / checked_flows.at(i).size();
        }
      }
    }
    std::cout << "flow test 3" << std::endl;
    for(i = 0; i < checked_flows.size(); i++){
      for(j = 0; j < checked_flows.at(i).size(); j++){
        checked_flows.at(i).at(j).velocity.x -= translation_flows.at(i).velocity.x;
        checked_flows.at(i).at(j).velocity.y -= translation_flows.at(i).velocity.y;
        checked_flows.at(i).at(j).velocity.z -= translation_flows.at(i).velocity.z;
      }
    }
    std::cout << "flow test 4" << std::endl;
    jsk_recognition_msgs::BoundingBoxArray box_msg;
    box_msg.header = flow->header;
    for(i = 0; i < labeled_boxes.size(); i++){
      box_msg.boxes.push_back(labeled_boxes.at(i));
    }
    box_pub_.publish(box_msg);
    //check flow

  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FlowSegmentation,nodelet::Nodelet);

