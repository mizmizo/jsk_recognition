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
    sub_box_.subscribe(*pnh_, "box", 1);
    sub_flow_.subscribe(*pnh_, "flow", 10);
    sub_only_flow_ = pnh_->subscribe("flow", 5, &FlowSegmantation::flow_extract, this);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_target_);
    sync_->registerCallback(boost::bind(&FlowSegmentation::box_extract, this, _1, _2));
  }

  void FlowSegmentation::unsubscribe()
  {
    sub_box_.unsubscribe();
    sub_flow_.unsubscribe();
  }

  void box_extract(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &box)
  {
    vital_checker_->poke();
    if(!(labeled_boxes.boxes.size() > 0))
      {
        unlabeled_boxes = box;
      } else {
      //todo
      
    }
    
  }


  void flow_extract(const jsk_recognition_msgs::Flow3DArrayStampedConstPtr &flow)
  {
    if(!(labeled_boxes.boxes.size() > 0)) return;
    flow_lebeling(); //todo
    check_flow();
    compute_boundingbox_vel();

  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FlowSegmentation,
                        nodelet::Nodelet);

