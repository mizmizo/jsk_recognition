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

#ifndef JSK_PCL_ROS_FLOW_SEGMENTATION_H_
#define JSK_PCL_ROS_FLOW_SEGMENTATION_H_

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/Flow3DArrayStamped.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "jsk_pcl_ros/pcl_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_topic_tools/diagnostic_nodelet.h"

namespace jsk_pcl_ros
{
  class FlowSegmentation: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    FlowSegmentation(): DiagnosticNodelet("FlowSegmentation") { }
    virtual void onInit();
    virtual bool comparebox(const jsk_recognition_msgs::BoundingBox& input_box,
                            uint& label);
    virtual void box_extract(const jsk_recognition_msgs::BoundingBoxArrayConstPtr &box);
    virtual void flow_extract(const jsk_recognition_msgs::Flow3DArrayStampedConstPtr &flow);
  protected:
    virtual void subscribe();
    virtual void unsubscribe();
    
    static uint32_t colorRGBAToUInt32(std_msgs::ColorRGBA c)
    {
        uint8_t r, g, b;
        r = (uint8_t)(c.r * 255);
        g = (uint8_t)(c.g * 255);
        b = (uint8_t)(c.b * 255);
        return ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);
    }

    ros::Subscriber sub_box_;
    ros::Subscriber sub_flow_;
    boost::mutex mutex_;
    ros::Publisher box_pub_;
    tf::TransformBroadcaster br_;
    std::string tf_prefix_;
    bool publish_tf_;
    Counter cluster_counter_;
    std::vector<jsk_recognition_msgs::BoundingBox> unlabeled_boxes;
    std::vector<jsk_recognition_msgs::BoundingBox> labeled_boxes;
  };

}

#endif
