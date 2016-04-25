// -*- mode: c++ -*-
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

#include "jsk_perception/binarize.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void Binarize::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void Binarize::subscribe()
  {
    sub_img_ = pnh_->subscribe("input", 1, &Binarize::convert, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void Binarize::unsubscribe()
  {
    sub_img_.shutdown();
  }

  void Binarize::convert(
    const sensor_msgs::Image::ConstPtr& img_msg)
  {
    vital_checker_->poke();
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, "mono8");
    cv::Mat img(img_msg->height, img_msg->width, CV_8UC1);
    cv_ptr->image.copyTo(img);
    cv::Mat bin_img;
    cv_bridge::CvImagePtr bin_ptr;
    bin_ptr = cv_bridge::toCvCopy(img_msg, "mono8");
    cv::threshold(img, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    bin_img.copyTo(bin_ptr->image);
    sensor_msgs::ImagePtr bin_msg= bin_ptr->toImageMsg();
    pub_.publish(bin_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::Binarize, nodelet::Nodelet);
