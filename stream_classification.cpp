/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <object_msgs/Object.h>
#include <object_msgs/Objects.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#define LINESPACING 50
#define DEFAULT_PARALLEL_FLAG 1
#define MOVEWINDOW 1000

// UI Enhancement constants
#define TEXT_FONT cv::FONT_HERSHEY_SIMPLEX
#define TEXT_SCALE 0.8
#define TEXT_THICKNESS 2
#define SHADOW_OFFSET 2

// UI Helper Functions
cv::Scalar getConfidenceColor(double confidence) {
  if (confidence >= 0.8) return cv::Scalar(0, 255, 0);      // Green for high confidence
  else if (confidence >= 0.6) return cv::Scalar(0, 255, 255); // Yellow for medium confidence
  else return cv::Scalar(0, 165, 255);                      // Orange for low confidence
}

void drawTextWithShadow(cv::Mat& image, const std::string& text, cv::Point position, 
                       cv::Scalar color, int fontFace, double fontScale, int thickness) {
  // Draw shadow
  cv::putText(image, text, cv::Point(position.x + SHADOW_OFFSET, position.y + SHADOW_OFFSET), 
              fontFace, fontScale, cv::Scalar(0, 0, 0), thickness + 1);
  // Draw main text
  cv::putText(image, text, position, fontFace, fontScale, color, thickness);
}

void drawTextWithBackground(cv::Mat& image, const std::string& text, cv::Point position, 
                           cv::Scalar textColor, cv::Scalar bgColor, int fontFace, double fontScale, int thickness) {
  int baseline = 0;
  cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  
  // Draw background rectangle
  cv::Point bgTopLeft(position.x - 5, position.y - textSize.height - 5);
  cv::Point bgBottomRight(position.x + textSize.width + 5, position.y + baseline + 5);
  cv::rectangle(image, bgTopLeft, bgBottomRight, bgColor, -1);
  
  // Draw text
  cv::putText(image, text, position, fontFace, fontScale, textColor, thickness);
}

int getFPS()
{
  static int fps = 0;
  static boost::posix_time::ptime duration_start = boost::posix_time::microsec_clock::local_time();
  static int frame_cnt = 0;

  frame_cnt++;

  boost::posix_time::ptime current = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration msdiff = current - duration_start;

  if (msdiff.total_milliseconds() > 1000)
  {
    fps = frame_cnt;
    frame_cnt = 0;
    duration_start = current;
  }

  return fps;
}

int parallel_flag;
void syncCb(const sensor_msgs::ImageConstPtr& img, const object_msgs::Objects::ConstPtr& objs)
{
  cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;
  int cnt = 0;

 // Draw header
  std::stringstream header_ss;
  header_ss << "F.Project 2022 - Live Classification Stream";
  drawTextWithBackground(cvImage, header_ss.str(), cv::Point(LINESPACING, LINESPACING), 
                        cv::Scalar(255, 255, 255), cv::Scalar(50, 50, 50), TEXT_FONT, TEXT_SCALE + 0.2, TEXT_THICKNESS);

  for (auto obj : objs->objects_vector)
  {
    std::stringstream ss;
    ss << obj.object_name << ": " << std::fixed << std::setprecision(1) << obj.probability * 100 << '%';
    
    // Get confidence-based color
    cv::Scalar textColor = getConfidenceColor(obj.probability);
    
    // Draw text with shadow for better visibility
    drawTextWithShadow(cvImage, ss.str(), cv::Point(LINESPACING, LINESPACING * (++cnt + 1)), 
                      textColor, TEXT_FONT, TEXT_SCALE, TEXT_THICKNESS);
  }

   if (parallel_flag == 0)
  {
    cv::imshow("F.Project 2022 - Live Classification (Single Device)", cvImage);
  }
  else
  {
    cv::namedWindow("F.Project 2022 - Live Classification (Multiple Devices)");
    cv::moveWindow("F.Project 2022 - Live Classification (Multiple Devices)", MOVEWINDOW, 0);
    cv::imshow("F.Project 2022 - Live Classification (Multiple Devices)", cvImage);
  }

  int key = cv::waitKey(5);
  if (key == 13 || key == 27 || key == 32 || key == 113)
  {
    ros::shutdown();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movidius_ncs_example_stream");
  ros::NodeHandle nh("~");

  parallel_flag = DEFAULT_PARALLEL_FLAG;
  if (!nh.getParam("parallel_flag", parallel_flag))
  {
    ROS_WARN("param parallel_flag not set, use default");
  }
  ROS_INFO_STREAM("use parallel_flag = " << parallel_flag);

  if (parallel_flag == 0)
  {
    message_filters::Subscriber<sensor_msgs::Image> camSub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<object_msgs::Objects> objSub(nh, "/movidius_ncs_nodelet/classified_objects_single", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, object_msgs::Objects> sync(camSub, objSub, 60);
    sync.registerCallback(boost::bind(&syncCb, _1, _2));
    ros::spin();
  }
  else
  {
    message_filters::Subscriber<sensor_msgs::Image> camSub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<object_msgs::Objects> objSub(nh, "/movidius_ncs_nodelet/classified_objects_multiple",
                                                             1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, object_msgs::Objects> sync(camSub, objSub, 60);
    sync.registerCallback(boost::bind(&syncCb, _1, _2));
    ros::spin();
  }

  return 0;
}
