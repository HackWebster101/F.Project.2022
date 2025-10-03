/*
 * Copyright (c) 2017-2025 Intel Corporation
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

#include "movidius_ncs_lib/utils.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

namespace movidius_ncs_lib
{
cv::Mat Utils::loadAndPreprocessImage(const std::string& image_path,
                                     int network_dimension,
                                     const std::vector<float>& mean,
                                     float scale)
{
  cv::Mat image = cv::imread(image_path);
  if (image.empty())
  {
    throw std::runtime_error("Failed to load image: " + image_path);
  }

  // Resize maintaining aspect ratio and pad if necessary
  cv::Mat resized = resizeMaintainingAspectRatio(image,
                                               cv::Size(network_dimension, network_dimension));

  // Normalize the image
  std::vector<float> tensor_data = imageToTensor(resized, network_dimension, mean, scale);

  // Convert back to cv::Mat for compatibility
  cv::Mat result(network_dimension, network_dimension, CV_32FC3);
  memcpy(result.data, tensor_data.data(), tensor_data.size() * sizeof(float));

  return result;
}

std::vector<float> Utils::imageToTensor(const cv::Mat& image,
                                       int network_dimension,
                                       const std::vector<float>& mean,
                                       float scale)
{
  std::vector<float> tensor_data;

  // Convert to float and normalize
  for (int c = 0; c < 3; ++c)
  {
    for (int h = 0; h < image.rows; ++h)
    {
      for (int w = 0; w < image.cols; ++w)
      {
        cv::Vec3b pixel = image.at<cv::Vec3b>(h, w);
        float value = static_cast<float>(pixel[c]) / scale - mean[c];
        tensor_data.push_back(value);
      }
    }
  }

  return tensor_data;
}

cv::Mat Utils::drawDetections(const cv::Mat& image,
                            const std::vector<Detection>& detections,
                            const std::vector<std::string>& class_names,
                            float threshold)
{
  cv::Mat result = image.clone();

  for (const auto& det : detections)
  {
    if (det.confidence >= threshold)
    {
      // Draw bounding box
      cv::rectangle(result, det.box, cv::Scalar(0, 255, 0), 2);

      // Draw label
      std::string label = class_names[det.class_id] + ": " +
                         std::to_string(det.confidence).substr(0, 4);

      int baseline = 0;
      cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                           0.5, 1, &baseline);

      cv::rectangle(result,
                   cv::Point(det.box.x, det.box.y - label_size.height),
                   cv::Point(det.box.x + label_size.width, det.box.y),
                   cv::Scalar(0, 255, 0), cv::FILLED);

      cv::putText(result, label,
                 cv::Point(det.box.x, det.box.y - 2),
                 cv::FONT_HERSHEY_SIMPLEX, 0.5,
                 cv::Scalar(0, 0, 0), 1);
    }
  }

  return result;
}

std::vector<cv::Rect> Utils::convertToAbsoluteCoordinates(
  const std::vector<std::vector<float>>& boxes,
  int image_width,
  int image_height)
{
  std::vector<cv::Rect> result;

  for (const auto& box : boxes)
  {
    if (box.size() >= 4)
    {
      int x = static_cast<int>(box[0] * image_width);
      int y = static_cast<int>(box[1] * image_height);
      int w = static_cast<int>(box[2] * image_width);
      int h = static_cast<int>(box[3] * image_height);

      result.push_back(cv::Rect(x, y, w, h));
    }
  }

  return result;
}

std::vector<Utils::Detection> Utils::applyNMS(
  const std::vector<Detection>& detections,
  float threshold)
{
  std::vector<Detection> result = detections;

  // Sort by confidence (highest first)
  std::sort(result.begin(), result.end(),
           [](const Detection& a, const Detection& b) {
             return a.confidence > b.confidence;
           });

  std::vector<bool> suppressed(result.size(), false);

  for (size_t i = 0; i < result.size(); ++i)
  {
    if (suppressed[i]) continue;

    for (size_t j = i + 1; j < result.size(); ++j)
    {
      if (suppressed[j]) continue;

      float iou = calculateIoU(result[i].box, result[j].box);
      if (iou > threshold)
      {
        suppressed[j] = true;
      }
    }
  }

  // Remove suppressed detections
  result.erase(std::remove_if(result.begin(), result.end(),
                             [&suppressed](const Detection& det) {
                               return suppressed[&det - &result[0]];
                             }), result.end());

  return result;
}

float Utils::calculateIoU(const cv::Rect& box1, const cv::Rect& box2)
{
  int x1 = std::max(box1.x, box2.x);
  int y1 = std::max(box1.y, box2.y);
  int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
  int y2 = std::min(box1.y + box1.height, box2.y + box2.height);

  int intersection_width = std::max(0, x2 - x1);
  int intersection_height = std::max(0, y2 - y1);
  int intersection_area = intersection_width * intersection_height;

  int union_area = box1.width * box1.height + box2.width * box2.height - intersection_area;

  if (union_area == 0) return 0.0f;

  return static_cast<float>(intersection_area) / union_area;
}

cv::Mat Utils::resizeMaintainingAspectRatio(const cv::Mat& image,
                                           const cv::Size& target_size)
{
  cv::Mat result;

  float aspect_ratio = static_cast<float>(image.cols) / image.rows;
  int new_width, new_height;

  if (aspect_ratio > 1.0f)
  {
    // Image is wider
    new_width = target_size.width;
    new_height = static_cast<int>(target_size.width / aspect_ratio);
  }
  else
  {
    // Image is taller
    new_height = target_size.height;
    new_width = static_cast<int>(target_size.height * aspect_ratio);
  }

  cv::resize(image, result, cv::Size(new_width, new_height));

  // Create square canvas and center the image
  cv::Mat canvas(target_size.height, target_size.width, image.type(),
                cv::Scalar(128, 128, 128));  // Gray background

  int x_offset = (target_size.width - new_width) / 2;
  int y_offset = (target_size.height - new_height) / 2;

  result.copyTo(canvas(cv::Rect(x_offset, y_offset, new_width, new_height)));

  return canvas;
}

std::vector<Utils::Detection> Utils::yoloToDetections(
  const std::vector<float>& yolo_boxes,
  int image_width,
  int image_height)
{
  std::vector<Detection> detections;

  // YOLO format: [x, y, w, h, confidence, class_id] per detection
  for (size_t i = 0; i < yolo_boxes.size(); i += 6)
  {
    if (i + 5 >= yolo_boxes.size()) break;

    float x = yolo_boxes[i + 0];
    float y = yolo_boxes[i + 1];
    float w = yolo_boxes[i + 2];
    float h = yolo_boxes[i + 3];
    float confidence = yolo_boxes[i + 4];
    int class_id = static_cast<int>(yolo_boxes[i + 5]);

    // Convert to absolute coordinates
    int abs_x = static_cast<int>((x - w / 2.0f) * image_width);
    int abs_y = static_cast<int>((y - h / 2.0f) * image_height);
    int abs_w = static_cast<int>(w * image_width);
    int abs_h = static_cast<int>(h * image_height);

    Detection det;
    det.box = cv::Rect(abs_x, abs_y, abs_w, abs_h);
    det.confidence = confidence;
    det.class_id = class_id;

    detections.push_back(det);
  }

  return detections;
}

std::vector<Utils::Detection> Utils::ssdToDetections(
  const std::vector<float>& ssd_boxes)
{
  std::vector<Detection> detections;

  // SSD format: [x1, y1, x2, y2, confidence, class_id] per detection
  for (size_t i = 0; i < ssd_boxes.size(); i += 6)
  {
    if (i + 5 >= ssd_boxes.size()) break;

    float x1 = ssd_boxes[i + 0];
    float y1 = ssd_boxes[i + 1];
    float x2 = ssd_boxes[i + 2];
    float y2 = ssd_boxes[i + 3];
    float confidence = ssd_boxes[i + 4];
    int class_id = static_cast<int>(ssd_boxes[i + 5]);

    Detection det;
    det.box = cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                      static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
    det.confidence = confidence;
    det.class_id = class_id;

    detections.push_back(det);
  }

  return detections;
}

}  // namespace movidius_ncs_lib
