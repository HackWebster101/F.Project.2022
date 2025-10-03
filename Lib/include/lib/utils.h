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

#ifndef MOVIDIUS_NCS_LIB_UTILS_H_
#define MOVIDIUS_NCS_LIB_UTILS_H_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace movidius_ncs_lib
{
/**
 * @brief Utility functions for NCS operations
 */
class Utils
{
public:
  /**
   * @brief Load image from file path and preprocess for neural network
   * @param image_path Path to the image file
   * @param network_dimension Dimension expected by the network
   * @param mean Mean values for normalization
   * @param scale Scale factor for normalization
   * @return Preprocessed cv::Mat image
   */
  static cv::Mat loadAndPreprocessImage(const std::string& image_path,
                                       int network_dimension,
                                       const std::vector<float>& mean,
                                       float scale);

  /**
   * @brief Convert OpenCV image to network input tensor format
   * @param image Input image
   * @param network_dimension Network input dimension
   * @param mean Mean values for normalization
   * @param scale Scale factor for normalization
   * @return Processed tensor data
   */
  static std::vector<float> imageToTensor(const cv::Mat& image,
                                         int network_dimension,
                                         const std::vector<float>& mean,
                                         float scale);

  /**
   * @brief Draw detection results on image
   * @param image Input image
   * @param detections Detection results
   * @param class_names Class names for labels
   * @param threshold Confidence threshold
   * @return Image with drawn detections
   */
  static cv::Mat drawDetections(const cv::Mat& image,
                               const std::vector<Detection>& detections,
                               const std::vector<std::string>& class_names,
                               float threshold = 0.5);

  /**
   * @brief Convert relative coordinates to absolute image coordinates
   * @param boxes Bounding boxes in relative format [x, y, w, h]
   * @param image_width Image width
   * @param image_height Image height
   * @return Bounding boxes in absolute format
   */
  static std::vector<cv::Rect> convertToAbsoluteCoordinates(
    const std::vector<std::vector<float>>& boxes,
    int image_width,
    int image_height);

  /**
   * @brief Apply non-maximum suppression to detection results
   * @param detections Detection results
   * @param threshold IoU threshold for suppression
   * @return Filtered detection results
   */
  static std::vector<Detection> applyNMS(
    const std::vector<Detection>& detections,
    float threshold = 0.4);

  /**
   * @brief Calculate intersection over union of two bounding boxes
   * @param box1 First bounding box
   * @param box2 Second bounding box
   * @return IoU value
   */
  static float calculateIoU(const cv::Rect& box1, const cv::Rect& box2);

  /**
   * @brief Resize image maintaining aspect ratio
   * @param image Input image
   * @param target_size Target size
   * @return Resized image
   */
  static cv::Mat resizeMaintainingAspectRatio(const cv::Mat& image,
                                             const cv::Size& target_size);

  /**
   * @brief Convert YOLO format to standard bounding box format
   * @param yolo_boxes YOLO format boxes [x, y, w, h, confidence, class_id]
   * @param image_width Image width
   * @param image_height Image height
   * @return Detection objects
   */
  static std::vector<Detection> yoloToDetections(
    const std::vector<float>& yolo_boxes,
    int image_width,
    int image_height);

  /**
   * @brief Convert SSD format to standard bounding box format
   * @param ssd_boxes SSD format boxes [x1, y1, x2, y2, confidence, class_id]
   * @return Detection objects
   */
  static std::vector<Detection> ssdToDetections(
    const std::vector<float>& ssd_boxes);

private:
  // Detection structure for internal use
  struct Detection
  {
    cv::Rect box;
    float confidence;
    int class_id;
    std::string class_name;
  };
};

}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_UTILS_H_
