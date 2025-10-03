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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <movidius_ncs_lib/ncs.h>
#include <movidius_ncs_lib/utils.h>
#include <movidius_ncs_lib/performance_monitor.h>
#include <movidius_ncs_lib/config_manager.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

class BatchProcessor
{
public:
  BatchProcessor(ros::NodeHandle& nh, const std::string& model_name)
    : nh_(nh), model_name_(model_name), performance_monitor_("batch_performance.log")
  {
    // Load configuration
    config_manager_ = std::make_shared<ConfigManager>("../config/" + model_name + ".yaml");

    // Initialize NCS device
    auto model_config = config_manager_->getModelConfig(model_name);
    if (!model_config)
    {
      ROS_ERROR("Failed to load model configuration for: %s", model_name.c_str());
      return;
    }

    ncs_ = std::make_unique<NCS>(
      0,  // device index
      Device::LogLevel::INFO,
      model_config->type,
      model_config->graph_file_path,
      model_config->category_file_path,
      model_config->network_dimension,
      model_config->mean,
      model_config->scale,
      model_config->top_n
    );

    // Image transport for output
    image_transport::ImageTransport it(nh_);
    output_pub_ = it.advertise("/ncs/batch/results", 10);

    // Subscribe to input directory or camera
    std::string input_method;
    nh_.param<std::string>("input_method", input_method, "directory");

    if (input_method == "directory")
    {
      std::string input_directory;
      nh_.param<std::string>("input_directory", input_directory, "../data/images/");
      processDirectory(input_directory);
    }
    else
    {
      image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1,
        &BatchProcessor::imageCallback, this);
      ROS_INFO("Subscribed to camera feed. Press Ctrl+C to stop.");
      ros::spin();
    }
  }

private:
  void processDirectory(const std::string& directory_path)
  {
    ROS_INFO("Processing images in directory: %s", directory_path.c_str());

    std::vector<std::string> image_files;
    cv::glob(directory_path + "/*.jpg", image_files, false);
    cv::glob(directory_path + "/*.jpeg", image_files, false);
    cv::glob(directory_path + "/*.png", image_files, false);
    cv::glob(directory_path + "/*.bmp", image_files, false);

    if (image_files.empty())
    {
      ROS_WARN("No image files found in directory: %s", directory_path.c_str());
      return;
    }

    ROS_INFO("Found %zu images to process", image_files.size());

    int batch_size = 10;
    nh_.param<int>("batch_size", batch_size, 10);

    for (size_t i = 0; i < image_files.size(); i += batch_size)
    {
      size_t end_idx = std::min(i + batch_size, image_files.size());
      std::vector<std::string> batch(image_files.begin() + i, image_files.begin() + end_idx);

      processBatch(batch);
    }

    // Generate performance report
    std::string report = performance_monitor_.generateReport();
    ROS_INFO("\n%s", report.c_str());

    // Export to CSV
    performance_monitor_.exportToCSV("batch_results.csv");
  }

  void processBatch(const std::vector<std::string>& image_files)
  {
    std::vector<cv::Mat> processed_images;
    std::vector<std::string> original_files = image_files;

    for (const auto& file_path : image_files)
    {
      cv::Mat image = cv::imread(file_path);
      if (image.empty())
      {
        ROS_WARN("Failed to load image: %s", file_path.c_str());
        continue;
      }

      // Preprocess image
      auto model_config = config_manager_->getModelConfig(model_name_);
      cv::Mat processed = Utils::loadAndPreprocessImage(
        file_path,
        model_config->network_dimension,
        model_config->mean,
        model_config->scale
      );

      processed_images.push_back(processed);
      processed_files_.push_back(file_path);
    }

    if (processed_images.empty()) return;

    // Start timing
    performance_monitor_.startTimer("batch_processing");

    // Process batch
    std::vector<sensor_msgs::ImagePtr> results;

    for (size_t i = 0; i < processed_images.size(); ++i)
    {
      const auto& image = processed_images[i];
      const auto& file_path = processed_files_[processed_files_.size() - processed_images.size() + i];

      // Load tensor
      ncs_->loadTensor(image);

      // Perform inference
      if (model_config->type == "classification")
      {
        ncs_->classify();
        auto result = ncs_->getClassificationResult();

        // Create result visualization
        cv::Mat original_image = cv::imread(file_path);
        cv::Mat result_image = createClassificationResultImage(original_image, result);

        // Convert to ROS message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();
        results.push_back(msg);

        // Record metrics
        PerformanceMetrics metrics;
        metrics.inference_time_ms = result->time_taken;
        metrics.preprocessing_time_ms = 0;  // Already included in load time
        metrics.total_time_ms = result->time_taken;
        metrics.frame_count = 1;
        metrics.fps = 1000.0 / result->time_taken;
        metrics.operation_type = "classification";
        metrics.model_name = model_name_;
        metrics.image_width = original_image.cols;
        metrics.image_height = original_image.rows;

        performance_monitor_.recordMetrics(metrics);
      }
      else if (model_config->type == "detection")
      {
        ncs_->detect();
        auto result = ncs_->getDetectionResult();

        // Create result visualization
        cv::Mat original_image = cv::imread(file_path);
        cv::Mat result_image = createDetectionResultImage(original_image, result);

        // Convert to ROS message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();
        results.push_back(msg);

        // Record metrics
        PerformanceMetrics metrics;
        metrics.inference_time_ms = result->time_taken;
        metrics.total_time_ms = result->time_taken;
        metrics.frame_count = 1;
        metrics.fps = 1000.0 / result->time_taken;
        metrics.operation_type = "detection";
        metrics.model_name = model_name_;
        metrics.image_width = original_image.cols;
        metrics.image_height = original_image.rows;

        performance_monitor_.recordMetrics(metrics);
      }
    }

    processed_files_.clear();

    // Publish results
    for (const auto& result : results)
    {
      output_pub_.publish(result);
    }

    performance_monitor_.stopTimer("batch_processing");

    ROS_INFO("Processed batch of %zu images", processed_images.size());
  }

  cv::Mat createClassificationResultImage(const cv::Mat& image,
                                         const ClassificationResultPtr& result)
  {
    cv::Mat result_image = image.clone();

    // Draw top classifications
    for (size_t i = 0; i < std::min(result->items.size(), size_t(5)); ++i)
    {
      const auto& item = result->items[i];

      // Draw rectangle for label
      cv::Rect label_rect(10, 10 + i * 30, 300, 25);
      cv::rectangle(result_image, label_rect, cv::Scalar(0, 0, 0), -1);

      // Draw label text
      std::string label = item.category + ": " +
                         std::to_string(item.probability).substr(0, 4);
      cv::putText(result_image, label, cv::Point(15, 25 + i * 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    }

    return result_image;
  }

  cv::Mat createDetectionResultImage(const cv::Mat& image,
                                    const DetectionResultPtr& result)
  {
    cv::Mat result_image = image.clone();

    // Draw detections
    for (const auto& detection : result->items)
    {
      cv::Rect box(
        static_cast<int>(detection.x),
        static_cast<int>(detection.y),
        static_cast<int>(detection.w),
        static_cast<int>(detection.h)
      );

      // Draw bounding box
      cv::rectangle(result_image, box, cv::Scalar(0, 255, 0), 2);

      // Draw label
      std::string label = detection.category + ": " +
                         std::to_string(detection.probability).substr(0, 4);

      cv::putText(result_image, label,
                  cv::Point(detection.x, detection.y - 5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, 255, 0), 2);
    }

    return result_image;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

      // Process single image
      std::vector<std::string> batch = {"camera_frame"};
      processBatch(batch);

      ros::Duration(0.1).sleep();  // Small delay between frames
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("CV bridge exception: %s", e.what());
    }
  }

  ros::NodeHandle nh_;
  std::string model_name_;
  std::unique_ptr<NCS> ncs_;
  std::shared_ptr<ConfigManager> config_manager_;
  image_transport::Publisher output_pub_;
  PerformanceMonitor performance_monitor_;
  std::vector<std::string> processed_files_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "batch_processing_example");

  if (argc < 2)
  {
    ROS_ERROR("Usage: batch_processing_example <model_name>");
    ROS_ERROR("Available models: alexnet, googlenet, mobilenetssd, etc.");
    return 1;
  }

  std::string model_name = argv[1];

  ros::NodeHandle nh("~");
  BatchProcessor processor(nh, model_name);

  return 0;
}
