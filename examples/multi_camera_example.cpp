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
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

class MultiCameraProcessor
{
public:
  MultiCameraProcessor(ros::NodeHandle& nh, const std::string& model_name, int num_devices)
    : nh_(nh), model_name_(model_name), num_devices_(num_devices),
      running_(true), performance_monitor_("multi_camera_performance.log")
  {
    // Load configuration
    config_manager_ = std::make_shared<ConfigManager>("../config/" + model_name + ".yaml");

    // Initialize multiple NCS devices
    auto model_config = config_manager_->getModelConfig(model_name);
    if (!model_config)
    {
      ROS_ERROR("Failed to load model configuration for: %s", model_name.c_str());
      return;
    }

    for (int i = 0; i < num_devices_; ++i)
    {
      try
      {
        auto ncs = std::make_unique<NCS>(
          i,  // device index
          Device::LogLevel::INFO,
          model_config->type,
          model_config->graph_file_path,
          model_config->category_file_path,
          model_config->network_dimension,
          model_config->mean,
          model_config->scale,
          model_config->top_n
        );

        ncs_devices_.push_back(std::move(ncs));

        // Create image transport for each device
        image_transport::ImageTransport it(nh_);
        auto pub = it.advertise("/ncs/camera_" + std::to_string(i) + "/results", 10);
        output_pubs_.push_back(pub);

        ROS_INFO("Initialized NCS device %d", i);
      }
      catch (const std::exception& e)
      {
        ROS_WARN("Failed to initialize NCS device %d: %s", i, e.what());
      }
    }

    if (ncs_devices_.empty())
    {
      ROS_ERROR("No NCS devices available");
      return;
    }

    // Subscribe to multiple camera topics
    image_transport::ImageTransport it(nh_);
    for (int i = 0; i < num_devices_; ++i)
    {
      std::string topic_name = "/camera_" + std::to_string(i) + "/image_raw";
      auto sub = it.subscribe(topic_name, 1,
        std::bind(&MultiCameraProcessor::imageCallback, this,
                 std::placeholders::_1, i));
      input_subs_.push_back(sub);

      ROS_INFO("Subscribed to %s", topic_name.c_str());
    }

    // Start processing thread
    processing_thread_ = std::thread(&MultiCameraProcessor::processingLoop, this);

    ROS_INFO("Multi-camera processor started with %zu NCS devices", ncs_devices_.size());
  }

  ~MultiCameraProcessor()
  {
    running_ = false;
    if (processing_thread_.joinable())
    {
      processing_thread_.join();
    }
  }

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg, int device_id)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (device_id >= static_cast<int>(image_queues_.size()))
    {
      image_queues_.resize(device_id + 1);
    }

    image_queues_[device_id].push(msg);

    // Limit queue size to prevent memory overflow
    if (image_queues_[device_id].size() > 10)
    {
      image_queues_[device_id].pop();
    }
  }

  void processingLoop()
  {
    ros::Rate rate(30);  // 30 Hz processing rate

    while (running_ && ros::ok())
    {
      for (size_t device_id = 0; device_id < ncs_devices_.size(); ++device_id)
      {
        sensor_msgs::ImageConstPtr msg;

        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (!image_queues_[device_id].empty())
          {
            msg = image_queues_[device_id].front();
            image_queues_[device_id].pop();
          }
        }

        if (msg)
        {
          processImage(msg, device_id);
        }
      }

      rate.sleep();
    }
  }

  void processImage(const sensor_msgs::ImageConstPtr& msg, int device_id)
  {
    try
    {
      cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

      if (image.empty() || device_id >= static_cast<int>(ncs_devices_.size()))
      {
        return;
      }

      auto& ncs = ncs_devices_[device_id];
      auto model_config = config_manager_->getModelConfig(model_name_);

      // Start timing
      auto start_time = std::chrono::high_resolution_clock::now();

      // Preprocess image
      cv::Mat processed = Utils::loadAndPreprocessImage(
        "",  // Empty path since we have the image data
        model_config->network_dimension,
        model_config->mean,
        model_config->scale
      );

      // Convert ROS image to OpenCV and preprocess
      cv::Mat resized;
      cv::resize(image, resized, cv::Size(model_config->network_dimension,
                                        model_config->network_dimension));

      // Normalize the image
      resized.convertTo(resized, CV_32FC3);
      resized /= model_config->scale;

      // Apply mean subtraction
      for (int c = 0; c < 3; ++c)
      {
        resized.forEach<cv::Vec3f>([&](cv::Vec3f& pixel, const int* position) -> void
        {
          pixel[c] -= model_config->mean[c];
        });
      }

      // Load tensor and perform inference
      ncs->loadTensor(resized);

      if (model_config->type == "classification")
      {
        ncs->classify();
        auto result = ncs->getClassificationResult();

        // Create result visualization
        cv::Mat result_image = createClassificationResultImage(image, result);

        // Record metrics
        auto end_time = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - start_time).count();

        PerformanceMetrics metrics;
        metrics.inference_time_ms = result->time_taken;
        metrics.preprocessing_time_ms = processing_time - result->time_taken;
        metrics.total_time_ms = processing_time;
        metrics.frame_count = 1;
        metrics.fps = 1000.0 / processing_time;
        metrics.operation_type = "classification";
        metrics.model_name = model_name_;
        metrics.image_width = image.cols;
        metrics.image_height = image.rows;

        performance_monitor_.recordMetrics(metrics);

        // Publish result
        sensor_msgs::ImagePtr result_msg =
          cv_bridge::CvImage(msg->header, "bgr8", result_image).toImageMsg();
        output_pubs_[device_id].publish(result_msg);
      }
      else if (model_config->type == "detection")
      {
        ncs->detect();
        auto result = ncs->getDetectionResult();

        // Create result visualization
        cv::Mat result_image = createDetectionResultImage(image, result);

        // Record metrics
        auto end_time = std::chrono::high_resolution_clock::now();
        double processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - start_time).count();

        PerformanceMetrics metrics;
        metrics.inference_time_ms = result->time_taken;
        metrics.preprocessing_time_ms = processing_time - result->time_taken;
        metrics.total_time_ms = processing_time;
        metrics.frame_count = 1;
        metrics.fps = 1000.0 / processing_time;
        metrics.operation_type = "detection";
        metrics.model_name = model_name_;
        metrics.image_width = image.cols;
        metrics.image_height = image.rows;

        performance_monitor_.recordMetrics(metrics);

        // Publish result
        sensor_msgs::ImagePtr result_msg =
          cv_bridge::CvImage(msg->header, "bgr8", result_image).toImageMsg();
        output_pubs_[device_id].publish(result_msg);
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("CV bridge exception for device %d: %s", device_id, e.what());
    }
    catch (std::exception& e)
    {
      ROS_ERROR("Processing exception for device %d: %s", device_id, e.what());
    }
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

    // Add device indicator
    cv::putText(result_image, "Device " + std::to_string(&result_image - &image),
                cv::Point(10, image.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

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

  ros::NodeHandle nh_;
  std::string model_name_;
  int num_devices_;
  std::atomic<bool> running_;
  std::vector<std::unique_ptr<NCS>> ncs_devices_;
  std::vector<image_transport::Subscriber> input_subs_;
  std::vector<image_transport::Publisher> output_pubs_;
  std::vector<std::queue<sensor_msgs::ImageConstPtr>> image_queues_;
  std::mutex queue_mutex_;
  std::thread processing_thread_;
  std::shared_ptr<ConfigManager> config_manager_;
  PerformanceMonitor performance_monitor_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_camera_example");

  if (argc < 3)
  {
    ROS_ERROR("Usage: multi_camera_example <model_name> <num_devices>");
    ROS_ERROR("Example: multi_camera_example mobilenetssd 2");
    return 1;
  }

  std::string model_name = argv[1];
  int num_devices = std::atoi(argv[2]);

  if (num_devices <= 0)
  {
    ROS_ERROR("Number of devices must be positive");
    return 1;
  }

  ros::NodeHandle nh("~");
  MultiCameraProcessor processor(nh, model_name, num_devices);

  ros::spin();

  return 0;
}
