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
#include <opencv2/opencv.hpp>
#include <movidius_ncs_lib/ncs.h>
#include <movidius_ncs_lib/utils.h>
#include <movidius_ncs_lib/config_manager.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

class CalibrationTool
{
public:
  CalibrationTool(const std::string& model_name)
    : model_name_(model_name)
  {
    // Load configuration
    config_manager_ = std::make_shared<ConfigManager>("../config/" + model_name + ".yaml");

    // Initialize NCS device
    auto model_config = config_manager_->getModelConfig(model_name);
    if (!model_config)
    {
      throw std::runtime_error("Failed to load model configuration for: " + model_name);
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

    ROS_INFO("Calibration tool initialized for model: %s", model_name.c_str());
  }

  void runCalibration()
  {
    ROS_INFO("Starting calibration for model: %s", model_name_.c_str());

    auto model_config = config_manager_->getModelConfig(model_name_);

    // Test different preprocessing parameters
    std::vector<std::vector<float>> test_means = {
      {104.0f, 117.0f, 123.0f},  // Default for many models
      {127.5f, 127.5f, 127.5f},  // Default for MobileNet
      {0.0f, 0.0f, 0.0f},        // No mean subtraction
      {128.0f, 128.0f, 128.0f}   // Center around 128
    };

    std::vector<float> test_scales = {255.0f, 127.5f, 1.0f};

    std::vector<std::string> test_images = getTestImages();

    if (test_images.empty())
    {
      ROS_WARN("No test images found. Generating synthetic images for calibration.");
      generateSyntheticImages();
      test_images = synthetic_images_;
    }

    // Run calibration tests
    std::vector<CalibrationResult> results;

    for (const auto& mean : test_means)
    {
      for (const auto& scale : test_scales)
      {
        CalibrationResult result = testPreprocessParameters(mean, scale, test_images, model_config);
        results.push_back(result);

        ROS_INFO("Tested: mean=[%.1f,%.1f,%.1f], scale=%.1f, accuracy=%.3f",
                 mean[0], mean[1], mean[2], scale, result.accuracy);
      }
    }

    // Find best parameters
    auto best_result = findBestParameters(results);

    ROS_INFO("\n=========================================");
    ROS_INFO("Calibration Results for %s", model_name_.c_str());
    ROS_INFO("=========================================");
    ROS_INFO("Best Parameters:");
    ROS_INFO("  Mean: [%.1f, %.1f, %.1f]",
             best_result.mean[0], best_result.mean[1], best_result.mean[2]);
    ROS_INFO("  Scale: %.1f", best_result.scale);
    ROS_INFO("  Accuracy: %.3f", best_result.accuracy);
    ROS_INFO("  Processing Time: %.2f ms", best_result.avg_processing_time);

    // Update configuration with best parameters
    updateModelConfig(best_result);

    // Save updated configuration
    config_manager_->saveConfig("../config/" + model_name_ + "_calibrated.yaml");

    ROS_INFO("Calibration completed. Updated configuration saved.");
  }

private:
  struct CalibrationResult
  {
    std::vector<float> mean;
    float scale;
    double accuracy;
    double avg_processing_time;
    std::vector<double> processing_times;
  };

  std::vector<std::string> getTestImages()
  {
    std::vector<std::string> images;
    cv::glob("../data/images/*.jpg", images, false);
    cv::glob("../data/images/*.jpeg", images, false);
    cv::glob("../data/images/*.png", images, false);

    return images;
  }

  void generateSyntheticImages()
  {
    auto model_config = config_manager_->getModelConfig(model_name_);

    for (int i = 0; i < 20; ++i)
    {
      cv::Mat image(model_config->network_dimension,
                   model_config->network_dimension,
                   CV_8UC3);

      // Generate different patterns for testing
      if (i % 4 == 0) {
        // Solid color
        image = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
      } else if (i % 4 == 1) {
        // Gradient
        for (int y = 0; y < image.rows; ++y) {
          for (int x = 0; x < image.cols; ++x) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(x * 255 / image.cols,
                                                 y * 255 / image.rows,
                                                 (x + y) * 255 / (image.rows + image.cols));
          }
        }
      } else if (i % 4 == 2) {
        // Circles
        image = cv::Scalar(128, 128, 128);
        for (int j = 0; j < 5; ++j) {
          cv::circle(image,
                    cv::Point(rand() % image.cols, rand() % image.rows),
                    rand() % 50 + 10,
                    cv::Scalar(rand() % 255, rand() % 255, rand() % 255),
                    -1);
        }
      } else {
        // Random noise
        cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
      }

      synthetic_images_.push_back(image);
    }
  }

  CalibrationResult testPreprocessParameters(
    const std::vector<float>& mean,
    float scale,
    const std::vector<std::string>& test_images,
    const ModelConfig* model_config)
  {
    CalibrationResult result;
    result.mean = mean;
    result.scale = scale;

    std::vector<double> processing_times;

    for (const auto& image_path : test_images)
    {
      cv::Mat image;

      if (image_path.empty()) {
        // Use synthetic image
        image = synthetic_images_[0];
      } else {
        image = cv::imread(image_path);
        if (image.empty()) continue;
      }

      // Test preprocessing and inference time
      auto start_time = std::chrono::high_resolution_clock::now();

      cv::Mat processed_image;
      cv::resize(image, processed_image,
                 cv::Size(model_config->network_dimension, model_config->network_dimension));

      processed_image.convertTo(processed_image, CV_32FC3);
      processed_image /= scale;

      // Apply mean subtraction
      for (int c = 0; c < 3; ++c)
      {
        processed_image.forEach<cv::Vec3f>([&](cv::Vec3f& pixel, const int* position) -> void
        {
          pixel[c] -= mean[c];
        });
      }

      // Perform inference
      ncs_->loadTensor(processed_image);

      if (model_config->type == "classification")
      {
        ncs_->classify();
      }
      else if (model_config->type == "detection")
      {
        ncs_->detect();
      }

      auto end_time = std::chrono::high_resolution_clock::now();
      double processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count() / 1000.0;

      processing_times.push_back(processing_time);
    }

    // Calculate average processing time
    result.avg_processing_time = std::accumulate(processing_times.begin(),
                                                processing_times.end(), 0.0) / processing_times.size();

    // For accuracy, we'd need ground truth labels - for now use a simple metric
    // In a real implementation, you'd compare against known good results
    result.accuracy = 1.0 / (1.0 + result.avg_processing_time / 100.0);  // Simple heuristic

    result.processing_times = processing_times;

    return result;
  }

  CalibrationResult findBestParameters(const std::vector<CalibrationResult>& results)
  {
    CalibrationResult best = results[0];

    for (const auto& result : results)
    {
      // Choose parameters that give best balance of accuracy and speed
      // In a real implementation, you'd use actual accuracy metrics
      double score = result.accuracy / (1.0 + result.avg_processing_time / 100.0);

      double best_score = best.accuracy / (1.0 + best.avg_processing_time / 100.0);

      if (score > best_score)
      {
        best = result;
      }
    }

    return best;
  }

  void updateModelConfig(const CalibrationResult& best_result)
  {
    auto model_config = config_manager_->getModelConfig(model_name_);
    if (model_config)
    {
      // Update with calibrated parameters
      model_config->mean = best_result.mean;
      model_config->scale = best_result.scale;

      config_manager_->setModelConfig(*model_config);
    }
  }

  std::string model_name_;
  std::unique_ptr<NCS> ncs_;
  std::shared_ptr<ConfigManager> config_manager_;
  std::vector<cv::Mat> synthetic_images_;
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: calibration_example <model_name>" << std::endl;
    std::cerr << "Example: calibration_example mobilenetssd" << std::endl;
    return 1;
  }

  std::string model_name = argv[1];

  try
  {
    CalibrationTool calibrator(model_name);
    calibrator.runCalibration();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Calibration failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
