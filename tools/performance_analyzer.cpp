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
#include <movidius_ncs_lib/performance_monitor.h>
#include <movidius_ncs_lib/config_manager.h>
#include <chrono>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <cmath>

class PerformanceAnalyzer
{
public:
  PerformanceAnalyzer(const std::string& model_name, int num_devices = 1)
    : model_name_(model_name), num_devices_(num_devices)
  {
    // Load configuration
    config_manager_ = std::make_shared<ConfigManager>("../config/" + model_name + ".yaml");

    // Initialize NCS devices
    auto model_config = config_manager_->getModelConfig(model_name);
    if (!model_config)
    {
      throw std::runtime_error("Failed to load model configuration for: " + model_name);
    }

    for (int i = 0; i < num_devices_; ++i)
    {
      try
      {
        auto ncs = std::make_unique<NCS>(
          i,  // device index
          Device::LogLevel::WARNING,
          model_config->type,
          model_config->graph_file_path,
          model_config->category_file_path,
          model_config->network_dimension,
          model_config->mean,
          model_config->scale,
          model_config->top_n
        );

        ncs_devices_.push_back(std::move(ncs));
        ROS_INFO("Initialized NCS device %d", i);
      }
      catch (const std::exception& e)
      {
        ROS_WARN("Failed to initialize NCS device %d: %s", i, e.what());
      }
    }

    if (ncs_devices_.empty())
    {
      throw std::runtime_error("No NCS devices available");
    }
  }

  void runAnalysis()
  {
    ROS_INFO("Starting comprehensive performance analysis for %s", model_name_.c_str());

    auto model_config = config_manager_->getModelConfig(model_name_);

    // Test different batch sizes
    std::vector<int> batch_sizes = {1, 2, 4, 8, 16};

    // Test different image sizes
    std::vector<int> image_sizes = {224, 300, 416, 512};

    // Test different input types
    std::vector<std::string> input_types = {"synthetic", "real"};

    std::vector<AnalysisResult> results;

    for (int batch_size : batch_sizes)
    {
      for (int image_size : image_sizes)
      {
        for (const std::string& input_type : input_types)
        {
          AnalysisResult result = runTest(batch_size, image_size, input_type, model_config);
          results.push_back(result);

          ROS_INFO("Test: batch=%d, size=%dx%d, type=%s, throughput=%.2f FPS",
                   batch_size, image_size, image_size, input_type.c_str(), result.throughput_fps);
        }
      }
    }

    // Generate comprehensive report
    generateReport(results);

    // Export results
    exportResults(results);
  }

private:
  struct AnalysisResult
  {
    int batch_size;
    int image_size;
    std::string input_type;
    double avg_inference_time_ms;
    double throughput_fps;
    double latency_ms;
    double memory_usage_mb;
    double power_consumption_w;
    std::vector<double> inference_times;
  };

  AnalysisResult runTest(int batch_size, int image_size,
                        const std::string& input_type,
                        const ModelConfig* model_config)
  {
    AnalysisResult result;
    result.batch_size = batch_size;
    result.image_size = image_size;
    result.input_type = input_type;

    // Generate test data
    std::vector<cv::Mat> test_images = generateTestData(batch_size, image_size, input_type);

    // Warm up
    for (int i = 0; i < 5; ++i)
    {
      runBatchInference(test_images, model_config);
    }

    // Actual measurement
    int num_iterations = 20;
    std::vector<double> inference_times;

    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_iterations; ++i)
    {
      double batch_time = runBatchInference(test_images, model_config);
      inference_times.push_back(batch_time);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();

    // Calculate metrics
    result.avg_inference_time_ms = std::accumulate(inference_times.begin(),
                                                  inference_times.end(), 0.0) / inference_times.size();

    result.throughput_fps = (batch_size * num_iterations * 1000.0) / total_time;
    result.latency_ms = result.avg_inference_time_ms / batch_size;
    result.memory_usage_mb = estimateMemoryUsage(image_size, batch_size);
    result.power_consumption_w = estimatePowerConsumption(result.throughput_fps);
    result.inference_times = inference_times;

    return result;
  }

  double runBatchInference(const std::vector<cv::Mat>& images,
                          const ModelConfig* model_config)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& image : images)
    {
      cv::Mat processed_image;
      cv::resize(image, processed_image,
                 cv::Size(model_config->network_dimension, model_config->network_dimension));

      processed_image.convertTo(processed_image, CV_32FC3);
      processed_image /= model_config->scale;

      // Apply mean subtraction
      for (int c = 0; c < 3; ++c)
      {
        processed_image.forEach<cv::Vec3f>([&](cv::Vec3f& pixel, const int* position) -> void
        {
          pixel[c] -= model_config->mean[c];
        });
      }

      // Use first available device
      ncs_devices_[0]->loadTensor(processed_image);

      if (model_config->type == "classification")
      {
        ncs_devices_[0]->classify();
      }
      else if (model_config->type == "detection")
      {
        ncs_devices_[0]->detect();
      }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time).count() / 1000.0;
  }

  std::vector<cv::Mat> generateTestData(int batch_size, int image_size,
                                       const std::string& input_type)
  {
    std::vector<cv::Mat> images;

    for (int i = 0; i < batch_size; ++i)
    {
      cv::Mat image(image_size, image_size, CV_8UC3);

      if (input_type == "synthetic")
      {
        // Generate synthetic patterns
        cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

        // Add some structure
        cv::circle(image, cv::Point(rand() % image_size, rand() % image_size),
                   rand() % 30 + 10, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), -1);
      }
      else
      {
        // Use real images if available, otherwise fallback to synthetic
        std::vector<std::string> real_images;
        cv::glob("../data/images/*.jpg", real_images, false);

        if (!real_images.empty())
        {
          cv::Mat real_image = cv::imread(real_images[i % real_images.size()]);
          if (!real_image.empty())
          {
            cv::resize(real_image, image, cv::Size(image_size, image_size));
          }
        }
        else
        {
          cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        }
      }

      images.push_back(image);
    }

    return images;
  }

  double estimateMemoryUsage(int image_size, int batch_size)
  {
    // Rough estimation of memory usage
    double image_memory = image_size * image_size * 3 * sizeof(uint8_t) * batch_size / (1024.0 * 1024.0);
    double tensor_memory = image_size * image_size * 3 * sizeof(float) * batch_size / (1024.0 * 1024.0);
    double model_memory = 50.0;  // Approximate model memory usage in MB

    return image_memory + tensor_memory + model_memory;
  }

  double estimatePowerConsumption(double throughput_fps)
  {
    // Rough estimation based on typical NCS power consumption
    // NCS typically consumes 1-3W depending on load
    double base_power = 1.0;  // Base power consumption
    double power_per_fps = 0.01;  // Additional power per FPS

    return base_power + (throughput_fps * power_per_fps);
  }

  void generateReport(const std::vector<AnalysisResult>& results)
  {
    ROS_INFO("\n=========================================");
    ROS_INFO("Performance Analysis Report");
    ROS_INFO("=========================================");

    // Find best configurations
    auto best_throughput = std::max_element(results.begin(), results.end(),
      [](const AnalysisResult& a, const AnalysisResult& b) {
        return a.throughput_fps < b.throughput_fps;
      });

    auto best_latency = std::min_element(results.begin(), results.end(),
      [](const AnalysisResult& a, const AnalysisResult& b) {
        return a.latency_ms < b.latency_ms;
      });

    auto best_efficiency = std::max_element(results.begin(), results.end(),
      [](const AnalysisResult& a, const AnalysisResult& b) {
        double efficiency_a = a.throughput_fps / a.power_consumption_w;
        double efficiency_b = b.throughput_fps / b.power_consumption_w;
        return efficiency_a < efficiency_b;
      });

    ROS_INFO("\nBest Throughput Configuration:");
    ROS_INFO("  Batch Size: %d", best_throughput->batch_size);
    ROS_INFO("  Image Size: %dx%d", best_throughput->image_size, best_throughput->image_size);
    ROS_INFO("  Input Type: %s", best_throughput->input_type.c_str());
    ROS_INFO("  Throughput: %.2f FPS", best_throughput->throughput_fps);
    ROS_INFO("  Latency: %.2f ms", best_throughput->latency_ms);

    ROS_INFO("\nBest Latency Configuration:");
    ROS_INFO("  Batch Size: %d", best_latency->batch_size);
    ROS_INFO("  Image Size: %dx%d", best_latency->image_size, best_latency->image_size);
    ROS_INFO("  Input Type: %s", best_latency->input_type.c_str());
    ROS_INFO("  Latency: %.2f ms", best_latency->latency_ms);
    ROS_INFO("  Throughput: %.2f FPS", best_latency->throughput_fps);

    ROS_INFO("\nBest Power Efficiency Configuration:");
    ROS_INFO("  Batch Size: %d", best_efficiency->batch_size);
    ROS_INFO("  Image Size: %dx%d", best_efficiency->image_size, best_efficiency->image_size);
    ROS_INFO("  Input Type: %s", best_efficiency->input_type.c_str());
    ROS_INFO("  Efficiency: %.2f FPS/W", best_efficiency->throughput_fps / best_efficiency->power_consumption_w);
  }

  void exportResults(const std::vector<AnalysisResult>& results)
  {
    std::ofstream file("performance_analysis_" + model_name_ + ".csv");

    if (!file.is_open())
    {
      ROS_WARN("Failed to open output file for results");
      return;
    }

    file << "BatchSize,ImageSize,InputType,AvgInferenceTime,ThroughputFPS,Latency,MemoryUsageMB,PowerConsumptionW\n";

    for (const auto& result : results)
    {
      file << result.batch_size << ","
           << result.image_size << ","
           << result.input_type << ","
           << result.avg_inference_time_ms << ","
           << result.throughput_fps << ","
           << result.latency_ms << ","
           << result.memory_usage_mb << ","
           << result.power_consumption_w << "\n";
    }

    ROS_INFO("Results exported to performance_analysis_%s.csv", model_name_.c_str());
  }

  std::string model_name_;
  int num_devices_;
  std::vector<std::unique_ptr<NCS>> ncs_devices_;
  std::shared_ptr<ConfigManager> config_manager_;
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: performance_analyzer <model_name> [num_devices]" << std::endl;
    std::cerr << "Example: performance_analyzer mobilenetssd 1" << std::endl;
    return 1;
  }

  std::string model_name = argv[1];
  int num_devices = 1;

  if (argc >= 3)
  {
    num_devices = std::atoi(argv[2]);
  }

  try
  {
    PerformanceAnalyzer analyzer(model_name, num_devices);
    analyzer.runAnalysis();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Performance analysis failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
