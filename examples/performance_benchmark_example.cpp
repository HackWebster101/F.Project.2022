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
#include <iomanip>

class PerformanceBenchmark
{
public:
  PerformanceBenchmark(const std::string& model_name, int num_iterations = 100)
    : model_name_(model_name), num_iterations_(num_iterations),
      performance_monitor_("benchmark_" + model_name + ".log")
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
      Device::LogLevel::WARNING,  // Minimal logging for benchmarking
      model_config->type,
      model_config->graph_file_path,
      model_config->category_file_path,
      model_config->network_dimension,
      model_config->mean,
      model_config->scale,
      model_config->top_n
    );

    ROS_INFO("Initialized NCS for model: %s", model_name.c_str());
  }

  void runBenchmark()
  {
    ROS_INFO("Starting benchmark for model: %s", model_name_.c_str());
    ROS_INFO("Number of iterations: %d", num_iterations_);

    auto model_config = config_manager_->getModelConfig(model_name_);

    // Generate test images or use existing ones
    std::vector<cv::Mat> test_images = generateTestImages();

    // Warm up
    ROS_INFO("Warming up...");
    for (int i = 0; i < 10; ++i)
    {
      runInference(test_images[i % test_images.size()], model_config);
    }

    // Actual benchmark
    ROS_INFO("Running benchmark...");
    std::vector<double> inference_times;
    std::vector<double> preprocessing_times;
    std::vector<double> total_times;

    for (int i = 0; i < num_iterations_; ++i)
    {
      cv::Mat test_image = test_images[i % test_images.size()];
      auto [inference_time, preprocessing_time, total_time] =
        runInference(test_image, model_config);

      inference_times.push_back(inference_time);
      preprocessing_times.push_back(preprocessing_time);
      total_times.push_back(total_time);

      if ((i + 1) % 10 == 0)
      {
        ROS_INFO("Completed %d/%d iterations", i + 1, num_iterations_);
      }
    }

    // Calculate statistics
    auto stats = calculateStatistics(inference_times);

    ROS_INFO("\n=========================================");
    ROS_INFO("Benchmark Results for %s", model_name_.c_str());
    ROS_INFO("=========================================");
    ROS_INFO("Iterations: %d", num_iterations_);
    ROS_INFO("Mean Inference Time: %.2f ms", stats.mean);
    ROS_INFO("Median Inference Time: %.2f ms", stats.median);
    ROS_INFO("Min Inference Time: %.2f ms", stats.min);
    ROS_INFO("Max Inference Time: %.2f ms", stats.max);
    ROS_INFO("Std Dev Inference Time: %.2f ms", stats.std_dev);
    ROS_INFO("95th Percentile: %.2f ms", stats.percentile_95);
    ROS_INFO("99th Percentile: %.2f ms", stats.percentile_99);
    ROS_INFO("FPS (mean): %.2f", 1000.0 / stats.mean);

    // Preprocessing stats
    auto prep_stats = calculateStatistics(preprocessing_times);
    ROS_INFO("\nPreprocessing Statistics:");
    ROS_INFO("Mean Preprocessing Time: %.2f ms", prep_stats.mean);

    // Total time stats
    auto total_stats = calculateStatistics(total_times);
    ROS_INFO("\nTotal Processing Statistics:");
    ROS_INFO("Mean Total Time: %.2f ms", total_stats.mean);
    ROS_INFO("FPS (total): %.2f", 1000.0 / total_stats.mean);

    // Record final metrics
    PerformanceMetrics metrics;
    metrics.inference_time_ms = stats.mean;
    metrics.preprocessing_time_ms = prep_stats.mean;
    metrics.total_time_ms = total_stats.mean;
    metrics.frame_count = num_iterations_;
    metrics.fps = 1000.0 / total_stats.mean;
    metrics.operation_type = model_config->type;
    metrics.model_name = model_name_;
    metrics.image_width = model_config->network_dimension;
    metrics.image_height = model_config->network_dimension;

    performance_monitor_.recordMetrics(metrics);

    // Generate report
    std::string report = performance_monitor_.generateReport();
    std::cout << "\n" << report;

    // Export to CSV
    performance_monitor_.exportToCSV("benchmark_" + model_name_ + "_results.csv");
  }

private:
  struct Statistics
  {
    double mean;
    double median;
    double min;
    double max;
    double std_dev;
    double percentile_95;
    double percentile_99;
  };

  std::vector<cv::Mat> generateTestImages()
  {
    std::vector<cv::Mat> images;
    auto model_config = config_manager_->getModelConfig(model_name_);

    // Create synthetic test images
    for (int i = 0; i < 10; ++i)
    {
      cv::Mat image(model_config->network_dimension,
                   model_config->network_dimension,
                   CV_8UC3);

      // Generate random colorful pattern
      cv::randu(image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

      // Add some geometric shapes
      cv::circle(image, cv::Point(rand() % model_config->network_dimension,
                                 rand() % model_config->network_dimension),
                 20, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), -1);

      images.push_back(image);
    }

    return images;
  }

  std::tuple<double, double, double> runInference(const cv::Mat& image,
                                                 const ModelConfig* model_config)
  {
    auto start_total = std::chrono::high_resolution_clock::now();

    // Preprocessing
    auto start_preprocessing = std::chrono::high_resolution_clock::now();

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

    auto end_preprocessing = std::chrono::high_resolution_clock::now();
    double preprocessing_time = std::chrono::duration_cast<std::chrono::microseconds>(
      end_preprocessing - start_preprocessing).count() / 1000.0;

    // Inference
    auto start_inference = std::chrono::high_resolution_clock::now();

    ncs_->loadTensor(processed_image);

    if (model_config->type == "classification")
    {
      ncs_->classify();
    }
    else if (model_config->type == "detection")
    {
      ncs_->detect();
    }

    auto end_inference = std::chrono::high_resolution_clock::now();
    double inference_time = std::chrono::duration_cast<std::chrono::microseconds>(
      end_inference - start_inference).count() / 1000.0;

    auto end_total = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration_cast<std::chrono::microseconds>(
      end_total - start_total).count() / 1000.0;

    return {inference_time, preprocessing_time, total_time};
  }

  Statistics calculateStatistics(const std::vector<double>& times)
  {
    Statistics stats;

    if (times.empty())
    {
      stats.mean = stats.median = stats.min = stats.max = stats.std_dev = 0.0;
      return stats;
    }

    // Calculate mean
    stats.mean = std::accumulate(times.begin(), times.end(), 0.0) / times.size();

    // Calculate min/max
    stats.min = *std::min_element(times.begin(), times.end());
    stats.max = *std::max_element(times.begin(), times.end());

    // Calculate median
    std::vector<double> sorted_times = times;
    std::sort(sorted_times.begin(), sorted_times.end());
    size_t mid = sorted_times.size() / 2;
    if (sorted_times.size() % 2 == 0)
    {
      stats.median = (sorted_times[mid - 1] + sorted_times[mid]) / 2.0;
    }
    else
    {
      stats.median = sorted_times[mid];
    }

    // Calculate standard deviation
    double sum_sq = 0.0;
    for (double t : times)
    {
      sum_sq += (t - stats.mean) * (t - stats.mean);
    }
    stats.std_dev = std::sqrt(sum_sq / times.size());

    // Calculate percentiles
    size_t idx_95 = static_cast<size_t>(times.size() * 0.95);
    size_t idx_99 = static_cast<size_t>(times.size() * 0.99);

    if (idx_95 < sorted_times.size())
      stats.percentile_95 = sorted_times[idx_95];
    else
      stats.percentile_95 = sorted_times.back();

    if (idx_99 < sorted_times.size())
      stats.percentile_99 = sorted_times[idx_99];
    else
      stats.percentile_99 = sorted_times.back();

    return stats;
  }

  std::string model_name_;
  int num_iterations_;
  std::unique_ptr<NCS> ncs_;
  std::shared_ptr<ConfigManager> config_manager_;
  PerformanceMonitor performance_monitor_;
};

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: performance_benchmark_example <model_name> [num_iterations]" << std::endl;
    std::cerr << "Example: performance_benchmark_example mobilenetssd 100" << std::endl;
    return 1;
  }

  std::string model_name = argv[1];
  int num_iterations = 100;

  if (argc >= 3)
  {
    num_iterations = std::atoi(argv[2]);
  }

  try
  {
    PerformanceBenchmark benchmark(model_name, num_iterations);
    benchmark.runBenchmark();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Benchmark failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
