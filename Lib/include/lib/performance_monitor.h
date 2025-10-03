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

#ifndef MOVIDIUS_NCS_LIB_PERFORMANCE_MONITOR_H_
#define MOVIDIUS_NCS_LIB_PERFORMANCE_MONITOR_H_

#include <chrono>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

namespace movidius_ncs_lib
{
/**
 * @brief Performance metrics for NCS operations
 */
struct PerformanceMetrics
{
  double inference_time_ms;
  double preprocessing_time_ms;
  double postprocessing_time_ms;
  double total_time_ms;
  int frame_count;
  double fps;
  std::string operation_type;  // "classification" or "detection"
  std::string model_name;
  int image_width;
  int image_height;
};

/**
 * @brief Performance monitoring and benchmarking utility
 */
class PerformanceMonitor
{
public:
  /**
   * @brief Constructor
   * @param log_file_path Path to log file for performance data
   */
  explicit PerformanceMonitor(const std::string& log_file_path = "");

  /**
   * @brief Destructor
   */
  ~PerformanceMonitor();

  /**
   * @brief Start timing for a specific operation
   * @param operation_name Name of the operation to time
   */
  void startTimer(const std::string& operation_name);

  /**
   * @brief Stop timing for a specific operation
   * @param operation_name Name of the operation to stop timing
   */
  void stopTimer(const std::string& operation_name);

  /**
   * @brief Record performance metrics
   * @param metrics Performance metrics to record
   */
  void recordMetrics(const PerformanceMetrics& metrics);

  /**
   * @brief Get average performance metrics
   * @return Average metrics across all recorded operations
   */
  PerformanceMetrics getAverageMetrics() const;

  /**
   * @brief Get performance metrics for a specific model
   * @param model_name Name of the model
   * @return Average metrics for the specified model
   */
  PerformanceMetrics getModelMetrics(const std::string& model_name) const;

  /**
   * @brief Generate performance report
   * @return Formatted performance report string
   */
  std::string generateReport() const;

  /**
   * @brief Export performance data to CSV file
   * @param filename Output CSV filename
   */
  void exportToCSV(const std::string& filename) const;

  /**
   * @brief Reset all performance data
   */
  void reset();

  /**
   * @brief Enable/disable logging to file
   * @param enable Whether to enable file logging
   */
  void enableFileLogging(bool enable);

private:
  struct Timer
  {
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    bool running = false;
  };

  std::vector<PerformanceMetrics> metrics_history_;
  std::map<std::string, Timer> active_timers_;
  std::ofstream log_file_;
  bool file_logging_enabled_;

  /**
   * @brief Calculate elapsed time in milliseconds
   * @param start Start time point
   * @param end End time point
   * @return Elapsed time in milliseconds
   */
  double calculateElapsedMs(const std::chrono::high_resolution_clock::time_point& start,
                           const std::chrono::high_resolution_clock::time_point& end) const;
};

}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_PERFORMANCE_MONITOR_H_
