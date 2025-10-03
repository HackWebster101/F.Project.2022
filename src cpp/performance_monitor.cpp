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

#include "movidius_ncs_lib/performance_monitor.h"
#include <algorithm>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace movidius_ncs_lib
{
PerformanceMonitor::PerformanceMonitor(const std::string& log_file_path)
  : file_logging_enabled_(!log_file_path.empty())
{
  if (file_logging_enabled_)
  {
    log_file_.open(log_file_path, std::ios::out | std::ios::app);
    if (log_file_.is_open())
    {
      log_file_ << "Timestamp,Operation,Model,InferenceTime,PreprocessingTime,"
                << "PostprocessingTime,TotalTime,FPS,ImageSize\n";
    }
  }
}

PerformanceMonitor::~PerformanceMonitor()
{
  if (log_file_.is_open())
  {
    log_file_.close();
  }
}

void PerformanceMonitor::startTimer(const std::string& operation_name)
{
  auto& timer = active_timers_[operation_name];
  timer.start = std::chrono::high_resolution_clock::now();
  timer.running = true;
}

void PerformanceMonitor::stopTimer(const std::string& operation_name)
{
  auto it = active_timers_.find(operation_name);
  if (it != active_timers_.end() && it->second.running)
  {
    it->second.end = std::chrono::high_resolution_clock::now();
    it->second.running = false;
  }
}

void PerformanceMonitor::recordMetrics(const PerformanceMetrics& metrics)
{
  metrics_history_.push_back(metrics);

  // Log to file if enabled
  if (file_logging_enabled_ && log_file_.is_open())
  {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    log_file_ << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << ","
              << metrics.operation_type << ","
              << metrics.model_name << ","
              << std::fixed << std::setprecision(2)
              << metrics.inference_time_ms << ","
              << metrics.preprocessing_time_ms << ","
              << metrics.postprocessing_time_ms << ","
              << metrics.total_time_ms << ","
              << metrics.fps << ","
              << metrics.image_width << "x" << metrics.image_height << "\n";
  }

  // Keep only last 1000 records to prevent memory bloat
  if (metrics_history_.size() > 1000)
  {
    metrics_history_.erase(metrics_history_.begin(),
                          metrics_history_.begin() + (metrics_history_.size() - 1000));
  }
}

PerformanceMetrics PerformanceMonitor::getAverageMetrics() const
{
  if (metrics_history_.empty())
  {
    return PerformanceMetrics{0, 0, 0, 0, 0, 0, "", "", 0, 0};
  }

  PerformanceMetrics avg = {0, 0, 0, 0, 0, 0, "", "", 0, 0};

  for (const auto& metrics : metrics_history_)
  {
    avg.inference_time_ms += metrics.inference_time_ms;
    avg.preprocessing_time_ms += metrics.preprocessing_time_ms;
    avg.postprocessing_time_ms += metrics.postprocessing_time_ms;
    avg.total_time_ms += metrics.total_time_ms;
    avg.fps += metrics.fps;
    avg.frame_count += metrics.frame_count;
  }

  size_t count = metrics_history_.size();
  avg.inference_time_ms /= count;
  avg.preprocessing_time_ms /= count;
  avg.postprocessing_time_ms /= count;
  avg.total_time_ms /= count;
  avg.fps /= count;
  avg.frame_count /= count;

  return avg;
}

PerformanceMetrics PerformanceMonitor::getModelMetrics(const std::string& model_name) const
{
  std::vector<PerformanceMetrics> model_metrics;

  for (const auto& metrics : metrics_history_)
  {
    if (metrics.model_name == model_name)
    {
      model_metrics.push_back(metrics);
    }
  }

  if (model_metrics.empty())
  {
    return PerformanceMetrics{0, 0, 0, 0, 0, 0, "", model_name, 0, 0};
  }

  PerformanceMetrics avg = {0, 0, 0, 0, 0, 0, "", model_name, 0, 0};

  for (const auto& metrics : model_metrics)
  {
    avg.inference_time_ms += metrics.inference_time_ms;
    avg.preprocessing_time_ms += metrics.preprocessing_time_ms;
    avg.postprocessing_time_ms += metrics.postprocessing_time_ms;
    avg.total_time_ms += metrics.total_time_ms;
    avg.fps += metrics.fps;
    avg.frame_count += metrics.frame_count;
    if (metrics.image_width > 0 && metrics.image_height > 0)
    {
      avg.image_width = metrics.image_width;
      avg.image_height = metrics.image_height;
    }
  }

  size_t count = model_metrics.size();
  avg.inference_time_ms /= count;
  avg.preprocessing_time_ms /= count;
  avg.postprocessing_time_ms /= count;
  avg.total_time_ms /= count;
  avg.fps /= count;
  avg.frame_count /= count;

  return avg;
}

std::string PerformanceMonitor::generateReport() const
{
  std::stringstream report;

  report << "=========================================\n";
  report << "NCS Performance Monitor Report\n";
  report << "=========================================\n\n";

  if (metrics_history_.empty())
  {
    report << "No performance data available.\n";
    return report.str();
  }

  // Overall statistics
  PerformanceMetrics avg = getAverageMetrics();
  report << "Overall Statistics:\n";
  report << "  Total Operations: " << metrics_history_.size() << "\n";
  report << "  Average Inference Time: " << std::fixed << std::setprecision(2)
         << avg.inference_time_ms << " ms\n";
  report << "  Average Preprocessing Time: " << avg.preprocessing_time_ms << " ms\n";
  report << "  Average Postprocessing Time: " << avg.postprocessing_time_ms << " ms\n";
  report << "  Average Total Time: " << avg.total_time_ms << " ms\n";
  report << "  Average FPS: " << avg.fps << "\n\n";

  // Model-specific statistics
  std::vector<std::string> models;
  for (const auto& metrics : metrics_history_)
  {
    if (std::find(models.begin(), models.end(), metrics.model_name) == models.end())
    {
      models.push_back(metrics.model_name);
    }
  }

  report << "Model-Specific Statistics:\n";
  for (const auto& model : models)
  {
    PerformanceMetrics model_avg = getModelMetrics(model);
    report << "  " << model << ":\n";
    report << "    Average Inference Time: " << model_avg.inference_time_ms << " ms\n";
    report << "    Average FPS: " << model_avg.fps << "\n";
    report << "    Operations: " << model_avg.frame_count << "\n";
  }

  return report.str();
}

void PerformanceMonitor::exportToCSV(const std::string& filename) const
{
  std::ofstream file(filename);

  if (!file.is_open())
  {
    throw std::runtime_error("Failed to open file for CSV export: " + filename);
  }

  file << "Timestamp,Operation,Model,InferenceTime,PreprocessingTime,"
       << "PostprocessingTime,TotalTime,FPS,ImageSize\n";

  for (const auto& metrics : metrics_history_)
  {
    file << "2025-01-01 00:00:00,"  // Placeholder timestamp
         << metrics.operation_type << ","
         << metrics.model_name << ","
         << std::fixed << std::setprecision(2)
         << metrics.inference_time_ms << ","
         << metrics.preprocessing_time_ms << ","
         << metrics.postprocessing_time_ms << ","
         << metrics.total_time_ms << ","
         << metrics.fps << ","
         << metrics.image_width << "x" << metrics.image_height << "\n";
  }
}

void PerformanceMonitor::reset()
{
  metrics_history_.clear();
  active_timers_.clear();
}

void PerformanceMonitor::enableFileLogging(bool enable)
{
  if (enable && !file_logging_enabled_)
  {
    // File logging was disabled, do nothing for now
    file_logging_enabled_ = false;
  }
  else if (!enable && file_logging_enabled_)
  {
    // Close log file if open
    if (log_file_.is_open())
    {
      log_file_.close();
    }
    file_logging_enabled_ = false;
  }
}

double PerformanceMonitor::calculateElapsedMs(
  const std::chrono::high_resolution_clock::time_point& start,
  const std::chrono::high_resolution_clock::time_point& end) const
{
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  return duration.count() / 1000.0;
}

}  // namespace movidius_ncs_lib
