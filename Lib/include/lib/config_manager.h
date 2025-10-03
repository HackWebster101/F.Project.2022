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

#ifndef MOVIDIUS_NCS_LIB_CONFIG_MANAGER_H_
#define MOVIDIUS_NCS_LIB_CONFIG_MANAGER_H_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace movidius_ncs_lib
{
/**
 * @brief Model configuration structure
 */
struct ModelConfig
{
  std::string name;
  std::string type;  // "classification" or "detection"
  std::string graph_file_path;
  std::string category_file_path;
  int network_dimension;
  std::vector<float> mean;
  float scale;
  int top_n;
  std::string description;
};

/**
 * @brief Device configuration structure
 */
struct DeviceConfig
{
  int device_index;
  Device::LogLevel log_level;
  bool enable_profiling;
  int max_inferences;
  std::string thermal_monitoring;
};

/**
 * @brief Configuration manager for NCS operations
 */
class ConfigManager
{
public:
  /**
   * @brief Constructor
   * @param config_file_path Path to YAML configuration file
   */
  explicit ConfigManager(const std::string& config_file_path = "");

  /**
   * @brief Destructor
   */
  ~ConfigManager();

  /**
   * @brief Load configuration from file
   * @param config_file_path Path to configuration file
   * @return true if successful, false otherwise
   */
  bool loadConfig(const std::string& config_file_path);

  /**
   * @brief Save current configuration to file
   * @param config_file_path Path to save configuration file
   * @return true if successful, false otherwise
   */
  bool saveConfig(const std::string& config_file_path) const;

  /**
   * @brief Get model configuration by name
   * @param model_name Name of the model
   * @return Model configuration, or nullptr if not found
   */
  ModelConfig* getModelConfig(const std::string& model_name);

  /**
   * @brief Get all available model configurations
   * @return Vector of model configurations
   */
  std::vector<ModelConfig> getAllModelConfigs() const;

  /**
   * @brief Add or update model configuration
   * @param config Model configuration to add/update
   */
  void setModelConfig(const ModelConfig& config);

  /**
   * @brief Remove model configuration
   * @param model_name Name of the model to remove
   * @return true if removed, false if not found
   */
  bool removeModelConfig(const std::string& model_name);

  /**
   * @brief Get device configuration
   * @return Device configuration
   */
  DeviceConfig getDeviceConfig() const;

  /**
   * @brief Set device configuration
   * @param config Device configuration to set
   */
  void setDeviceConfig(const DeviceConfig& config);

  /**
   * @brief Get configuration value by key
   * @param key Configuration key
   * @param default_value Default value if key not found
   * @return Configuration value
   */
  template<typename T>
  T getConfigValue(const std::string& key, const T& default_value = T()) const;

  /**
   * @brief Set configuration value
   * @param key Configuration key
   * @param value Configuration value
   */
  template<typename T>
  void setConfigValue(const std::string& key, const T& value);

  /**
   * @brief Check if configuration is valid
   * @return true if configuration is valid, false otherwise
   */
  bool isValid() const;

  /**
   * @brief Get list of available model names
   * @return Vector of model names
   */
  std::vector<std::string> getAvailableModels() const;

  /**
   * @brief Create default configuration
   * @return true if successful, false otherwise
   */
  bool createDefaultConfig();

private:
  YAML::Node config_yaml_;
  std::map<std::string, ModelConfig> model_configs_;
  DeviceConfig device_config_;
  bool config_loaded_;

  /**
   * @brief Parse model configuration from YAML
   * @param node YAML node containing model configuration
   * @return Parsed model configuration
   */
  ModelConfig parseModelConfig(const YAML::Node& node);

  /**
   * @brief Parse device configuration from YAML
   * @param node YAML node containing device configuration
   * @return Parsed device configuration
   */
  DeviceConfig parseDeviceConfig(const YAML::Node& node);

  /**
   * @brief Convert model configuration to YAML
   * @param config Model configuration to convert
   * @return YAML node
   */
  YAML::Node modelConfigToYaml(const ModelConfig& config) const;

  /**
   * @brief Convert device configuration to YAML
   * @param config Device configuration to convert
   * @return YAML node
   */
  YAML::Node deviceConfigToYaml(const DeviceConfig& config) const;
};

}  // namespace movidius_ncs_lib

#endif  // MOVIDIUS_NCS_LIB_CONFIG_MANAGER_H_
