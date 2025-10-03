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

#include "movidius_ncs_lib/config_manager.h"
#include <fstream>
#include <iostream>
#include <algorithm>

namespace movidius_ncs_lib
{
ConfigManager::ConfigManager(const std::string& config_file_path)
  : config_loaded_(false)
{
  if (!config_file_path.empty())
  {
    loadConfig(config_file_path);
  }
  else
  {
    createDefaultConfig();
  }
}

ConfigManager::~ConfigManager()
{
}

bool ConfigManager::loadConfig(const std::string& config_file_path)
{
  try
  {
    config_yaml_ = YAML::LoadFile(config_file_path);

    // Parse model configurations
    if (config_yaml_["models"] && config_yaml_["models"].IsSequence())
    {
      for (const auto& model_node : config_yaml_["models"])
      {
        ModelConfig config = parseModelConfig(model_node);
        model_configs_[config.name] = config;
      }
    }

    // Parse device configuration
    if (config_yaml_["device"])
    {
      device_config_ = parseDeviceConfig(config_yaml_["device"]);
    }

    config_loaded_ = true;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to load configuration: " << e.what() << std::endl;
    config_loaded_ = false;
    return false;
  }
}

bool ConfigManager::saveConfig(const std::string& config_file_path) const
{
  try
  {
    YAML::Node root;

    // Add model configurations
    for (const auto& pair : model_configs_)
    {
      root["models"].push_back(modelConfigToYaml(pair.second));
    }

    // Add device configuration
    root["device"] = deviceConfigToYaml(device_config_);

    // Add additional configuration values
    for (const auto& pair : config_yaml_)
    {
      if (pair.first.as<std::string>() != "models" &&
          pair.first.as<std::string>() != "device")
      {
        root[pair.first] = pair.second;
      }
    }

    std::ofstream file(config_file_path);
    if (!file.is_open())
    {
      return false;
    }

    file << root;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to save configuration: " << e.what() << std::endl;
    return false;
  }
}

ModelConfig* ConfigManager::getModelConfig(const std::string& model_name)
{
  auto it = model_configs_.find(model_name);
  if (it != model_configs_.end())
  {
    return &(it->second);
  }
  return nullptr;
}

std::vector<ModelConfig> ConfigManager::getAllModelConfigs() const
{
  std::vector<ModelConfig> configs;
  for (const auto& pair : model_configs_)
  {
    configs.push_back(pair.second);
  }
  return configs;
}

void ConfigManager::setModelConfig(const ModelConfig& config)
{
  model_configs_[config.name] = config;
}

bool ConfigManager::removeModelConfig(const std::string& model_name)
{
  auto it = model_configs_.find(model_name);
  if (it != model_configs_.end())
  {
    model_configs_.erase(it);
    return true;
  }
  return false;
}

DeviceConfig ConfigManager::getDeviceConfig() const
{
  return device_config_;
}

void ConfigManager::setDeviceConfig(const DeviceConfig& config)
{
  device_config_ = config;
}

template<typename T>
T ConfigManager::getConfigValue(const std::string& key, const T& default_value) const
{
  try
  {
    if (config_yaml_[key])
    {
      return config_yaml_[key].as<T>();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to get config value for key '" << key << "': " << e.what() << std::endl;
  }
  return default_value;
}

template<typename T>
void ConfigManager::setConfigValue(const std::string& key, const T& value)
{
  config_yaml_[key] = value;
}

bool ConfigManager::isValid() const
{
  return config_loaded_ && !model_configs_.empty();
}

std::vector<std::string> ConfigManager::getAvailableModels() const
{
  std::vector<std::string> models;
  for (const auto& pair : model_configs_)
  {
    models.push_back(pair.first);
  }
  return models;
}

bool ConfigManager::createDefaultConfig()
{
  try
  {
    // Create default model configurations
    ModelConfig alexnet;
    alexnet.name = "alexnet";
    alexnet.type = "classification";
    alexnet.network_dimension = 227;
    alexnet.mean = {104.0f, 117.0f, 123.0f};
    alexnet.scale = 255.0f;
    alexnet.top_n = 5;
    alexnet.description = "AlexNet for image classification";
    model_configs_[alexnet.name] = alexnet;

    ModelConfig googlenet;
    googlenet.name = "googlenet";
    googlenet.type = "classification";
    googlenet.network_dimension = 224;
    googlenet.mean = {104.0f, 117.0f, 123.0f};
    googlenet.scale = 255.0f;
    googlenet.top_n = 5;
    googlenet.description = "GoogleNet for image classification";
    model_configs_[googlenet.name] = googlenet;

    ModelConfig mobilenetssd;
    mobilenetssd.name = "mobilenetssd";
    mobilenetssd.type = "detection";
    mobilenetssd.network_dimension = 300;
    mobilenetssd.mean = {127.5f, 127.5f, 127.5f};
    mobilenetssd.scale = 127.5f;
    mobilenetssd.top_n = 5;
    mobilenetssd.description = "MobileNet SSD for object detection";
    model_configs_[mobilenetssd.name] = mobilenetssd;

    // Create default device configuration
    device_config_.device_index = 0;
    device_config_.log_level = Device::LogLevel::WARNING;
    device_config_.enable_profiling = false;
    device_config_.max_inferences = 1000;
    device_config_.thermal_monitoring = "enabled";

    config_loaded_ = true;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to create default configuration: " << e.what() << std::endl;
    return false;
  }
}

ModelConfig ConfigManager::parseModelConfig(const YAML::Node& node)
{
  ModelConfig config;

  if (node["name"]) config.name = node["name"].as<std::string>();
  if (node["type"]) config.type = node["type"].as<std::string>();
  if (node["network_dimension"]) config.network_dimension = node["network_dimension"].as<int>();

  if (node["mean"] && node["mean"].IsSequence())
  {
    config.mean.clear();
    for (const auto& val : node["mean"])
    {
      config.mean.push_back(val.as<float>());
    }
  }

  if (node["scale"]) config.scale = node["scale"].as<float>();
  if (node["top_n"]) config.top_n = node["top_n"].as<int>();
  if (node["description"]) config.description = node["description"].as<std::string>();

  return config;
}

DeviceConfig ConfigManager::parseDeviceConfig(const YAML::Node& node)
{
  DeviceConfig config;

  if (node["device_index"]) config.device_index = node["device_index"].as<int>();
  if (node["log_level"])
  {
    std::string log_level_str = node["log_level"].as<std::string>();
    if (log_level_str == "DEBUG") config.log_level = Device::LogLevel::DEBUG;
    else if (log_level_str == "INFO") config.log_level = Device::LogLevel::INFO;
    else if (log_level_str == "WARNING") config.log_level = Device::LogLevel::WARNING;
    else if (log_level_str == "ERROR") config.log_level = Device::LogLevel::ERROR;
    else config.log_level = Device::LogLevel::WARNING;
  }

  if (node["enable_profiling"]) config.enable_profiling = node["enable_profiling"].as<bool>();
  if (node["max_inferences"]) config.max_inferences = node["max_inferences"].as<int>();
  if (node["thermal_monitoring"]) config.thermal_monitoring = node["thermal_monitoring"].as<std::string>();

  return config;
}

YAML::Node ConfigManager::modelConfigToYaml(const ModelConfig& config) const
{
  YAML::Node node;

  node["name"] = config.name;
  node["type"] = config.type;
  node["network_dimension"] = config.network_dimension;

  for (float val : config.mean)
  {
    node["mean"].push_back(val);
  }

  node["scale"] = config.scale;
  node["top_n"] = config.top_n;
  node["description"] = config.description;

  return node;
}

YAML::Node ConfigManager::deviceConfigToYaml(const DeviceConfig& config) const
{
  YAML::Node node;

  node["device_index"] = config.device_index;

  switch (config.log_level)
  {
    case Device::LogLevel::DEBUG:
      node["log_level"] = "DEBUG";
      break;
    case Device::LogLevel::INFO:
      node["log_level"] = "INFO";
      break;
    case Device::LogLevel::WARNING:
      node["log_level"] = "WARNING";
      break;
    case Device::LogLevel::ERROR:
      node["log_level"] = "ERROR";
      break;
  }

  node["enable_profiling"] = config.enable_profiling;
  node["max_inferences"] = config.max_inferences;
  node["thermal_monitoring"] = config.thermal_monitoring;

  return node;
}

// Template specializations for common types
template<>
std::string ConfigManager::getConfigValue<std::string>(const std::string& key,
                                                      const std::string& default_value) const
{
  try
  {
    if (config_yaml_[key])
    {
      return config_yaml_[key].as<std::string>();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to get string config value for key '" << key << "': " << e.what() << std::endl;
  }
  return default_value;
}

template<>
int ConfigManager::getConfigValue<int>(const std::string& key, const int& default_value) const
{
  try
  {
    if (config_yaml_[key])
    {
      return config_yaml_[key].as<int>();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to get int config value for key '" << key << "': " << e.what() << std::endl;
  }
  return default_value;
}

template<>
bool ConfigManager::getConfigValue<bool>(const std::string& key, const bool& default_value) const
{
  try
  {
    if (config_yaml_[key])
    {
      return config_yaml_[key].as<bool>();
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to get bool config value for key '" << key << "': " << e.what() << std::endl;
  }
  return default_value;
}

}  // namespace movidius_ncs_lib
