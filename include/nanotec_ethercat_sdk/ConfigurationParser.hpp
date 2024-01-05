#pragma once

#include <yaml-cpp/yaml.h>

#include <message_logger/message_logger.hpp>
#include <string>

#include "nanotec_ethercat_sdk/Configuration.hpp"

namespace nanotec {
/*!
 * @brief	Read configuration data from a yaml file
 */
class ConfigurationParser {
 public:
  /*!
   * no default constructor
   */
  ConfigurationParser() = delete;

  /*!
   * @brief	Constructor
   * Using an absolute path may be preferable, depending on the way the final
   * program is being run.
   * @param filename	The path to the configuration file
   */
  explicit ConfigurationParser(const std::string& filename);

  /*!
   * @brief	Constructor
   * This enables the use of a yaml node instead of a yaml file for the
   * configuration. This is useful for the creation of nested config files
   * @param configNode	The yaml node containing the configuration data
   */
  explicit ConfigurationParser(YAML::Node configNode);

  /*!
   * @brief	return the configuration
   * The Configuration object is filled according to the specified configuration
   * file. Any missing parameters will be automatically filled with well tested
   * default values.
   * @return	A configured nanotec::ethercat::Configuration object
   */
  Configuration getConfiguration() const;

 private:
  /*!
   * @brief	Parse the configuration data
   * @param configNode	yaml node containing the config data
   */
  void parseConfiguration(YAML::Node configNode);
  /*!
   * The Configuration object
   */
  Configuration configuration_;
};

}  // namespace nanotec
