/**
 * @file diagnostic_tools.cpp
 * @author Daniel Campos(daniel.f.campos@inesctec.pt)
 *
 * @brief
 * Diagnostic Library to monitor system status
 *
 * @version 0.1
 * @date 2020-02-17
 *
 * @copyright Copyright (c) 2020
 *
 */

#include "diagnostic_tools.h"

namespace diag_tools {

  DiagnosticTools::DiagnosticTools(ros::NodeHandle& n, const std::string name, const std::string hardware_id)
  {
    configure(n, name, hardware_id);
  }

  DiagnosticTools::DiagnosticTools(){}

  void DiagnosticTools::configure(ros::NodeHandle &n, const std::string name, const std::string hardware_id)
  {
    m_last_check_freq = ros::Time().now();
    m_current_freq = 0.0;
    m_times_not_ok = 0;

    m_diagnostic_pub = n.advertise<diagnostic_msgs::DiagnosticArray>(n.getNamespace() + "/diagnostics", 1);
    m_diagnostic.name = name;
    m_diagnostic.hardware_id = hardware_id;
  }

  void DiagnosticTools::setLevel(const char level, const std::string message)
  {
    m_diagnostic.level = level;

    switch (m_diagnostic.level)
    {
      case diagnostic_msgs::DiagnosticStatus::OK:
        m_diagnostic.message = "Ok";
        break;
      case diagnostic_msgs::DiagnosticStatus::WARN:
        m_diagnostic.message = "Warning";
        break;
      case diagnostic_msgs::DiagnosticStatus::ERROR:
        m_diagnostic.message = "Error";
        break;
      default:
        m_diagnostic.message = "Stale";
    }

    if (message.compare("none") != 0)
      m_diagnostic.message += " - " + message;

    // Publish diagnostic message
    publish();
  }

  void DiagnosticTools::add(const std::string key, const bool value)
  {
    // change value to string
    add(key, (value) ? (std::string("True")) : (std::string("False")));
  }

  void DiagnosticTools::add(const std::string key, const int value)
  {
    // change value to string
    add(key, std::to_string(value));
  }

  void DiagnosticTools::add(const std::string key, const double value)
  {
    // change value to string
    add(key, std::to_string(value));
  }

  void DiagnosticTools::add(const std::string key, const char* value)
  {
    add(key, std::string(value));
  }

  void DiagnosticTools::add(const std::string key, const std::string value)
  {
    // Create KeyValue type
    diagnostic_msgs::KeyValue key_value;
    key_value.key = key;
    key_value.value = value;

    // Search for key
    del(key);

    // Add Key Value
    m_diagnostic.values.push_back(key_value);
  }

  void DiagnosticTools::del(const std::string key)
  {
    std::vector<diagnostic_msgs::KeyValue>::iterator it;

    // Search for key
    for(it = m_diagnostic.values.begin(); it != m_diagnostic.values.end(); it++)
      if (it->key.compare(key) == 0)
        break;

    // If key already found in 'values' delete it
    if (it != m_diagnostic.values.end())
      m_diagnostic.values.erase(it);
  }

  void DiagnosticTools::publish()
  {
    m_current_freq = 1.0 / (ros::Time().now().toSec() - m_last_check_freq.toSec());
    add("Diag frequency (Hz)", m_current_freq);
    m_last_check_freq = ros::Time().now();

    // Published as a warning, error or stale
    if (m_diagnostic.level != diagnostic_msgs::DiagnosticStatus::OK)
      m_times_not_ok++;
    else
      m_times_not_ok = 0;

    // Publish diagnostic
    diagnostic_msgs::DiagnosticArray diagnostic_array;
    diagnostic_array.header.stamp = ros::Time::now();
    diagnostic_array.header.frame_id = m_diagnostic.name;
    diagnostic_array.status.push_back(m_diagnostic);
    m_diagnostic_pub.publish(diagnostic_array);
  }

  double DiagnosticTools::getCurrentFreq()
  {
    return m_current_freq;
  }

  unsigned int DiagnosticTools::getTimesNotOK()
  {
    return m_times_not_ok;
  }

  void DiagnosticTools::addConsecutiveNOKs()
  {
    add("Consecutive NOKs (iter)", static_cast<int>(m_times_not_ok));
  }
}
