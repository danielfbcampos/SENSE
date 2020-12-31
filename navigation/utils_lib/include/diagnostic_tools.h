/**
 * @file diagnostic_tools.h
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


#ifndef DIAGNOSTIC_TOOLS_H
#define DIAGNOSTIC_TOOLS_H

#include <ros/ros.h>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>


namespace diag_tools {

  class DiagnosticTools
  {
    public:
      /**
       * @brief DiagnosticTools - sets the diagnostics publisher and configures the name and hardware_id for the node n
       * @param n - node handler name
       * @param name - name of the test component
       * @param hardware_id - hardware unique identifier
       */
      DiagnosticTools(ros::NodeHandle& n, const std::string name, const std::string hardware_id);
      DiagnosticTools();

      /**
       * @brief configure
       * @param n - node handler name
       * @param name - name of the test component
       * @param hardware_id - hardware unique identifier
       */
      void configure(ros::NodeHandle& n, const std::string name, const std::string hardware_id);

      /**
       * @brief setLevel - Sets the level and the message of the Diagnostics message
       * @param level    - Severety level, diagnostic_msgs::DiagnosticStatus enum
       * @param message  - Description of the status
       */
      void setLevel(const char level, const std::string message = "none");

      /**
       * @brief add - Adds a diagnostics entry with a bool value
       * @param key - Value label
       * @param value - Value to track over time
       */
      void add(const std::string key, const bool value);

      /**
       * @brief add - Adds a diagnostics entry with a int value
       * @param key - Value label
       * @param value - Value to track over time
       */
      void add(const std::string key, const int value);

      /**
       * @brief add - Adds a diagnostics entry with a double value
       * @param key - Value label
       * @param value - Value to track over time
       */
      void add(const std::string key, const double value);

      /**
       * @brief add - Adds a diagnostics entry with a char* value
       * @param key - Value label
       * @param value - Value to track over time
       */
      void add(const std::string key, const char* value);

      /**
       * @brief add - Adds a diagnostics entry with a string value
       * @param key - Value label
       * @param value - Value to track over time
       */
      void add(const std::string key, const std::string value);

      /**
       * @brief del - Deletes a diagnostics entry
       * @param key - Key value to delete
       */
      void del(const std::string key);


      /**
       * @brief publish - Publish the diagnostics message
       */
      void publish();

      /**
       * @brief getCurrentFreq - Gets current diagnostics frequency
       * @return
       */
      double getCurrentFreq();

      /**
       * @brief getTimesNotOK - Returns counter with the number of times the diagnostic message was in a state other than OK
       * @return
       */
      unsigned int getTimesNotOK();

      /**
       * @brief addConsecutiveNOKs - Adds the not ok counter to the diagnostic
       */
      void addConsecutiveNOKs();

    protected:


    private:
      // Diagnostic msg
      diagnostic_msgs::DiagnosticStatus m_diagnostic;

      // ROS Publisher
      ros::Publisher m_diagnostic_pub;

      // Variables to check frequency
      ros::Time m_last_check_freq;
      double m_current_freq;

      // To check the times that the diagnostics are in a state different from OK (warn, error, stale)
      unsigned int m_times_not_ok;
  };
}
#endif


