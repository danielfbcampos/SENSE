#ifndef UTILSCRAS_H
#define UTILSCRAS_H

#include <chrono>
#include <random>
#include <limits>

//Eigen
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>

//ROS
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

//ROS MSGs includes
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

//TF includes
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZ PointXYZ; //!< Data type definition for a PointXYZ
typedef pcl::PointXYZI PointXYZI; //!< Data type definition for a PointXYZI
typedef pcl::PointNormal PointXYZNormal; //!< Data type definition for a PointXYZNormal
typedef pcl::PointXYZINormal PointXYZINormal; //!< Data type definition for a PointXYZINormal
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ; //!< Data type definition for a PointCloud with PointXYZ
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI; //!< Data type definition for a PointCloud with PointXYZI
typedef pcl::PointCloud<PointXYZNormal> PointCloudXYZNormal; //!< Data type definition for a PointCloud with PointXYZNormal
typedef pcl::PointCloud<PointXYZINormal> PointCloudXYZINormal; //!< Data type definition for a PointCloud with PointXYZINormal

template<typename T1, typename T2, typename T3>
using triplet = std::tuple<T1, T2, T3>;

enum State { NOK=0, OK };

class Utils
{
public:
  /**
     * @brief
     * sgn - get signal from number
     * @tparam T - Type independent (float, double, int)
     * @param val - number to get signal from
     * @return T
     */
  template <typename T>
  static T sgn(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  /**
     * @brief
     * normalizeAngle - normalizes angle between -pi and pi
     * @tparam T - Type independent (float, double, int)
     * @param angle - angle to be normalized
     * @return T
     */
  template <typename T>
  static T normalizeAngle(T angle)
  {
    T sign = sgn(angle);
    angle = fabs(angle);

    while(angle > M_PI)
      angle -= 2*M_PI;

    angle = sign*angle;
    return angle;
  }

  /**
   * @brief absAngleDiff - Get absolute angular difference
   * @tparam T - Type independent (float, double, int)
   * @param init - init angle
   * @param goal - goal angle
   * @return
   */
  template <typename T>
  static T absAngleDiff(T init, T goal)
  {
    return fabs(normalizeAngle<T>(goal-init));
  }

  /**
   * @brief distance - Get distance (xx, yy and zz)
   * @tparam T - Type independent (float, double, int)
   * @param init - init pose
   * @param goal - goal pose
   * @return
   */
  template <typename T>
  static T distance(geometry_msgs::Pose init, geometry_msgs::Pose goal)
  {
    return static_cast<T>(std::sqrt(std::pow(goal.position.x-init.position.x,2) + std::pow(goal.position.y-init.position.y,2) + std::pow(goal.position.z-init.position.z,2)));
  }

  /**
   * @brief distance - Get distance (xx, yy and zz)
   * @tparam T - Type independent (float, double, int)
   * @param init - init pose
   * @param goal - goal pose
   * @return
   */
  template <typename T>
  static T distance(geometry_msgs::PoseStamped init, geometry_msgs::PoseStamped goal)
  {
    return static_cast<T>(std::sqrt(std::pow(goal.pose.position.x-init.pose.position.x,2) + std::pow(goal.pose.position.y-init.pose.position.y,2) + std::pow(goal.pose.position.z-init.pose.position.z,2)));
  }

  /**
   * @brief distance - Get distance (xx, yy and zz)
   * @tparam T - Type independent (float, double, int)
   * @param init - init pose
   * @param goal - goal pose
   * @return
   */
  template <typename T>
  static T distance(Eigen::Vector3d init, Eigen::Vector3d goal)
  {
    return static_cast<T>(std::sqrt(std::pow(goal.x()-init.x(),2) + std::pow(goal.y()-init.y(),2) + std::pow(goal.z()-init.z(),2)));
  }

  /**
   * @brief distance2D - Get distance (xx, yy)
   * @tparam T - Type independent (float, double, int)
   * @param init - init pose
   * @param goal - goal pose
   * @return
   */
  template <typename T>
  static T distance2D(geometry_msgs::Pose init, geometry_msgs::Pose goal)
  {
    return static_cast<T>(std::sqrt(std::pow(goal.position.x-init.position.x,2) + std::pow(goal.position.y-init.position.y,2)));
  }

  /**
   * @brief distance2D - Get distance (xx, yy)
   * @tparam T - Type independent (float, double, int)
   * @param init - init pose
   * @param goal - goal pose
   * @return
   */
  template <typename T>
  static T distance2D(geometry_msgs::PoseStamped init, geometry_msgs::PoseStamped goal)
  {
    return static_cast<T>(std::sqrt(std::pow(goal.pose.position.x-init.pose.position.x,2) + std::pow(goal.pose.position.y-init.pose.position.y,2)));
  }

  /**
   * @brief toDeg - rad angle to degree
   * @tparam T - Type independent (float, double, int)
   * @param ang_rad - angle in radians
   * @return
   */
  template <typename T>
  static T toDeg(T ang_rad)
  {
    return ang_rad*180.0/M_PI;
  }

  /**
   * @brief toRad - degree angle to rad
   * @tparam T - Type independent (float, double, int)
   * @param ang_deg - angle in radians
   * @return
   */
  template <typename T>
  static T toRad(T ang_deg)
  {
    return ang_deg*M_PI/180.0;
  }

  /**
   * @brief publishData - data publisher template for any type of msg
   * @param pub_
   * @param data_
   */
  template <class DataType>
  static inline void publishData(ros::Publisher& pub_, const DataType& data_)
  {
    if(pub_.getNumSubscribers()>0)
      pub_.publish(data_);

    return;
  }

  /**
   * @brief getRPY - get roll, pitch and yaw
   * @param pose - pose to extract
   * @return
   */
  static triplet<double, double, double> getRPY(geometry_msgs::PoseStamped pose);
  static triplet<double, double, double> getRPY(geometry_msgs::Pose pose);

  /**
   * @brief tf_calc - obtain tf from source frame to target frame
   * @param listener
   * @param target - Pose stamped transform result. VERY IMPORTANT TO FILL CORRECTLY THE HEADER!
   * @param source - Pose stamped to be transformed. VERY IMPORTANT TO FILL CORRECTLY THE HEADER!
   * @return true if no exception found
   */
  static bool tf_calc(tf::TransformListener& listener_, geometry_msgs::PoseStamped &target, geometry_msgs::PoseStamped source);

  /**
   * @brief tfCalcTransform - obtain transform from source frame to target frame
   * @param transf_output - output transform
   * @param listener
   * @param stamp - timestamp for transform
   * @param target_frame - desired (to) frame name
   * @param source_frame - source (from) frame name
   * @param wait_time - wait for transform time default 0.1s
   * @return
   */
  static bool tfCalcTransform(tf::StampedTransform & transf_output, tf::TransformListener& listener, const ros::Time stamp, const std::string target_frame, const std::string source_frame, double wait_time=0.1);

  /**
   * @brief getOdom - obtain pose of the robot frame in the odom frame at time stamp
   * @param listener_
   * @param stamp - timestamp from odom
   * @param odom_frame - odometry frame name
   * @param robot_frame - robot frame name
   * @return
   */
  static std::pair<bool, geometry_msgs::PoseStamped> getOdom(tf::TransformListener& listener, const ros::Time stamp, const std::string odom_frame, const std::string robot_frame);

  /**
   * @brief pclToROS - convert pcl PointCloudXYZ to sensor_msgs::PointCloud2
   * @param cloud_in - PointCloudXYZ::Ptr
   * @param cloud_out - sensor_msgs::PointCloud2
   */
  static void pclToROS(const PointCloudXYZ::Ptr& cloud_in, sensor_msgs::PointCloud2& cloud_out);

  /**
   * @brief pclToROS - convert pcl PointCloudXYZ to sensor_msgs::PointCloud2
   * @param cloud_in - PointCloudXYZ
   * @param cloud_out - sensor_msgs::PointCloud2
   */
  static void pclToROS(const PointCloudXYZ& cloud_in, sensor_msgs::PointCloud2& cloud_out);

  /**
   * @brief rosToPCL - convert sensor_msgs::PointCloud2 to pcl PointCloudXYZ::Ptr
   * @param cloud_in - sensor_msgs::PointCloud2
   * @param cloud_out - PointCloudXYZ::Ptr
   */
  static void rosToPCL(const sensor_msgs::PointCloud2& cloud_in, PointCloudXYZ::Ptr& cloud_out);

  /**
   * @brief rosToPCL - convert sensor_msgs::PointCloud2 to pcl PointCloudXYZ::Ptr
   * @param cloud_in - sensor_msgs::PointCloud2
   * @param cloud_out - PointCloudXYZ
   */
  static void rosToPCL(const sensor_msgs::PointCloud2& cloud_in, PointCloudXYZ& cloud_out);

  /**
   * @brief scanToPCLHP - convert laser scan to point cloud with high precision
   * @param listener - tf listener
   * @param scan - scan to convert
   * @param target_frame - desired frame
   * @param output_cloud - output pointcloud
   * @return
   */
  static bool scanToPCLHP(tf::TransformListener & listener, const sensor_msgs::LaserScan & scan, const std::string & target_frame, sensor_msgs::PointCloud2 & output_cloud);

protected:
private:
  Utils();
};

#endif


