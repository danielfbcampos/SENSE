#include "utils_cras.h"

/**
 * @brief getRPY - get roll, pitch and yaw
 * @param pose - pose to extract
 * @return
 */
triplet<double, double, double> Utils::getRPY(geometry_msgs::PoseStamped pose)
{
  // Gets roll pitch and yaw
  double roll, pitch, yaw;

  tf::Quaternion head_cur(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
  tf::Matrix3x3 m_cur(head_cur);
  m_cur.getRPY(roll,pitch,yaw);

  return triplet<double, double, double>(roll, pitch, yaw);
}

/**
 * @brief getRPY - get roll, pitch and yaw
 * @param pose - pose to extract
 * @return
 */
triplet<double, double, double> Utils::getRPY(geometry_msgs::Pose pose)
{
  // Gets roll pitch and yaw
  double roll, pitch, yaw;

  tf::Quaternion head_cur(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  tf::Matrix3x3 m_cur(head_cur);
  m_cur.getRPY(roll,pitch,yaw);

  return triplet<double, double, double>(roll, pitch, yaw);
}

/**
 * @brief tf_calc - obtain tf from source frame to target frame
 * @param listener
 * @param target - Pose stamped transform result. VERY IMPORTANT TO FILL CORRECTLY THE HEADER!
 * @param source - Pose stamped to be transformed. VERY IMPORTANT TO FILL CORRECTLY THE HEADER!
 * @return true if no exception found
 */
bool Utils::tf_calc(tf::TransformListener& listener_, geometry_msgs::PoseStamped &target, geometry_msgs::PoseStamped source)
{
  if(target.header.frame_id == source.header.frame_id) // If equal frame no need for transform
  {
    target = source;
    return true;
  }

  try
  {
    listener_.transformPose(target.header.frame_id, source, target); // Transform from Source to Target
    return true;
  }
  catch (tf::TransformException& e) {
    ROS_INFO ("Not saving scan due to tf lookup exception: %s",
              e.what());
    return false;
  }
}

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
bool Utils::tfCalcTransform(tf::StampedTransform & transf_output, tf::TransformListener& listener, const ros::Time stamp, const std::string target_frame, const std::string source_frame, double wait_time /*=0.1*/)
{
  try
  {
    listener.waitForTransform(target_frame, source_frame, stamp, ros::Duration(wait_time));
    listener.lookupTransform(target_frame, source_frame, stamp, transf_output);
  }
  catch (tf::TransformException& e){
    ROS_INFO ("Not saving scan due to tf lookup exception: %s",
              e.what());
    return false;
  }

  return true;
}

/**
 * @brief getOdom - obtain pose of the robot frame in the odom frame at time stamp
 * @param listener_
 * @param stamp - timestamp from odom
 * @param odom_frame - odometry frame name
 * @param robot_frame - robot frame name
 * @return
 */
std::pair<bool, geometry_msgs::PoseStamped> Utils::getOdom(tf::TransformListener& listener, const ros::Time stamp, const std::string odom_frame, const std::string robot_frame)
{
  bool state = true;
  geometry_msgs::PoseStamped odom, robot;

  odom.header.stamp     = robot.header.stamp   = stamp;
  odom.header.frame_id  = odom_frame;
  robot.header.frame_id = robot_frame;

  odom.pose.position.x  = odom.pose.position.y = odom.pose.position.z = 0.0;
  odom.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
  robot.pose            = odom.pose;

  if(!tf_calc(listener, odom, robot))
  {
    state = false;
  }

  return std::make_pair(state, odom);
}


/**
 * @brief pclToROS - convert pcl PointCloudXYZ to sensor_msgs::PointCloud2
 * @param cloud_in - PointCloudXYZ::Ptr
 * @param cloud_out - sensor_msgs::PointCloud2
 */
void Utils::pclToROS(const PointCloudXYZ::Ptr& cloud_in, sensor_msgs::PointCloud2& cloud_out)
{
  pcl::toROSMsg(*cloud_in, cloud_out);
  return;
}

/**
  * @brief pclToROS - convert pcl PointCloudXYZ to sensor_msgs::PointCloud2
  * @param cloud_in - PointCloudXYZ
  * @param cloud_out - sensor_msgs::PointCloud2
  */
void Utils::pclToROS(const PointCloudXYZ& cloud_in, sensor_msgs::PointCloud2& cloud_out)
{
  pcl::toROSMsg(cloud_in, cloud_out);
  return;
}

/**
  * @brief rosToPCL - convert sensor_msgs::PointCloud2 to pcl PointCloudXYZ::Ptr
  * @param cloud_in - sensor_msgs::PointCloud2
  * @param cloud_out - PointCloudXYZ::Ptr
  */
void Utils::rosToPCL(const sensor_msgs::PointCloud2& cloud_in, PointCloudXYZ::Ptr& cloud_out)
{
  pcl::fromROSMsg(cloud_in, *cloud_out);
  return;
}

/**
  * @brief rosToPCL - convert sensor_msgs::PointCloud2 to pcl PointCloudXYZ::Ptr
  * @param cloud_in - sensor_msgs::PointCloud2
  * @param cloud_out - PointCloudXYZ
  */
void Utils::rosToPCL(const sensor_msgs::PointCloud2& cloud_in, PointCloudXYZ& cloud_out)
{
  pcl::fromROSMsg(cloud_in, cloud_out);
  return;
}

/**
  * @brief scanToPCLHP - convert laser scan to point cloud with high precision
  * @param scan - scan to convert
  * @param target_frame - desired frame
  * @param output_cloud - output pointcloud
  * @return
  */
bool Utils::scanToPCLHP(tf::TransformListener & listener, const sensor_msgs::LaserScan & scan, const std::string & target_frame, sensor_msgs::PointCloud2 & output_cloud)
{
  laser_geometry::LaserProjection projector_;

  if(!listener.waitForTransform(scan.header.frame_id,
                                target_frame,
                                scan.header.stamp + ros::Duration().fromSec(static_cast<double>(scan.ranges.size()*scan.time_increment)),
                                ros::Duration(0.1)))
    return false;

  //Tranform laser to pointcloud
  projector_.transformLaserScanToPointCloud(target_frame, scan, output_cloud, listener);
  return true;
}
