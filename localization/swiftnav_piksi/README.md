# swiftnav_piksi - OBSOLETE

This package has been replaced by the SwiftNav supported  ROS package. The repo
is at https://github.com/swift-nav/swiftnav_ros. Wiki documentation for the new
package is at http://wiki.ros.org/swiftnav_ros

This package is a ROS release of a driver for Swift Navigation's Piksi RTK GPS receiver module.
A pair of Piksi modules connected by a wireless link provides the location of each receiver
relative to the other with accuracy as good as a couple of centimetres, and in addition,
each Piksi module provides its location with typical GPS accuracy (about 3 meters).
Typically one module is a stationary base station, which may be located at a surveyed point,
while the other is mounted on a rover and provides ROS navigation software with a highly
accurate position relative to the base station.
