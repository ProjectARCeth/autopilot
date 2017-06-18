#include "arc_tools/tf_kit.hpp"

namespace arc_tools {

void tfBroadcaster(Eigen::Vector4d quat, Eigen::Vector3d position, std::string from, std::string to){
  // Init static broadcaster.
  static tf::TransformBroadcaster broadcaster;
  //Set orientation and vector.
  tf::Quaternion tf_quat(quat(0), quat(1), quat(2), quat(3));
  tf::Vector3 tf_vector(position(0), position(1), position(2)+0.2);
  //Setting tf - broadcast from odom to rear_axle.
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf_quat, tf_vector),
                           ros::Time::now(), from, to));
}
}//namespace arc_tools.
