#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <grasp_utilities/object_marker_reconfigurationConfig.h>

void callback(grasp_utilities::object_marker_reconfigurationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f", 
            config.forearm_height, config.forearm_angle, 
            config.pos_x, config.pos_y, config.pos_z,
            config.dim_x, config.dim_y, config.dim_z, config.form);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_marker_reconfiguration");

  dynamic_reconfigure::Server<grasp_utilities::object_marker_reconfigurationConfig> server;
  dynamic_reconfigure::Server<grasp_utilities::object_marker_reconfigurationConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}