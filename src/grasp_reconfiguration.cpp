#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <grasp_utilities/grasp_reconfigurationConfig.h>

void callback(grasp_utilities::grasp_reconfigurationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %f %f", 
            config.number_fingers, config.min_pressure_threshold, 
            config.max_pressure_threshold, config.position_step);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_reconfiguration");

  dynamic_reconfigure::Server<grasp_utilities::grasp_reconfigurationConfig> server;
  dynamic_reconfigure::Server<grasp_utilities::grasp_reconfigurationConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}