#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <grasp_utilities/tactile_servo_reconfigurationConfig.h>

void callback(grasp_utilities::tactile_servo_reconfigurationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d", 
            config.p, config.i, 
            config.d);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tactile_servo_reconfiguration");

  dynamic_reconfigure::Server<grasp_utilities::tactile_servo_reconfigurationConfig> server;
  dynamic_reconfigure::Server<grasp_utilities::tactile_servo_reconfigurationConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}