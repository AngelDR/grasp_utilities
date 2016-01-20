#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include "../include/pa10/pa10_wrapper.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"



class NodeClass {

private:

  ros::Publisher base_pub;
  ros::Publisher s2_pub;
  ros::Publisher s3_pub;
  ros::Publisher e1_pub; 
  ros::Publisher e2_pub;
  ros::Publisher w1_pub; 
  ros::Publisher w2_pub;

public:
  ros::NodeHandle n;
  ros::Subscriber sub;
     //--- defining a callback function---
  void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
  {
    ROS_INFO("Recibida trayectoria");
    moveit_msgs::RobotTrajectory _robot_trajectory = msg->trajectory[0];
    //ROS_INFO("Robot trajectory");
    trajectory_msgs::JointTrajectory _joint_trajectory = _robot_trajectory.joint_trajectory;
    //ROS_INFO("Joint trajectory");
    std::vector<trajectory_msgs::JointTrajectoryPoint> _points = _joint_trajectory.points;
    //ROS_INFO("Points vector");

    PA10Wrapper pa10;
    ros::Rate hz (20);
    //angulos[0]<< 0,0,0,0,0,0,0;
    Eigen::VectorXf angulos [] = {Eigen::VectorXf (7)};
    angulos[0]<< 0,0,0,0,0,0,0;
    pa10.goTo(angulos[0]);
    sleep(1.0);    
    pa10.modAxs();
    //sleep(1.0);
    Eigen::VectorXf joint_positions = Eigen::VectorXf (7);
    //std::vector<std_msgs::Float64> joint_positions;

    std_msgs::Float64 value_1, value_2, value_3, value_4, value_5, value_6, value_7;
    value_1.data=0;
    value_2.data=0;
    value_3.data=0;
    value_4.data=0;
    value_5.data=0;
    value_6.data=0;
    value_7.data=0;

    /**for(int i=0; i<200;i++){
      value_1.data+=0.0025;
      value_4.data-=0.0025;
      joint_positions << (float) value_1.data, (float) value_2.data, (float) value_3.data, (float) value_4.data, (float) value_5.data,(float) value_6.data, (float) value_7.data;
           //joint_positions << 0.003962332959198028, 0.0515037272636724, 0.07705049507275152, -0.13779913773443253, 0.04189187387812674, 0.10722692157583646, -0.08539089352257592;

      //pa10.odrAxs(joint_positions);
      pa10.odrAxs(joint_positions);
      ROS_INFO("Valor articular  %f", joint_positions[0]);
      hz.sleep();
    }*/

    int num_pos_intermedias = 500;
    for(int i=0; i < _points.size() - 1; i++){
        //publish data -> controles Gazebo
        for(int paso = 0; paso < num_pos_intermedias; paso++){		
	  	    value_1.data = _points.at(i).positions[0] + (paso * ((_points.at(i+1).positions[0]  - _points.at(i).positions[0] )/ num_pos_intermedias));
        	//ROS_INFO("Valor articular 0 %f", value_1.data);
          //base_pub.publish(value_1);
		      value_2.data =  _points.at(i).positions[1] + (paso * ((_points.at(i+1).positions[1]  - _points.at(i).positions[1] )/ num_pos_intermedias));
        	//ROS_INFO("Valor articular 1 %f", value_2.data);
          //s2_pub.publish(value_2);
		      value_3.data =  _points.at(i).positions[2] + (paso * ((_points.at(i+1).positions[2]  - _points.at(i).positions[2] )/ num_pos_intermedias));
        	//ROS_INFO("Valor articular 2 %f", value_3.data);
         	//s3_pub.publish(value_3);
		      value_4.data =  _points.at(i).positions[3] + (paso * ((_points.at(i+1).positions[3]  - _points.at(i).positions[3] )/ num_pos_intermedias));
        	//ROS_INFO("Valor articular 3 %f", value_4.data);
          //e1_pub.publish(value_4);
		      value_5.data =  _points.at(i).positions[4] + (paso * ((_points.at(i+1).positions[4]  - _points.at(i).positions[4] )/ num_pos_intermedias));		
        	//ROS_INFO("Valor articular 4 %f", value_5.data);
          //e2_pub.publish(value_5);
		      value_6.data =  _points.at(i).positions[5] + (paso * ((_points.at(i+1).positions[5]  - _points.at(i).positions[5] )/ num_pos_intermedias));
        	//ROS_INFO("Valor articular 5 %f", value_6.data);
          //w1_pub.publish(value_6);
		      value_7.data =  _points.at(i).positions[6] + (paso * ((_points.at(i+1).positions[6]  - _points.at(i).positions[6] )/ num_pos_intermedias));
        	//ROS_INFO("Valor articular 6 %f", value_7.data);
         	//w2_pub.publish(value_7);

          joint_positions << (float) value_1.data, (float) value_2.data, (float) value_3.data, (float) value_4.data, (float) value_5.data,(float) value_6.data, (float) value_7.data;
	         //joint_positions << 0.003962332959198028, 0.0515037272636724, 0.07705049507275152, -0.13779913773443253, 0.04189187387812674, 0.10722692157583646, -0.08539089352257592;

	         pa10.odrAxs(joint_positions);

	         //hz.sleep();

          //ros::spinOnce;
            sleep(0.010);
        }

        ROS_INFO("Punto de trayectoria %d", i);
        // IF CONECTAR CON PA10 real = 1
        //pa10.odrAxs(joint_positions);
        //hz.sleep(); 
    }


        
  }

  //Constructor
  NodeClass()
  {
    base_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_base_to_s2_position_controller/command", 1000);
    s2_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_s2_to_s3_position_controller/command", 1000);
    s3_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_s3_to_e1_position_controller/command", 1000);
    e1_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_e1_to_e2_position_controller/command", 1000);
    e2_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_e2_to_w1_position_controller/command", 1000);
    w1_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_w1_to_w2_position_controller/command", 1000);
    w2_pub = n.advertise<std_msgs::Float64>("/shadow_pa10_7dof/joint_w2_to_eef_position_controller/command", 1000);

  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectoryExecution");
  NodeClass* a = new NodeClass();
  a->sub = a->n.subscribe("/move_group/display_planned_path", 1000, &NodeClass::trajectoryCallback,a);

  ros::spin();
  return 0;
}
