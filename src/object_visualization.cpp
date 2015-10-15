#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "std_msgs/Float64.h"
#include "tekscan_client/GetPressureMap.h"
#include <math.h>
#define PI 3.14159265

#include <Eigen/Dense>
#include <tf/transform_datatypes.h>

using namespace Eigen;

int main(int argc, char** argv){
	ros::init(argc, argv, "object_visualization");
	ros::NodeHandle node;

	tf::TransformBroadcaster br,br_obj;
	tf::Transform transform, transform_obj;
	//geometry_msgs::TransformStamped transformStampedff;
	//geometry_msgs::TransformStamped transformStampedmf;
	// Clases para transformaciones con tf2
	tf2_ros::Buffer tfBuffer;
	tf::TransformListener tfListener; 
	geometry_msgs::TransformStamped ff_transf;


	// >> Marker visualization forces
	ros::Publisher object_marker_pub = node.advertise<visualization_msgs::Marker>( "object_marker", 0 );
	visualization_msgs::Marker  object_marker;

	object_marker.header.frame_id = "forearm_proj";
	object_marker.ns = "object";
	object_marker.id = 5;
	object_marker.type = visualization_msgs::Marker::CUBE;
	object_marker.action = visualization_msgs::Marker::ADD;
	// Todo: anadir params
	object_marker.scale.x = 0.135; // Param
	object_marker.scale.y = 0.045; // Param
	object_marker.scale.z = 0.077; //
	object_marker.color.a = 1.0; // Don't forget to set the alpha!
	object_marker.color.r = 0.0;
	object_marker.color.g = 1.0;
	object_marker.color.b = 0.0;

  ros::Rate rate(10.0);
 	Matrix4d transformation_matrix;

  while (node.ok()){

      // Get param : forma del objeto
      int object_form;
      if (node.getParam("/object_marker_reconfiguration/form", object_form))
      {
        ROS_INFO("Forma del objeto : %d", object_form); 
        object_marker.type = object_form;
      }
      else
      {
        object_form = 1;
      }

   	  // Get param : posicion forearm (altura & rotacion)
   	  double forearm_height, forearm_angle;
   	  if (node.getParam("/object_marker_reconfiguration/forearm_height", forearm_height))
      {
        ROS_INFO("Altura del brazo : %f", forearm_height); 
      }
      else
      {
        forearm_height = 0.3;
      }

   	  if (node.getParam("/object_marker_reconfiguration/forearm_angle", forearm_angle))
      {
        ROS_INFO("Angulo del brazo : %f", forearm_angle); 
      }
      else
      {
        forearm_angle = 0.5;
      }

      // Get param: dimensiones objec (x,y,z)
      double dim_x, dim_y, dim_z;
      if (node.getParam("/object_marker_reconfiguration/dim_x", dim_x))
      {
        ROS_INFO("Dim x : %f", dim_x); 
      }
      else
      {
        dim_x = 0.135;
      }

   	  if (node.getParam("/object_marker_reconfiguration/dim_y", dim_y))
      {
        ROS_INFO("Dim y : %f", dim_y); 
      }
      else
      {
        dim_y = 0.045;
      }

      if (node.getParam("/object_marker_reconfiguration/dim_z", dim_z))
      {
        ROS_INFO("dim_z : %f", dim_z); 
      }
      else
      {
        dim_z = 0.077;
      }

      // Get param: Posicion objeto (x,y,z)
      double pos_x, pos_y, pos_z;
      if (node.getParam("/object_marker_reconfiguration/pos_x", pos_x))
      {
        ROS_INFO("Pos x : %f", pos_x); 
      }
      else
      {
        pos_x = 0.0;
      }

   	  if (node.getParam("/object_marker_reconfiguration/pos_y", pos_y))
      {
        ROS_INFO("Pos y : %f", pos_y); 
      }
      else
      {
        pos_y = 0.0225;
      }

      if (node.getParam("/object_marker_reconfiguration/pos_z", pos_z))
      {
        ROS_INFO("pos_z : %f", pos_z); 
      }
      else
      {
        pos_z = 0.3;
      }

      // Define rotation matrix (forearm, forearm_proj_plane)
      //tf::Matrix3x3 rotation_(1,0,0,0, cos(forearm_angle), -sin(forearm_angle),0, sin(forearm_angle), cos(forearm_angle));
      tf::Matrix3x3 rotation_(cos(-forearm_angle),0,sin(-forearm_angle),0,1,0,-sin(-forearm_angle),0, cos(-forearm_angle));
      //tf::Matrix3x3 rotation_(cos(forearm_angle), -sin(forearm_angle),0,sin(forearm_angle),cos(forearm_angle),0,0,0,1);

      double roll_, pitch_, yaw_;

      rotation_.getEulerYPR(yaw_,pitch_, roll_);

  	

  		// Crear frame en proyeccion forearm
    	transform.setOrigin(tf::Vector3(0.0, -forearm_height * cos(forearm_angle), forearm_height * sin(forearm_angle) ) );
    	transform.setRotation(tf::Quaternion(yaw_,pitch_,roll_));
    	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "forearm", "forearm_proj"));


    	transform_obj.setOrigin(tf::Vector3(pos_x, pos_y, pos_z));
    	transform_obj.setRotation(tf::Quaternion(0,0,0,1));
    	br_obj.sendTransform(tf::StampedTransform(transform_obj, ros::Time::now(), "forearm_proj", "object_frame"));

    	//Posicion objecto(respecto a forearm_proy): x = 0; y = dim(y)/2; z = thumb.z(respecto a forarm_proy) + dim(z)/2
    	// Obtener desde modulo vision ->

    	object_marker.header.stamp = ros::Time();
	    object_marker.pose.position.x = pos_x;
	    object_marker.pose.position.y = pos_y;
	    object_marker.pose.position.z = pos_z;

      object_marker.scale.x = dim_x;
      object_marker.scale.y = dim_y; 
      object_marker.scale.z = dim_z; 
    	rate.sleep();  

    	object_marker_pub.publish(object_marker);
    	// Grasp matrix: rotacion puntos dedos-> tf 

  }

};