#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "tekscan_client/GetPressureMap.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <visualization_msgs/Marker.h>
using namespace std;


int main(int argc, char** argv){
  ros::init(argc, argv, "shadow_tf_listener");
  ros::NodeHandle node;
  
  tf2_ros::Buffer tfBuffer;
  tf::TransformListener tfListener;
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0; 

  // Marker visualizacion valores de Modulo young: (texto con valores de presion-desplazamiento-moduloYoung) 
  ros::Publisher young_mod_marker_pub = node.advertise<visualization_msgs::Marker>( "young_module_marker", 0 );
  visualization_msgs::Marker  young_mod_marker;

  // Publisher para 
  young_mod_marker.header.frame_id = "shadow_world_frame";
  young_mod_marker.ns = "young_modulus";
  young_mod_marker.id = 7;
  young_mod_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  young_mod_marker.action = visualization_msgs::Marker::ADD;
  // Todo: anadir params
  young_mod_marker.scale.x = 0.1; // Param
  young_mod_marker.scale.y = 0.1; // Param
  young_mod_marker.scale.z = 0.1; //
  young_mod_marker.color.a = 1.0; // Don't forget to set the alpha!
  young_mod_marker.color.r = 0.0;
  young_mod_marker.color.g = 1.0;
  young_mod_marker.color.b = 0.0;
  std::string text_of_marker = "";

  std::string lista_frames[5] = {"thtip","fftip","mftip","rftip","lftip"};
  tf::StampedTransform transform[5];
  tf::StampedTransform object_transform;
    
  ros::Rate rate(10.0);
  
  int iteration = 0;
  double displacement[5] = {0,0,0,0,0};
  double initial_displacement[5] = {0,0,0,0,0};
  double stretchRatio[5] = {0,0,0,0,0};
  double stress[5] = {0,0,0,0,0};
  double young_modulus[5] = {0,0,0,0,0};

  // Get param : numero dedos
  int num_fingers_exp;
  if (node.getParam("/grasp_reconfiguration/number_fingers", num_fingers_exp))
  {
    ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
  }
  else{
    num_fingers_exp = 3;
  }

  // Obtener posicion centro objeto:
  try{
    ros::Time now = ros::Time::now();
    tfListener.waitForTransform("forearm", "object_frame", now, ros::Duration(3.0));
    tfListener.lookupTransform("forearm", "object_frame", now, object_transform);
  }catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // Archivos 
  ofstream pose_file;
  ofstream tactil_file;
  ofstream young_file;
  pose_file.open ("/home/aurova/Desktop/pruebas/resultados/positions.txt");
  if(pose_file.is_open())
    ROS_INFO("Archivo posiciones abierto");
  tactil_file.open ("/home/aurova/Desktop/pruebas/resultados/pressure.txt");
  if(tactil_file.is_open())
    ROS_INFO("Archivo presion abierto");
  young_file.open ("/home/aurova/Desktop/pruebas/resultados/young.txt");
  if(young_file.is_open())
    ROS_INFO("Archivo modulo young");
  

  // Ejecución continua
  while (node.ok()){
    //Obtener transformaciones
    text_of_marker = "";
    for(int id = 0;id < num_fingers_exp; id++)
    {
      try{
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform("forearm", lista_frames[id], now, ros::Duration(3.0));
        tfListener.lookupTransform("forearm", lista_frames[id], now, transform[id]);

        // Obtain displacement for each finger
        displacement[id] = sqrt(pow(transform[id].getOrigin().getX() - object_transform.getOrigin().getX(),2)
          + pow(transform[id].getOrigin().getY()-object_transform.getOrigin().getY(),2)
          + pow(transform[id].getOrigin().getZ()-object_transform.getOrigin().getZ(),2));

        // Initial vectors:
        if(iteration==0){
            initial_displacement[id]=displacement[id];
        }

        // Strain: stretch ratio
        //stretchRatio[id] = displacement[id] / initial_displacement[id];

        // Strain: cauchy strain
        stretchRatio[id] = (initial_displacement[id] - displacement[id]) / initial_displacement[id];


      }catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
      }
    } 

      
    // Obtener mapa de presión - Stress[]
    if (pressure_client.call(srv_pressure))
    {
      for(int id=0; id<num_fingers_exp; id++){
        stress[id] = srv_pressure.response.applied_force[id] / 2.46;
        young_modulus[id] = stress[id] / stretchRatio[id];
        text_of_marker += ">Dedo: " + boost::lexical_cast<std::string>(id) + "\n \n";
        text_of_marker += ">>Radio desplazamiento: " + boost::lexical_cast<std::string>(stretchRatio[id]) + "\n";
        text_of_marker += ">>Stress: " + boost::lexical_cast<std::string>(stress[id]) + "\n";
        text_of_marker += ">>Young mod.: " + boost::lexical_cast<std::string>(young_modulus[id]) + "\n";
        text_of_marker += "\n \n";
      }
    }
    else
    {
      ROS_ERROR("Failed to call service pressure");
      return 1;
    }


    young_mod_marker.text = text_of_marker;
    young_mod_marker.pose.position.z = 1.0;
    young_mod_marker.pose.position.x = 0.5;
    young_mod_marker_pub.publish(young_mod_marker);
    ROS_INFO("Published");


    // Escribir en archivo si esta activada escritura:
    int write;
    if(node.getParam("/grasp_reconfiguration/write_files", write))
    {
      ROS_INFO("Escribir en archivo : %d"); 
    }
    else{
      write = 0;
    }

    if(write)
    {
      for(int id=0; id<5; id++){
        // if: id <= num_dedos escribir valores leidos
        if(id<num_fingers_exp){
          pose_file << transform[id].getOrigin().getX() << " " << transform[id].getOrigin().getY() << " " << transform[id].getOrigin().getZ() << " ";
          tactil_file << stress[id] << " " << stress[id]*2.46 << " ";
          young_file << displacement[id] << " " << stretchRatio[id] << " " << young_modulus[id] << " " ;
        }
        // else: completar con ceros -> archivo siempre con el mismo num de columns
        else{
          pose_file << 0 << " " << 0 << " " << 0 << " ";
          tactil_file << 0 << " " << 0 << " ";
          young_file << 0 << " " << 0 << " " << 0 << " ";  
        }
      } 
      pose_file << iteration << "\n";
      tactil_file << iteration << "\n";
      young_file << iteration << "\n";
    }

    iteration++;
    rate.sleep();

  }

  pose_file.close();
  tactil_file.close();
  young_file.close();
  return 0;
};
