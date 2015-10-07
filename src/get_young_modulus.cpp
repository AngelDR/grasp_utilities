#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tekscan_client/GetPressureMap.h"

#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;


double getStretchRatio(geometry_msgs::TransformStamped thumb_initial, geometry_msgs::TransformStamped thumb_current,
 geometry_msgs::TransformStamped finger_initial, geometry_msgs::TransformStamped finger_current)
{
  double initial_lenght = sqrt(pow((thumb_initial.transform.translation.x -  finger_initial.transform.translation.x),2)
      + pow((thumb_initial.transform.translation.y - finger_initial.transform.translation.y),2)
      + pow((thumb_initial.transform.translation.z - finger_initial.transform.translation.z),2)
      ); 

  ROS_INFO("Initial lenght: %f", initial_lenght);
  double current_lenght = sqrt(pow((thumb_current.transform.translation.x -  finger_current.transform.translation.x),2)
      + pow((thumb_current.transform.translation.y - finger_current.transform.translation.y),2)
      + pow((thumb_current.transform.translation.z - finger_current.transform.translation.z),2)
      ); 
  ROS_INFO("Current lenght: %f", current_lenght);

  double stretchRatio = current_lenght / initial_lenght;

  return stretchRatio;

}


int main(int argc, char** argv){
  ros::init(argc, argv, "shadow_tf_listener");
  ros::NodeHandle node;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0; 
    
  ros::Rate rate(10.0);
  
  ofstream myfile;
  ofstream tactil_file;
  myfile.open ("/home/aurova/Desktop/pruebas/resultados/positions.txt");
  if(myfile.is_open())
    ROS_INFO("Archivo posiciones abierto");
   
  tactil_file.open ("/home/aurova/Desktop/pruebas/resultados/pressure.txt");
  if(tactil_file.is_open())
    ROS_INFO("Archivo presion abierto");
  
  int iteration = 0;
  double displacement = 0;
  double strain = 0;
  double young_modulus = 0;

  geometry_msgs::TransformStamped th_initial_transf;
  geometry_msgs::TransformStamped ff_initial_transf;
  geometry_msgs::TransformStamped mf_initial_transf;
  geometry_msgs::TransformStamped rf_initial_transf;
  geometry_msgs::TransformStamped lf_initial_transf;

  geometry_msgs::TransformStamped th_current_transf;
  geometry_msgs::TransformStamped ff_current_transf;

  while (node.ok()){
    geometry_msgs::TransformStamped transformSt;
    // get timestamp
    try{
      // obtener posiciones tips: 
      
      // transform -> get pose ff
      transformSt = tfBuffer.lookupTransform("forearm", "thtip",
                               ros::Time(0));
      
      if(iteration == 0)
      {
        th_initial_transf = transformSt;
        th_current_transf = transformSt;
      } 
      else
        th_current_transf = transformSt;
      
      displacement = sqrt(pow((transformSt.transform.translation.x-th_initial_transf.transform.translation.x),2)
			+ pow((transformSt.transform.translation.y-th_initial_transf.transform.translation.y),2)
			+ pow((transformSt.transform.translation.z-th_initial_transf.transform.translation.z),2)
			); 
      
      //transform.getRotaton();
      std::cout << "x " << transformSt.transform.translation.x;
      std::cout << " ";
      std::cout << "Y " << transformSt.transform.translation.y;
      std::cout << " ";
      std::cout << "Z " << transformSt.transform.translation.z;
      std::cout << " ";  
      
      myfile << transformSt.transform.translation.x;
      myfile << " ";
      myfile << transformSt.transform.translation.y;
      myfile << " ";
      myfile << transformSt.transform.translation.z;
      myfile << " ";
      myfile << displacement;
      myfile << " ";

    
      
      
      // transform -> get pose ff
      transformSt = tfBuffer.lookupTransform("forearm", "fftip",
                               ros::Time(0));
      
      if(iteration == 0){
        ff_initial_transf = transformSt;
        ff_current_transf = transformSt;
      }
      else
        ff_current_transf = transformSt;

      displacement = sqrt(pow((transformSt.transform.translation.x-ff_initial_transf.transform.translation.x),2)
		  + pow((transformSt.transform.translation.y-ff_initial_transf.transform.translation.y),2)
		  + pow((transformSt.transform.translation.z-ff_initial_transf.transform.translation.z),2)
		  ); 
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      myfile << transformSt.transform.translation.x;
      myfile << " ";
      myfile << transformSt.transform.translation.y;
      myfile << " ";
      myfile << transformSt.transform.translation.z;
      myfile << " ";
      myfile << displacement;
      myfile << " ";
      
      // transform -> get pose mf
      transformSt = tfBuffer.lookupTransform("forearm", "mftip",
                               ros::Time(0));
      if(iteration == 0) mf_initial_transf = transformSt;
      
      displacement = sqrt(pow((transformSt.transform.translation.x-mf_initial_transf.transform.translation.x),2)
		  + pow((transformSt.transform.translation.y-mf_initial_transf.transform.translation.y),2)
		  + pow((transformSt.transform.translation.z-mf_initial_transf.transform.translation.z),2)
		  ); 
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      myfile << transformSt.transform.translation.x;
      myfile << " ";
      myfile << transformSt.transform.translation.y;
      myfile << " ";
      myfile << transformSt.transform.translation.z;
      myfile << " ";
      myfile << displacement;
      myfile << " ";
      
      // transform -> get pose rf
      transformSt = tfBuffer.lookupTransform("forearm", "rftip",
                               ros::Time(0));
      if(iteration == 0) rf_initial_transf = transformSt;
      
      displacement = sqrt(pow((transformSt.transform.translation.x-rf_initial_transf.transform.translation.x),2)
		  + pow((transformSt.transform.translation.y-rf_initial_transf.transform.translation.y),2)
		  + pow((transformSt.transform.translation.z-rf_initial_transf.transform.translation.z),2)
		  ); 
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      myfile << transformSt.transform.translation.x;
      myfile << " ";
      myfile << transformSt.transform.translation.y;
      myfile << " ";
      myfile << transformSt.transform.translation.z;
      myfile << " ";
      myfile << displacement;
      myfile << " ";
      
      // transform -> get pose lf
      transformSt = tfBuffer.lookupTransform("forearm", "lftip",
                               ros::Time(0));
      if(iteration == 0) lf_initial_transf = transformSt;
      
      displacement = sqrt(pow((transformSt.transform.translation.x-lf_initial_transf.transform.translation.x),2)
		  + pow((transformSt.transform.translation.y-lf_initial_transf.transform.translation.y),2)
		  + pow((transformSt.transform.translation.z-lf_initial_transf.transform.translation.z),2)
		  ); 
      /**
      myfile << transform.getOrigin().x();
      myfile << " ";
      myfile << transform.getOrigin().y();
      myfile << " ";
      myfile << transform.getOrigin().z();
      myfile << " ";
      */
      
      myfile << transformSt.transform.translation.x;
      myfile << " ";
      myfile << transformSt.transform.translation.y;
      myfile << " ";
      myfile << transformSt.transform.translation.z;
      myfile << " ";
      myfile << displacement;
      myfile << " ";
      
      // Guardar en archivos posiciones (posth, posff, posmf, posrf, poslf, timestamp)
    
      /**myfile << "\n"; */
      myfile << iteration;
      myfile << "\n";
      
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    
    // Obtener mapa de presión
    if (pressure_client.call(srv_pressure))
    {
      // guardar en archivos fuerza/presión

	  tactil_file << srv_pressure.response.applied_force[0];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.applied_force[1];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.applied_force[2];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.applied_force[3];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.applied_force[4];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.force_deviation[0];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.force_deviation[1];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.force_deviation[2];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.force_deviation[3];
	  tactil_file << " ";
	  tactil_file << srv_pressure.response.force_deviation[4];
	  tactil_file << " ";
	  tactil_file << iteration;
	  tactil_file << "\n";



    // Obtener strain (deformacion)   stretch ratio = l /L  ; l = longitud actual; L = longitud inicial
    strain = getStretchRatio(th_initial_transf,th_current_transf,ff_initial_transf,ff_current_transf);
    young_modulus = (srv_pressure.response.applied_force[0] + srv_pressure.response.applied_force[1]) / strain;

    ROS_INFO("Current stretch ratio : %f", strain);
    ROS_INFO("Current young modulus: %f", young_modulus);

    }
    else
    {
      ROS_ERROR("Failed to call service pressure");
      return 1;
    }
    iteration++;
    rate.sleep();
  }
  
  myfile.close();
  tactil_file.close();

  ROS_INFO("Archivo cerrado");
  return 0;
};
