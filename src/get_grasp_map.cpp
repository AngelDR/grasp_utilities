#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>

#include "std_msgs/Float64.h"
#include "tekscan_client/GetPressureMap.h"
#include <math.h>
#include <iostream>
#include <fstream>
#define PI 3.14159265

#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <visualization_msgs/Marker.h>
#include <string>

using namespace Eigen;
using namespace std;

// Funcion para obtener valor singular minimo de la matriz de agarre
double getMinimumSingularValue(const MatrixXd &grasp_matrix)
{   
  MatrixXd base = grasp_matrix * grasp_matrix.transpose();
  SelfAdjointEigenSolver<MatrixXd> eigen_solver;
  eigen_solver.compute(base);
  //ROS_INFO_STREAM("Eigenvalues of the grasp matrix are: \n" << eigen_solver.eigenvalues().transpose());
  // singular values:

  std::vector<double> v_singular_values;
  for(int i=0; i<6 ; i++){
    v_singular_values.push_back(sqrt(eigen_solver.eigenvalues()[i]));
  }
  double min = v_singular_values[0];
  for(int i=0; i<6 ; i++){
    if(v_singular_values[i] < min) min = v_singular_values[i];
  }
  return min;
}


// Funcion para obtener valor singular maximo de la matriz de agarre
double getMaximumSingularValue(const MatrixXd &grasp_matrix)
{   
  MatrixXd base = grasp_matrix * grasp_matrix.transpose();
  SelfAdjointEigenSolver<MatrixXd> eigen_solver;
  eigen_solver.compute(base);
  //ROS_INFO_STREAM("Eigenvalues of the grasp matrix are: \n" << eigen_solver.eigenvalues().transpose());
  // singular values:

  std::vector<double> v_singular_values;
  for(int i=0; i<6 ; i++){
    v_singular_values.push_back(sqrt(eigen_solver.eigenvalues()[i]));
  }
  double max = v_singular_values[0];
  for(int i=0; i<6 ; i++){
    if(v_singular_values[i] > max) max = v_singular_values[i];
  }
  return max;
}

// Funcion para obtner indice de isotropia de la matriz de agarre
double isotropyOfGraspMap(const MatrixXd &grasp_matrix){
  return getMinimumSingularValue(grasp_matrix) / getMaximumSingularValue(grasp_matrix);
}



int main(int argc, char** argv){
	ros::init(argc, argv, "get_grasp_map");
	ros::NodeHandle node;

	tf2_ros::Buffer tfBuffer;
	tf::TransformListener tfListener; 

  ros::Rate rate(10.0);
 	Matrix4d transformation_matrix;

  // Marker visualizacion valores de grasp Matrix: (texto con valor singular minimo/maximo/isotropia) 
  ros::Publisher grasp_marker_pub = node.advertise<visualization_msgs::Marker>( "grasp_matrix_marker", 0 );
  visualization_msgs::Marker  grasp_matrix_marker;

  grasp_matrix_marker.header.frame_id = "shadow_world_frame";
  grasp_matrix_marker.ns = "grasp";
  grasp_matrix_marker.id = 5;
  grasp_matrix_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  grasp_matrix_marker.action = visualization_msgs::Marker::ADD;
  // Todo: anadir params
  grasp_matrix_marker.scale.x = 0.1; // Param
  grasp_matrix_marker.scale.y = 0.1; // Param
  grasp_matrix_marker.scale.z = 0.1; //
  grasp_matrix_marker.color.a = 1.0; // Don't forget to set the alpha!
  grasp_matrix_marker.color.r = 0.0;
  grasp_matrix_marker.color.g = 1.0;
  grasp_matrix_marker.color.b = 0.0;

  std::string lista_frames[6] = {"object_frame","thtip","fftip","mftip","rftip","lftip"};
  tf::StampedTransform transform[6];


  // Archivos 
  ofstream grasp_map_file;
  grasp_map_file.open ("/home/aurova/Desktop/pruebas/resultados/grasp_map_values.txt");
  if(grasp_map_file.is_open())
    ROS_INFO("Archivo para valores de grasp matrix");
  int iteration = 0;

	while (node.ok()){

    // Get param : numero dedos
    int num_fingers_exp;
    if (node.getParam("/grasp_reconfiguration/number_fingers", num_fingers_exp))
    {
      ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
    }
    else{
      num_fingers_exp = 4;
    }
    MatrixXd grasp_matrix(6,4*(num_fingers_exp));

    // Rellenar grasp_map : frame referencia = forearm
    for(int id = 0;id <= num_fingers_exp; id++)
    {
      try{
        ros::Time now = ros::Time::now();
        tfListener.waitForTransform("forearm", lista_frames[id], now, ros::Duration(3.0));
        tfListener.lookupTransform("forearm", lista_frames[id], now, transform[id]);
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    // Rellenar grasp_map
    for(int num_fila = 0; num_fila < 3; num_fila++){
        int id_finger = 1;
        for(int num_col = 0; num_col < 4*num_fingers_exp ; num_col+=4){
          tf::Quaternion q = transform[id_finger].getRotation();
          tf::Matrix3x3 rotation_matrix(q);
          grasp_matrix(num_fila,num_col)=rotation_matrix[num_fila].getX(); 
          grasp_matrix(num_fila,num_col+1)=rotation_matrix[num_fila].getY();
          grasp_matrix(num_fila,num_col+2)=rotation_matrix[num_fila].getZ();
          grasp_matrix(num_fila,num_col+3)=0;
          id_finger++;
      }
    }

    // Obtener vectores centro-contacto para rellenar segunda parte grasp_map
    MatrixXd matriz_desplazamientos(num_fingers_exp,3);
    for(int i = 0; i < num_fingers_exp; i++){
      matriz_desplazamientos(i,0) = transform[i+1].getOrigin().getX() - transform[0].getOrigin().getX();
      matriz_desplazamientos(i,1) = transform[i+1].getOrigin().getY() - transform[0].getOrigin().getY();
      matriz_desplazamientos(i,2) = transform[i+1].getOrigin().getZ() - transform[0].getOrigin().getZ(); 
    }


    // Rellenar segunda parte
    // Rellenar grasp_map
    for(int num_fila = 3; num_fila < 6; num_fila++){
        int id_finger = 1;
        for(int num_col = 0; num_col < 4*num_fingers_exp ; num_col+=4){

          // obtener [p x]
          tf::Quaternion q = transform[id_finger].getRotation();
          tf::Matrix3x3 rotation_matrix(q);  
          Matrix3d matriz_desplazamiento_i;
          matriz_desplazamiento_i <<  0, -matriz_desplazamientos(id_finger-1,2),matriz_desplazamientos(id_finger-1,1), 
                                    matriz_desplazamientos(id_finger-1,2),0,-matriz_desplazamientos(id_finger-1,0),
                                    -matriz_desplazamientos(id_finger-1,1),matriz_desplazamientos(id_finger-1,0),0;    

          Matrix3d rotacion_aux;
          rotacion_aux <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), 
                        rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(),
                        rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ();
          matriz_desplazamiento_i = matriz_desplazamiento_i * rotacion_aux;

          grasp_matrix(num_fila,num_col)=matriz_desplazamiento_i(num_fila-3,0); 
          grasp_matrix(num_fila,num_col+1)=matriz_desplazamiento_i(num_fila-3,1);
          grasp_matrix(num_fila,num_col+2)=matriz_desplazamiento_i(num_fila-3,2);
          grasp_matrix(num_fila,num_col+3)=rotacion_aux(num_fila-3,2);
          id_finger++;
      }
    }

    ROS_INFO_STREAM("/Grasp matrix: \n" << grasp_matrix);
    std::string text_of_marker = "Minimo valor singular: " + boost::lexical_cast<std::string>(getMinimumSingularValue(grasp_matrix)) + " \nMaximo valor singular: " + boost::lexical_cast<std::string>(getMaximumSingularValue(grasp_matrix))
      + " \nIsotropia: " + boost::lexical_cast<std::string>(isotropyOfGraspMap(grasp_matrix)); 
    grasp_matrix_marker.text = text_of_marker;
    grasp_matrix_marker.pose.position.z = 1.0;
    grasp_marker_pub.publish(grasp_matrix_marker);


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
      grasp_map_file << boost::lexical_cast<std::string>(getMinimumSingularValue(grasp_matrix)) << " " << boost::lexical_cast<std::string>(getMaximumSingularValue(grasp_matrix))
      << " " << boost::lexical_cast<std::string>(isotropyOfGraspMap(grasp_matrix)) <<" ";
      grasp_map_file << iteration << "\n"; 
    }

    iteration++;
  }

  grasp_map_file.close();
};