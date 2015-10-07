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
#define PI 3.14159265

#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

using namespace Eigen;

int main(int argc, char** argv){
	ros::init(argc, argv, "get_grasp_map");
	ros::NodeHandle node;

	tf2_ros::Buffer tfBuffer;
	tf::TransformListener tfListener; 

  ros::Rate rate(10.0);
 	Matrix4d transformation_matrix;
  SelfAdjointEigenSolver<MatrixXd> eigen_solver;


 	// Get param : numero dedos
  int num_fingers_exp;
  if (node.getParam("/grasp_reconfigure/number_fingers", num_fingers_exp))
  {
    ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
  }
  else{
    num_fingers_exp = 4;
  }

  std::string lista_frames[6] = {"object_frame","thtip","fftip","mftip","rftip","lftip"};
  tf::StampedTransform transform[6];

	while (node.ok()){
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



          grasp_matrix(num_fila,num_col)=matriz_desplazamiento_i(num_fila,num_col); 
          grasp_matrix(num_fila,num_col+1)=matriz_desplazamiento_i(num_fila,num_col+1);
          grasp_matrix(num_fila,num_col+2)=matriz_desplazamiento_i(num_fila,num_col+2);
          grasp_matrix(num_fila,num_col+3)=rotacion_aux(num_fila,2);
          id_finger++;
      }
    }

    MatrixXd base = grasp_matrix * grasp_matrix.transpose();
    eigen_solver.compute(base);
    ROS_INFO_STREAM("Eigenvalues of the grasp matrix are: \n" << eigen_solver.eigenvalues().transpose());
    ROS_INFO_STREAM("/Grasp matrix: \n" << grasp_matrix);
  }

};