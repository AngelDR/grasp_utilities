/* Author: Angel Delgado
 Organization: Universidad de Alicante
 Comentarios: codigo provisional -> hacer modular
 */

#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/SVD> 

 // Files
#include <iostream>
#include <fstream>
#include <math.h>

using namespace Eigen;
using namespace std;


/**
Main:  nodo control tactil
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dh_params");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string tip_name, group_name, f_desired_param, sensor_frame;
  std::string base_name[5] = {"ffknuckle","mfknuckle","rfknuckle","lfmetacarpal","thbase"};
  std::string ff_links[6] = {"palm","ffknuckle","ffproximal","ffmiddle","ffdistal","fftip"};
  std::string mf_links[6] = {"palm","mfknuckle","mfproximal","mfmiddle","mfdistal","mftip"};
  std::string rf_links[6] = {"palm","rfknuckle","rfproximal","rfmiddle","rfdistal","rftip"};
  std::string lf_links[6] = {"palm","lfmetacarpal","lfknuckle","lfproximal","lfmiddle","lfdistal"};
  std::string th_links[6] = {"palm","thbase","thproximal","thhub","thmiddle","thdistal"};
  std::string finger_name[5] = {"first finger","middle finger","ring finger","little finger","thumb"};

  // Clases para transformaciones con tf2
  tf2_ros::Buffer tfBuffer;
  tf::TransformListener tfListener; 
  geometry_msgs::TransformStamped _transf;

  // File:   
  ofstream DH_file;
  DH_file.open ("/home/aurova/Desktop/pruebas/resultados/DH/dh_params.txt");
  if(DH_file.is_open())
    ROS_INFO("Archivo abierto");

  Matrix4d transformation_matrix;
  ros::Time now = ros::Time::now();
  tf::StampedTransform transform_s;
  try{
  	tfListener.waitForTransform("palm", "ffknuckle", now, ros::Duration(2.0));
	tfListener.lookupTransform("palm", "ffknuckle", now, transform_s);
	}
	catch(tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
	}


  // Transformaciones base:
  for(int i=0; i<5 ; i++){

	try{
	  ros::Time now = ros::Time::now();
	  tfListener.waitForTransform("palm", base_name[i], now, ros::Duration(2.0));
	  tfListener.lookupTransform("palm", base_name[i], now, transform_s);
	  tf::Quaternion q = transform_s.getRotation();
	  tf::Matrix3x3 rotation_matrix(q);
	  transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform_s.getOrigin().getX(), 
	                            rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform_s.getOrigin().getY(),
	                            rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform_s.getOrigin().getZ(),
	                            0,0,0,1;

	   // Escribir en archivo:
	    DH_file << "Finger: " << finger_name[i];
	    DH_file << "\n" << "Base Transformation matrix: \n";
	   	DH_file <<  rotation_matrix[0].getX() << " " << rotation_matrix[0].getY() << " " << rotation_matrix[0].getZ() << " " << transform_s.getOrigin().getX() << "\n" 
	            <<  rotation_matrix[1].getX() << " " << rotation_matrix[1].getY() << " " << rotation_matrix[1].getZ() << " " << transform_s.getOrigin().getY() << "\n"
	            <<  rotation_matrix[2].getX() << " " << rotation_matrix[2].getY() << " " << rotation_matrix[2].getZ() << " " << transform_s.getOrigin().getZ() << "\n"
	            << 0 << " " << 0 << " " << 0 << " " << 1 << "\n";

	    DH_file << "\n";

	}catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
		continue;
	}
	}


	// DH params:
	DH_file << "DH PARAMETERS \n";

	double d, theta, r, alpha;

	std:string joints[6];
  	for(int i=0; i<5 ; i++){
  		if(i==0){ joints[0] = ff_links[0]; joints[1] = ff_links[1]; joints[2] = ff_links[2]; joints[3] = ff_links[3]; joints[4] = ff_links[4]; joints[5] = ff_links[5];}
  		else if(i==1) { joints[0] = mf_links[0]; joints[1] = mf_links[1]; joints[2] = mf_links[2]; joints[3] = mf_links[3]; joints[4] = mf_links[4]; joints[5] = mf_links[5];}
  		else if(i==2) { joints[0] = rf_links[0]; joints[1] = rf_links[1]; joints[2] = rf_links[2]; joints[3] = rf_links[3]; joints[4] = rf_links[4]; joints[5] = rf_links[5];}
  		else if(i==3) { joints[0] = lf_links[0]; joints[1] = lf_links[1]; joints[2] = lf_links[2]; joints[3] = lf_links[3]; joints[4] = lf_links[4]; joints[5] = lf_links[5];}
  		else if(i==4) { joints[0] = th_links[0]; joints[1] = th_links[1]; joints[2] = th_links[2]; joints[3] = th_links[3]; joints[4] = th_links[4]; joints[5] = th_links[5];}

  		DH_file << "Params for finger: " << finger_name[i] << "\n";

  		for(int j=0; j<5; j++){
			try{
			  ros::Time now = ros::Time::now();
			  tf::StampedTransform transform_s;
			  tfListener.waitForTransform(joints[j], joints[j+1], now, ros::Duration(1.0));
			  tfListener.lookupTransform(joints[j], joints[j+1], now, transform_s);
			  tf::Quaternion q = transform_s.getRotation();
			  tf::Matrix3x3 rotation_matrix(q);
			  transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform_s.getOrigin().getX(), 
			                            rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform_s.getOrigin().getY(),
			                            rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform_s.getOrigin().getZ(),
			                            0,0,0,1;

			   // Escribir en archivo:
			   d = transformation_matrix(2,3) * 1000;
			   theta = acos(transformation_matrix(0,0));
			   r = (transformation_matrix(0,3) / transformation_matrix(0,0)) * 1000;
			   alpha = asin(transformation_matrix(2,1));

			   DH_file << d << " " << theta << " " << r << " " << alpha << "\n"; 


			}catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
		}
		DH_file << "\n";

		// falta thtip
		try{
			  ros::Time now = ros::Time::now();
			  tf::StampedTransform transform_s;
			  tfListener.waitForTransform("thdistal", "thtip", now, ros::Duration(1.0));
			  tfListener.lookupTransform("thdistal", "thtip", now, transform_s);
			  tf::Quaternion q = transform_s.getRotation();
			  tf::Matrix3x3 rotation_matrix(q);
			  transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform_s.getOrigin().getX(), 
			                            rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform_s.getOrigin().getY(),
			                            rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform_s.getOrigin().getZ(),
			                            0,0,0,1;

			   // Escribir en archivo:
			   d = transformation_matrix(2,3) * 1000;
			   theta = acos(transformation_matrix(0,0));
			   r = (transformation_matrix(0,3) / transformation_matrix(0,0)) * 1000;
			   alpha = asin(transformation_matrix(2,1));

			   DH_file << d << " " << theta << " " << r << " " << alpha << "\n"; 


			}catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
	}
	DH_file.close();

  
  ros::shutdown();  
  return 0;
}
