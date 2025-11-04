 /*
 * mht.cpp
 *
 *  Created aprox on: 07/07/2013 (aprox)
 *      Author: Ely Repiso
 */
// Copyright (C) from 2013-until now Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Ely Repiso
// All rights reserved.
/*
*
*      Created on: 2013 by Ely Repiso and published first as her TFC on 13/12/2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2- humble migration) 
*      Author: Ely Repiso (from 2013 and currently).
*      Furthermore, this code is a modification and extension extracted from the theory of the open source papers of Donald Reid IEEE Transaction on Automatic Control 1979 and Kai Oliver Arras ICRA2008. We never had their code, therefore, we implemented this code from scratch, only taking inspiration from their articles.
*     
*      License (for other authors that will not be the original one): CC BY-NC-ND 4.0 
*      (Attribution-NonCommercial-#NoDerivatives 4.0 International)
*       https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
*
*      This license does not allow other authors to modify or to take profit from these works. 
*      Then, for #modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
*  collaborations (for #journals with other researchers, formal collaborations between UPC and companies, and so on).
*
* Please, to use it cite: Vaquero, Victor, et al. "Low cost, robust and real time system for detecting and tracking moving objects to automate cargo handling in port terminals." Robot 2015: Second Iberian Robotics Conference: Advances in Robotics, Volume 2. Cham: Springer International Publishing, 2015.
*
*  Redistribution and use in source and binary forms, without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*   * Additionally, remember that the derivatives without collaboration 
*     Of the original authors are prohibited
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
***********************************************************
*/

#include "mht.h"
#include <stdio.h>
#include <math.h>
#include <list>


Cmht::Cmht() :
cargo_ANTS(false),
debug_cruces_fallos(false),
debug_odometry_(false),
debug_cruces_v_(false),
debug_cruces_2015_(false),
debug_groups_(false),
pruebas_C_(true),
counter_(0),
dt_(0.05),
total_dt_(0.0),
scene_tracks_people_prediction_(Cperson_abstract::Low_pass_linear_regression_filtering,Cperson_abstract::Manual_remove),
//scene_tracks_people_prediction_(Cperson_abstract::Linear_regression_filtering,Cperson_abstract::Manual_remove),
threshold_distance_(2), //TODO: ojo!!! está cambiada de 2 a 4, para probar lo de los super clusters! (distancia buen sin usar covarianzas es 2!!!!)
threshold_distance_groups_(2.5),
threshold_no_deteccion_(0.4), //threshold_no_deteccion_=0.4 the previous value that was well. (el normal, anterior.)
covariance_no_track_(20),
count_my_tracks_(1), //initialization of the counter of my tracks, the first time that I create an object mht.
count_my_clusters_actual_(1),
count_my_clusters_(1), //initialization of the counter of my tracks, the first time that I create an object mht.
threshold_remove_cluster_(0.4),
max_num_tracks_in_cluster_allowed_(6),
perimeter_distance_(10000.0),
vehicle_prediction_(Cperson_abstract::Low_pass_linear_regression_filtering,Cperson_abstract::Manual_remove),
total_px_(0.0),
total_py_(0.0),
total_oz_(0.0),
first_time_(true),
count_my_groups_(1),
debug_companion_akp_(false),//, // se pone a true para debugar la database.
generate_database_companion_(false), // TODO! si se usa fuera de generar database, poner a false.
fuse_tibi_and_teo_tracks_(false),
debug_fuse_tibi_and_teo_tracks_(false),
mht_velocity_margin_(0.1)
{
	std::string data_file="/home/erepiso/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/14_junio/txt_tracks/grupo1/general_results/results_database352_v4.txt";
	results_file_=data_file;
	new_matlab_file();

	// folders:
	// /home/ely7787/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/7_junio/txts_tracks/grupo1/results_database1.txt
	// /home/ely7787/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/8_junio/txts_tracks/grupo1/results_database1.txt
	// /home/ely7787/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/9_junio/txts_tracks/grupo1/results_database1.txt
	// /home/ely7787/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/12_junio/txts_tracks/grupo(1-9)/results_database1.txt
	// /home/ely7787/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/14_junio/txts_tracks/grupo(1-9)/results_database1.txt
	// /home/ely7787/bags/rosbags_akp_learning_parametros_fuerzas/rosbags_database/16_junio/txts_tracks/grupo(1-5)/results_database1.txt

}

void Cmht::new_matlab_file()
{
	iteration_=1;
//// //this->config_.map_path + "/" + this->config_.map_filename
	//std::string data_file, data_file2;
	//data_file="/home/ely7787/iri-lab/labrobotica/restricted/algorithms/people_prediction/branches/ely_people_prediction_companion/data_results/results";
	//data_file2=".txt";
	//std::string final_file= data_file + "/" + data_file2;
	std::ofstream fileMatlab;
	// const char *direc_file_name_;
    //direc_file_name_= results_file_.c_str();
    //home/ely7787/iri-lab/labrobotica/restricted/algorithms/people_prediction/branches/ely_people_prediction_companion_robot/1_data_results
    fileMatlab.open (results_file_.c_str(), std::ofstream::out | std::ofstream::app);
    // FILE HEADER
    fileMatlab << "% ***** Autogenerated File - database companion people tracks tibi and teo  ELY ***** \n\n";
    fileMatlab << "%  The file contains: all the tracks of people for one experiment of the database \n\n";
    fileMatlab << " id=[];\n"; // (iteration, elements of this iter)
    fileMatlab << " x=[];\n";
    fileMatlab << " y=[];\n";
    fileMatlab << " t=[];\n";
    fileMatlab << " vx=[];\n";
    fileMatlab << " vy=[];\n";
    fileMatlab << " pd=[];\n";
    fileMatlab << " Cxx=[];\n";
    fileMatlab << " Cxy=[];\n";
    fileMatlab << " Cyx=[];\n";
    fileMatlab << " Cyy=[];\n";
    fileMatlab.close();

}

void Cmht::save_tracks_in_txt_tibi_and_teo_companion_database(){

//}
//void Cmht::printToMatlab()
//{
	//std::cout << " (save tracks in file)  "<< std::endl;
	std::ofstream fileMatlab;
	fileMatlab.open (results_file_.c_str(), std::ofstream::out | std::ofstream::app);
    //fileMatlab << " clc,\n clear all,\n close all \n\n\n";
	fileMatlab << " \n \n \n % New iteration! \n";


	unsigned int element=1;
	//for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	for ( unsigned int e=0; e<exit_params_mht_.size(); e++)
	{
		// see if the track is confirmed
			SpointV_cov track_act=exit_params_mht_[e].getSpointV_cov();

			// IMPORTANT, save the time with all decimals, find in google cout full precision (is related to the form that the cout show the number)
			//std::cout.precision(17);

			double time=track_act.time_stamp;
			double horas=(time/1000000000000.0);

			double resto_h=fmod( time, 1000000000000.0 );
			//std::cout << " horas= " <<horas<< "; (min) resto_h="<<resto_h<<" float_time_stamp_"<<float_time_stamp_/1000000000000.0<<"; float_time_stamp_="<<float_time_stamp_<< std::endl;
			//std::iostream.precision(17);

			//if(exit_params_mht_[e].id<30){
				fileMatlab <<std::setprecision(17);

				fileMatlab << " \n \n \n % New track \n";
				fileMatlab << "\n id("<<iteration_<<","<<element<<")="<<exit_params_mht_[e].id<<";\n";
				fileMatlab << " x("<<iteration_<<","<<element<<")="<<track_act.x<<";\n";
				fileMatlab << " y("<<iteration_<<","<<element<<")="<<track_act.y<<";\n";

				std::setprecision(17);
				fileMatlab << " t("<<iteration_<<","<<element<<")="<<resto_h<<";\n";
				//fileMatlab.write(reinterpret_cast<char*>(&resto_h), sizeof(resto_h));
				//fprintf(fileMatlab,'%6.2f %12.8f\r\n',A);

				fileMatlab << "\n vx("<<iteration_<<","<<element<<")="<<track_act.vx<<";\n";
				fileMatlab << "vy("<<iteration_<<","<<element<<")="<<track_act.vy<<";\n";
				fileMatlab << "pd("<<iteration_<<","<<element<<")="<<exit_params_mht_[e].probability_of_detection<<";\n";
				fileMatlab << "Cxx("<<iteration_<<","<<element<<")="<<track_act.cov[0]<<";\n";
				fileMatlab << "Cxy("<<iteration_<<","<<element<<")="<<track_act.cov[1]<<";\n";
				fileMatlab << "Cyx("<<iteration_<<","<<element<<")="<<track_act.cov[4]<<";\n";
				fileMatlab << "Cyy("<<iteration_<<","<<element<<")="<<track_act.cov[5]<<";\n";
			//}


			//std::cout << " id= " <<exit_params_mht_[e].id<< "; t="<<track_act.time_stamp/1000000000000.0<<"; pd= "<<exit_params_mht_[e].probability_of_detection<< std::endl;
			//std::cout << " (x= " <<track_act.x<<"; y="<<track_act.y<<") ; (vx="<<track_act.vx<<"; vy="<<track_act.vy<<")"<< std::endl;
			//std::cout << " [Cxx= " <<track_act.cov[0]<<"; Cxy="<<track_act.cov[1]<<"] \n ; [Cyx="<<track_act.cov[4]<<"; Cyy="<<track_act.cov[5]<<"]"<< std::endl;


			element++;

	}
	if(!exit_params_mht_.empty()){
		iteration_++;
	}


	// salvar obstaculos por si hacen falta luego. No se puede, se tendra que lanzar el companion y guardarlos de all o ver donde estan y ponerlos hard coded en el archivo.


	fileMatlab.close();
}

Cmht::~Cmht()
{
}

void Cmht::get_total_velocity(Cvelocity velocity)
{

	// pose del vehiculo en esta iteración!!!
	total_px_ = velocity.get_total_px();
	total_py_ = velocity.get_total_py();
	total_oz_ = velocity.get_total_oz();
	time_act_veh_ = velocity.get_total_time();
}

void Cmht::update_total_velocity()
{

	//std::cout << " ENTRO EN update_total_velocity (mht)" << std::endl;

	int id = 3;
	if (first_time_)
	{

		SpointV_cov vehicle_pose_associate;
		vehicle_pose_associate = SpointV_cov();
		std::vector<double> vehicle_cov;
		vehicle_cov.reserve(16);
		vehicle_cov.resize(16, 0.0);
		vehicle_cov = vehicle_pose_associate.cov;
		vehicle_cov[0] = 0.5; // cov_xx
		vehicle_cov[1] = 0.0; //cov_xy
		vehicle_cov[4] = 0.0; //cov_yx
		vehicle_cov[5] = 0.5; //cov_yy

		std::vector<SdetectionObservation> vehicle_track;
		vehicle_track.clear();

		vehicle_pose_ = SpointV_cov(total_px_, total_py_, time_act_veh_, 0.0,0.0, vehicle_cov);
		//;

		vehicle_track.push_back(SdetectionObservation(id, vehicle_pose_));
		if(debug_cruces_fallos){
		std::cout << "\n vehicle_track_.back().print() " << std::endl;
		vehicle_track.back().print();
		}
    // update_scene(const std::vector<SdetectionObservation>& observation,bool& we_have_person_companion)
    bool we_have_person_companion=false;
		vehicle_prediction_.update_scene(vehicle_track,we_have_person_companion);
		first_time_ = false;

	}
	else
	{

		// prediccion y busqueda vehiculo en objeto de Gonzalo.
		Cperson_abstract* vehicle_obj;
		vehicle_prediction_.find_person(id, &vehicle_obj);

		SpointV_cov vehicle_pose_associate;
		vehicle_pose_associate = SpointV_cov();
		vehicle_pose_associate = vehicle_obj->pointV_propagation(dt_);

		std::vector<double> vehicle_cov;
		vehicle_cov.reserve(16);
		vehicle_cov.resize(16, 0.0);
		vehicle_cov = vehicle_pose_associate.cov;
		vehicle_cov[0] = 0.5; // cov_xx
		vehicle_cov[1] = 0.0; //cov_xy
		vehicle_cov[4] = 0.0; //cov_yx
		vehicle_cov[5] = 0.5; //cov_yy

		std::vector<SdetectionObservation> vehicle_track;
		vehicle_track.clear();

		vehicle_pose_ = SpointV_cov(total_px_, total_py_, time_act_veh_,vehicle_pose_associate.vx, vehicle_pose_associate.vy,vehicle_cov);
		//;

		vehicle_track.push_back(SdetectionObservation(id, vehicle_pose_));
		if(debug_cruces_fallos){
		std::cout << "\n vehicle_track_.back().print() " << std::endl;
		vehicle_track.back().print();
		if (vehicle_pose_associate.vx != 0)
		{
			double vehicle_orientation = vehicle_pose_associate.orientation();
			std::cout << "\n vehicle_orientation=" << vehicle_orientation<< std::endl;
		}
		}

    // update_scene(const std::vector<SdetectionObservation>& observation,bool& we_have_person_companion)
    bool we_have_person_companion=false;

		vehicle_prediction_.update_scene(vehicle_track,we_have_person_companion);
	}
	//std::cout << " SALGO DE update_total_velocity (mht)" << std::endl;
}

void Cmht::mht_odom_update(Codometry & odome)
{
	// Function that change the window of SpointV_cov of each person for made its predictions.
	// translate from tha last frame to the new robot local frame the position, velocity and the matrix of covariance



	//if(cargo_ANTS){ //TODO: ver si para tibi y dabo también iria bien.
		update_total_velocity();
	//}


		if(debug_odometry_){
				std::cout << " ENTRO EN odom_update (mht)" << std::endl;
		}
	 	if(debug_odometry_){
			double linear_vx=vehicle_pose_.vx;
			double linear_vy=vehicle_pose_.vy;		
			std::cout << "linear_vx" <<  vehicle_pose_.vx << std::endl;
			std::cout << "linear_vy" <<  vehicle_pose_.vy << std::endl;
		}


	double R; //Traslación del frame
	double thetaZ; // rotación del frame que te da la nueva odometria.
	double dt_act;
	Eigen::Matrix3d odom_eigen_tf;

	R = odome.get_translation();
	thetaZ = odome.get_rotation();
	dt_act = odome.get_dt();
	odom_eigen_tf = odome.get_global_tf_odom();
	//std::cout<< " \n odom_eigen_tf= \n" <<odom_eigen_tf<< std::endl;
	
	std::vector<double> vect_odom_eigen_tf;
	vect_odom_eigen_tf.reserve(9);
	vect_odom_eigen_tf.resize(9, 0.0);
	vect_odom_eigen_tf[0] = (double) odom_eigen_tf(0, 0);
	vect_odom_eigen_tf[1] = (double) odom_eigen_tf(0, 1); 
	vect_odom_eigen_tf[2] = (double) odom_eigen_tf(0, 2);
	vect_odom_eigen_tf[3] = (double) odom_eigen_tf(1, 0); 
	vect_odom_eigen_tf[4] = (double) odom_eigen_tf(1, 1);
	vect_odom_eigen_tf[5] = (double) odom_eigen_tf(1, 2); 
	vect_odom_eigen_tf[6] = (double) odom_eigen_tf(2, 0); 
	vect_odom_eigen_tf[7] = (double) odom_eigen_tf(2, 1); 
	vect_odom_eigen_tf[8] = (double) odom_eigen_tf(2, 2); 

	

	actual_wz_=odome.get_actual_velocity_Wz_();

	if((R==0)&&(thetaZ==0))
	{
		R = odome.get_ant_translation();
		thetaZ = odome.get_ant_rotation();
	}

	if(debug_odometry_){
		std::cout << " R=" << R << std::endl;
		std::cout << " thetaZ=" << thetaZ << std::endl;
		std::cout << " dt_act=" << dt_act << std::endl;
	}
	if(debug_odometry_){
	std::cout << " vehicle velocity with odometry" << std::endl;
	}
	double linear_dVx=0;
	double linear_dVy=0;
	//double linear_dVx2=0;
	//double linear_dVy2=0;
	//double linear_dVx3=0;
	//double linear_dVy3=0;
	//double linear_dVx4=0;
	//double linear_dVy4=0;
	//double linear_dVx5=0;
	//double linear_dVy5=0;
	double inc_vx=odome.get_linear_vx();
	double inc_vy=odome.get_linear_vy();
	double inc_dt=odome.get_dt();
	if(first_odom_){
		first_odom_=false;
	}else{

		linear_dVx=inc_vx;
		linear_dVy=inc_vy;
		linear_vx_=inc_vx;
		linear_vy_=inc_vy;
		/*linear_dVx2=inc_vx*dt_;
		linear_dVy2=inc_vy*dt_;
		linear_dVx3=inc_vx/dt_;
		linear_dVy3=inc_vy/dt_;
		linear_dVx4=inc_vx*inc_dt;
		linear_dVy4=inc_vy*inc_dt;
		linear_dVx5=inc_vx/inc_dt;
		linear_dVy5=inc_vy/inc_dt;*/
	}
	if(debug_odometry_){
	std::cout <<"linear_Vx="<< linear_dVx<< std::endl;
	std::cout <<"linear_Vy="<< linear_dVy << std::endl;
	/*std::cout <<" con * dt, linear_dVx2="<< linear_dVx2<< std::endl;
	std::cout <<"con * dt, linear_dVy2="<< linear_dVy2 << std::endl;
	std::cout <<" con / dt, linear_dVx3="<< linear_dVx3<< std::endl;
	std::cout <<"con / dt, linear_dVy3="<< linear_dVy3 << std::endl;
	std::cout <<" con * Odometry_dt, linear_dVx4="<< linear_dVx4<< std::endl;
	std::cout <<"con * Odometry_dt, linear_dVy4="<< linear_dVy4 << std::endl;
	std::cout <<" con / Odometry_dt, linear_dVx5="<< linear_dVx5<< std::endl;
	std::cout <<"con / Odometry_dt, linear_dVy5="<< linear_dVy5 << std::endl;*/
	}

	double dx = odome.get_dx_();
	double dy = odome.get_dy_();
	//std::cout << "!!! (mht) fuera for!!!" << std::endl;
	if(debug_companion_akp_){
	std::cout << "mytracks_list_.size()="<<mytracks_list_.size() << std::endl;
	}
	
	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
	   //std::cout << "!!! (mht) dentro for!!!" << std::endl;
		int id = it_myTrack->get_idTracker();
		//std::cout<< "\n ID Kalman correct to propagation track " << id << std::endl;
		SpointV_cov kalman_state = it_myTrack->get_kalman_update();

		double dist_robot_people_x=kalman_state.x;
		double dist_robot_people_y=kalman_state.y;
		//double v_rot_x= -sqrt((dist_robot_people_y*actual_wz_)*(dist_robot_people_y*actual_wz_));
		//double v_rot_y= sqrt((dist_robot_people_x*actual_wz_)*(dist_robot_people_x*actual_wz_));
		double v_rot_x=-dist_robot_people_y*actual_wz_; //+linear_vx_;
		double v_rot_y= dist_robot_people_x*actual_wz_;//+linear_vx_;

		if(debug_odometry_){
			std::cout<< "\n v_rot_x="<< v_rot_x << std::endl;
			std::cout<< "\n v_rot_y="<< v_rot_y << std::endl;

			std::cout<< "\n vehicle_pose_.vx="<< vehicle_pose_.vx << std::endl;
			std::cout<< "\n vehicle_pose_.vy="<< vehicle_pose_.vy << std::endl;

			std::cout<< "\n v_rot_x*dt_act ="<< v_rot_x*dt_act << std::endl;
			std::cout<< "\n v_rot_y*dt_act ="<< v_rot_y*dt_act  << std::endl;
		}
		//double inc_ang_rot_velocity=sqrt((v_rot_x)*(v_rot_x)+(v_rot_y)*(v_rot_y))*dt_act;
		//std::cout<< "\n inc_ang_rot_velocity="<< inc_ang_rot_velocity << std::endl;

		//thetaZ=thetaZ-sqrt((v_rot_x*dt_act/4)*(v_rot_x*dt_act/4)+(v_rot_y*dt_act/4)*(v_rot_y*dt_act/4));
		//thetaZ=thetaZ+inc_ang_rot_velocity;
		//std::cout<< "\n Kalman correct to propagation track " << std::endl;
		//kalman_state.print();
		bool debug;
		debug=true;
		scene_tracks_people_prediction_.rotate_and_traslate_scene(id,R,thetaZ,dx,dy,v_rot_x,v_rot_y,vect_odom_eigen_tf);
	}

	//odome.reset_odometry();

}


void Cmht::mht_Cscene_se_in_tracks()
{
	scene_tracks_people_prediction_.see_tracks_on_scene();
}


void Cmht::mht_update(std::vector<Sdetection>& obs, double external_time_stamp)
{
// The main function for mht.
	if(debug_companion_akp_){
		std::cout<< std::endl<< std::endl<< std::endl;
		std::cout<< " INICIO ITER mht_update  obs.size()"<< obs.size()<< std::endl;
		std::cout<< "generate_database_companion_="<<generate_database_companion_<< std::endl;
		std::cout<< "fuse_tibi_and_teo_tracks_="<<fuse_tibi_and_teo_tracks_<< std::endl;
		std::cout<< "debug_fuse_tibi_and_teo_tracks_="<<debug_fuse_tibi_and_teo_tracks_<<"; threshold_distance_="<<threshold_distance_<< std::endl;
		std::cout<<"; threshold_distance_="<<threshold_distance_<< std::endl;
	// SEE TRACKS IN SCENE:
	//scene_tracks_people_prediction_.see_tracks_on_scene();
	}

	//if(obs.size()>0){
	clock_t tStart = clock();
	if(debug_cruces_fallos){
		std::cout << " ENTRO EN mht_update (!!!)" << std::endl;
	}

	if (cargo_ANTS)
	{
		threshold_distance_ = 1;
		aug_cov_prediction = 5.0;
		covariance_no_track_ = 1000;
		scene_tracks_people_prediction_.set_filtering_time_window(2.5);

		//		std::cout<< " DETECCIONES CON (0,0) (!!!)" << std::endl;
		// sacar por pantalla las detecciones.
		/*	for(unsigned int it_obs=0; it_obs<obs.size(); it_obs++){
		 obs[it_obs].print_spoint_DET();
		 }*/

		// INICIO eliminar detecciones en (0,0)
		// eliminate the Sdetection outside the region around the robot. Find the position of the obs to eliminate.
		std::vector<unsigned int> pos_obs_to_eliminate0;
		for (unsigned int it_obs = 0; it_obs < obs.size(); it_obs++)
		{
			//obs[it_obs].print_spoint_DET();
			if ((obs[it_obs].x == 0) && (obs[it_obs].y == 0))
			{
				pos_obs_to_eliminate0.push_back(it_obs);
			}
		}

		//eliminate the obs
		for (unsigned int it_obs_erase = 0;it_obs_erase < pos_obs_to_eliminate0.size(); it_obs_erase++)
		{
			obs.erase(obs.begin() + pos_obs_to_eliminate0[it_obs_erase]);
			for (unsigned int it = 0; it < pos_obs_to_eliminate0.size(); it++)
			{
				pos_obs_to_eliminate0[it] = pos_obs_to_eliminate0[it] - 1;
			}
		}

		/*std::cout << " DETECCIONES SIN!! (0,0) (!!!)" << std::endl;
		for (unsigned int it_obs = 0; it_obs < obs.size(); it_obs++)
		{
			obs[it_obs].print_spoint_DET();
		}*/

	}
	else
	{
		//std::cout << " aug_cov_prediction="<<aug_cov_prediction << std::endl;
		//aug_cov_prediction = 0.4; // la buena para global es de 0.4
		//scene_tracks_people_prediction_.set_filtering_time_window(2.5); // le baje el tiempo de computo de la velocidad!!! (para que sea más exacta a la de la persona)
		scene_tracks_people_prediction_.set_filtering_time_window(1.5);
	}

	// eliminate the Sdetection outside the region around the robot. Find the position of the obs to eliminate.
	std::vector<unsigned int> pos_obs_to_eliminate;

	for (unsigned int it_obs = 0; it_obs < obs.size(); it_obs++)
	{
		//obs[it_obs].print_spoint_DET();
		if ((obs[it_obs].x > perimeter_distance_)|| (obs[it_obs].x < -perimeter_distance_)|| (obs[it_obs].y > perimeter_distance_)|| (obs[it_obs].y < -perimeter_distance_))
		{
			pos_obs_to_eliminate.push_back(it_obs);
		}
	}

	//eliminate the obs
	for (unsigned int it_obs_erase = 0;it_obs_erase < pos_obs_to_eliminate.size(); it_obs_erase++)
	{
		obs.erase(obs.begin() + pos_obs_to_eliminate[it_obs_erase]);
		for (unsigned int it = 0; it < pos_obs_to_eliminate.size(); it++)
		{
			pos_obs_to_eliminate[it] = pos_obs_to_eliminate[it] - 1;
		}
	}
	if (debug_cruces_fallos)
	{
		std::cout << " //eliminate the obs. obs.size()=" << obs.size()<< std::endl;
	}
	//if(obs.size()>0){  // haz update SOLO si tienes detecciones!!! (luego se puede cambiar para que propague los tracks cuando no tenga detecciones y le toque)
	//	std::cout<< "\n \n INICIO!!!!!!!!!!!!!!!!! \n \n" << std::endl;
	counter_ = counter_ + 1;
	//	std::cout<< "\n iteration:" <<  counter_ << std::endl;
	//obs==> contains the detections.
	//scene_==>contains the tracks.
	//if (obs.empty())
	//	return;

	if (!obs.empty()){
		time_stamp_ = 0;
		time_stamp_ = obs[0].time_stamp;
		float_time_stamp_=(float) obs[0].time_stamp;
		//		std::cout<< "\n in_time_stamp:" << obs[0].time_stamp << std::endl;
		//		std::cout<< "\n in_time_stamp_anterior:" << time_stamp_anterior_<< std::endl;
		/*if(fuse_tibi_and_teo_tracks_){
			dt_ =0.2;
			std::cout<< "\n time_stamp_ =" << time_stamp_ <<"; time_stamp_anterior_="<<time_stamp_anterior_<< std::endl;
			std::cout<< "\n time_stamp_ - time_stamp_anterior_" << time_stamp_ - time_stamp_anterior_ << std::endl;
		}
		else{*/
		//std::cout << "\n (1) INICIAL dt_:" << dt_ << std::endl;
			dt_ = (time_stamp_ - time_stamp_anterior_); // divido el dt entre dos, porque se me alternan laser delantero y trasero y sino pierdo tracks!
		//}
			if(dt_>1.0){
				dt_=0.05;
			}

			if(debug_fuse_tibi_and_teo_tracks_){
				std::cout<< "\n time_stamp_ =" << time_stamp_ <<"; time_stamp_anterior_="<<time_stamp_anterior_<< std::endl;
				std::cout<< "\n time_stamp_ - time_stamp_anterior_" << time_stamp_ - time_stamp_anterior_ << std::endl;
			}
		//
		if(debug_companion_akp_){
			std::cout << "\n (2) INICIAL dt_:" << dt_ << std::endl;
		}
		total_dt_=total_dt_+dt_;
		//std::cout << "\n total_dt_:" << dt_ << std::endl;
		if(cargo_ANTS){ //TODO: ver si para tibi y dabo también iria bien.
			update_total_velocity();
		}

		initialization_param_of_associations(obs); //initialization of all the tracks to NO asociated tracks. and emptied the before associations detection<->track.

		//if (debug_cruces_fallos)
		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " after initialization_param_of_associations(obs)"<< std::endl;
		}

		//		std::cout<< "\n numero de detecciones:" <<  obs.size() << std::endl;
		//		std::cout<< "\n mht threshold_distance_:" <<  threshold_distance_ << std::endl;
		//		std::cout<< "\n mht threshold_no_deteccion_:" <<  threshold_no_deteccion_ << std::endl;



		association_by_distances(obs); //associate the detections with the tracks.
		//if (debug_cruces_fallos)
		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " after association_by_distances( obs );" << std::endl;
		}
		// Debug_associations:
		//std::cout<< "\n \n before find_two_dets_in_one_track:" << std::endl;
		//		mht_look_tracks_associaten_in_one_detection( );

		if(!generate_database_companion_){
			mht_find_two_dets_in_one_track();
		}

		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout<< "\n \n after find_two_dets_in_one_track:" << std::endl;
		}
		//		mht_look_tracks_associaten_in_one_detection();

		mht_made_clusters(obs); // separation detections and tracks in clusters.


		//if (debug_cruces_fallos)
		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " after mht_made_clusters( obs );" << std::endl;
		}

		mht_update_cluster_tracks(obs); // va antes que tracks, para hacer ya despues el update de todos los tracks juntos!, muy similar al update_tracks, o almenos a la association_by_distances

		//if (debug_cruces_fallos)
		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " after mht_update_cluster_tracks" << std::endl;
		}

		if (cargo_ANTS)
		{
			std::cout << " \n \n \n Entro en GRUPS" << std::endl;
			update_groups();
		}

		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " before mht_update_tracks" << std::endl;
		}

		mht_update_tracks(obs); // use my tracker in each track and detection taking in to a count it's particular situation.

		//if (debug_cruces_fallos)
		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " after mht_update_tracks" << std::endl;
		}

		mht_exit_clusters();

		//if (debug_cruces_fallos)
		if(debug_fuse_tibi_and_teo_tracks_)
		{
			std::cout << " after mht_exit_clusters();" << std::endl;
		}

		if(debug_cruces_2015_){
			mht_print_results(); //for the visualization of results in C++.
			std::cout << " mytracks_list_.size()=" <<  mytracks_list_.size() <<std::endl;
			printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
		}
	}else{
		//std::cout << " ENTRO EN ELSE obs()=empty! " << std::endl;
		if(debug_companion_akp_){
			std::cout << " ENTRO EN ELSE obs()=empty! " << std::endl;
		}
		time_stamp_=external_time_stamp; // coger time stamp actual.
		if(debug_companion_akp_){
			std::cout<< "\n in_time_stamp=external_time_stamp=" << external_time_stamp<< std::endl;
			std::cout<< "\n in_time_stamp_anterior:" << time_stamp_anterior_<< std::endl;
		}
		dt_ = (time_stamp_ - time_stamp_anterior_);


		//if(dt_>0.2){
		//		dt_=0.02;
		//	}

		if(debug_companion_akp_){
			std::cout<< "\n (initial) dt_=" << dt_<< std::endl;
		}
		initialization_param_of_associations(obs);
		mht_continuation_tracks(obs);
		mht_remove_tracks_by_probability();
		mht_confirm_tracks_by_bool();
		mht_propagation_tracks();
	} // end if(!obs.empty())
	//}
//	}   //end of if(obs.size()>0){, this have to be here, at that moment.
	//if(!fuse_tibi_and_teo_tracks_){
		eliminate_new_tracks_bad_associated();
	//}
	//std::cout<<"; !!!!antes out update.mht!!!!"<< std::endl;
	//scene_tracks_people_prediction_.see_tracks_on_scene();


	// save data in txt. for tibi and teo database.
		if(fuse_tibi_and_teo_tracks_){
			save_tracks_in_txt_tibi_and_teo_companion_database();
		}


	/*std::cout << " mytracks_list_.size()="<<mytracks_list_.size() << std::endl;
    std::cout << " mycluster_list_.size()="<<mycluster_list_.size() << std::endl;
    std::cout << " mycluster_list_actual_.size()="<<mycluster_list_actual_.size() << std::endl;
    std::cout << " mygroup_list_.size()="<<mygroup_list_.size() << std::endl;
    std::cout << " scene_tracks_people_prediction_.person_list_.size()="<< scene_tracks_people_prediction_.get_scene2()->size()<< std::endl;

	std::cout << " exit mht c++" << std::endl;*/

}



void Cmht::mht_look_tracks_associaten_in_one_detection()
{
//function for visualize the cluster before made the Sclusters

	for (unsigned int i = 0; i < OneDetection_ntracks_association_.size(); i++)
	{

		for (unsigned int j = 0;j < OneDetection_ntracks_association_[i].size(); j++)
		{

			std::cout << "\n Detection:" << i << std::endl;
			std::cout << "\n Tracks associated with this detection:"<< OneDetection_ntracks_association_[i][j] << std::endl;
			std::cout << "\n Distance Det <--> this Track "<< vector_distances_[i][j] << std::endl;
		}
	}

	for (unsigned int i = 0; i < count_my_tracks_; i++)
	{
		std::cout << "\n track  " << i << std::endl;
		std::cout << "\n track_no_associated=  " << track_no_associated_[i]<< std::endl;
	}

}

//mht_find_two_dets_in_one_track== this function change when take in to account the velocities in laser detections.

void Cmht::mht_find_two_dets_in_one_track()
{
// This function made a solution for the complex case of two detections inside one track.

	//std::vector< std::vector<unsigned int> > Temp_OneDetection_ntracks_association;
	//Temp_OneDetection_ntracks_association.reserve(OneDetection_ntracks_association_.size());
	std::vector<int> id_track_associated_to_this_det; //vector de ints que contiene la posicion de la deteccion más cercana a este track, para luego quedarme solo con esa!!!
	std::vector<double> min_distance; // -1,case detection not associated!!!! saltarmelo!
	std::vector<bool> temp_track_no_associated; //temporal vector for tracks not associated with detection.

// Initialization of variables.

	for (unsigned int i = 0; i < OneDetection_ntracks_association_.size(); i++)
	{
		id_track_associated_to_this_det.push_back(-1); //id==-1 if not have track associated this detection and if 1det<-->1track)
	}

	for (unsigned int i = 0; i < OneDetection_ntracks_association_.size(); i++)
	{
		min_distance.push_back(100); //id==-1 if not have track associated this detection and if 1det<-->1track)
	}
	std::vector<bool> two_dets_in_one_track;
	//bool one_det_n_tracks=false;
	//std::vector<unsigned int> temp_associationsTracks_oneDetection;
	//temp_associationsTracks_oneDetection.reserve(mytracks_list_.size());

	for (unsigned int j = 0; j < OneDetection_ntracks_association_.size(); j++)
	{
		two_dets_in_one_track.push_back(false);
	}

// segunda pasada, eliminar detecciones associadas al mismo track siendo un unico track !!!

	bool one_track_n_dets = false;
	std::vector<bool> detection_without_track;
	for (unsigned int i = 0; i < OneDetection_ntracks_association_.size(); i++)
	{
		detection_without_track.push_back(false); //id==-1 if not have track associated this detection!!! (new track debido a que det1<-->track1, det2 <-->track1 (dos detecciones asociadas al mismo track)).
	}
	for (unsigned int i_detection = 0;i_detection < OneDetection_ntracks_association_.size();i_detection++)
	{

		tracks_first_det_.clear();

		if ((!OneDetection_ntracks_association_[i_detection].empty())&& (OneDetection_ntracks_association_[i_detection].size() == 1))
		{ //check that the detection is  associated to only 1 track.

			tracks_first_det_ = OneDetection_ntracks_association_[i_detection];

			if (tracks_first_det_.size() == 1) //find matches for the tracks of this detection if this detection is associated onli to one track!.
			{

				for (unsigned int i_detection2 = 0;i_detection2 < OneDetection_ntracks_association_.size();i_detection2++)
				{

					if ((!OneDetection_ntracks_association_[i_detection2].empty())&& (OneDetection_ntracks_association_[i_detection2].size()== 1))
					{

						tracks_other_det_for_compare_.clear();
						tracks_other_det_for_compare_ =OneDetection_ntracks_association_[i_detection2]; //get the tracks for this other detection.

						//for( unsigned int ind_track2=0; ind_track2 < tracks_other_det_for_compare_.size(); ind_track2++ )
						//{    //compare all of the tracks in this other detection with this actual track in the first detection in cluster.
						// only have one track for compare!!!
						if ((tracks_first_det_[0]== tracks_other_det_for_compare_[0])&& (i_detection != i_detection2)) // only the first asociated track for comprare
						{

							one_track_n_dets = true;

//		    							std::cout<< "\n have n detection <--> 1 track" << std::endl;
//		    							std::cout<< "\n tam_detection:" << OneDetection_ntracks_association_[i_detection].size() << std::endl;
//		    							std::cout<< "\n tam_detection2:" << OneDetection_ntracks_association_[i_detection2].size() << std::endl;

							//case1: ( 1 Track <--> n Detections)

//		    								std::cout<< "\n case1: ( 1 Track <--> n Detections) "<< std::endl;

							if (vector_distances_[i_detection][0]< vector_distances_[i_detection2][0])
							{
								//det1 <--> track1 ==> det2=new track.

								detection_without_track[i_detection2] = true;
								no_new_track_[i_detection2] == true; //OJO, hago que la detección que no se asocia no genere nuevo track!!! (para comprovaciones futuras de fallos)
							}
							else
							{
								//det2 <--> track1 ==> det1=new track.
								//OneDetection_ntracks_association_[i_detection].clear();
								detection_without_track[i_detection] = true;
								no_new_track_[i_detection] == true;

							}

						}

						//}
					}
				}

			}

		}

	}
	// clean the detections inside one track that are far of the track.
	// not have two detections associated to one only track.
	if (one_track_n_dets)
	{

		std::vector<std::vector<unsigned int> > temp_OneDetection_ntracks_association =OneDetection_ntracks_association_;
		OneDetection_ntracks_association_.clear();
		std::vector<unsigned int> vect_tracks_temp;

		for (unsigned int i_detection = 0;i_detection < temp_OneDetection_ntracks_association.size();i_detection++)
		{
			vect_tracks_temp.clear();

			if ((!temp_OneDetection_ntracks_association[i_detection].empty())&& (!detection_without_track[i_detection])) //check that the detection is associated to track.
			{
				//case 1 detection <---> n Track associated (change to association with the nearest track)

				vect_tracks_temp.push_back(temp_OneDetection_ntracks_association[i_detection][0]);

			}

			OneDetection_ntracks_association_.push_back(vect_tracks_temp);

		}
	}
}

void Cmht::initialization_param_of_associations(std::vector<Sdetection>& obs)
{
// Initialization for some parameters of mht, for each iteration.

	//initialization of parameters for the associations between detections and tracks.
	//initialization of all the tracks to NO associated tracks. and emptied the before associations detection<->track.

	OneDetection_ntracks_association_.clear(); //erase the before associations of one detection to all of possible tracks for association (tracks at distance < 2).
	OneDetection_ntracks_association_.reserve(obs.size()); //space reservation for the rows (filas)... num_detecciones. Then every vector that I create, I will have to reserve space for columns=num_tracks;

	num_det_iter_ = obs.size();

	track_no_associated_.clear();
	track_no_associated_.reserve(count_my_tracks_); //(mytracks_list_.size() ); //ojo!!! estas empiezan en 0, pero los tracks empiezan el 1!! cuidado!

	for (unsigned int j = 0; j < count_my_tracks_; j++) //mytracks_list_.size(); j++)
	{ //put all tracks to value no associate tracks.
		track_no_associated_.push_back(true); // This variable have size equal to all the number of tracks in the scene and this value is no associated=true=1.
//		std::cout<< "\n track_no_associated_[" <<j<<"]="<< track_no_associated_[j] << std::endl;
//		std::cout<< " track_no_associated_[id_track_actual=1] :" <<  track_no_associated_[j]<< std::endl;
	}
	//track_no_associated_.resize(mytracks_list_.size(),true);

	//counter control
	if (count_my_tracks_ > 10000)
		count_my_tracks_ = 1;

	//counter control
	if (count_my_clusters_ > 10000)
		count_my_clusters_ = 1;

	if (count_my_clusters_actual_ > 10000)
		count_my_clusters_actual_ = 1;

	if (count_my_groups_ > 10000)
		count_my_groups_ = 1;

	vector_distances_.clear(); //erase the distances for this iteration.
	//vector distances initialization.
	/*for(unsigned int j=0; j<count_my_tracks_;j++)//mytracks_list_.size(); j++)
	 {
	 vector_distances_.push_back(100.0); //this variable have size of all number of tracks in the scene, and be 100, at the moment detection no associated.

	 }*/

// after initializing the tracks change the association distance to 1m
	/*if (count_my_tracks_ > 50)
	{
		set_threshold_distance(1); //para que cuando este ya en marcha asocie mejor cada deteccion con su track.
	}*/

	detection_in_cluster_.clear();
	detection_in_cluster_.reserve(obs.size());
//initialization param cluster
	for (unsigned int j = 0; j < obs.size(); j++)
	{
		detection_in_cluster_.push_back(false);
	}

	// inicialization no_new_track.
	no_new_track_.clear();
	for (unsigned int j = 0; j < obs.size(); j++)
	{
		no_new_track_.push_back(false);
	}

	track_no_asociated_in_cross_situation_.clear();
	for (unsigned int j = 0; j < count_my_tracks_; j++) //mytracks_list_.size(); j++)
	{
		track_no_asociated_in_cross_situation_.push_back(false); // This variable have size equal to all the number of tracks in the scene and this value is no associated=true=1.

	}

	exit_track_cargo_ANTS_.clear();
	for (unsigned int j = 0; j < count_my_tracks_; j++) //mytracks_list_.size(); j++)
	{
		exit_track_cargo_ANTS_.push_back(true); // This variable have size equal to all the number of tracks in the scene and this value is no associated=true=1.
	}

}

void Cmht::association_by_distances(std::vector<Sdetection>& obs)
{
//This make the detection<-->track association by distances.

//paso1: Distance calculation!!! and made the "clusters" according to the vectors track_no_asociated_ and vector_distances_
	//for then apply the functions of track.cpp that require each cluster.
	//std::cout<< " \n \n Function --> association_by_distances:" <<  std::endl;

	double distance_laser_track = 0.0;
	int iteration_distance = 0;

	std::vector<unsigned int> associationsTracks_oneDetection; //vector where I save the tracks associated to a detection.
	associationsTracks_oneDetection.reserve(mytracks_list_.size());

	std::vector<int> associationsDetetections_oneTrack;
	associationsDetetections_oneTrack.reserve(obs.size());

	std::vector<double> vector_distances_det; //save the distance vector for this detection.
	vector_distances_.reserve(obs.size());

	for (unsigned int i_detection = 0; i_detection < obs.size(); i_detection++)
	{
		//std::cout<< "\n print detection" << std::endl;
		//obs[i_detection].print();

		associationsTracks_oneDetection.clear();
		vector_distances_det.clear();
		iteration_distance = 0;

		for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
		{

			int id_track_actual;
			id_track_actual = it_myTrack->get_idTracker(); //get my id, for search this track propagated that Gonzalo passes me with the same id.
			//save the distances taking into account the order of each track id's from high to low value.
			Cperson_abstract* person_obj;

			//track_pose.print();
			if (debug_cruces_fallos)
			{
				std::cout<< "\n ASOCIATION, tracks que hay en Ctrack id_track_actual="<< id_track_actual << std::endl;
			}

			if (scene_tracks_people_prediction_.find_person(id_track_actual,&person_obj))
			{
				//person_obj->get_current_pointV().print();
				//std::cout<<"(before person propagation) id_track_actual=" << id_track_actual <<"; dt_="<<dt_ << std::endl;

				//if(fuse_tibi_and_teo_tracks_){
				//	dt_=5.0;
				//}

				SpointV_cov track_poseV = person_obj->pointV_propagation(dt_); //calculation of the increment!. Get the track pose, by the function and the pointer to the tracks list.
				//std::cout<<"dt_" << dt_  << std::endl;
				//std::cout<< "\n track_poseV_actual=" << std::endl;
				//track_poseV.print();

				std::vector<double> track_cov;
				track_cov.reserve(4);
				track_cov.resize(4, 0.0);
				track_cov[0] = track_poseV.cov[0] + aug_cov_prediction;
				track_cov[1] = track_poseV.cov[1];
				track_cov[2] = track_poseV.cov[4];
				track_cov[3] = track_poseV.cov[5] + aug_cov_prediction;
				Spoint_cov track_pose(track_poseV.x, track_poseV.y,track_poseV.time_stamp, track_cov);

//				std::cout<< "\n (distance) track_pose_actual="<<id_track_actual << std::endl;
//				track_poseV.print();

				Spoint_cov detection_actual(obs[i_detection].x,obs[i_detection].y, obs[i_detection].time_stamp,obs[i_detection].cov); //TODO: Calculo distancias, ojo, al cambiar SpointV_cov lo mismo cambia como calcular la distancia.

				//				std::cout<< "\n (function distances) detection_actual=" << std::endl;
//				detection_actual.print();
				//Spoint detection_actualV( obs[i_detection].x , obs[i_detection].y,obs[i_detection].time_stamp );

//				double distance_laser_track_sin_cov = track_pose.distance(detection_actual); //TODO:  future (change to cov_dist)
//				distance_laser_track = track_pose.cov_dist(detection_actual);
				//double distv;
				//double distxv;

				//distance_laser_track = track_pose.distance(detection_actual);//,&distv,&distxv);
				double distance_laser_track_con_cov =track_pose.cov_dist(detection_actual);
				if (cargo_ANTS)
				{
					distance_laser_track = track_pose.cov_dist(detection_actual);
					//distance_laser_track = track_pose.distance(detection_actual);
					//std::cout << " (cargo_ANTS) distance_laser_track=" << distance_laser_track<< std::endl;
				}
				else
				{
					//distance_laser_track = track_pose.cov_dist(detection_actual); //,&distv,&distxv);
					distance_laser_track =track_pose.distance(detection_actual); //TODO: usar distancia con covarianza como en cargo_ANTS, pero hace falta reducir más la distancia de asociación (o las covarianzas=creo que es solo la distancia de asociacion, pero mirar).
					//std::cout << " (normal_dist) distance_laser_track=" << distance_laser_track<< std::endl;
				}
				//std::cout<<" \n DISTANCE: sin cov distance_laser_track:" <<  distance_laser_track << std::endl;
			   // std::cout<<" \n DISTANCE: con cov distance_laser_track:" <<  distance_laser_track_con_cov << std::endl;
			   // std::cout<<"   del track: "<<id_track_actual<< std::endl;
			   //std::cout<<"   detetección número: "<< i_detection << std::endl;


			   //if(fuse_tibi_and_teo_tracks_){
				//   threshold_distance_=0.5; // TODO: ojo, quitar luego y ver donde se cambia el threshold distance hard coded
			   //}


				if ((distance_laser_track < threshold_distance_) && (distance_laser_track > 0))
				{ //create the cluster with this corresponding detection and all the tracks that contains. (creo ya el cluster correspondiente con la detección y los tracks que la contengan.)
					iteration_distance = iteration_distance + 1;

					track_no_associated_[id_track_actual] = false; //(id_track_actual-1) porque mis id_track empiezan en el 1. track_no_asociado=0-->el track está asociado.
//				    std::cout<< " vector_bool_tracks_no_associados size:" <<  track_no_associated_.size()<< std::endl;
//
					associationsTracks_oneDetection.push_back(id_track_actual); //vector of tracks associated with this actual detection.

					vector_distances_det.push_back(distance_laser_track); //I save only the distance of the track that is a shorter distance for this detection.

//std::cout<< "DISTANCE" << std::endl;
					//if (debug_cruces_fallos)
					if (debug_companion_akp_)
					{
						std::cout << "numero de deteccion actual:"<< i_detection << std::endl;
						std::cout << "track associado:" << id_track_actual<< std::endl;
						std::cout << "distance_laser_track=" << distance_laser_track<< std::endl;
					}
//							    std::cout<< " SIZE tracks_associados_a deteccion actual:" <<  associationsTracks_oneDetection.size()<< std::endl;
//							    std::cout<< " tracks_associados_a deteccion actual:" <<  id_track_actual << std::endl;
//std::cout<< "distancias de tracks_associados_a deteccion actual:" <<  vector_distances_det[iteration_distance]<< std::endl;
					//	detection_belongs_track=true;
				}else{
					//std::cout << "NO numero de deteccion actual:"<< i_detection << std::endl;
					//std::cout << "NO track associado:" << id_track_actual<< std::endl;
					//std::cout << "NO distance_laser_track=" << distance_laser_track<< std::endl;
				}
			}
		}

		vector_distances_.push_back(vector_distances_det); //TODO: ojo! las detecciones tambien van de la 1 a la 0 del vector.! (esto es menos problema que lo de los tracks creo que ya iría bien de normal)
		OneDetection_ntracks_association_.push_back(associationsTracks_oneDetection);

	}

	//if (debug_cruces_fallos)
	if (debug_companion_akp_)
	{
		std::cout << "OneDetection_ntracks_association_ FINAL ASSOCIATION"<< std::endl;
		for (unsigned int u = 0; u < OneDetection_ntracks_association_.size();u++)
		{
			for (unsigned int l = 0;l < OneDetection_ntracks_association_[u].size(); l++)
			{
				std::cout << " id_track => OneDetection_ntracks_association_[" << u << "]["<< l << "]." << OneDetection_ntracks_association_[u][l]<< std::endl;
				std::cout << " distance to det => vector_distances_det[" << u << "]["<< l << "]=" << vector_distances_[u][l]<< std::endl;

			}
		}
	}

	/*std::cout<< "\n vector_distances_.size" <<  vector_distances_.size()<< std::endl;
	std::cout<< "\n empty " << vector_distances_.empty() << std::endl;
	if(vector_distances_.empty()) //entra si el vector no está vacio
	 {
	 for(unsigned int i=0; i<vector_distances_.size();i++){
	 std::cout<< " \n de detección:" <<  i << std::endl;
	 std::cout<< " \n DISTANCE: distance_laser_track:" << (double) vector_distances_[i][0] << std::endl; //NO le gusta esto!!!
	 }
	 }*/
}

void Cmht::mht_inicialization_tracks(std::vector<Sdetection>& obs)
{
//This function make the tracker initialization for a new lonely detections that made a new track.
	if(debug_fuse_tibi_and_teo_tracks_){
		std::cout<< "\n IN mht_inicialization_tracks " << std::endl;
	}

	for (unsigned int i_detection = 0; i_detection < obs.size(); i_detection++)
	{
		if(debug_fuse_tibi_and_teo_tracks_){
			std::cout<< "\n (1) mht_inicialization_tracks " << std::endl;
		}

		if ((OneDetection_ntracks_association_[i_detection].empty())&& (no_new_track_[i_detection] == false)) //check that the detection is not associated to (ningún) track.
		{ // Case only detection without association of a track, create new track for this detection.
			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n mht_inicialization_tracks OneDetection_ntracks_association_[i_detection].empty()="<<OneDetection_ntracks_association_[i_detection].empty()<< "; no_new_track_[i_detection]="<<no_new_track_[i_detection] << std::endl;
			}
			//add track!

			std::list<Ctrack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
			it_insert_track = mytracks_list_.begin();

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (3) mht_inicialization_tracks " << std::endl;
			}

			if (!mytracks_list_.empty())
			{
				if(debug_fuse_tibi_and_teo_tracks_)
				{
					std::cout<< "\n (4) mht_inicialization_tracks " << std::endl;
				}

				while (it_insert_track != mytracks_list_.end())
				{
					it_insert_track++;
					if (it_insert_track->get_idTracker() > count_my_tracks_)
					{
						break;
					}

				}
			}

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (7) mht_inicialization_tracks " << std::endl;
			}
//			Ctrack track(count_my_tracks_);
			Ctrack new_track = Ctrack(count_my_tracks_);

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (8) mht_inicialization_tracks " << std::endl;
			}
			//show in the screen the inicialiced track.
//			std::cout<< " \n Inizalization track:" <<  count_my_tracks_<< std::endl;
//			obs[i_detection].print_spoint_DET();

			count_my_tracks_++;

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (9) mht_inicialization_tracks " << std::endl;
			}
			Ctrack::detection_type detection_type =(Ctrack::detection_type) obs[i_detection].type;

			if(fuse_tibi_and_teo_tracks_){
				detection_type=Ctrack::Laser;
			}

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (10) mht_inicialization_tracks " << std::endl;
			}
			new_track.set_laser_distance(100); //Is the first detection, doesn't have track, distance=100>threshold.
			new_track.set_velocity_margin(mht_velocity_margin_);

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (11) mht_inicialization_tracks " << std::endl;
			}
			Spoint_cov detec = obs[i_detection].getSpoint_cov();
//std::cout<< "\n (mht_inicialization_tracks) detection: " << std::endl;
//detec.print();

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (12) mht_inicialization_tracks " << std::endl;
			}
			new_track.ini_track(obs[i_detection].getSpoint_cov(),detection_type,dt_); //The initialization function made Kalman inside, like the other cases (update, etc).

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (13) mht_inicialization_tracks " << std::endl;
			}

			Spoint_ini_track_for_covariance_in_predictions_= obs[i_detection].getSpoint_cov();

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (14) mht_inicialization_tracks " << std::endl;
			}

			if(debug_companion_akp_){
				std::cout<< "\n print Spoint_ini_track_for_covariance_in_predictions_ (new track) " << std::endl;
				Spoint_ini_track_for_covariance_in_predictions_.print();
			}

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (15) mht_inicialization_tracks " << std::endl;
			}
			//std::cout<< "\n print new track " << std::endl;
//SpointV_cov track_ini=new_track.get_kalman_update();
//track_ini.print();
			mytracks_list_.insert(it_insert_track, new_track);

			if(debug_fuse_tibi_and_teo_tracks_)
			{
				std::cout<< "\n (16) mht_inicialization_tracks " << std::endl;
			}
			//end add track!
//			std::cout<< "\n new track, id_track:" <<count_my_tracks_<< std::endl;
		}
		//std::cout<< "\n (17) mht_inicialization_tracks " << std::endl;
	}
//	std::cout<< " \n exit of the initialization tracks function"<< std::endl;
	//std::cout<< "\n (18) mht_inicialization_tracks " << std::endl;
}

void Cmht::mht_association_det_with_tracks(std::vector<Sdetection>& obs)
{
//This function make the tracker update for the existent tracks associated with a actual detection.
	if (debug_cruces_fallos)
	{
		std::cout << "\n !!!!!!! mht_association_det_with_tracks " << std::endl;
	}

	for (unsigned int i_detection = 0; i_detection < obs.size(); i_detection++)
	{
		//std::cout<< "\n OneDetection_ntracks_association_[i_detection]" <<  OneDetection_ntracks_association_[i_detection].size() << std::endl;

		if ((OneDetection_ntracks_association_[i_detection].size() == 1)) // detection associated only to one track, NO cluster.
		{

			if (debug_cruces_fallos)
			{
				std::cout << "\n entro en if " << std::endl;
				for (unsigned int d = 0;d< OneDetection_ntracks_association_[i_detection].size();d++)
				{
					std::cout<< " OneDetection_ntracks_association_[i_detection=="<< i_detection << "][" << d << "].track="<< OneDetection_ntracks_association_[i_detection][d]<< std::endl;
				}
			}
			//case 1 det and 1 track associated.

			SpointV_cov track; //track that will be the detection associated, in each iteration!
			//double Probability_laser_detection=obs[i_detection].get_detector_probability();  //Para cambio de PD a la Pd del detector. (de momento desestimado este cambio pq era mezcla de conceptos.)
			Sdetection SdetDetection = obs[i_detection];
			Spoint_cov detection = SdetDetection.getSpoint_cov();

//						std::cout<< "\n (mht_association_det_with_tracks) detection " << std::endl;
//						detection.print();

			Ctrack::detection_type detection_type =
					(Ctrack::detection_type) obs[i_detection].type;
			unsigned int id_track_associate =OneDetection_ntracks_association_[i_detection][0]; //The tracks that this detection is associated.

			std::list<Ctrack>::iterator it_track_associate; //pointer to my track!
			Cperson_abstract* person_obj; //pointer to the track prediction that I get of Gonzalo.
			SpointV_cov track_pose_associate; //prediction SpointV_cov track associated.
			track_pose_associate = SpointV_cov(); //reset por si acaso no lo encuentra...

			if (debug_cruces_fallos)
			{
				std::cout << "\n antes de for " << std::endl;
			}
			for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin(); it_myTrack != mytracks_list_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			{
				if (debug_cruces_fallos)
				{
					std::cout << "\n [mht_association_det_with_tracks]: \n i_detection: "<< i_detection<< std::endl;
					std::cout << " \n id_track_associate: "<< id_track_associate << std::endl;
					std::cout<< " \n Tracks in my track list (it_myTrack->get_idTracker()) "<< it_myTrack->get_idTracker() << std::endl;
				}

				if (id_track_associate == it_myTrack->get_idTracker())
				{
					//search the predicted track and my track.
					it_track_associate = it_myTrack; // get the pointer to my track
					//get the pointer to predicted track, for made the SpointV_cov of this track. the prediction.

					if (scene_tracks_people_prediction_.find_person(id_track_associate, &person_obj)) //Get the person/detection of this track.
					{
						if (debug_cruces_fallos)
						{
							std::cout << " \n id_track_associate: "<< id_track_associate << std::endl;
							std::cout << " \n deteccion_associate: "<< i_detection << std::endl;
						}
						track_pose_associate = person_obj->pointV_propagation(dt_); //I save the person spose.
					}
				}
			}
			if (debug_cruces_fallos)
			{
				std::cout << "\n DISTANCIA: "<< (double) vector_distances_[i_detection][0]<< std::endl;
			}

			it_track_associate->set_laser_distance((double) vector_distances_[i_detection][0]); //I put insithe the track.cpp the distance between the detection and the nearest tracks, for the good probability calculation.
			it_track_associate->set_velocity_margin(mht_velocity_margin_);

			//it_track_associate->set_laser_Pd_InTrack(Probability_laser_detection); ////Para cambio de PD a la Pd del detector. (de momento desestimado este cambio pq era mezcla de conceptos.) cojo la Pd del detector laser.
			if (debug_cruces_fallos)
			{
				std::cout << "\n Association! dt_:" << dt_ << std::endl;
				std::cout << "time_stamp_:" << time_stamp_ << std::endl;
				std::cout << "\n (ASSOCIATION) id_track_pose_associate: "<< id_track_associate << std::endl;
				track_pose_associate.print();
			}

			track_pose_associate.cov[0] = track_pose_associate.cov[0]+ aug_cov_prediction; // OJO!!! aquí es 0.4 para personas!!!
			track_pose_associate.cov[5] = track_pose_associate.cov[5]+ aug_cov_prediction;

			if (debug_cruces_fallos)
			{
				detection.print();
			}
			//track_pose_associate.print();
			if (debug_cruces_fallos)
			{
				std::cout << " \n tipo de detección: " << detection_type<< std::endl;
			}

			it_track_associate->update_track(detection, track_pose_associate,detection_type,dt_); //the defect type=Laser.

			bool track_no_associated_by_velocity =it_track_associate->get_track_no_associated_by_change_abrut_velocity();

			if (debug_cruces_fallos)
			{
				std::cout << " \n track_no_associated_by_velocity: "<< track_no_associated_by_velocity << std::endl;
			}

			if (track_no_associated_by_velocity == true)
			{
				track_no_associated_[it_track_associate->get_idTracker()] =true;
			}

			//show the associated track in the screen.
			if (debug_cruces_fallos)
			{
				std::cout << " \n Asociación track:" << id_track_associate<< std::endl;
				std::cout << "\n DISTANCIA: "<< vector_distances_[i_detection][0] << std::endl; //DISTANCIA: vector_distances_[i_detection][id_track_associate-1]
				std::cout << " \n Detección asociada :" << std::endl;
				obs[i_detection].print_spoint_DET(); //detection.
				track_pose_associate.print(); //pose que me retorna gonzalo
			}

		}
		//else if(OneDetection_ntracks_association_[i_detection].size()>1)
		//{
		//	for(unsigned int d=0; d<OneDetection_ntracks_association_[i_detection].size();d++){
		//		std::cout<< " OneDetection_ntracks_association_[i_detection=="<<i_detection<<"]["<<d<<".track=" << OneDetection_ntracks_association_[i_detection][d]  << std::endl;
		//	}

		//TODO: caso 1 det y n tracks associados
		//			std::cout<< " \n caso 1det <--> 2track :"  << std::endl;
		//			const std::vector<double> cov;
		//			cov.reserve(16);
		//			cov.resize(16,0.0);

		//			SpointV_cov track_x=SpointV_cov(1.0,20,1, cov);

		//			double X=track_x.x;
		//double Y=track_x;

		//}
	}
}

void Cmht::mht_continuation_tracks(std::vector<Sdetection>& obs)
{
//This function make the tracker update for the existent tracks without association with a actual detection.
	//if (debug_cruces_fallos)
	//{
	if(debug_companion_akp_){
		std::cout << " \n mht_continuation_tracks" << std::endl;
	}
	//}

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		//Search the id for the actual track.
		//	std::cout<< " \n TRACKS LIST mht_continuation_tracks" << std::endl;
		int id_track_actual; //id_track_actual==t=track.
		id_track_actual = it_myTrack->get_idTracker(); //cojo mi id, para buscar el track que me pasa Gonzalo con la misma id.
		Cperson_abstract* person_obj;
		SpointV_cov track_pose;

		if (debug_cruces_fallos)
		{
			std::cout << " \n track_no_associated_[id_track_actual] :"<< track_no_associated_[id_track_actual] << std::endl;
		}

		if (track_no_associated_[id_track_actual] == true) //track no associated=true!=1. (el 0 me lo saltaría porque empiezo en el track=1, mi count_my_tracks=1 la primera vez, por lo del robot de gonzalo, que tiene id=0.
		{
			//Search the track prediction that corresponds with this track.

//			scene_tracks_people_prediction_.print();
			//scene_tracks_.find_person(id_track_actual,&person_obj); //le he de pasar la direccion del puntero. (algo** ==es un puntero al puntero)
			//if (debug_cruces_fallos)
			//{
			if(debug_companion_akp_){
				std::cout << " \n no detecto a la persona!!!" << std::endl;
			}
			//}
//			std::cout<< "\n Propagation! dt_:" <<  dt_ << std::endl;
//			std::cout<< "time_stamp_:"<<time_stamp_<< std::endl;
			if (scene_tracks_people_prediction_.find_person(id_track_actual,&person_obj)) //find the person/detection associated with this track.
			{
				//if (debug_cruces_fallos)
				//{
				if(debug_companion_akp_){
					std::cout << " \n (track associado) si encuentro a la persona!!! dt_="<<dt_<< std::endl;
				}
				//}
				//	person_obj->get_current_pointV().print();

					/*SpointV_cov spoint_b=person_obj->get_current_pointV();
					std::cout << " (Spoint del predictor) spoint_b.x="<<spoint_b.x<<"; spoint_b.y="<<spoint_b.y<< std::endl;
					std::cout << " (Spoint del predictor) spoint_b.vx="<<spoint_b.vx<<"; spoint_b.vy="<<spoint_b.vy<< std::endl;
					SpointV_cov spoint_a=it_myTrack->get_anterior_trackV_guardado();
					double vx=spoint_a.vx;
					double vy=spoint_a.vy;
					double v_ac=sqrt(pow(vx,2)+pow(vy,2));
					std::cout << "v_ac="<<v_ac<<"; vx="<<vx<<"; vy="<<vy<< std::endl;*/
				//person_obj->set_desired_velocty(v_ac);
				track_pose = person_obj->pointV_propagation(dt_); //I get the spose of this person.
				if(debug_companion_akp_){
					std::cout << "track_pose propagated (GONZALO) "<< std::endl;
					track_pose.print();
				}
				/* uso propagacion a v_cte! (cuando no tengo track)*/
				SpointV_cov spoint_a=it_myTrack->get_anterior_trackV_guardado();
				double vx=spoint_a.vx;
				double vy=spoint_a.vy;
				double dx = vx*dt_ ;
				double dy = vy*dt_ ;
				if(debug_companion_akp_){
					std::cout << " spoint_a ant"<< std::endl;
					spoint_a.print();
				}
				//track_pose=spoint_a;
				spoint_a.x=spoint_a.x+dx;
				spoint_a.y=spoint_a.y+dy;
				spoint_a.time_stamp=spoint_a.time_stamp+dt_;
				//track_pose=spoint_a;
				if(debug_companion_akp_){
					std::cout << " spoint_a act new"<< std::endl;
					spoint_a.print();
				}

				//Strack(spoint_a.x+dx,spoint_a.y+dy,spoint_a.time_stamp_+dt_,spoint_a.vx,spoint_a.vy,spoint_a.cov) //Strack( double x_ , double y_ , double time_stamp_=0, double vx_=0.0, double vy_=0.0,
						//const std::vector<double>& cov_= std::vector<double>(), unsigned int id_=0, double probability_of_detection_=0.0 );

				//	track_pose.print();
			}
			//track_pose.print();
			//caso track solo
//			std::cout<< "\n  PROPAGATION (mht_continuation_tracks) track_pose, id_track "<< id_track_actual << std::endl;
//			track_pose.print();

			it_myTrack->set_track_no_asociated_in_cross_situation_track(track_no_asociated_in_cross_situation_);
			it_myTrack->set_time_stamp(time_stamp_);
			it_myTrack->set_laser_distance(100.0); //for the kalman_states, case of only track without detection,put distance>2.2=threshold for the association distance.
			it_myTrack->set_velocity_margin(mht_velocity_margin_);
			it_myTrack->update_trackn(track_pose, dt_); //TODO: (aquí tendrá que tener tipo el track, sino todos los cogerá como láser por defecto, mirar si hace falta...) tener en cuenta, que cuando no sea el tipo por defecto=laser, tendré que coger el tipo del struct que me pasan.

		}
		//sacar por pantalla track continuado, sin detección asociada.
		if (debug_cruces_fallos)
		{
			std::cout<< " \n Propagation track. propagado sin detección asociada:"<< id_track_actual << std::endl;
		}
		//track_pose.print(); //pose que me retorna gonzalo, la que se propagará
	}
}

void Cmht::mht_remove_tracks_by_probability()
{
//This function remove the tracks when it's probability is belong the probability threshold of elimination tracks.

	std::vector<std::list<Ctrack>::iterator> vector_punteros_a_mis_tracks;

	//if (debug_cruces_fallos)
	//{
	if(debug_companion_akp_){
		std::cout<< " REMOVE TRACKS: size lista de mis tracks (justo antes de borrar):"<< mytracks_list_.size() << std::endl;
	}
		//}
	//find tracks to erase.

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		SpointV_cov kalman_state = it_myTrack->get_kalman_update();
		int id = it_myTrack->get_idTracker();
		double sum_cov = kalman_state.cov[0] + kalman_state.cov[1]+ kalman_state.cov[4] + kalman_state.cov[5]; //+kalman_state.cov[2]+kalman_state.cov[3]+kalman_state.cov[6]+kalman_state.cov[7]+kalman_state.cov[8]+kalman_state.cov[9]+kalman_state.cov[10]+kalman_state.cov[11]+kalman_state.cov[12]+kalman_state.cov[13]+kalman_state.cov[14]+kalman_state.cov[15];

		//get the probability of the track, and if this probability is less than the value of threshold_eliminacion, erase this track.
		if (debug_cruces_fallos)
		{
			std::cout << " id_track=" << id << std::endl;
		}
		//kalman_state.print();

		if ((it_myTrack->get_probability() < threshold_no_deteccion_)|| (sum_cov > covariance_no_track_))
		{
			//save pointers to my tracks for delete. (guardar punteros a mis tracks a borrar.)
			vector_punteros_a_mis_tracks.push_back(it_myTrack);
			//remove predictions in person propagation
			scene_tracks_people_prediction_.remove_person(id);
			mht_remove_clusters_with_track_removed(id);
		}

	}

	//Delete tracks (borrar tracks a borrar encontrados.)

	for (unsigned int i = 0; i < vector_punteros_a_mis_tracks.size(); i++)
	{

		// remove this track in all of the existent clusters!!!.
		unsigned int id_track_ac =vector_punteros_a_mis_tracks[i]->get_idTracker();
		for (std::list<Ccluster>::iterator it_myCluster =mycluster_list_.begin(); it_myCluster != mycluster_list_.end();it_myCluster++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
		{
			std::vector<unsigned int> tracks_in_cluster =it_myCluster->get_tracks_in_cluster_mht();
			std::vector<unsigned int> pos_tracks_eliminate;
			// find the tracks for eliminate in cluster.
			for (unsigned int it_tracks_in_cluster = 0;it_tracks_in_cluster < tracks_in_cluster.size();it_tracks_in_cluster++)
			{
				if (id_track_ac == tracks_in_cluster[it_tracks_in_cluster])
				{
					pos_tracks_eliminate.push_back(it_tracks_in_cluster);
				}
			}
			// eliminate this tracks and realign the tracks.
			for (unsigned int it_erase = 0;it_erase < pos_tracks_eliminate.size(); it_erase++)
			{
				tracks_in_cluster.erase(tracks_in_cluster.begin()+ pos_tracks_eliminate[it_erase]);
				for (unsigned int it = 0; it < pos_tracks_eliminate.size();it++)
				{
					pos_tracks_eliminate[it] = pos_tracks_eliminate[it] - 1;
				}
			}
			it_myCluster->set_tracks_in_cluster_mht(tracks_in_cluster);
		}

		// remove this track in track list.
		// TODO: INI nuevo errase tracks, errase the prediction list ok, when erase tracks.
		scene_tracks_people_prediction_.remove_person(vector_punteros_a_mis_tracks[i]->get_idTracker());
		mht_remove_clusters_with_track_removed(vector_punteros_a_mis_tracks[i]->get_idTracker());
		// TODO: FIN nuevo errase tracks, errase the prediction list ok, when erase tracks.

		mytracks_list_.erase(vector_punteros_a_mis_tracks[i]); //vector_punteros_a_mis_tracks[i]==it_myTrack -> pointer to my track for delete.
	}

	vector_punteros_a_mis_tracks.clear(); //clear the vector of pointer for deleting tracks, for the next iteration.

	//if (debug_cruces_fallos)
	//{
	if(debug_companion_akp_){
		std::cout << " Tracks despues de REMOVE" << std::endl;
	}
	//}

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		int id = it_myTrack->get_idTracker();
		//if (debug_cruces_fallos)
		//{
		if(debug_companion_akp_){
			std::cout << " id_track=" << id << std::endl;
		}
		//}
	}

}

void Cmht::mht_confirm_tracks_by_bool()
{
	//This function confirm the tracks using the boolean result of the tracker obtained with it's probability of person detection.

	if (cargo_ANTS)
	{
		publish_only_one_track_in_groups();
		//publish_only_one_track_in_clusters(); //FUNCTION ONLY FOR CARGO ANTS
	}
	exit_params_mht_.clear(); //clear the exit parameters for the tracks, for filling with the parameters in this iteration.
	exit_propagations_mht_.clear();

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		if (it_myTrack->get_track_confirmed() == true)
		{
			//Track confirmed. (MIS TRACKS!)
			//guardar track en parámetros para sacar fuera, porque este track ya es persona, esta confirmado.
			Strack param_returned;
			SpointV_cov kalman_state = it_myTrack->get_kalman_update();
			unsigned int id = it_myTrack->get_idTracker();
			double prob = it_myTrack->get_probability();

			param_returned = Strack(kalman_state.x, kalman_state.y,kalman_state.time_stamp, kalman_state.vx, kalman_state.vy,kalman_state.cov, id, prob);
			if (cargo_ANTS)
			{
				if ((-1.2 < kalman_state.vx) && (kalman_state.vx < -0.94))
				{

				}
				else
				{
					if (exit_track_cargo_ANTS_[id] == true)
					{
						//if(kalman_state.x<0.1){
						//if((id==6)||(id==12)||(id==28)||(id==26)){

						//}else{
							exit_params_mht_.push_back(param_returned);
							//}
						//}
					}
				}
			}
			else
			{
				if(debug_companion_akp_){
					std::cout<< " (EXIT PARAMS) param_returned.id="<<param_returned.id<< std::endl;
					std::cout<< " (EXIT PARAMS) param_returned.probability="<<param_returned.probability_of_detection<< std::endl;
					param_returned.print();
				}
				//if(id<30){
					exit_params_mht_.push_back(param_returned);
				//}

			}
			// Sacar las predicciones de gonzalo (para ver las covarianzas, comparar!!!)
			Strack people_prediction;
			unsigned int id_prediction = it_myTrack->get_idTracker();
			double prob_prediction = 0.0;
			SpointV_cov track_propagado =it_myTrack->get_propagation_guardada();
			people_prediction = Strack(track_propagado.x, track_propagado.y,track_propagado.time_stamp, track_propagado.vx,track_propagado.vy, track_propagado.cov, id_prediction,prob_prediction);

			//std::cout<< "\n print prediction" << std::endl;
			//people_prediction.print();
			if(debug_companion_akp_){
			std::cout<< " (EXIT exit_propagations_mht_) people_prediction.id="<<people_prediction.id<< std::endl;
			}
			exit_propagations_mht_.push_back(people_prediction);
		}

	}

}

void Cmht::mht_update_tracks(std::vector<Sdetection>& obs)
{
	//This function is a "main" function for made the entire process of tracking.
	//if (debug_cruces_fallos)
	if(debug_fuse_tibi_and_teo_tracks_)
	{
		std::cout<< " antes de ASOCIATION DET WITH TRACKS mht_association_det_with_tracks(obs);"<< std::endl;
	}

	mht_association_det_with_tracks(obs);

	//if (debug_cruces_fallos)
	if(debug_fuse_tibi_and_teo_tracks_)
	{
		std::cout<< " antes de CONTINUATION TRACKS mht_continuation_tracks(obs)"<< std::endl;
	}

	mht_continuation_tracks(obs);

	//if (debug_cruces_fallos)
	if(debug_fuse_tibi_and_teo_tracks_)
	{
		std::cout << " antes de INITIALICE TRACkS mht_inicialization_tracks"<< std::endl;
	}

	mht_inicialization_tracks(obs);

	//if (debug_cruces_fallos)
	if(debug_fuse_tibi_and_teo_tracks_)
	{
		std::cout << " antes de  REMOVE mht_remove_tracks_by_probability();"<< std::endl;
	}

	mht_remove_tracks_by_probability();

	if(cargo_ANTS){
		mht_filter_by_velocity();
	}

	//if (debug_cruces_fallos)
	if(debug_fuse_tibi_and_teo_tracks_)
	{
		std::cout << " antes de CONFIRM" << std::endl;
	}

	mht_confirm_tracks_by_bool();

	//if (debug_cruces_fallos)
	if(debug_fuse_tibi_and_teo_tracks_)
	{
		std::cout << " antes de PROPAGATE tracks" << std::endl;
	}

	mht_propagation_tracks();

	//   mht_print_results();
}

void Cmht::mht_propagation_tracks()
{
	//This function update the resulting tracks, for the propagation node. (update the Kalman corrections of the tracker)

	//pass tracks to the node people_propagation.
	//if (debug_cruces_fallos)
	//{
	if(debug_companion_akp_){
		std::cout << "\n Entro en propagation tracks: " << std::endl;
	}
	//}
	kalman_tracks_out_.clear();
//	unsigned int size_number_intern_tracks=mytracks_list_.size();
//	std::cout<< "\n size_number_intern_tracks: " << size_number_intern_tracks << std::endl;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		int id = it_myTrack->get_idTracker();
		//std::cout<< "\n ID Kalman correct to propagation track " << id << std::endl;
		SpointV_cov kalman_state = it_myTrack->get_kalman_update();

		//std::cout<< "\n (ant) Kalman propagate:" << std::endl;
		//kalman_state.print();

		//kalman_state.cov=Spoint_ini_track_for_covariance_in_predictions_.cov; //OJO! en principio va bien. Pero personalmente no me gusta hacer esto.

		//std::cout<< "\n (before2) Kalman propagate:" << std::endl;
		//kalman_state.print();

		//kalman_state.cov[10]=0.001;
		//kalman_state.cov[15]=0.001;
		if(debug_companion_akp_){
			std::cout<< "\n (before3) Kalman propagate:" << std::endl;
			kalman_state.print();
		}

		//std::cout<< "\n Kalman correct to propagation track " << std::endl;
//		kalman_state.print();

		if (it_myTrack->get_if_track_is_update() == true)
		{

			/*Ini local tracker, compensate rotational velocity*/
			/*double dist_robot_people_x=kalman_state.x;
			double dist_robot_people_y=kalman_state.y;
			double wz=actual_wz_;
			double v_rot_x= sqrt((dist_robot_people_y*wz)*(dist_robot_people_y*wz));
			double v_rot_y= sqrt((dist_robot_people_x*wz)*(dist_robot_people_x*wz));
			double track_vx=kalman_state.vx + v_rot_x;
			double track_vy=kalman_state.vy + v_rot_y;
			kalman_state.vx=track_vx;
			kalman_state.vy=track_vy;*/
			/*Final local tracker, compensate rotational velocity*/

			kalman_tracks_out_.push_back(SdetectionObservation(id, kalman_state));
			//if (debug_cruces_fallos)
			//{
			if(debug_companion_akp_){
				std::cout << "\n (put tracks in scene_tracks_people_prediction_) kalman_tracks_out_.back().print() "<< std::endl;
				std::cout << "\n id_track="<<it_myTrack->get_idTracker()<< std::endl;
				kalman_tracks_out_.back().print();
			}
			//}
		}
	}


 // update_scene(const std::vector<SdetectionObservation>& observation,bool& we_have_person_companion)
    bool we_have_person_companion=false;

	scene_tracks_people_prediction_.update_scene(kalman_tracks_out_,we_have_person_companion); //change TODO: (kalman_tracks_out)

	time_stamp_anterior_ = time_stamp_;

	if (debug_cruces_fallos)
	{
		std::cout << "\n salgo de propagation tracks. " << std::endl;
	}
}

void Cmht::mht_print_results()
{
//This function print the external tracker results, and also the intern tracker results and the propagation results (I can choose what results I want to see each time)

	std::cout << std::endl << std::endl<< " Entro en return_results_Checkout_with_matlab" << std::endl;

	printf("\n  ");

	for (unsigned int k = 0; k < exit_params_mht_.size(); k++)
	{
		std::cout << " STRACKS = tracks que envio fuera! \n " << std::endl;

		std::cout << " track confirmado" << std::endl;
		std::cout << " id_track:" << exit_params_mht_[k].returnId()<< std::endl;
		std::cout << " probability:" << exit_params_mht_[k].returnProbability()<< std::endl;
		//std::cout<< " valor track_confirmed_:" << it_myTrack->get_track_confirmed() << std::endl;
		std::cout << " iteration:(num track que envio):" << k << std::endl;
		std::cout << " kalman_state_cov=[cov_xx,cov_xy,cov_xVx,cov_xVy]"<< std::endl;
		SpointV_cov kalman_state = exit_params_mht_[k].getSpointV_cov();
		std::cout << " kalman_state_cov=[cov_xx,cov_xy,cov_xVx,cov_xVy]"<< std::endl;
		kalman_state.print();
		std::cout << " kalman_state_velocity_orientation="<< kalman_state.orientation() << std::endl;
		std::cout << " kalman_state=[ cov_xx  , cov_xy  , cov_xVx  , cov_xVy  ]"<< std::endl;
		std::cout << " kalman_state=[ cov_yx  , cov_yy  , cov_yVx  , cov_yVy  ]"<< std::endl;
		std::cout << " kalman_state=[ cov_Vxx , cov_Vxy , cov_VxVx , cov_VxVy ]"<< std::endl;
		std::cout << " kalman_state=[ cov_Vyx , cov_Vyy , cov_VyVx , cov_VyVy ]"<< std::endl;

		std::cout << std::endl;
		std::cout << std::endl;

		double dist_robot_people_x=kalman_state.x;
		double dist_robot_people_y=kalman_state.y;
		double wz=actual_wz_;
		double v_rot_x= sqrt((dist_robot_people_y*wz)*(dist_robot_people_y*wz));
		double v_rot_y= sqrt((dist_robot_people_x*wz)*(dist_robot_people_x*wz));

		std::cout << " v_rot_x="<< v_rot_x << std::endl;
		std::cout << " v_rot_y="<< v_rot_y << std::endl;
		std::cout << " linear_vx="<< linear_vx_ << std::endl;
		std::cout << " lineal_vy="<< linear_vy_ << std::endl;
		double track_vx=kalman_state.vx + linear_vx_ + v_rot_x;
		double track_vy=kalman_state.vy + linear_vy_ + v_rot_y;
		std::cout << " track_vx="<< track_vx << std::endl;
		std::cout << " track_vy="<< track_vy << std::endl;

	}

	/*	for(unsigned int j=0; j< exit_propagations_mht_.size(); j++)
	 {
	 std::cout<< " PROPAGATIONS = propagations que envio fuera! \n " << std::endl;

	 std::cout<< " propagations" << std::endl;
	 std::cout<< " id_propagation:" <<  exit_propagations_mht_[j].returnId()<< std::endl;
	 std::cout<< " probability:" <<  exit_propagations_mht_[j].returnProbability() << std::endl;
	 //std::cout<< " valor track_confirmed_:" << it_myTrack->get_track_confirmed() << std::endl;
	 std::cout<< " iteration:(num propagation que envio):" << j<< std::endl;
	 std::cout<< " propagation_state: " << std::endl;
	 SpointV_cov propagation_state= exit_propagations_mht_[j].getSpointV_cov();
	 propagation_state.print();

	 }*/

	//if(exit_params_mht_.empty())
	//{
	//std::cout<< " track no confirmado" << std::endl;
	//scene_tracks_people_prediction_.print();
	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		std::cout << " TRACKS MIOS = tracks que tengo yo! \n" << std::endl;

		int id_track = it_myTrack->get_idTracker();
		std::cout << " id_track:" << id_track << std::endl;
		std::cout << " valor track_confirmed_:"<< it_myTrack->get_track_confirmed() << std::endl;
		SpointV_cov kalman_state = it_myTrack->get_kalman_update(); //TODO:como obtengo el spose de la clase SpointV_cov!???
		std::cout << " kalman_state.x:" << kalman_state.x << std::endl;
		std::cout << " kalman_state.y:" << kalman_state.y << std::endl;
		std::cout << " kalman_state_cov=[cov_xx,cov_xy,cov_xVx,cov_xVy]"<< std::endl;
		kalman_state.print();
		double probability = it_myTrack->get_probability();
		std::cout << " probability:" << probability << std::endl;
		std::cout << " kalman_state_velocity_orientation="<< kalman_state.orientation() << std::endl;

		std::cout << std::endl;
		std::cout << std::endl;

		double dist_robot_people_x=kalman_state.x;
		double dist_robot_people_y=kalman_state.y;
		double wz=actual_wz_;
		double v_rot_x= sqrt((dist_robot_people_y*wz)*(dist_robot_people_y*wz));
		double v_rot_y= sqrt((dist_robot_people_x*wz)*(dist_robot_people_x*wz));
		double v_rot_x2= dist_robot_people_y*wz;
		double v_rot_y2= dist_robot_people_x*wz;

		std::cout << " v_rot_x="<< v_rot_x << std::endl;
		std::cout << " v_rot_y="<< v_rot_y << std::endl;
		std::cout << " v_rot_x2="<< v_rot_x2 << std::endl;
		std::cout << " v_rot_y2="<< v_rot_y2 << std::endl;
		std::cout << " linear_vx="<< linear_vx_ << std::endl;
		std::cout << " lineal_vy="<< linear_vy_ << std::endl;
		double track_vx=kalman_state.vx + linear_vx_ + v_rot_x;   // vale para todos los casos, pero segun la odometria a veces no sale velocidad 0, estando parado!
		double track_vy=kalman_state.vy + linear_vy_ + v_rot_y;
		std::cout << " track_vx="<< track_vx << std::endl;
		std::cout << " track_vy="<< track_vy << std::endl;
	}
	//}
//	printf("\n \n");
}

// functions that change variables in dinamicReconfigure (change variables when are testing the tracker)

void Cmht::set_threshold_distance(double threshold_distance_in)
{
	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: threshold_distance

	threshold_distance_ = threshold_distance_in;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
			std::cout<< " \n CHANGE! CHANGE!!! set_threshold_distance: \n \n" <<  std::endl;
		it_myTrack->set_threshold_distance_InTrack(threshold_distance_in);
	}
}

void Cmht::set_threshold_confirmation_track(double threshold_confirmation_track_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: threshold_confirmation_track

	mht_threshold_confirmation_track_ = threshold_confirmation_track_in; //(posible)TODO: futura posible confirmacion de los tracks en la mht y no dentro de cada track... (mirar)

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
			std::cout<< " \n CHANGE! CHANGE!!! set_threshold_confirmation_track: \n \n" <<  std::endl;
		it_myTrack->set_threshold_confirmation_track_InTrack(threshold_confirmation_track_in);
	}

}

void Cmht::set_laser_Pd(double laser_Pd_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: laser_Pd

	mht_laser_Pd_ = laser_Pd_in;

	//me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
			std::cout<< " \n CHANGE! CHANGE!!! set_laser_Pd: \n \n" <<  std::endl;
		it_myTrack->set_laser_Pd_InTrack(laser_Pd_in);
	}
}

void Cmht::set_laser_beta_ft(double laser_beta_ft_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: laser_beta_ft

	mht_laser_beta_ft_ = laser_beta_ft_in;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
		std::cout<< " \n CHANGE! CHANGE!!! set_laser_beta_ft: \n \n" <<  std::endl;
		it_myTrack->set_laser_beta_ft_InTrack(laser_beta_ft_in);
	}
}

void Cmht::set_laser_beta_nt(double laser_beta_nt_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: laser_beta_nt

	mht_laser_beta_nt_ = laser_beta_nt_in;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
		std::cout<< " \n CHANGE! CHANGE!!! set_laser_beta_nt: \n \n" <<  std::endl;
		it_myTrack->set_laser_beta_nt_InTrack(laser_beta_nt_in);
	}
}

void Cmht::set_laser_beta_no_detection(double laser_beta_no_detection_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: laser_beta_no_detection

	mht_laser_beta_no_detection_ = laser_beta_no_detection_in;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
		std::cout<< " \n CHANGE! CHANGE!!! set_laser_beta_no_detection: \n \n" <<  std::endl;
		it_myTrack->set_laser_beta_no_detection_InTrack(
				laser_beta_no_detection_in);
	}
}

void Cmht::set_laser_beta_track_no_confirmed(double laser_beta_track_no_confirmed_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: laser_beta_track_no_confirmed

	mht_laser_beta_track_no_confirmed_ = laser_beta_track_no_confirmed_in;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
		std::cout<< " \n CHANGE! CHANGE!!! set_laser_beta_track_no_confirmed: \n \n" <<  std::endl;
		it_myTrack->set_laser_beta_track_no_confirmed_InTrack(laser_beta_track_no_confirmed_in);

	}
}

void Cmht::set_increment_iteration_track_no_confirmed(double increment_iteration_track_no_confirmed_in)
{

	//This function set externally (when we are testing the tracker) with dinamicReconfigure the parameter: increment_iteration_track_no_confirmed

	mht_increment_iteration_track_no_confirmed_ =increment_iteration_track_no_confirmed_in;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	//for( Ctrack it_myTrack : mytracks_list_)
	{
		std::cout<< "\n \n CHANGE! CHANGE!!! set_increment_iteration_track_no_confirmed \n \n" << std::endl;
		it_myTrack->set_increment_iteration_track_no_confirmed_InTrack(increment_iteration_track_no_confirmed_in);
	}
}

////////////////////////////////////////////////////////
/// Functions for crosses (clusters) of multi-people ///
////////////////////////////////////////////////////////

// OJO! crosses of people=clusters

void Cmht::mht_insert_cluster(const std::vector<unsigned int>& tracks,const std::vector<unsigned int>& detections,const std::vector<std::vector<double>>& distances,const std::vector<std::vector<unsigned int>>& tracks_associated)
{
	// This function Insert new clusters in mycluster_list_actual_   (list of actual clusters/crosses in this actual iteration.

	// if cluster not exist, and if not is a merge cluster.
	bool cluster_exist = false;
	if (!mycluster_list_actual_.empty())
	{ // if the list is not empty

		// Search in all the list if this cluster exist.
		for (std::list<Scluster>::iterator it_myCluster =mycluster_list_actual_.begin();it_myCluster != mycluster_list_actual_.end(); it_myCluster++) //me recorro mis clusters.
		{
			std::vector<unsigned int> tacks_in_cluster_list =it_myCluster->get_tracks_in_cluster();
			tacks_in_cluster_list=arrange_tracks(tacks_in_cluster_list);
			std::vector<unsigned int> tracks2=arrange_tracks(tracks);

			if (tracks2 == tacks_in_cluster_list)
			{ //(ya comprobado)TODO: comprobar si puedo comparar este vector ASI!!! ojo!
				cluster_exist = true;
			}
		}
	}

	//I search in my track list where I have to put this track for maintain the track list in order.
// make the cluster if not exist!

	if (cluster_exist == false)
	{ //(!cluster_exist) //(ya comprobado)TODO: ver si va también así.


		Scluster new_cluster = Scluster(count_my_clusters_actual_, tracks,detections, distances, tracks_associated);

/*      std::cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!! new_Scluster: Scluster distances:" << std::endl;
		std::vector<std::vector<double>> asoci_det=new_cluster.get_association_distances_Scluster();
		for(unsigned int o=0;o<asoci_det.size();o++){
			for(unsigned int w=0;w<asoci_det[o].size();w++){
				std::cout<< "det="<<o<< "asoci_det["<<o<<"]["<<w<<"]="<<asoci_det[o][w]<< std::endl;
			}
		}
*/
		//count_my_clusters_actual_++;
		it_insert_cluster_actual_ = mycluster_list_actual_.begin();

		if (!mycluster_list_actual_.empty())
		{
			while (it_insert_cluster_actual_ != mycluster_list_actual_.end())
			{
				it_insert_cluster_actual_++;
				if (it_insert_cluster_actual_->get_idCluster()<= count_my_clusters_actual_)
				{
					break;
				}
			}
		}
		mycluster_list_actual_.insert(it_insert_cluster_actual_, new_cluster);
		count_my_clusters_actual_++;
	}
}

//void Cmht::mht_merge_split_cluster( const std::vector<unsigned int>& tracks , const std::vector<unsigned int>& detections ){
// Merge clusters and split clusters!!! TODO
//}

void Cmht::mht_made_clusters(std::vector<Sdetection>& obs)
{
// This function Made the new clusters for this actual iteration (then you compare and update the existent clusters with this actual clusters)

	//reiniciar the cluster list and the counter.
	count_my_clusters_actual_ = 1; //reinicias el count_my_clusters_actual para esta iteración de cruces (clusters).
	mycluster_list_actual_.clear();

// 1- find clusters.
	detection_in_cluster_.clear();
	detection_in_cluster_.resize(obs.size());
	for (unsigned int j = 0; j < obs.size(); j++)
	{
		detection_in_cluster_.push_back(false);
	}

	for (unsigned int i_detection = 0; i_detection < obs.size(); i_detection++)
	{

		// ini variable clusters.
		bool is_cluster = false; //variable for assign a cluster when the detection can pertain to a two tracks or two detections inside two tracks, etc.
		std::vector<unsigned int> ini_detections;
		std::vector<unsigned int> ini_tracks;
		std::vector<std::vector<unsigned int>> ini_tracks_associated;
		ini_tracks_associated.resize(obs.size());
		std::vector<std::vector<double>> ini_distances;
		ini_distances.resize(obs.size());
		std::vector<bool> track_in_cluster; // mirar si track_in_cluster va también fuera o no!!! (ojo)!!!
		for (unsigned int m = 0; m < count_my_tracks_; m++)
		{
			track_in_cluster.push_back(false);
		}

		tracks_associated_this_detection_.clear(); //tracks for the first detection + tracks incorporated for other detections inside of the first tracks (tracks encadenados por culpa de detecciones diferentes).
		//como los tracks están en orden si incorporo un nuevo track de una deteccion nueva asociada, luego se añadiran el resto de detecciones que contenga ese track de indice superior.

		// ini make cluster for this det if it is necessary.

		if ((!OneDetection_ntracks_association_[i_detection].empty())) //check that the detection is not associated to (ningún) track.
		{ //if have detection associated to one track or have a track associated. Discard the cases of only one detection or only one track (only one track is discarded automatically.

			tracks_associated_this_detection_ =OneDetection_ntracks_association_[i_detection];
			std::vector<double> distances_associated_this_detection =vector_distances_[i_detection];
			//	std::cout<< "\n size_tracks_associated_:" <<  tracks_associated_this_detection_.size() << std::endl;

			if ((tracks_associated_this_detection_.size() > 1)&& (detection_in_cluster_[i_detection] == false))
			{
//    			std::cout<< "\n creo un NUEVO cluster:" << std::endl;
//				std::cout<< "\n i_detection:"<<i_detection << std::endl;

				is_cluster = true;
				ini_detections.push_back(i_detection);
				detection_in_cluster_[i_detection] = true;

				for (unsigned int k = 0;k < tracks_associated_this_detection_.size(); k++)
				{

//    				std::cout<< "\n tracks_associated_this_detection_[k]:"<<tracks_associated_this_detection_[k] << std::endl;
					//   				std::cout<< "\n distancia:"<<distances_associated_this_detection[k] << std::endl;
					ini_tracks.push_back(tracks_associated_this_detection_[k]);

					ini_tracks_associated[i_detection].push_back(tracks_associated_this_detection_[k]);
					ini_distances[i_detection].push_back(distances_associated_this_detection[k]);

					track_in_cluster[tracks_associated_this_detection_[k]] =true;
				}

				for (unsigned int ind_track = 0;ind_track < tracks_associated_this_detection_.size();ind_track++) //find matches for the tracks of this detection.
				{
					//for this tracks I have to compare each of them with the tracks associated for all of the other detections.

					for (unsigned int i_detection2 = 0; i_detection2 < obs.size();i_detection2++)
					{
						//std::cout<< "\n ind_track:" <<  ind_track << std::endl;
						//std::cout<< "\n tracks_associated_this_:" <<  tracks_associated_this_detection_[ind_track] << std::endl;

						tracks_other_detections_for_compare_.clear();
						tracks_other_detections_for_compare_ =OneDetection_ntracks_association_[i_detection2]; //get the tracks for this other detection.
						std::vector<double> distances_other_detections =vector_distances_[i_detection2];

						for (unsigned int ind_track2 = 0;ind_track2< tracks_other_detections_for_compare_.size();ind_track2++)
						{ //compare all of the tracks in this other detection with this actual track in the first detection in cluster.

							//std::cout<< "\n ind_track2:" <<  ind_track2 << std::endl;
							//std::cout<< "\n tracks_associated_other_:" <<   tracks_other_detections_for_compare_[ind_track2] << std::endl;

							if ((tracks_associated_this_detection_[ind_track]== tracks_other_detections_for_compare_[ind_track2])&& (i_detection != i_detection2))
							{
								//if first detection only have 1 track associated (1 track <--> 1 detection)
								if ((is_cluster == false)&& (detection_in_cluster_[i_detection]== false))
								{
									//std::cout<< "\n creo un cluster2:" << std::endl;
	//								std::cout<< "\n creo un NUEVO cluster:" << std::endl;
	//								std::cout<< "\n i_detection:"<<i_detection << std::endl;
									is_cluster = true;
									ini_detections.push_back(i_detection);

									detection_in_cluster_[i_detection] = true;
									for (unsigned int k = 0;k< tracks_associated_this_detection_.size();k++)
									{
	//									std::cout<< "\n tracks_associated_this_detection_[k]:"<<tracks_associated_this_detection_[k] << std::endl;
	//									std::cout<< "\n distancia:"<<vector_distances_[i_detection][k] << std::endl;
										if(track_in_cluster[tracks_associated_this_detection_[k]]==false){
											ini_tracks_associated[i_detection].push_back(tracks_associated_this_detection_[k]);
											ini_tracks.push_back(tracks_associated_this_detection_[k]);
											ini_distances[i_detection].push_back(distances_associated_this_detection[k]);
											track_in_cluster[tracks_associated_this_detection_[k]] =true;
										}
									}
								}

								//cases for the second detection.
								if (detection_in_cluster_[i_detection2] == false)
								{
									//std::cout<< "\n creo un cluster3:" << std::endl;
	//								std::cout<< "\n i_detection2:"<<i_detection2 << std::endl;
									is_cluster = true;
									ini_detections.push_back(i_detection2); //if cluster don't have this detection, I have to include it, otherwise not.
									detection_in_cluster_[i_detection2] = true;

									for (unsigned int l = 0;l< tracks_other_detections_for_compare_.size();l++)
									{

	//									std::cout<< "\n tracks_associated_this_detection_[k]:"<<tracks_associated_this_detection_[l] << std::endl;
	//									std::cout<< "\n distancia:"<<vector_distances_[i_detection2][l] << std::endl;

										ini_distances[i_detection2].push_back(distances_other_detections[l]);
										ini_tracks_associated[i_detection2].push_back(tracks_other_detections_for_compare_[l]);

										if (track_in_cluster[tracks_other_detections_for_compare_[l]]== false)
										{ // if not is this track in cluster, add it!

											ini_tracks.push_back(tracks_other_detections_for_compare_[l]);
											track_in_cluster[tracks_other_detections_for_compare_[l]] =true;
										}
									}
								}
							}
						}
					}
				}

			}
		}

		if (is_cluster == true)
		{
//    		std::cout<< "\n Insert cluster!:" << std::endl;
			mht_insert_cluster(ini_tracks, ini_detections, ini_distances,ini_tracks_associated);
			//count_my_clusters_actual_++;
		}
	}

// imprimir clusters por pantalla, comprobar clusters.
	if (debug_cruces_fallos)
	{
		std::cout << "\n print SClusters" << std::endl;
		for (std::list<Scluster>::iterator it_myCluster =mycluster_list_actual_.begin();it_myCluster != mycluster_list_actual_.end(); it_myCluster++) //me recorro mis clusters actuales.
		{
			it_myCluster->print_cluster();
		}
	}

}

void Cmht::find_Ctrack_in_cluster(std::vector<unsigned int> tracks)
{

// This function find the pointers to the tracker_tracks (my_tracks) that contain one cluster.

//	std::cout<< "\n find_Ctrack_in_cluster" << std::endl;
	it_Ctracks_cluster_mht_.clear();

//	std::cout<< "\n tracks.size()=" <<tracks.size()<< std::endl;

	for (unsigned int t = 0; t < tracks.size(); t++)
	{
		unsigned int id_track = tracks[t];
		std::list<Ctrack>::iterator act_it_Track;

		for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
		{
			if (it_myTrack->get_idTracker() == id_track)
			{
				act_it_Track = it_myTrack;
			}
		}
		it_Ctracks_cluster_mht_.push_back(act_it_Track);
	}
}

void Cmht::mht_insert_cluster_in_mht_cluster_list(Ccluster cluster_new)
{

	// This function insert new cluster in mycluster_list_ (this list contain the continuous existent clusters,
	// remaining clusters between time), if this cluster exist don't insert, in other code part
	// I make the merge of this actual cluster and the before cluster.

	// if cluster not exist, and if not is a merge cluster.
	bool cluster_exist = false;

	std::vector<unsigned int> tracks = cluster_new.get_tracks_in_cluster_mht();

	if (!mycluster_list_.empty())
	{ // if the list is not empty

		// Search in all the list if this cluster exist.
		for (std::list<Ccluster>::iterator it_myCluster2 =mycluster_list_.begin(); it_myCluster2 != mycluster_list_.end();it_myCluster2++) //me recorro mis clusters.
		{
			std::vector<unsigned int> tacks_in_cluster_list =it_myCluster2->get_tracks_in_cluster_mht();
			if (tracks == tacks_in_cluster_list)
			{ //TODO: comprobar si puedo comparar este vector ASI!!! ojo!
				cluster_exist = true;
			}
		}
	}

	//I search in my track list where I have to put this track for maintain the track list in order.
// make the cluster if not exist!
	if (cluster_exist == false)
	{ //(!cluster_exist) //TODO: ver si va también así.

		//count_my_clusters_actual_++;

		it_insert_cluster_ = mycluster_list_.begin();

		if (!mycluster_list_.empty())
		{
			while (it_insert_cluster_ != mycluster_list_.end())
			{
				it_insert_cluster_++;
				if (it_insert_cluster_->get_id_cluster_mht()
						> count_my_clusters_)
				{
					break;
				}
			}
		}

		mycluster_list_.insert(it_insert_cluster_, cluster_new);

	}

}

bool Cmht::mht_compare_clusters(std::vector<unsigned int> actual_tracks,std::vector<unsigned int> cluster_mht_tracks)
{

// This function compare the actual cluster with the before cluster.

	bool cluster_exist = false;

	// INICIO ordenar tracks en esos vectores de tracks de cada cluster
	std::vector<unsigned int> actual_tracks1=arrange_tracks(actual_tracks);
	std::vector<unsigned int> cluster_mht_tracks1=arrange_tracks(cluster_mht_tracks);
	// FIN ode

	for (unsigned int it_act_trac = 0; it_act_trac < actual_tracks1.size();it_act_trac++)
	{
		for (unsigned int it_mht_track = 0;it_mht_track < cluster_mht_tracks1.size(); it_mht_track++)
		{

			if (actual_tracks1[it_act_trac] == cluster_mht_tracks1[it_mht_track])
			{
				cluster_exist = true;
			}

		}
	}

	return cluster_exist;
}

std::vector<unsigned int> Cmht::mht_find_tracks_act_in_cluster_mht_for_update_cluster_list_mht(const std::vector<unsigned int>& tracks_act,const std::vector<unsigned int>& tracks_mht)
{

	// for merge clusters, (for clusters with some equal tracks) find the tracks that contain the before cluster and the tracks that contain only the actual cluster.

	std::vector<bool> tracks_act_in_cluster_mht; // tracks in cluster actual, that exist in the mht continuous cluster.
	std::vector<unsigned int> tracks_not_in_cluster;

	for (unsigned int size1; size1 < tracks_act.size(); size1++)
	{
		tracks_act_in_cluster_mht.push_back(false);
	}

	for (unsigned int it_tracks_act; it_tracks_act < tracks_act.size();it_tracks_act++)
	{
		for (unsigned int it_tracks_mht; it_tracks_mht < tracks_mht.size();it_tracks_mht++)
		{
			if (tracks_act[it_tracks_act] == tracks_mht[it_tracks_mht])
			{
				tracks_act_in_cluster_mht[it_tracks_act] = true;
			}
		}
		if (tracks_act_in_cluster_mht[it_tracks_act] == false)
		{
			// este track no esta en el cluster
			tracks_not_in_cluster.push_back(tracks_act[it_tracks_act]);
		}
	}

	return tracks_not_in_cluster; // thracks not in cluster mht (the remaining cluster)

}

std::vector<bool> Cmht::mht_find_tracks_mht_in_cluster_act_for_update_separate_clusters(const std::vector<unsigned int>& tracks_act,const std::vector<unsigned int>& tracks_mht,std::vector<unsigned int>& tracks_in_mht_but_not_in_actual_cluster)
{

// This function find the tracks that not contain the actual cluster, but are contained in the mht cluster (the before remaining cluster).

	std::vector<bool> tracks_mht_in_cluster_act; // tracks in cluster actual, that exist in the mht continuous cluster.

	for (unsigned int size1; size1 < tracks_mht.size(); size1++)
	{
		tracks_mht_in_cluster_act.push_back(false);
	}

	for (unsigned int it_tracks_mht; it_tracks_mht < tracks_mht.size();it_tracks_mht++)
	{
		for (unsigned int it_tracks_act; it_tracks_act < tracks_act.size();it_tracks_act++)
		{
			if (tracks_act[it_tracks_act] == tracks_mht[it_tracks_mht])
			{
				tracks_mht_in_cluster_act[it_tracks_mht] = true; // OJO!, con esto así, si reordeno los tracks la lio!!! (mirar si hay que reordenarlos o no!)
			}
		}

		if (tracks_mht_in_cluster_act[it_tracks_mht] == false)
		{
			// este track no esta en el cluster
			tracks_in_mht_but_not_in_actual_cluster.push_back(tracks_mht_in_cluster_act[it_tracks_mht]);
		}
	}

	return tracks_mht_in_cluster_act; // tracks not in actual cluster and it's tracks are contained in the mht cluster (the remaining cluster)
}

void Cmht::mht_update_mht_cluster_list(std::vector<Sdetection>& obs)
{

// This function make the update/the merge between the mht_cluster(before and remaining clusters) and the actual_clusters

// hay que buscar si el cluster para esta iteracion existe ya en mi lista de clusters de la clase Ccluster, y si no existe, crearlo!!! y luego hacerle el update.

// TODO: en esta funcion es donde puedo hacer el merge de clusters, etc si hace falta!!!
	for (std::list<Ccluster>::iterator it_myCluster_mht =mycluster_list_.begin(); it_myCluster_mht != mycluster_list_.end();it_myCluster_mht++) //me recorro mis clusters del mht.
	{
		it_myCluster_mht->set_bool_cluster_update_mht(false); // initialization of all cluster updates, before the update.
	}

//	std::cout<< "\n MHT UPDATE CLUSTER LIST 1!!! " << std::endl;
	std::vector<bool> actual_cluster_in_mht;
	for (unsigned int it = 0; it < mycluster_list_actual_.size(); it++) //me recorro mis clusters actuales.
	{
		actual_cluster_in_mht.push_back(false);
	}
//	std::cout<< "\n MHT UPDATE CLUSTER LIST 2!!! " << std::endl;
	unsigned int it = 0;
//	std::cout<< "\n mycluster_list_actual_.size():!!! " <<mycluster_list_actual_.size()<< std::endl;
//	std::cout<< "\n mycluster_list_mht_.size():!!! " <<mycluster_list_.size()<< std::endl;

	/// probar creación correcta de clusters en ROS
//	std::cout<< "\n NEW INICIO!!! COMPARE CLUSTERS !!! " << std::endl;
//	std::cout<< "\n inicio Cluster actual!" << std::endl;
	/*	for(std::list<Scluster>::iterator it_myCluster = mycluster_list_actual_.begin() ; it_myCluster!=mycluster_list_actual_.end() ; it_myCluster++ ) //me recorro mis clusters actuales.
	 {
	 it_myCluster->print_cluster();
	 }*/
//	std::cout<< "\n Inicio Cluster mht!" << std::endl;
	/*	for(std::list<Ccluster>::iterator it_myCluster_mht = mycluster_list_.begin() ; it_myCluster_mht!=mycluster_list_.end() ; it_myCluster_mht++ ) //me recorro mis clusters del mht.
	 {
	 it_myCluster_mht->print_Ccluster();
	 }
	 */

	if (debug_cruces_v_)
	{
		std::cout << "\n CLUSTER actual!" << std::endl;
	}

	if (debug_cruces_2015_)
	{
		std::cout << "\n mycluster_list_actual_.empty(): "<< mycluster_list_actual_.empty() << std::endl;
	}

	for (std::list<Scluster>::iterator it_myCluster =mycluster_list_actual_.begin();it_myCluster != mycluster_list_actual_.end(); it_myCluster++) //me recorro mis clusters actuales.
	{
		if (debug_cruces_2015_)
		{
			std::cout << "\n print cluster, id: " << it_myCluster->id_cluster_<< std::endl;
		}

//if(!cargo_ANTS){

		for (std::list<Ccluster>::iterator it_myCluster_mht =mycluster_list_.begin();it_myCluster_mht != mycluster_list_.end(); it_myCluster_mht++) //me recorro mis clusters del mht.
		{
			if (debug_cruces_2015_)
			{
				/*std::cout<< "\n Cluster EXISTE!!! COMPARE CLUSTERS !!! " << std::endl;
				 std::cout<< "\n Cluster actual!" << std::endl;
				 it_myCluster->print_cluster();
				 std::cout<< "\n Cluster mht!" << std::endl;
				 it_myCluster_mht->print_Ccluster();
				 std::cout<< "\n tracks, comparation " << std::endl;

				 std::cout<< "\n list_actual tracks size: "<< it_myCluster->tracks_.size() << std::endl;
				 for (unsigned int x=0; x<it_myCluster->tracks_.size();x++){
				 std::cout<< "\n list_actual, track["<<x<<"]= "<< it_myCluster->tracks_[x] << std::endl;
				 }

				 std::cout<< "\n mycluster_list_ tracks size: "<< it_myCluster_mht->get_tracks_in_cluster_mht().size() << std::endl;
				 for (unsigned int x=0; x<it_myCluster_mht->get_tracks_in_cluster_mht().size();x++){
				 std::cout<< "\n mycluster_list_, track["<<x<<"]= "<< it_myCluster_mht->get_tracks_in_cluster_mht()[x] << std::endl;
				 }*/
			}

			// for the case 2: cluster exist but this cluster change the tracks that its has.
			bool cluster_exist;
			cluster_exist = mht_compare_clusters(it_myCluster->tracks_,it_myCluster_mht->get_tracks_in_cluster_mht());

			if (debug_cruces_fallos)
			{
				std::cout << " cluster_exist=" << cluster_exist << std::endl;
			}
			// INICIO CASOS CLUSTERS!

			std::vector<unsigned int> tracks_mht1;
			std::vector<unsigned int> tracks1;

			tracks_mht1 = it_myCluster_mht->get_tracks_in_cluster_mht();
			tracks1 = it_myCluster->tracks_;
			bool the_cluster_have_the_same_tracks;
			the_cluster_have_the_same_tracks = arrange_tracks_of_one_cluster(tracks_mht1, tracks1);

			//if(it_myCluster->tracks_==it_myCluster_mht->get_tracks_in_cluster_mht()){ //TODO: si pasa algo raro, puede que sea que hay que reordenar los tracks en el cluster al hacerlo/guardarlo.
			if (debug_cruces_fallos)
			{
				std::cout << " the_cluster_have_the_same_tracks="<< the_cluster_have_the_same_tracks << std::endl;
			}

			if (the_cluster_have_the_same_tracks)
			{
				// CASE 1: Cluster exist and with the same number of tracks.
				if (debug_cruces_2015_)
				{
					std::cout << "\n 11111 update existent cluster! id:"<< it_myCluster_mht->get_id_cluster_mht()<< std::endl;
					std::cout << "\n (entro en if) Cluster EXISTE!!! "<< std::endl;
				}

				// 1- bool update_cluster==true. (This cluster has been update)
				actual_cluster_in_mht[it] = true;
				it_myCluster_mht->set_bool_cluster_update_mht(true);

				// 2- change the actual detection associated to the tracks. (These detections change in each iteration)
				it_myCluster_mht->set_detections_in_cluster_mht(it_myCluster->detections_);

				if (debug_cruces_2015_){
					std::vector<unsigned int> act_det=it_myCluster_mht->get_detections_in_cluster_mht();
					for(unsigned int g=0;g<act_det.size();g++){
						std::cout << "act_det["<<g<<"]"<< act_det[g]<< std::endl;
					}
				}

				// 3- change the actual distances between each track<-->detection associated.(change in each iteration)
				it_myCluster_mht->set_distances_in_cluster_mht(it_myCluster->association_distances_);

				if (debug_cruces_2015_){
					std::vector< std::vector<double> > asso_dist=it_myCluster_mht->get_association_distances_mht();
					for(unsigned int g=0;g<asso_dist.size();g++){
						for(unsigned int p=0;p<asso_dist[g].size();p++){
							std::cout << "det="<<g<<"asso_dist["<<g<<"]["<<p<<"]"<< asso_dist[g][p]<< std::endl;
						}
					}
				}

				//4- change tracks associated to these detections, because the tracks can be rearranged in other order.
				it_myCluster_mht->set_tracks_associated_in_cluster_mht(it_myCluster->tracks_associated_);

				if (debug_cruces_2015_){
					std::vector< std::vector<unsigned int> > trac_asso=it_myCluster_mht->get_tracks_associated_mht();
					for(unsigned int g=0;g<trac_asso.size();g++){
						for(unsigned int p=0;p<trac_asso[g].size();p++){
							std::cout << "det="<<g<<"trac_asso["<<g<<"]["<<p<<"]"<< trac_asso[g][p]<< std::endl;
						}
					}
				}
				// tracks quedan igual cambian!!! (OJO CAMBIAR)

				// 5- change the pointers to the tracks in the cluster, because the tracks can be rearranged in other order.
				find_Ctrack_in_cluster(it_myCluster->tracks_);
				it_myCluster_mht->set_Ctracks_cluster_mht(it_Ctracks_cluster_mht_);

				// 6- set true for the tracks that are in cross situations. (tracks in cluster)
				for (unsigned int it1 = 0; it1 < it_Ctracks_cluster_mht_.size();it1++)
				{
					it_Ctracks_cluster_mht_[it1]->set_track_in_coss_situatuion(true);
				}

/***********************************************************************/
				// See velocities!!!
				std::vector<unsigned int> tracks_in_mht =it_myCluster_mht->get_tracks_in_cluster_mht();
				std::vector<double> ini_vel_x =it_myCluster_mht->get_initial_velocities_x_mht();
				std::vector<double> ini_vel_y =it_myCluster_mht->get_initial_velocities_y_mht();
				std::vector<double> ini_ori =it_myCluster_mht->get_initial_orientations_mht();
				std::vector<double> ini_vel =it_myCluster_mht->get_initial_velocity_mht();
				std::vector<unsigned int> actual_detections =it_myCluster_mht->get_detections_in_cluster_mht(); // ELIMINAR cuando acabe con las comprobaciones de lo de los cruces en velocidades!!!
				std::vector<double> params ={ 2.3, 0.0, 0.0, 0.0, 0.0 }; //TODO temporal solution

				for (unsigned int it_tr = 0; it_tr < tracks_in_mht.size();it_tr++)
				{
					for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin();
							it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
					{

						if (tracks_in_mht[it_tr] == it_myTrack->get_idTracker())
						{
							SpointV_cov track;
							int id_track_actual = it_myTrack->get_idTracker();
							Cperson_abstract* person_obj;
							//SpointV_cov track_pose;

							if (scene_tracks_people_prediction_.find_person(id_track_actual, &person_obj)) //find the person/detection associated with this track.
							{
								track = person_obj->get_current_pointV(); //I get the spose of this person.
								//	track.print();

							}
							double gonzalo_orientation = track.orientation();

							if (debug_cruces_v_)
							{
								std::cout << "\n track in cluster, track id: "<< it_myTrack->get_idTracker()<< std::endl;
								std::cout << " gonzalo velocity.vx: "<< track.vx << std::endl;
								std::cout << " gonzalo velocity.vy: "<< track.vy << std::endl;
								std::cout << " Gonzalo velocity.V: "<< track.v() << std::endl;

								std::cout << " INICIAL velocity.vx: "<< ini_vel_x[it_tr] << std::endl;
								std::cout << " INICIAL velocity.vy: "<< ini_vel_y[it_tr] << std::endl;
								std::cout << " INICIAL velocity.V: "<< ini_vel[it_tr] << std::endl;

								std::cout << " Gonzalo angle_orientation: "<< gonzalo_orientation << std::endl;
								std::cout << " INICIAL angle_orientation: "<< ini_ori[it_tr] << "\n" << std::endl;

								/*check visually the angle differential between the inicial_track and the actual detection*/

								std::cout << "deteccion size="<< actual_detections.size()<< std::endl;
								std::cout << "track size="<< initial_crossing_tracks.size()<< std::endl;
							}
							for (unsigned int i_detection = 0;i_detection < actual_detections.size();i_detection++)
							{
								Spoint detection_actual(obs[i_detection].x,obs[i_detection].y,obs[i_detection].time_stamp);

								//initial_crossing_tracks=it_myCluster->tracks_associated_; // OJO!!!, mirar si usar estos esta bien!!!

								for (unsigned int i_ini_tracks = 0;i_ini_tracks< initial_crossing_tracks.size();i_ini_tracks++)
								{
									SpointV_cov inicial_track =initial_crossing_tracks[i_ini_tracks];

									if (debug_cruces_v_)
									{
										std::cout << "\n deteccion id="<< actual_detections[i_detection]<< std::endl;
										//detection_actual.print();
										std::cout << "track id="<< initial_id_crossing_tracks[i_ini_tracks]<< std::endl;
										//inicial_track.print();
										/*							   					std::cout<< "detection_actual.x="<<detection_actual.x<< std::endl;
										 std::cout<< "initial_track.x="<<inicial_track.x<< std::endl;
										 std::cout<< "detection_actual.y="<<detection_actual.y<< std::endl;
										 std::cout<< "initial_track.y="<<inicial_track.y<< std::endl;
										 double inc_x1=detection_actual.x-inicial_track.x;
										 double inc_y1=detection_actual.y-inicial_track.y;
										 std::cout<< "inc_x1="<<inc_x1<< std::endl;
										 std::cout<< "inc_y1="<<inc_y1<< std::endl;
										 double inc_x2=inicial_track.x-detection_actual.x;
										 double inc_y2=inicial_track.y-detection_actual.y;
										 std::cout<< "inc_x2="<<inc_x2<< std::endl;
										 std::cout<< "inc_y2="<<inc_y2<< std::endl;
										 */}

									double increm_time =inicial_track.time_stamp- detection_actual.time_stamp;
									//double angle_act=atan2(((detection_actual.y-inicial_track.y)/increm_time),((detection_actual.x-inicial_track.x)/increm_time));
									double angle_act = atan2(((inicial_track.y- detection_actual.y)/ increm_time),((inicial_track.x- detection_actual.x)/ increm_time));
									double diferetial_angle =inicial_track.angle_heading_point(detection_actual);

									if (debug_cruces_v_)
									{
										std::cout<< "angle_act (angle det_act/ini_track)="<< angle_act << std::endl;
										std::cout<< "Angulo diferencia entre la deteccion actual y los tracks del cluster="<< diferetial_angle<< std::endl;
									}

									//double cost=cost_to_go( inicial_track, detection_actual, &params);
									//TODO: coger track estado actual para el coste!!!, no va bien la ventana!!!

									SpointV_cov track;
									int id_track_actual =initial_id_crossing_tracks[i_ini_tracks];
									Cperson_abstract * person_obj;
									if (scene_tracks_people_prediction_.find_person(id_track_actual, &person_obj)) //find the person/detection associated with this track.
									{
										track =person_obj->get_current_pointV(); //I get the spose of this person.
										//	track.print();
									}
									double diferential_orientation = angle_act- track.orientation();
									double cost = cost_to_go(track,detection_actual, &params);

									if (debug_cruces_v_)
									{
										std::cout<< "Orientation track actual (id="<< initial_id_crossing_tracks[i_ini_tracks]<< ")=" << track.orientation()<< std::endl;

										std::cout << "diferential_orientation="<< diferential_orientation<< std::endl;

										std::cout<< "Cost to go from pose_track_actual to actual_detection="<< cost << std::endl;
									}
									// PD for orientation!!!
									double Pd_orient;
									double Pd_orient_mod;
									if ((diferetial_angle > -0.4)&& (diferetial_angle < 0.4))
									{
										Pd_orient = log(fabs(diferetial_angle))/ log(3);
										Pd_orient_mod = fabs(Pd_orient);
										//std::cout<< " Pd_orient: "<<Pd_orient<<"\n"<< std::endl;
										if (debug_cruces_v_)
										{
											std::cout << " Pd_orient_mod: "<< Pd_orient_mod<< std::endl;
										}
									}
									else if ((diferetial_angle < -0.4)&& (diferetial_angle > -0.8))
									{ // exponencial que emieze en 1 y decrezca hacia la izquierda.

										Pd_orient = 0.337/ fabs(diferetial_angle); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}
									}
									else if ((diferetial_angle > 0.4)&& (diferetial_angle < 0.8))
									{ // exponencial que emieze en 1 y decrezca hacia la derecha.

										Pd_orient = 0.337/ fabs(diferetial_angle);
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}
									}
									else if (diferetial_angle < -0.8)
									{

										Pd_orient = pow(3,-fabs(diferetial_angle));
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}

									}
									else if (diferetial_angle > 0.8)
									{

										Pd_orient = pow(3,-fabs(diferetial_angle));
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}

									}

									/// for see the different distances between tracks<--->detections, track1<--->track2
									SpointV_cov track_act;
									Cperson_abstract* person_obj_act;
									if (scene_tracks_people_prediction_.find_person(initial_id_crossing_tracks[i_ini_tracks],&person_obj_act)) //find the person/detection associated with this track.
									{
										track_act =person_obj->get_current_pointV(); //I get the spose of this person.
										//	track.print();

									}
									Spoint_cov track_pose(track_act.x,track_act.y, track_act.time_stamp,track_act.cov);
									Spoint_cov detection_actual_cov(obs[i_detection].x,obs[i_detection].y,obs[i_detection].time_stamp,obs[i_detection].cov);

									double distance_laser_track =track_pose.distance(detection_actual_cov);
									if (debug_cruces_v_)
									{
										std::cout << "distance detection("<< actual_detections[i_detection]<< ")<-->track("<< initial_id_crossing_tracks[i_ini_tracks]<< ")=" << distance_laser_track<< "\n" << std::endl;
									}
								}
							}

							/*fin check visually the angle differential between the inicial_track and the actual detection*/

							//Spoint_cov detection_actual( obs[i_detection].x , obs[i_detection].y,obs[i_detection].time_stamp,obs[i_detection].cov);
							double Pd_vel_x;
							if ((track.vx > ini_vel_x[it_tr] - 0.30)&& (track.vx < ini_vel_x[it_tr] + 0.20))
							{
								Pd_vel_x = 1;
							}
							else if (track.vx < ini_vel_x[it_tr] - 0.30)
							{ // exponencial que emieze en 1 y decrezca hacia la izquierda.

								Pd_vel_x = pow(5,-(ini_vel_x[it_tr] - 0.30 - track.vx)); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
							}
							else if (track.vx > ini_vel_x[it_tr] + 0.20)
							{ // exponencial que emieze en 1 y decrezca hacia la derecha.

								Pd_vel_x = pow(2,(ini_vel_x[it_tr] + 0.20 - track.vx));

							}
							if (debug_cruces_v_)
							{
								std::cout << "\n Pd_vel_x: " << Pd_vel_x<< std::endl;
							}
							// Pd for velocities and orientarions:
							double Pd_vel_y;
							if ((track.vy > ini_vel_y[it_tr] - 0.30)&& (track.vy < ini_vel_y[it_tr] + 0.20))
							{
								Pd_vel_y = 1;
							}
							else if (track.vy < ini_vel_y[it_tr] - 0.30)
							{ // exponencial que emieze en 1 y decrezca hacia la izquierda.

								Pd_vel_y = pow(5,-(ini_vel_y[it_tr] - 0.30 - track.vy)); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
							}
							else if (track.vy > ini_vel_y[it_tr] + 0.20)
							{ // exponencial que emieze en 1 y decrezca hacia la derecha.

								Pd_vel_y = pow(2,ini_vel_y[it_tr] + 0.20 - track.vy);

							}
							if (debug_cruces_v_)
							{
								std::cout << "Pd_vel_y: " << Pd_vel_y << "\n"<< std::endl;
							}
							// podría pedir aquí las predicciones futuras para las detecciones y añadirlas al cluster, o hacerlo dentro. (si lo hago aquí, en los casos de update del cluster, he de hacerlo también) Pero las velocidades NO se tocan!!!

							//	   			std::cout<< "\n SpointV_cov track "<< std::endl;
							//	   			track.print();

						}
					}
				}

				/* calculate distance track<-->track between tracks in crosses. */
				// para distancia entre tracks del cluster:
				std::vector<unsigned int> actual_id_crossing_tracks =it_myCluster->tracks_; // actual_id_crossing_tracks cambiado por initial_crossing_tracks. y falta entrar los tracks actuales a la clase cluster.
				std::vector<Spoint_cov> tracks_in_clust_a;
				std::vector<SpointV_cov> actual_tracks_in_clust;
				for (unsigned int i_ini_tracks_3 = 0;i_ini_tracks_3 < actual_id_crossing_tracks.size();i_ini_tracks_3++)
				{
					SpointV_cov track_3; //=it_myTrack->get_kalman_update();
					int id_track_actual_3 =actual_id_crossing_tracks[i_ini_tracks_3]; //it_myTrack->get_idTracker();
					Cperson_abstract* person_obj;
					if (scene_tracks_people_prediction_.find_person(id_track_actual_3, &person_obj)) //find the person/detection associated with this track.
					{
						track_3 = person_obj->get_current_pointV(); //I get the spose of this person.
																	//	track.print();
					}
					Spoint_cov track_3c(track_3.x, track_3.y,track_3.time_stamp, track_3.cov);
					tracks_in_clust_a.push_back(track_3c);
					actual_tracks_in_clust.push_back(track_3);
				}

				//it_myCluster_mht->set_actual_tracks_in_cluster_mht(actual_tracks_in_clust);

				std::vector<double> vector_distance_between_tracks;

				for (unsigned int i_t = 0; i_t < tracks_in_clust_a.size();i_t++)
				{
					Spoint_cov track_i = tracks_in_clust_a[i_t];
					for (unsigned int j_t = 0; j_t < tracks_in_clust_a.size();j_t++)
					{
						if (i_t != j_t)
						{
							Spoint_cov track_j = tracks_in_clust_a[j_t];
							double distan = track_i.distance(track_j);
							if (debug_cruces_v_)
							{
								std::cout << "distance track("<< actual_id_crossing_tracks[i_t]<< ")<-->track("<< actual_id_crossing_tracks[j_t]<< ")=" << distan << std::endl;
							}
							//std::cout<< "track_i:("<<initial_id_crossing_tracks[i_t]<<")"<< std::endl;
							//track_i.print();
							//std::cout<< "track_j:("<<initial_id_crossing_tracks[j_t]<<")"<< std::endl;
							//track_j.print();
							vector_distance_between_tracks.push_back(distan);
						}
					}
				}

				//10- Set the new distances between the tracks that are in cluster.

				it_myCluster_mht->set_distances_between_tracks_in_cluster_mht(vector_distance_between_tracks);

				/*FIN calculate distance track<-->track between tracks in crosses. */

				/*	std::vector<unsigned int> detections=it_myCluster_mht->get_detections_in_cluster_mht();
				 std::vector<SpointV_cov> initial_tracks=it_myCluster_mht->get_initial_tracks_mht();
				 for(unsigned int detec=0;detec<detections.size();detec++){
				 for(unsigned int ini_track=0; ini_track<initial_tracks.size(); ini_track++){
				 double dt=obs[detections[detec]].time_stamp-initial_tracks[ini_track].time_stamp;
				 if ( dt  < 0.001 ) dt = 0.1;

				 double vx_act=(obs[detections[detec]].x-initial_tracks[ini_track].x)/dt;
				 double vy_act=(obs[detections[detec]].y-initial_tracks[ini_track].y)/dt;
				 double orient_act=atan2(vx_act,vy_act);
				 std::cout<< "\n  obs["<<detections[detec]<<"].x: "<<obs[detections[detec]].x << std::endl;
				 std::cout<< "\n  obs["<<detections[detec]<<"].y: "<<obs[detections[detec]].y << std::endl;
				 std::cout<< "\n  initial_tracks["<<tracks_in_mht[ini_track]<<"].x: "<<initial_tracks[ini_track].x << std::endl; // poner id del track!
				 std::cout<< "\n  initial_tracks["<<tracks_in_mht[ini_track]<<"].y: "<<initial_tracks[ini_track].y << std::endl;
				 std::cout<< "\n  dt: "<<dt << std::endl;

				 std::cout<< "\n  vx_act: "<<vx_act << std::endl;
				 std::cout<< "\n  vy_act: "<<vy_act << std::endl;
				 std::cout<< "\n  orient_act: "<<orient_act<< std::endl;

				 }
				 }*/
				if (debug_cruces_v_)
				{
					it_myCluster_mht->print_Ccluster();
				}

/***********************************************************************/
			}
			else if ((cluster_exist == true)&& (!the_cluster_have_the_same_tracks))
			{
				// CASE 2 and 3: Cluster exist but this cluster change the tracks that its has. Include more or less tracks.
				if (debug_cruces_2015_)
				{
					std::cout<< "!!!!!!!!!!!!3333 The same cluster with more or less tracks exist"<< std::endl;
					//std::cout<< "\n SIMILAR cluster exist:" << std::endl;
					//std::cout<< "\n id mht:" <<it_myCluster_mht->get_id_cluster_mht()<< std::endl;
					//std::cout<< "\n id actual:" <<it_myCluster->id_cluster_<< std::endl;
				}

				// 1- bool update_cluster==true. (This cluster has been update)
				actual_cluster_in_mht[it] = true;
				it_myCluster_mht->set_bool_cluster_update_mht(true);

				// 2- change the actual detection associated to the tracks. (These detections change in each iteration)
				it_myCluster_mht->set_detections_in_cluster_mht(it_myCluster->detections_);

				// 3- change the actual distances between each track<-->detection associated.(change in each iteration)
				it_myCluster_mht->set_distances_in_cluster_mht(it_myCluster->association_distances_);

				//4- change tracks associated to these detections, because the tracks can be rearranged in other order.
				it_myCluster_mht->set_tracks_associated_in_cluster_mht(it_myCluster->tracks_associated_);

				//5- change tracks in cluster because they change, are less or more!
				it_myCluster_mht->set_tracks_in_cluster_mht(it_myCluster->tracks_);

				// 6- change the pointers to the tracks in the cluster, because the tracks can be rearranged in other order.
				find_Ctrack_in_cluster(it_myCluster->tracks_);
				it_myCluster_mht->set_Ctracks_cluster_mht(it_Ctracks_cluster_mht_);

				// 7- set true for the tracks that are in cross situations. (tracks in cluster)
				for (unsigned int it1 = 0; it1 < it_Ctracks_cluster_mht_.size();it1++)
				{
					it_Ctracks_cluster_mht_[it1]->set_track_in_coss_situatuion(true); // estos tracks estan en cluster!
				}

				if (debug_cruces_2015_)
				{
					it_myCluster_mht->print_Ccluster();
				}

/***********************************************************************/
				// See velocities!!!
				std::vector<unsigned int> tracks_in_mht =it_myCluster_mht->get_tracks_in_cluster_mht();
				std::vector<double> ini_vel_x =it_myCluster_mht->get_initial_velocities_x_mht();
				std::vector<double> ini_vel_y =it_myCluster_mht->get_initial_velocities_y_mht();
				std::vector<double> ini_ori =it_myCluster_mht->get_initial_orientations_mht();
				std::vector<double> ini_vel =it_myCluster_mht->get_initial_velocity_mht();
				std::vector<unsigned int> actual_detections =it_myCluster_mht->get_detections_in_cluster_mht(); // ELIMINAR cuando acabe con las comprobaciones de lo de los cruces en velocidades!!!
				std::vector<double> params ={ 2.3, 0.0, 0.0, 0.0, 0.0 }; //TODO temporal solution

				for (unsigned int it_tr = 0; it_tr < tracks_in_mht.size();it_tr++)
				{
					for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
					{

						if (tracks_in_mht[it_tr] == it_myTrack->get_idTracker())
						{
							SpointV_cov track;
							int id_track_actual = it_myTrack->get_idTracker();
							Cperson_abstract* person_obj;
							if (scene_tracks_people_prediction_.find_person(id_track_actual, &person_obj)) //find the person/detection associated with this track.
							{
								track = person_obj->get_current_pointV(); //I get the spose of this person.
								//	track.print();
							}
							double gonzalo_orientation = track.orientation();
							if (debug_cruces_v_)
							{
								std::cout << "\n track in cluster, track id: "<< it_myTrack->get_idTracker()<< std::endl;
								std::cout << " gonzalo velocity.vx: "<< track.vx << std::endl;
								std::cout << " gonzalo velocity.vy: "<< track.vy << std::endl;
								std::cout << " Gonzalo velocity.V: "<< track.v() << std::endl;
								std::cout << " INICIAL velocity.vx: "<< ini_vel_x[it_tr] << std::endl;
								std::cout << " INICIAL velocity.vy: "<< ini_vel_y[it_tr] << std::endl;
								std::cout << " INICIAL velocity.V: "<< ini_vel[it_tr] << std::endl;

								std::cout << " Gonzalo angle_orientation: "<< gonzalo_orientation << std::endl;
								std::cout << " INICIAL angle_orientation: "<< ini_ori[it_tr] << "\n" << std::endl;
							}

							for (unsigned int i_detection = 0;i_detection < actual_detections.size();i_detection++)
							{
								Spoint detection_actual(obs[i_detection].x,obs[i_detection].y,obs[i_detection].time_stamp);

								for (unsigned int i_ini_tracks = 0;i_ini_tracks< initial_crossing_tracks.size();i_ini_tracks++)
								{
									SpointV_cov inicial_track =initial_crossing_tracks[i_ini_tracks];
									if (debug_cruces_v_)
									{
										std::cout << "\n deteccion id="<< actual_detections[i_detection]<< std::endl;
										//detection_actual.print();
										std::cout << "track id="<< initial_id_crossing_tracks[i_ini_tracks]<< std::endl;
									}
									//inicial_track.print();
									double increm_time =inicial_track.time_stamp- detection_actual.time_stamp;
									//double angle_act=atan2(((detection_actual.y-inicial_track.y)/increm_time),((detection_actual.x-inicial_track.x)/increm_time));
									double angle_act = atan2(((inicial_track.y- detection_actual.y)/ increm_time),((inicial_track.x- detection_actual.x)/ increm_time));
									if (debug_cruces_v_)
									{
										std::cout<< "angle_act (angle det_act/ini_track)="<< angle_act << std::endl;
									}
									double diferetial_angle =inicial_track.angle_heading_point(detection_actual);
									if (debug_cruces_v_)
									{
										std::cout<< "Angulo diferencia entre la deteccion actual y los tracks del cluster="<< diferetial_angle<< std::endl;
									}
									// DIFERENTIAL_ANGLE, cuanto más cercano a 0, más similares son las orientaciones de esos dos puntos y es el que pertenece a ese track.
									//  DIFERENTIAL_ANGLE<0.2 (11 grados), de desvio en orientación en el cruce son permitidos.
									// en cruces normales con orientaciones de 180grados de diferencia, DIFERENTIAL_ANGLE=0.40 significa que es la detección de la otra persona, se situa esa deteccion
									// en la linea paralela a esta.

									//double cost=cost_to_go( inicial_track, detection_actual, &params);
									SpointV_cov track;
									int id_track_actual =initial_id_crossing_tracks[i_ini_tracks];
									Cperson_abstract * person_obj;
									if (scene_tracks_people_prediction_.find_person(id_track_actual, &person_obj)) //find the person/detection associated with this track.
									{
										track =person_obj->get_current_pointV(); //I get the spose of this person.
										//	track.print();
									}
									if (debug_cruces_v_)
									{
										std::cout<< "Orientation track actual (id="<< initial_id_crossing_tracks[i_ini_tracks]<< ")=" << track.orientation()<< std::endl;
									}
									double diferential_orientation = angle_act- track.orientation();
									if (debug_cruces_v_)
									{
										std::cout << "diferential_orientation="<< diferential_orientation<< std::endl;
									}
									double cost = cost_to_go(track,detection_actual, &params);
									if (debug_cruces_v_)
									{
										std::cout<< "Cost to go from pose_track_actual to actual_detection="<< cost << std::endl;
									}
									// PD for orientation!!!
									double Pd_orient;
									double Pd_orient_mod = 0.0;
									if ((diferetial_angle > -0.2)&& (diferetial_angle < 0.2))
									{

										Pd_orient = log(fabs(diferetial_angle))/ log(5);
										Pd_orient_mod = fabs(Pd_orient);
										//std::cout<< "Pd_orient: "<<Pd_orient<<"\n"<< std::endl;
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient_mod: "<< Pd_orient_mod<< std::endl;
										}
									}
									else if ((diferetial_angle < -0.2)&& (diferetial_angle > -0.8))
									{ // exponencial que emieze en 1 y decrezca hacia la izquierda.

										Pd_orient = pow(0.5,fabs(diferetial_angle)); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}
									}
									else if ((diferetial_angle > 0.2)&& (diferetial_angle < 0.8))
									{ // exponencial que emieze en 1 y decrezca hacia la derecha.

										Pd_orient = pow(0.5,fabs(diferetial_angle));
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}
									}
									else if (diferetial_angle < -0.8)
									{
										Pd_orient = pow(3,-fabs(diferetial_angle));
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}
									}
									else if (diferetial_angle > 0.8)
									{
										Pd_orient = pow(3,-fabs(diferetial_angle));
										if (debug_cruces_v_)
										{
											std::cout << "Pd_orient: "<< Pd_orient << std::endl;
										}
									}

									/// for see the different distances between tracks<--->detections, track1<--->track2
									SpointV_cov track_act;
									Cperson_abstract* person_obj_act;
									if (scene_tracks_people_prediction_.find_person(initial_id_crossing_tracks[i_ini_tracks],&person_obj_act)) //find the person/detection associated with this track.
									{
										track_act =person_obj->get_current_pointV(); //I get the spose of this person.
										//	track.print();
									}
									Spoint_cov track_pose(track_act.x,track_act.y, track_act.time_stamp,track_act.cov);
									Spoint_cov detection_actual_cov(obs[i_detection].x,obs[i_detection].y,obs[i_detection].time_stamp,obs[i_detection].cov);

									double distance_laser_track =track_pose.distance(detection_actual_cov);
									if (debug_cruces_v_)
									{
										std::cout << "\n distance detection("<< actual_detections[i_detection]<< ")<-->track("<< initial_id_crossing_tracks[i_ini_tracks]<< ")=" << distance_laser_track<< "\n" << std::endl;
									}
								}
							}

							/*fin check visually the angle differential between the inicial_track and the actual detection*/

							double Pd_vel_x;
							if (((track.vx > ini_vel_x[it_tr] - 0.30)&& (track.vx < ini_vel_x[it_tr] + 0.20))|| (track.vx == 0)|| (ini_vel_x[it_tr] == 0))
							{
								Pd_vel_x = 1;
							}
							else if (track.vx < ini_vel_x[it_tr] - 0.30)
							{ // exponencial que emieze en 1 y decrezca hacia la izquierda.

								Pd_vel_x = pow(5,-(ini_vel_x[it_tr] - 0.30 - track.vx)); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
							}
							else if (track.vx > ini_vel_x[it_tr] + 0.20)
							{ // exponencial que emieze en 1 y decrezca hacia la derecha.

								Pd_vel_x = pow(2,(ini_vel_x[it_tr] + 0.20 - track.vx));

							}
							if (debug_cruces_v_)
							{
								std::cout << "\n Pd_vel_x: " << Pd_vel_x<< std::endl;
							}
							// Pd for velocities and orientarions:
							double Pd_vel_y;
							if (((track.vy > ini_vel_y[it_tr] - 0.30)&& (track.vy < ini_vel_y[it_tr] + 0.20))|| (track.vy == 0)|| (ini_vel_y[it_tr] == 0))
							{
								Pd_vel_y = 1;
							}
							else if (track.vy < ini_vel_y[it_tr] - 0.30)
							{ // exponencial que emieze en 1 y decrezca hacia la izquierda.

								Pd_vel_y = pow(5,-(ini_vel_y[it_tr] - 0.30 - track.vy)); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
							}
							else if (track.vy > ini_vel_y[it_tr] + 0.20)
							{ // exponencial que emieze en 1 y decrezca hacia la derecha.

								Pd_vel_y = pow(2,ini_vel_y[it_tr] + 0.20 - track.vy);

							}
							if (debug_cruces_v_)
							{
								std::cout << "\n Pd_vel_y: " << Pd_vel_y<< std::endl;
							}
							// podría pedir aquí las predicciones futuras para las detecciones y añadirlas al cluster, o hacerlo dentro. (si lo hago aquí, en los casos de update del cluster, he de hacerlo también) Pero las velocidades NO se tocan!!!
							//	   			std::cout<< "\n SpointV_cov track "<< std::endl;
							//	   			track.print();

						}
					}
				}

				/* INI calculate distance track<-->track between tracks in crosses. */
				// para distancia entre tracks del cluster:
				std::vector<Spoint_cov> tracks_in_clust_a;
				std::vector<SpointV_cov> actual_tracks_in_clust;
				for (unsigned int i_ini_tracks_3 = 0;i_ini_tracks_3 < initial_id_crossing_tracks.size();i_ini_tracks_3++)
				{
					SpointV_cov track_3; //=it_myTrack->get_kalman_update();
					int id_track_actual_3 =initial_id_crossing_tracks[i_ini_tracks_3]; //it_myTrack->get_idTracker();
					Cperson_abstract* person_obj;
					if (scene_tracks_people_prediction_.find_person(id_track_actual_3, &person_obj)) //find the person/detection associated with this track.
					{
						track_3 = person_obj->get_current_pointV(); //I get the spose of this person.
																	//	track.print();
					}
					Spoint_cov track_3c(track_3.x, track_3.y,track_3.time_stamp, track_3.cov);
					tracks_in_clust_a.push_back(track_3c);
					actual_tracks_in_clust.push_back(track_3);
				}

				//it_myCluster_mht->set_actual_tracks_in_cluster_mht(actual_tracks_in_clust);

				std::vector<double> vector_distance_between_tracks;

				for (unsigned int i_t = 0; i_t < tracks_in_clust_a.size();i_t++)
				{
					Spoint_cov track_i = tracks_in_clust_a[i_t];
					for (unsigned int j_t = 0; j_t < tracks_in_clust_a.size();j_t++)
					{
						if (i_t != j_t)
						{
							Spoint_cov track_j = tracks_in_clust_a[j_t];
							double distan = track_i.distance(track_j);

							if (debug_cruces_v_)
							{
								std::cout << "\n distance track("<< initial_id_crossing_tracks[i_t]<< ")<-->track("<< initial_id_crossing_tracks[j_t]<< ")=" << distan << std::endl;
								std::cout << "\n track_i:("<< initial_id_crossing_tracks[i_t]<< ")" << std::endl;
								track_i.print();
								std::cout << "track_j:("<< initial_id_crossing_tracks[j_t]<< ")" << std::endl;
								track_j.print();
							}
							vector_distance_between_tracks.push_back(distan);
						}
					}
				}
				it_myCluster_mht->set_distances_between_tracks_in_cluster_mht(vector_distance_between_tracks);
				/*FIN calculate distance track<-->track between tracks in crosses. */

				/*		std::vector<unsigned int> detections=it_myCluster_mht->get_detections_in_cluster_mht();
				 std::vector<SpointV_cov> initial_tracks=it_myCluster_mht->get_initial_tracks_mht();
				 for(unsigned int detec=0;detec<detections.size();detec++){
				 for(unsigned int ini_track=0; ini_track<initial_tracks.size(); ini_track++){
				 double vx_act=(obs[detections[detec]].x-initial_tracks[ini_track].x)/(obs[detections[detec]].time_stamp-initial_tracks[ini_track].time_stamp);
				 double vy_act=(obs[detections[detec]].y-initial_tracks[ini_track].y)/(obs[detections[detec]].time_stamp-initial_tracks[ini_track].time_stamp);
				 double orient_act=atan2(vx_act,vy_act);
				 std::cout<< "\n  vx_act: "<<vx_act << std::endl;
				 std::cout<< "\n  vy_act: "<<vy_act << std::endl;
				 std::cout<< "\n  orient_act: "<<orient_act<< std::endl;

				 }
				 }*/
				if (debug_cruces_v_)
				{
					it_myCluster_mht->print_Ccluster();
				}

			}
			// TODO: merge similar clusters!!!
			// find part of tracks that are equal.
			// insert the new tracks of the cluster and the new detections, and distances, for the new detections included!
/***********************************************************************/
		}

//}
		std::vector<unsigned int> it_myCluster_tracks = it_myCluster->tracks_;
		unsigned int tracks_in_cluster_size = it_myCluster_tracks.size();
		if(debug_cruces_fallos){
			std::cout << "\n !!!!!!!!! tracks_in_cluster_size="<< tracks_in_cluster_size << std::endl;
		}

		if ((actual_cluster_in_mht[it] == false)&& (tracks_in_cluster_size > 1))
		{
			// This cluster doesn't exist at this moment in the clusters list mht, add it!!!
			if (debug_cruces_v_)
			{
				std::cout<< "\n IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII!!!!!!!!!!!"<< std::endl;
				std::cout << "\n made new cluster!: id:"<< it_myCluster->id_cluster_ << std::endl;
			}

/***********************************************************************/
			find_Ctrack_in_cluster(it_myCluster->tracks_);
			std::vector<bool> tracks_separados_act;
			for (unsigned int z; z < it_myCluster->tracks_.size(); z++)
			{
				tracks_separados_act.push_back(false);
			}

			// find track inicial velocities for each track in cluster.
			std::vector<double> initial_Velocities_x;
			std::vector<double> initial_Velocities_y;
			std::vector<double> initial_orientations;
			std::vector<double> initial_v;
			std::vector<bool> vx_vy_ori_to_use_in_cluster;
			std::vector<SpointV_cov> initial_tracks;
			initial_crossing_tracks.clear();
			initial_id_crossing_tracks.clear();
			for (unsigned int it_tr = 0; it_tr < it_myCluster->tracks_.size();it_tr++)
			{
				for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
				{

					if (it_myCluster->tracks_[it_tr]== it_myTrack->get_idTracker())
					{

						SpointV_cov track;
						int id_track_actual = it_myTrack->get_idTracker();

						if (!it_myTrack->get_check_window_track_empty())
						{ // if deque is not empty, get it's first element.
							if (debug_cruces_v_)
							{
								std::cout<< "\n deque is NOT empty, have a window of the position for this track. "<< std::endl;
							}
							track = it_myTrack->get_first_window_track(); // probar si es mejor usar la ventana temporal esta en los cruces!!! y no exactamente la posición actual en el cruce.
							if (debug_cruces_v_)
							{
								std::cout << "\n track id: "<< it_myTrack->get_idTracker()<< "\n pos track cruce actual:"<< std::endl;
								std::cout<< "\n pos track cruce inicial ventana:"<< std::endl;
								track.print();
							}
						}
						else
						{
							Cperson_abstract* person_obj;
							if (scene_tracks_people_prediction_.find_person(id_track_actual, &person_obj)) //find the person/detection associated with this track.
							{
								if (debug_cruces_v_)
								{
									std::cout<< "\n deque is empty, get prediction current position. "<< std::endl;
								}
								track = person_obj->get_current_pointV(); //I get the spose of this person.
								if (debug_cruces_v_)
								{
									std::cout << "\n track id: "<< it_myTrack->get_idTracker()<< "\n pos track cruce actual:"<< std::endl;
									track.print();
								}
							}
						}

						initial_Velocities_x.push_back(track.vx);
						initial_Velocities_y.push_back(track.vy);
						initial_orientations.push_back(track.orientation());
						initial_v.push_back(track.v());
						initial_tracks.push_back(track);
						if (debug_cruces_v_)
						{
							std::cout<< "\n GONZALO track in cluster, track id: "<< id_track_actual << std::endl;
							std::cout << "\n Gonzalo INITIAL velocity.vx: "<< track.vx << std::endl;
							std::cout << "\n Gonzalo INITIAL velocity.vy: "<< track.vy << std::endl;
							std::cout << "\n Gonzalo INITIAL velocity.V: "<< track.v() << std::endl;
							std::cout<< "\n Gonzalo INITIAL angle_orientation: "<< track.orientation() << std::endl;
							std::cout << "\n Gonzalo INITIAL SpointV_cov track "<< std::endl; // hacer un spoint initial para los tracks iniciales al entrar al cluster para luego comparar con los SpointV_cov que tienes luego.
							track.print();
						}
						// TODO: pensar si con push_back o con posición de velocidad en el num/id del track.
//			   			std::cout<< "\n track in cluster, track id: "<<it_myTrack->get_idTracker() << std::endl;
//			   			std::cout<< "\n INITIAL velocity.vx: "<<track_ini.vx << std::endl;
//			   			std::cout<< "\n INITIAL velocity.vy: "<<track_ini.vy << std::endl;

//			   			double angle_orientation=atan2(track_ini.vy,track_ini.vx);
//			   			initial_orientations.push_back(angle_orientation);
//			   			std::cout<< "\n INITIAL angle_orientation: "<<angle_orientation << std::endl;
//			   			std::cout<< "\n INITIAL SpointV_cov track "<< std::endl;  // hacer un spoint initial para los tracks iniciales al entrar al cluster para luego comparar con los SpointV_cov que tienes luego.
//			   			track_ini.print();
						initial_crossing_tracks.push_back(track);
						initial_id_crossing_tracks.push_back(id_track_actual);
						// podría pedir aquí las predicciones futuras para las detecciones y añadirlas al cluster, o hacerlo dentro. (si lo hago aquí, en los casos de update del cluster, he de hacerlo también) Pero las velocidades NO se tocan!!!
					}
				}
			}

			// Inicio Show the diference in orientations for the tracks
			for (unsigned int it_ori = 0;it_ori < initial_crossing_tracks.size(); it_ori++)
			{
				for (unsigned int jt_ori = it_ori + 1;jt_ori < initial_crossing_tracks.size(); jt_ori++)
				{
					if (it_ori != jt_ori)
					{
						SpointV_cov i_track = initial_crossing_tracks[it_ori];
						SpointV_cov j_track = initial_crossing_tracks[jt_ori];
						double diferetial_orientation_mod = sqrt((i_track.orientation() - j_track.orientation())* (i_track.orientation()- j_track.orientation()));
						if (debug_cruces_v_)
						{
							std::cout << "Angulo diferencia entre la id_track"<< initial_id_crossing_tracks[it_ori]<< " y  id_track"<< initial_id_crossing_tracks[jt_ori]<< "del cluster="<< diferetial_orientation_mod << std::endl;
						}
					}
				}
			}
			// Fin Show the diference in orientations for the tracks

			/* INI calculate distance track<-->track between tracks in crosses. */
			// para distancia entre tracks del cluster:
			std::vector<Spoint_cov> tracks_in_clust_a;
			std::vector<SpointV_cov> actual_tracks_in_clust;
			for (unsigned int i_ini_tracks_3 = 0;i_ini_tracks_3 < initial_id_crossing_tracks.size();i_ini_tracks_3++)
			{
				SpointV_cov track_3; //=it_myTrack->get_kalman_update();
				int id_track_actual_3 =initial_id_crossing_tracks[i_ini_tracks_3]; //it_myTrack->get_idTracker();
				Cperson_abstract* person_obj;
				if (scene_tracks_people_prediction_.find_person(id_track_actual_3, &person_obj)) //find the person/detection associated with this track.
				{
					track_3 = person_obj->get_current_pointV(); //I get the spose of this person.
																//	track.print();
				}
				Spoint_cov track_3c(track_3.x, track_3.y, track_3.time_stamp,track_3.cov);
				tracks_in_clust_a.push_back(track_3c);
				actual_tracks_in_clust.push_back(track_3);
			}

			std::vector<double> vector_distance_between_tracks;

			for (unsigned int i_t = 0; i_t < tracks_in_clust_a.size(); i_t++)
			{
				Spoint_cov track_i = tracks_in_clust_a[i_t];
				for (unsigned int j_t = 0; j_t < tracks_in_clust_a.size();j_t++)
				{
					if (i_t != j_t)
					{
						Spoint_cov track_j = tracks_in_clust_a[j_t];
						double distan = track_i.distance(track_j);
						if (debug_cruces_v_)
						{
							std::cout << "\n distance track("<< initial_id_crossing_tracks[i_t]<< ")<-->track("<< initial_id_crossing_tracks[j_t] << ")="<< distan << std::endl;
							std::cout << "\n track_i:("<< initial_id_crossing_tracks[i_t] << ")"<< std::endl;
							track_i.print();
							std::cout << "track_j:("<< initial_id_crossing_tracks[j_t] << ")"<< std::endl;
							track_j.print();
						}
						vector_distance_between_tracks.push_back(distan);
					}
				}
			}
			/*FIN calculate distance track<-->track between tracks in crosses. */

			if (debug_cruces_v_)
			{
				it_myCluster->print_cluster();
			}
			// generate the new Cluster.

			// prueba initial crossing tracks.
			if (debug_cruces_v_)
			{
				for (unsigned int i = 0; i < initial_crossing_tracks.size();i++)
				{
					std::cout << "\n Entro en print inicial crosing tracks "<< std::endl;
					initial_crossing_tracks[i].print();
				}
			}
/***********************************************************************/

			// 1- Create the cluster and insert it in my cluster_list.
			Ccluster cluster_new(count_my_clusters_, it_myCluster->tracks_,it_myCluster->detections_,it_myCluster->association_distances_,it_myCluster->tracks_associated_, it_Ctracks_cluster_mht_,tracks_separados_act, initial_Velocities_x,initial_Velocities_y, initial_orientations, initial_tracks,initial_id_crossing_tracks, vx_vy_ori_to_use_in_cluster,initial_v, vector_distance_between_tracks,actual_tracks_in_clust);
			// insertion of new cluster in my_cluster_list in mht!!!.
			mht_insert_cluster_in_mht_cluster_list(cluster_new);
			actual_cluster_in_mht[it] = true;
			if (debug_cruces_v_){
				cluster_new.print_Ccluster();
			}
			count_my_clusters_++;
		}

		it++;
	}

}

void Cmht::mht_update_cluster_tracks(std::vector<Sdetection>& obs)
{

// This function make the "main" for the clusters. (make the creation, merge, and mht_tracking_update for the clusters and for the tracks in each cluster.

	// update parecido al de los tracks!
	//std::cout << "(1)  mht_update_cluster_tracks "<< std::endl;
	mht_update_mht_cluster_list(obs);
	//std::cout << "(2)  mht_update_cluster_tracks "<< std::endl;
	mht_remove_repeated_clusters();
	//std::cout << "(3)  mht_update_cluster_tracks "<< std::endl;
	for (std::list<Ccluster>::iterator it_myCluster_mht =mycluster_list_.begin(); it_myCluster_mht != mycluster_list_.end();it_myCluster_mht++) //me recorro mis clusters del mht.
	{
		//std::cout << "(4)  mht_update_cluster_tracks "<< std::endl;
		it_myCluster_mht->set_temp_count_my_tracks_mht(count_my_tracks_);
		it_myCluster_mht->set_temp_count_my_clusters_mht(count_my_clusters_);
		// update clustero o update_no_cluster según convenga.
		//std::cout << "(5)  mht_update_cluster_tracks "<< std::endl;
		std::vector<unsigned int> tracks_in_cluster =it_myCluster_mht->get_tracks_in_cluster_mht();

		if(tracks_in_cluster.size()>max_num_tracks_in_cluster_allowed_)
		{
			//it_myCluster_mht->update_cluster(obs); // make the update of each cluster.
			mht_cluster_update_with_many_tracks(it_myCluster_mht);
			it_myCluster_mht->set_bool_cluster_update_mht(false);
			//it_myCluster_mht->update_cluster(obs); // make the update of each cluster.
		}else{
			it_myCluster_mht->update_cluster(obs); // make the update of each cluster.
		}
		//std::cout << "(6)  mht_update_cluster_tracks "<< std::endl;
	}
	//std::cout << "(7)  mht_update_cluster_tracks "<< std::endl;
	mht_remove_clusters_by_probability();
	//std::cout << "(8)  mht_update_cluster_tracks "<< std::endl;
	if (!mycluster_list_.empty())
	{
		mht_update_tracks_with_winner_Pd_cluster(); // make the update for each track in the existent clusters.
	}
	//std::cout << "(9)  mht_update_cluster_tracks "<< std::endl;
}
///////////////////////////////////////////////////
void Cmht::mht_cluster_update_with_many_tracks(std::list<Ccluster>::iterator it_myCluster1 ){

	//This function is for allow cluster bejavior with a lot of tracks in cluster.
	// Don't make de complicated and better cluster association that make my tracker, but allow not be blocked in the cases that is imposible make my cluster calculations.
	// dejar cada deteccion asociada a un unico track, el más cercano!.

	std::vector<int> id_track_associated_to_this_det; //vector de ints que contiene la posicion de la deteccion más cercana a este track, para luego quedarme solo con esa!!!
	std::vector<double> min_distance;											// -1,case detection not associated!!!! saltarmelo!
	//std::vector<bool> temp_track_no_associated;  //temporal vector for tracks not associated with detection.
	bool one_det_n_tracks=false;

// Initialization of variables.

	for(unsigned int i=0; i<OneDetection_ntracks_association_.size();i++){
		id_track_associated_to_this_det.push_back(-1);
		//id==-1 if not have track associated this detection and if 1det<-->1track)
	}

	for(unsigned int i=0; i<OneDetection_ntracks_association_.size();i++){
		min_distance.push_back(100); //id==-1 if not have track associated this detection and if 1det<-->1track)
	}

		std::vector<unsigned int> detections_in_cluster=it_myCluster1->get_detections_in_cluster_mht();

		for(unsigned int i=0;i<detections_in_cluster.size();i++)
		{
			unsigned int detection_ac=detections_in_cluster[i];
			if(OneDetection_ntracks_association_[detection_ac].size()>1)
			{
		 		one_det_n_tracks=true;
				//busca el track a menor distancia y quedate con el!!!
				min_distance[detection_ac]=vector_distances_[detection_ac][0];
				id_track_associated_to_this_det[detection_ac]=OneDetection_ntracks_association_[detection_ac][0];

				for(unsigned int track_i=1; track_i<OneDetection_ntracks_association_[detection_ac].size();track_i++){
					if( min_distance[detection_ac] > vector_distances_[detection_ac][track_i] ){
						min_distance[detection_ac]=vector_distances_[detection_ac][track_i];
						id_track_associated_to_this_det[detection_ac]=OneDetection_ntracks_association_[detection_ac][track_i];
					}

				}

			}
		}

	   if(one_det_n_tracks){

		   std::vector<bool> temp_track_no_associated_=track_no_associated_;
		   track_no_associated_.clear();
		   	//temp_track_no_associated=track_no_associated_;
		   	for(unsigned int j=0; j<count_my_tracks_;j++)
		   	{
		   		track_no_associated_.push_back(true);
		   		// This variable have size equal to all the number of tracks in the scene and this value is no associated=true=1.
		   	}

		   std::vector< std::vector<unsigned int> > temp_OneDetection_ntracks_association= OneDetection_ntracks_association_;
		   std::vector< std::vector<double> > temp_vector_distances_=vector_distances_;
		   OneDetection_ntracks_association_.clear();
		   vector_distances_.clear();
		   std::vector<unsigned int> vect_tracks_temp;
		   std::vector<double> vect_distances_temp;

		   for (unsigned int i_detection = 0; i_detection< temp_OneDetection_ntracks_association.size(); i_detection++ )
		   {
			   vect_tracks_temp.clear();
			   vect_distances_temp.clear();

			   if( (!temp_OneDetection_ntracks_association[i_detection].empty()) && (id_track_associated_to_this_det[i_detection]!=-1))
			   //check that the detection is not associated to (ningún) track.
			   {
				   //case 1 detection <---> n Track associated (change to association with the nearest track)
				   vect_distances_temp.push_back(min_distance[i_detection]);
			   	   vect_tracks_temp.push_back( id_track_associated_to_this_det[i_detection] );
			   	   //de esta deteccion cojo el track que esta en la posicion que me marca pos_track_associated_to_this_det
			   	   // para esta deteccion!!!)
			   	   track_no_associated_[ id_track_associated_to_this_det[i_detection] ]=false;

			   }else if( (id_track_associated_to_this_det[i_detection]==-1) &&  (!temp_OneDetection_ntracks_association[i_detection].empty()) ){

				   //case 1 detection <---> 1 track o 1 detection <---> n track de otro cluster! (remain with the same association)
				   vect_distances_temp=temp_vector_distances_[i_detection];
				   vect_tracks_temp=temp_OneDetection_ntracks_association[i_detection];
				   for(unsigned int h=0; h<temp_OneDetection_ntracks_association[i_detection].size();h++){
				   	track_no_associated_[ temp_OneDetection_ntracks_association[i_detection][h] ]=temp_track_no_associated_[temp_OneDetection_ntracks_association[i_detection][h]];
				   }

			   }

			   OneDetection_ntracks_association_.push_back( vect_tracks_temp );
			   vector_distances_.push_back(vect_distances_temp);
		   }
	   }
	   if(!generate_database_companion_){
	   mht_find_two_dets_in_one_track(); // mirar que no haya un track con dos detecciones dentro!!!
	   }
}

void Cmht::mht_remove_clusters_by_probability()
{
// This function removes the clusters with less probability than a threshold of elimination.
	// And also remove clusters with only one track

	std::vector<std::list<Ccluster>::iterator> vector_punteros_a_mis_clusters;

//	std::cout<< " size lista de mis tracks (justo antes de borrar):" <<  mytracks_list_.size() << std::endl;

	//find clusters to erase.
	for (std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();it_myCluster != mycluster_list_.end(); it_myCluster++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		// for remove clusters by probability
		if ((it_myCluster->get_Pd_cluster_mht() < threshold_remove_cluster_))
		{
			//save pointers to my tracks for delete. (guardar punteros a mis tracks a borrar.)
			vector_punteros_a_mis_clusters.push_back(it_myCluster);
			//remove predictions in person propagation
		}

		// for remove clusters with only one track
		std::vector<unsigned int> cluster_tracks_mht =it_myCluster->get_tracks_in_cluster_mht();
		unsigned int tracks_size = cluster_tracks_mht.size();

		if (tracks_size == 1)
		{
			//save pointers to my tracks for delete. (guardar punteros a mis tracks a borrar.)
			vector_punteros_a_mis_clusters.push_back(it_myCluster);
		}

	}

	//Delete tracks (borrar tracks a borrar encontrados.)

	for (unsigned int i = 0; i < vector_punteros_a_mis_clusters.size(); i++)
	{
		//remove this track
		std::vector<unsigned int> tracks_mht =vector_punteros_a_mis_clusters[i]->get_tracks_in_cluster_mht();

		for (unsigned int it1 = 0; it1 < tracks_mht.size(); it1++)
		{
			for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin(); it_myTrack != mytracks_list_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			{
				if (tracks_mht[it1] == it_myTrack->get_idTracker())
				{
					it_Ctracks_cluster_mht_[it1]->set_track_in_coss_situatuion(false); // estos tracks ya no estan en cluster!
				}
			}
		}

		mycluster_list_.erase(vector_punteros_a_mis_clusters[i]); //vector_punteros_a_mis_tracks[i]==it_myTrack -> pointer to my track for delete.
	}

	vector_punteros_a_mis_clusters.clear(); //clear the vector of pointer for deleting tracks, for the next iteration.
}

void Cmht::mht_remove_clusters_with_track_removed(unsigned int id)
{
// This function removes the clusters that have one track that is removed.

	std::vector<std::list<Ccluster>::iterator> vector_punteros_a_mis_clusters_with_track_delete;

	//find clusters to erase.
	for (std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();it_myCluster != mycluster_list_.end(); it_myCluster++)
	{
		std::vector<unsigned int> cluster_tracks_mht =it_myCluster->get_tracks_in_cluster_mht();

		for (unsigned int Ctra = 0; Ctra < cluster_tracks_mht.size(); Ctra++)
		{
			if (id == cluster_tracks_mht[Ctra])
			{
				//save pointers to my tracks for delete. (guardar punteros a mis tracks a borrar.)
				vector_punteros_a_mis_clusters_with_track_delete.push_back(it_myCluster);
			}
		}
	}

	for (unsigned int i = 0;i < vector_punteros_a_mis_clusters_with_track_delete.size(); i++)
	{
		std::vector<unsigned int> tracks_mht =vector_punteros_a_mis_clusters_with_track_delete[i]->get_tracks_in_cluster_mht();

		for (unsigned int it1 = 0; it1 < tracks_mht.size(); it1++)
		{
			for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin(); it_myTrack != mytracks_list_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			{
				if (tracks_mht[it1] == it_myTrack->get_idTracker())
				{
					it_Ctracks_cluster_mht_[it1]->set_track_in_coss_situatuion(false); // estos tracks ya no estan en cluster!
				}
			}
		}

		//remove this Cluster
		mycluster_list_.erase(vector_punteros_a_mis_clusters_with_track_delete[i]);
	}

	vector_punteros_a_mis_clusters_with_track_delete.clear(); //clear the vector of pointer for deleting clusters, for the next iteration.
}

void Cmht::mht_remove_repeated_clusters()
{
	//This Function: If have two clusters with some same tracks, remove the cluster with the biggest id_cluster
	// two clusters with the same tracks occur when two clusters are combined, the best form is removing hiar one of the two ecual clusters!!! (no los puedes mezclar, y los dos acaban con los mismos tracks y todo igual, lo unico diferente son los id's y probabilides.
	//TODO: ojo! mirar las velocidades, los tracks iniciales, para velocidades, que esos puede que haya que incluirlos TODOS en el cluster que me quede!
	// OJO, SI! hay que arreglar lo de velocidades!

	std::vector<std::list<Ccluster>::iterator> vector_punteros_a_mis_clusters_a_borrar;
	std::vector<bool> cluster_ids_a_borrar;
	for(unsigned int cl=0; cl<count_my_clusters_;cl++){
		cluster_ids_a_borrar.push_back(false);
	}
		//find clusters to erase.
		for (std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();it_myCluster != mycluster_list_.end(); it_myCluster++)
		{
		//std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();
			for (std::list<Ccluster>::iterator it_myCluster2 = mycluster_list_.begin();it_myCluster2 != mycluster_list_.end(); it_myCluster2++)
			{
				unsigned int id_clust1=it_myCluster->get_id_cluster_mht();
				unsigned int id_clust2=it_myCluster2->get_id_cluster_mht();
				if((id_clust1!=id_clust2)&&(cluster_ids_a_borrar[id_clust1]==false)&&(cluster_ids_a_borrar[id_clust2]==false)){
					std::vector<unsigned int> cluster_tracks_mht1 =it_myCluster->get_tracks_in_cluster_mht();
					std::vector<unsigned int> cluster_tracks_mht2 =it_myCluster2->get_tracks_in_cluster_mht();

					bool repeated_cluster=mht_compare_clusters(cluster_tracks_mht1,cluster_tracks_mht2);

					if(repeated_cluster){
						// cluster exist, find the biggest id for remove!
						if(id_clust1>id_clust2){
							if(cluster_ids_a_borrar[id_clust1]==false){
								vector_punteros_a_mis_clusters_a_borrar.push_back(it_myCluster);
								cluster_ids_a_borrar[id_clust1]=true;
							}
						}else{
							if(cluster_ids_a_borrar[id_clust2]==false){
								vector_punteros_a_mis_clusters_a_borrar.push_back(it_myCluster2);
								cluster_ids_a_borrar[id_clust2]=true;
							}
						}
					}

				}

			}
		}

		for (unsigned int i = 0;i < vector_punteros_a_mis_clusters_a_borrar.size(); i++)
		{
			//remove this Cluster
			mycluster_list_.erase(vector_punteros_a_mis_clusters_a_borrar[i]);
		}

		vector_punteros_a_mis_clusters_a_borrar.clear(); //clear the vector of pointer for deleting clusters, for the next iteration.

}

void Cmht::mht_update_tracks_with_winner_Pd_cluster()
{
// This function make the update for the tracks in the clusters(tracks in crosses situations).

//	std::cout<< "\n mht_update_tracks_with_winner_Pd_cluster (change individual tracks taking in to account associations in clusters) " << std::endl;


	for (std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();it_myCluster != mycluster_list_.end(); it_myCluster++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{

		if (it_myCluster->get_bool_cluster_update_mht() == true)
		{
			Eigen::MatrixXi winner_hypothesis_actual_cluster;
			std::vector<unsigned int> detections_cluster;
			std::vector<unsigned int> tracks_cluster;
			std::vector<std::vector<double>> association_distances_cluster;
			std::vector<std::vector<unsigned int>> tracks_associated_cluster;

			winner_hypothesis_actual_cluster =it_myCluster->get_winner_hypothesis_cluster_mht();

			/*std::cout<< "\n it_myCluster->get_id_cluster_mht()="<<it_myCluster->get_id_cluster_mht() << std::endl;
			std::cout<< "\n winner_hypothesis_actual_cluster" << std::endl;
			for(int i_d=0; i_d<winner_hypothesis_actual_cluster.rows();i_d++){
				for(int i_d2=0; i_d2<winner_hypothesis_actual_cluster.cols();i_d2++){
					std::cout<< "winner_hypothesis_actual_cluster=("<<i_d<<","<<i_d2<<")="<<winner_hypothesis_actual_cluster(i_d,i_d2)<<std::endl;
				}
			}*/

			detections_cluster = it_myCluster->get_detections_in_cluster_mht();
			tracks_cluster = it_myCluster->get_tracks_in_cluster_mht();

			association_distances_cluster =it_myCluster->get_association_distances_mht();
			tracks_associated_cluster =it_myCluster->get_tracks_associated_mht();

			for (int iter_subhypothesis = 0;iter_subhypothesis < winner_hypothesis_actual_cluster.cols();iter_subhypothesis++)
			{

				unsigned int actual_detection =detections_cluster[iter_subhypothesis];
				unsigned int actual_track_associated =winner_hypothesis_actual_cluster(0, iter_subhypothesis); // fila 0, columna= sub hypothesis que toque...

				//			std::cout<< "\n actual_detection =" << actual_detection <<" track associated=" <<actual_track_associated<< std::endl;

				OneDetection_ntracks_association_[actual_detection].clear();
				if (actual_track_associated <= count_my_tracks_)
				{
					//std::cout<< "\n detection =" << actual_detection <<" actual_track_associated=" <<actual_track_associated<< std::endl;
					OneDetection_ntracks_association_[actual_detection].push_back(actual_track_associated);
				}
				//std::cout<< "\n detection =" << actual_detection <<" actual_track_associated=" <<actual_track_associated<< std::endl;

				unsigned int pos_track_associated = 0;
				//find pos track
				for (unsigned int pos = 0;pos< association_distances_cluster[actual_detection].size();pos++)
				{
					if (tracks_associated_cluster[actual_detection][pos]== actual_track_associated)
					{
						pos_track_associated = pos;
					}
				}
				double dist_association_det_track =association_distances_cluster[actual_detection][pos_track_associated]; //ver como coger bien esta distancia!!! ojo!!! Important!!!
				vector_distances_[actual_detection].clear();
				vector_distances_[actual_detection].push_back(dist_association_det_track);
//				std::cout<< "\n association distance =" << dist_association_det_track << std::endl;
			}

			// Tracks without detection associated. (if id_track not are in the winner hypothesis, this track is not associated)

			std::vector<bool> track_associated;
			for (unsigned int it_tracks_cluster = 0;it_tracks_cluster < tracks_cluster.size();it_tracks_cluster++)
			{
				track_associated.push_back(false);
			}

			for (unsigned int it_tracks_cluster = 0;it_tracks_cluster < tracks_cluster.size();it_tracks_cluster++)
			{
				for (int it_tracks_hipotesis = 0;it_tracks_hipotesis< winner_hypothesis_actual_cluster.cols();it_tracks_hipotesis++)
				{
					unsigned int winner_act_hypothesis=(unsigned int) winner_hypothesis_actual_cluster(0,it_tracks_hipotesis);
					if (tracks_cluster[it_tracks_cluster]==winner_act_hypothesis)
					{
						track_associated[it_tracks_cluster] = true;
					}

				}
			}


			for (unsigned int it_tracks_cluster = 0;it_tracks_cluster < track_associated.size();it_tracks_cluster++)
			{
				if (track_associated[it_tracks_cluster] == false)
				{
					unsigned int pos = tracks_cluster[it_tracks_cluster];
					track_no_associated_[pos] = true; // track no associated.
					track_no_asociated_in_cross_situation_[pos] = true;
//								std::cout<< "\n track_no_associated_ =" << pos << std::endl;
				}
			}


			// TODO: ojo! si es fa (0) o new track la hypothesis esos casos no los contemplo. (en teoría NUNCA se dará el caso fa=0, pero ojo si da problemas mirar eso...)
			// Case new tracks in hypothesis.

			if (detections_cluster.size() > tracks_cluster.size())
			{ // I have more detections than tracks.
				for (int it_hypothesis = 0;it_hypothesis < winner_hypothesis_actual_cluster.cols();it_hypothesis++)
				{
					unsigned int winner_act_hypothesis=(unsigned int) winner_hypothesis_actual_cluster(0, it_hypothesis);
					if (winner_act_hypothesis> count_my_tracks_)
					{ //this detection is a new track
						unsigned int this_det =detections_cluster[it_hypothesis];
						OneDetection_ntracks_association_[this_det].clear(); // esta deteccion no tiene ningún track asociado!

						unsigned int rows= (unsigned int) winner_hypothesis_actual_cluster.rows();
						if (rows> tracks_cluster.size())
						{
							no_new_track_[this_det] = false;
						}
						else
						{
							no_new_track_[this_det] = true; // además de esto, cuando se elimina un track he de eliminarlo del cluster.
						}
//						std::cout<< "\n detection without track =" << this_det << std::endl;
					}
				}
			}

		}
		else
		{ //cluster update false, el cluster se degrada (la pd se ha degradado antes y el cluster si hace falta ha sifo eliminado)
		  //, caso para que actuen velocidades y tal en separacion en clusters.

		}
	}
}

void Cmht::mht_exit_clusters()
{
// this function complete the ExitClusters, for publish the mht_tracker.

	exit_clusters_mht_.clear(); //clear the exit parameters for the tracks, for filling with the parameters in this iteration.
//	std::cout<< "\n print clusters =" << std::endl;
	for (std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();it_myCluster != mycluster_list_.end(); it_myCluster++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		//if(it_myCluster->get_cluster_confirmed_mht()==true)
		//{
		//Track confirmed. (MIS TRACKS!)
		//guardar track en parámetros para sacar fuera, porque este track ya es persona, esta confirmado.
		Exitcluster cluster_returned;
		//SpointV_cov kalman_state=it_myTrack->get_kalman_update();
		//unsigned int id=it_myTrack->get_idTracker();
		//double prob=it_myTrack->get_probability();
		unsigned int id_cluster = it_myCluster->get_id_cluster_mht();
		std::vector<unsigned int> tracks =it_myCluster->get_tracks_in_cluster_mht(); //TODO: necesito la posicion de los tracks.
		// TODO: rellenar las posiciones de los tracks buscando en la lista de tracks!

		std::vector<SpointV_cov> position_tracks; // position for the tracks before the correction step. (correction Kalman)

		for (unsigned int it_track = 0; it_track < tracks.size(); it_track++)
		{
			unsigned int track_act = tracks[it_track];
			for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin(); it_myTrack != mytracks_list_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			{
				if (track_act == it_myTrack->get_idTracker())
				{
					SpointV_cov pose_track;
					pose_track = it_myTrack->get_kalman_update();
					position_tracks.push_back(pose_track);
//						pose_track.print();
				}
			}

		}

		// find the min x and y of all tracks!	and
		// made the median distance with the positions of all tracks!
		if (position_tracks.size() > 1)
		{
			double pose_centroid_x = 0.0;
			double pose_centroid_y = 0.0;
			double min_pose_x = position_tracks[0].x;
			double min_pose_y = position_tracks[0].y;
			double medium_V_cluster = 0.0;

			std::vector<double> cov;
			for (unsigned int it = 0; it < 4; it++)
			{
				cov.push_back(0.0);
			}

			for (unsigned int pos = 0; pos < position_tracks.size(); pos++)
			{
				pose_centroid_x = pose_centroid_x + position_tracks[pos].x;
				pose_centroid_y = pose_centroid_y + position_tracks[pos].y;
				cov[0] = cov[0] + position_tracks[pos].cov[0] + 5;
				cov[1] = cov[1] + position_tracks[pos].cov[1];
				cov[2] = cov[2] + position_tracks[pos].cov[4];
				cov[3] = cov[3] + position_tracks[pos].cov[5] + 5;

				if (position_tracks[pos].x < min_pose_x)
				{
					min_pose_x = position_tracks[pos].x;
				}

				if (position_tracks[pos].y < min_pose_y)
				{
					min_pose_y = position_tracks[pos].y;
				}
				// calculate medium velocity
				medium_V_cluster = medium_V_cluster + position_tracks[pos].v();
			}

			pose_centroid_x = (pose_centroid_x/ ((double) position_tracks.size())); //+min_pose_x;
			pose_centroid_y = (pose_centroid_y/ ((double) position_tracks.size())); //+min_pose_y;
			medium_V_cluster = (medium_V_cluster/ ((double) position_tracks.size()));

//				std::cout<< "\n min_pose_x =" << min_pose_x << std::endl;
//				std::cout<< "\n min_pose_y =" << min_pose_y << std::endl;
//				std::cout<< "\n (double) position_tracks.size() =" << (double) position_tracks.size() << std::endl;
//				std::cout<< "\n pose_centroid_x =" << pose_centroid_x << std::endl;
//				std::cout<< "\n pose_centroid_y =" << pose_centroid_y << std::endl;
			Spoint centroid_cluster(pose_centroid_x, pose_centroid_y); // centroid for the cluster, calculate the median distance with the positions of all tracks!

			Spoint_cov centroid_cluster_with_cluster_area(pose_centroid_x,pose_centroid_y, 0.0, cov);
//				std::cout<< "\n centroid_cluster_with_cluster_area =" << std::endl;
//				centroid_cluster_with_cluster_area.print();

			double Pd_cluster = it_myCluster->get_Pd_cluster_mht(); //valor inicial y por defecto ==1. TODO: quitar!!! aquí no hacen falta!!!
			std::vector<bool> tracks_separados =it_myCluster->get_tracks_separados_mht(); // true if tracks are separated. TODO: mirar si hace falta, creo que es más en la ineterna del mht que hace falta...
//				double cluster_life=it_myCluster->get_cluster_life_mht();

			// Inicial, see if is a group or a crosse.
			bool equal_velocity_tracks = false;
			bool equal_orientation_tracks = false;

			for (unsigned int pos = 0; pos < position_tracks.size(); pos++)
			{
				for (unsigned int pos2 = pos + 1; pos2 < position_tracks.size();pos2++)
				{
					if ((pos != pos2)&& ((position_tracks[pos].v()< position_tracks[pos2].v() + 0.15)&& (position_tracks[pos].v()> position_tracks[pos2].v() - 0.15)))
					{
						equal_velocity_tracks = true;
					}
					else
					{
						equal_velocity_tracks = false;
					}

					if ((pos != pos2)&& ((position_tracks[pos].orientation()< position_tracks[pos2].orientation() + 0.15)&& (position_tracks[pos].orientation()> position_tracks[pos2].orientation()- 0.15)))
					{
						equal_orientation_tracks = true;
					}
					else
					{
						equal_orientation_tracks = false;
					}

				}
			}
			// Final, see if is a group or a crosse.

			cluster_returned = Exitcluster(id_cluster, tracks, position_tracks,centroid_cluster, centroid_cluster_with_cluster_area,Pd_cluster, medium_V_cluster, equal_velocity_tracks,equal_orientation_tracks);
			//std::cout << "\n !!!!!!!!!!!!!!! id Exitcluster=" << id_cluster<< std::endl;
			exit_clusters_mht_.push_back(cluster_returned);
		}
		//}

	}

}

void Cmht::print_clusters_mht()
{

// This function is for printing the ExitClusters

	for (std::list<Ccluster>::iterator it_myCluster_mht =mycluster_list_.begin(); it_myCluster_mht != mycluster_list_.end();it_myCluster_mht++) //me recorro mis clusters del mht.
	{
		it_myCluster_mht->print_Ccluster();
	}
}

// function Cos_to_go to one Spoint to another, for cost for crosses situations.

double Cmht::cost_to_go(SpointV target1_out, const Spoint& target2,const std::vector<double>* params)
{
	//std::cout<< "\n cost to go!!! "<< std::endl;
	double result = 0.0;
	unsigned int cont = 0;
	Sforce f_goal;
	SpointV target1 = target1_out;
	SpointV before_target1;
	double distance_between_targets1 = 0.0;

	double distance = target1.distance(target2);
	if (distance < 0.5)
	{
		target1.vx = target1.vx / 2;
		target1.vy = target1.vy / 2;
	}
	else
	{
		//target1.vx=target1.vx;
		//target1.vy=target1.vy;
	}
	double dist_before = 10;

	while ((distance > 0.01) && (cont < 40) && (dist_before >= distance)&& (distance_between_targets1 < distance)) //&& (target1.x<target2.x)) //0.3*0.3
	{
		//social_forces_param_to_person_ defined in Cscene_abstract
		f_goal = force_goal(Sdestination(0, target2.x, target2.y), target1,params);
		result += f_goal.module2();
		before_target1 = target1;
		target1 = target1.propagate(dt_, f_goal, target1.v());
		distance_between_targets1 = target1.distance(before_target1);
		dist_before = distance;
		distance = target1.distance(target2);
		cont++;
		//std::cout<< "\n count="<<cont<< std::endl;
		//std::cout<< "\n distance between target1 and target2="<<distance<< std::endl;
		//std::cout<< "\n dt_="<<dt_<< std::endl;
		//std::cout<< "\n target1.v()="<<target1.v()<< std::endl;
		//std::cout<< "\n distance_between_targets1="<<distance_between_targets1<< std::endl;
		//std::cout<< "\n target1 (track):" << std::endl;
		//target1.print();
		//std::cout<< "\n target2 (detection):" << std::endl;
		//target2.print();
		//std::cout<< "\n result="<<result << std::endl;
	}
	return result;
}



/*** Functions for groups!  ***/

void Cmht::distances_for_groups_mht()
{
	//Function that generate groups of tracks, tacking into account the distance between tracks.
	//vect_dist_between_tracks_=[ vec_dist track1 to the other tracks if distance is less than a threshold] [ vec_dist track2 to the other tracks if distance is less than a threshold] ... [ vec_dist trackn to the other tracks if distance is less than a threshold];
	vect_dist_between_tracks_.clear();
	vect_dist_between_tracks_.reserve(mytracks_list_.size());
	std::vector<double> vector_distances_track_to_all_tracks; // Example: vec_dist track1 to the other tracks=[dist track1<-->track2,dist track1<-->track3] dist<threshold_distance_groups_
	vector_distances_track_to_all_tracks.clear();
	vector_distances_track_to_all_tracks.reserve(mytracks_list_.size());

	//groups_onetrack_ntracks_associate_= [vect of idTracks grouped with track1][vect of idTracks grouped with track2] ... [vect of idTracks grouped with trackn]
	groups_onetrack_ntracks_associate_.clear();
	groups_onetrack_ntracks_associate_.reserve(mytracks_list_.size());
	track1_associated_mht_.clear();
	track1_associated_mht_.reserve(mytracks_list_.size());
	std::vector<unsigned int> track1_associated; //vector where I save the if of the tracks gruped with this track by distance.
	track1_associated.clear();
	track1_associated.reserve(mytracks_list_.size());
	std::vector<unsigned int> track2_associated; //vector where I save the if of the tracks gruped with this track by distance.
	track2_associated.clear();
	track2_associated.reserve(mytracks_list_.size());

	for (std::list<Ctrack>::iterator it_myTrack1 = mytracks_list_.begin();it_myTrack1 != mytracks_list_.end(); it_myTrack1++)
	{
		vector_distances_track_to_all_tracks.clear();
		track2_associated.clear();
		track1_associated.clear();

		SpointV_cov track_poseV1;
		track_poseV1 = it_myTrack1->get_kalman_update();

		unsigned int id_track1 = it_myTrack1->get_idTracker();

		for (std::list<Ctrack>::iterator it_myTrack2 = it_myTrack1;it_myTrack2 != mytracks_list_.end(); it_myTrack2++)
		{
			unsigned int id_track2 = it_myTrack2->get_idTracker();
			SpointV_cov track_poseV2;
			track_poseV2 = it_myTrack2->get_kalman_update();

			double distance_track_to_track;
			distance_track_to_track = track_poseV1.cov_dist(track_poseV2);

			if(debug_groups_){
				std::cout << " (out if) id_track1=" << id_track1 << "<--->id_track2="<< id_track2 << std::endl;
				std::cout << "(out if) distance_track_to_track="<< distance_track_to_track << std::endl;
			}

			if ((distance_track_to_track < threshold_distance_groups_)&& (id_track1 != id_track2))
			{
				vector_distances_track_to_all_tracks.push_back(distance_track_to_track);
				track2_associated.push_back(id_track2);
				track1_associated.push_back(id_track1);
				if(debug_groups_){
					std::cout << " id_track1=" << id_track1 << "<--->id_track2="<< id_track2 << std::endl;
					std::cout << "distance_track_to_track="<< distance_track_to_track << std::endl;
				}
			}

		}

		if (!vector_distances_track_to_all_tracks.empty())
		{
			vect_dist_between_tracks_.push_back(vector_distances_track_to_all_tracks);
			groups_onetrack_ntracks_associate_.push_back(track2_associated);
			track1_associated_mht_.push_back(track1_associated);
		}

	}

	if(debug_groups_){
		std::cout << "Show final vectors" << std::endl;
		for (unsigned int i1 = 0; i1 < vect_dist_between_tracks_.size(); i1++)
		{
			for (unsigned int i2 = 0; i2 < vect_dist_between_tracks_[i1].size();i2++)
			{
				std::cout << " id_track1[" << i1 << "][" << i2 << "]="<< track1_associated_mht_[i1][i2] << "<--->id_track2="<< groups_onetrack_ntracks_associate_[i1][i2] << std::endl;
				std::cout << "distance_track_to_track[" << i1 << "][" << i2 << "]="<< vect_dist_between_tracks_[i1][i2] << std::endl;
			}
		}

	}

}

void Cmht::make_groups_mht()
{
	//Function that make the groups of tracks, using the vectors generated in the function distances_for_groups_mht, (that take into account the distance for arrange the tracks).

	tracks_en_grupo_.clear(); // variable for know if the track is in one group. tracks_en_grupo_[track_id]=true if is in one group. false if not.
	tracks_en_grupo_.reserve(mytracks_list_.size());

	for (unsigned int j = 0; j < count_my_tracks_; j++)
	{
		tracks_en_grupo_.push_back(false);
	}

	std::vector<unsigned int> tracks_group_act;
	tracks_group_act.clear();
	tracks_group_act.reserve(mytracks_list_.size());

	std::vector<SpointV_cov> SpointVcovtracks_group_act;
	tracks_group_act.clear();
	tracks_group_act.reserve(mytracks_list_.size());

	for (unsigned int g = 0; g < groups_onetrack_ntracks_associate_.size(); g++)
	{
		tracks_group_act.clear();
		tracks_group_act.reserve(mytracks_list_.size());

		if(tracks_en_grupo_[track1_associated_mht_[g][0]]==false){
				tracks_group_act.push_back(track1_associated_mht_[g][0]);
				tracks_en_grupo_[track1_associated_mht_[g][0]]=true;

			for (unsigned int g_t_ac = 0;g_t_ac < groups_onetrack_ntracks_associate_[g].size(); g_t_ac++)
			{
				if(tracks_en_grupo_[groups_onetrack_ntracks_associate_[g][g_t_ac]]==false){
					tracks_group_act.push_back(groups_onetrack_ntracks_associate_[g][g_t_ac]);
					tracks_en_grupo_[groups_onetrack_ntracks_associate_[g][g_t_ac]]=true;
				}
			}
			for (unsigned int g_act = 0;g_act < groups_onetrack_ntracks_associate_[g].size(); g_act++)
			{
				if(debug_groups_){
					std::cout << "Entro en for g_act !!!" << std::endl;
				}
				unsigned int track2 = groups_onetrack_ntracks_associate_[g][g_act];

				for (unsigned int g3 = 0;g3 < groups_onetrack_ntracks_associate_.size(); g3++)
				{
					if(debug_groups_){
						std::cout << "Entro en for g3 !!!" << std::endl;
						std::cout << "track2="<<track2<< std::endl;
						std::cout << "track1_associated_mht_[g3][0]="<<track1_associated_mht_[g3][0]<< std::endl;
					}

					if (track2 == track1_associated_mht_[g3][0])
					{
						if(debug_groups_){
							std::cout << "Entro en if !!!" << std::endl;
						}
						for (unsigned int g_t_ac2 = 0;g_t_ac2< groups_onetrack_ntracks_associate_[g3].size();g_t_ac2++)
						{
							if(debug_groups_){
								std::cout << "Entro en for g_t_ac2!!!" << std::endl;
							}
							if(tracks_en_grupo_[groups_onetrack_ntracks_associate_[g3][g_t_ac2]]==false){
								tracks_group_act.push_back(groups_onetrack_ntracks_associate_[g3][g_t_ac2]);
								tracks_en_grupo_[groups_onetrack_ntracks_associate_[g3][g_t_ac2]]=true;
							}
						}
					}
				}

			}

			if(debug_groups_){
				for(unsigned int x=0;x<tracks_group_act.size();x++){
					std::cout << " tracks_group_act["<<x<<"]="<<tracks_group_act[x] << std::endl;
				}
			}

			unsigned int id_group_act;
			bool group_no_exist = find_group(tracks_group_act, id_group_act);

			for(unsigned int xt=0;xt<tracks_group_act.size();xt++ ){
				for (std::list<Ctrack>::iterator it_myTrack2 = mytracks_list_.begin();it_myTrack2 != mytracks_list_.end(); it_myTrack2++)
				{
					if(tracks_group_act[xt]==it_myTrack2->get_idTracker()){
						SpointVcovtracks_group_act.push_back(it_myTrack2->get_kalman_update());
					}
				}
			}

			if(!tracks_group_act.empty())
			{
				if (group_no_exist)
				{
					if(debug_groups_){
						std::cout << " Entro en group No exist, create new group! " << std::endl;
					}
					Cgroups group_new(count_my_groups_, tracks_group_act, SpointVcovtracks_group_act);

					count_my_groups_++;
					it_insert_group_ = mygroup_list_.begin();

					if (!mygroup_list_.empty())
					{
						while (it_insert_group_ != mygroup_list_.end())
						{
							it_insert_group_++;
							if (it_insert_group_->get_id_group() > count_my_groups_)
							{
								break;
							}
						}
					}
					group_new.set_update_group(true);
					mygroup_list_.insert(it_insert_group_, group_new);

					if(debug_groups_){
						std::cout << " \n id_in create new cluster" << id_group_act << std::endl;
					}
				}
				else
				{
					if(debug_groups_){
						std::cout << " Entro en group exist " << std::endl;
						std::cout << " \n id_group_act" << id_group_act << std::endl;
					}
					for (std::list<Cgroups>::iterator it_myGroup =mygroup_list_.begin(); it_myGroup != mygroup_list_.end();it_myGroup++)
					{
						if (id_group_act == it_myGroup->get_id_group())
						{
							if(debug_groups_){
								std::cout << " Entro en if update group " << std::endl;
							}
							it_myGroup->set_update_group(true);
							it_myGroup->set_tracks_group(tracks_group_act);
							it_myGroup->set_SpointVcov_tracks_group(SpointVcovtracks_group_act);
						}
					}
				}
			}
			//////////////////////////////
		} // fin if primero dentro del for!!!
	}

	if(debug_groups_){
		std::cout << "\n \n \n !!! Print my groups !!!" << std::endl;
		for (std::list<Cgroups>::iterator it_myGroup = mygroup_list_.begin();it_myGroup != mygroup_list_.end(); it_myGroup++)
		{
			it_myGroup->print_groups();
		}
		mht_print_results(); //for the visualization of results in C++.
	}
}

bool Cmht::find_group(std::vector<unsigned int> tracks_group_act,unsigned int& id_group)
{
	// Function that find one existing group.
	if(debug_groups_){
		std::cout << " \n Entro en FIND_GROUP" << std::endl;
	}
	bool no_group = true;

	std::vector<unsigned int> tracks_group_act_arr;
	tracks_group_act_arr = arrange_tracks(tracks_group_act);

	if(debug_groups_){
		for (unsigned int mi = 0; mi < tracks_group_act_arr.size(); mi++)
		{
			std::cout << " \n tracks_group_act_arr[" << mi << "]:"<< tracks_group_act_arr[mi] << std::endl;
		}
	}

	for (std::list<Cgroups>::iterator it_myGroup = mygroup_list_.begin();it_myGroup != mygroup_list_.end(); it_myGroup++)
	{
		std::vector<unsigned int> tracks_in_group;
		tracks_in_group = it_myGroup->get_tracks_group();
		std::vector<unsigned int> tracks_in_group_arr;
		tracks_in_group_arr = arrange_tracks(tracks_in_group);
		// si hay algun track igual a los de algún grupo, existe el grupo!!!
		for(unsigned int j1=0;j1<tracks_group_act_arr.size();j1++)
		{
			for(unsigned int j2=0;j2<tracks_in_group_arr.size();j2++)
			{
				if(tracks_group_act_arr[j1]==tracks_in_group_arr[j2]){
					// the group exist, if one track is the same.
					no_group = false;
					// update del grupo!!!
					id_group = it_myGroup->get_id_group();
					if(debug_groups_){
						std::cout << " \n id_group" << id_group << std::endl;
					}
				}
			}

		}

	}

	if(debug_groups_){
		std::cout << " \n no_group" << no_group << std::endl;
		std::cout << "\n \n \n !!! Print my groups after find_group!!!" << std::endl;
		for (std::list<Cgroups>::iterator it_myGroup = mygroup_list_.begin();it_myGroup != mygroup_list_.end(); it_myGroup++)
		{
			it_myGroup->print_groups();
		}
	}

	return no_group;
}

void Cmht::restart_groups()
{
	// Function that restard the groups, first in one mht iteration All the groups are not updated!
	if(debug_groups_){
		std::cout << " \n entro en restart groups"  << std::endl;
	}
	for (std::list<Cgroups>::iterator it_myGroup = mygroup_list_.begin();it_myGroup != mygroup_list_.end(); it_myGroup++)
	{
		it_myGroup->set_update_group(false);
	}

}

void Cmht::remove_groups()
{
	// Function that remove groups that not exist now! (ins the actual mht iteration)
	if(debug_groups_){
		std::cout << " \n entro en remove group"  << std::endl;
	}
	std::vector<std::list<Cgroups>::iterator> vector_punteros_a_mis_grupos_for_errase;

	// first find the groups that have to be deleted.
	for (std::list<Cgroups>::iterator it_myGroup = mygroup_list_.begin();it_myGroup != mygroup_list_.end(); it_myGroup++)
	{
		if (it_myGroup->get_update_group()==false)
		{
			if(debug_groups_){
				std::cout << " \n remove group id_group" << it_myGroup->get_id_group()<< std::endl;
			}
			vector_punteros_a_mis_grupos_for_errase.push_back(it_myGroup); //save pointers to my groups for delete.
		}
	}

	// finally, delete these groups.
	for (unsigned int i = 0;i < vector_punteros_a_mis_grupos_for_errase.size(); i++)
	{
		//remove this group
		mygroup_list_.erase(vector_punteros_a_mis_grupos_for_errase[i]);
	}

	vector_punteros_a_mis_grupos_for_errase.clear();

	if(debug_groups_){
		std::cout << "\n !!! Print groups That NOT removed !!!" << std::endl;
		for (std::list<Cgroups>::iterator it_myGroup = mygroup_list_.begin();it_myGroup != mygroup_list_.end(); it_myGroup++)
		{
			it_myGroup->print_groups();
		}
	}
}

void Cmht::publish_only_one_track_in_groups()
{
	// FOR CARGO ANTS ONLY, Find the track with less track_id, this track is the only track that is send out.
	if(debug_groups_){
		std::cout<< " !!!!!!!!!!!!!!!!!!!!!!publish_only_one_track_in_groups!!!! " << std::endl;
	}
	for (std::list<Cgroups>::iterator it_mygroup = mygroup_list_.begin();it_mygroup != mygroup_list_.end(); it_mygroup++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		it_mygroup->generation_group_track_pose();
		//it_mygroup->print_groups();
		if(debug_groups_){
			std::cout<< " id_group= "<<it_mygroup->get_id_group() << std::endl;
		}
		std::vector<unsigned int> tracks_group_mht =it_mygroup->get_tracks_group();
		unsigned int id_track_id_menor = tracks_group_mht[0];
		unsigned int num_tracks = tracks_group_mht.size();

		for (unsigned int j = 0; j < num_tracks; j++)
		{
			if (tracks_group_mht[j] < id_track_id_menor)
			{
				id_track_id_menor = tracks_group_mht[j];
			}
		}
		if(debug_groups_){
			std::cout<< " id_track_id_menor= "<<id_track_id_menor << std::endl;
		}

			for (unsigned int iter = 0; iter < tracks_group_mht.size();iter++)
			{
				if (tracks_group_mht[iter] == id_track_id_menor)
				{
					exit_track_cargo_ANTS_[tracks_group_mht[iter]] = true;
					for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
					{
						if(it_myTrack->get_idTracker()==id_track_id_menor){
							SpointV_cov trackA=it_myTrack->get_kalman_update();
							it_mygroup->set_tracks_with_lessID_group(trackA);
							//trackA.print();
						}
					}
				}
				else
				{
					exit_track_cargo_ANTS_[tracks_group_mht[iter]] = false;
				}

				if(debug_groups_){
					std::cout<< " exit_track_cargo_ANTS_[track="<<tracks_group_mht[iter]<<"]= "<<exit_track_cargo_ANTS_[tracks_group_mht[iter]] << std::endl;
				}
			}
	}
}


void Cmht::central_point_groups()
{

	for (std::list<Cgroups>::iterator it_mygroup = mygroup_list_.begin();it_mygroup != mygroup_list_.end(); it_mygroup++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		it_mygroup->generation_group_track_pose();
		//it_mygroup->print_groups();
	}
}

///////////
void Cmht::groups_out()
{
	//This function update the resulting groups, for the node.
	//std::cout << "\n Entro en groups out: " << std::endl;

	exitGroups_.clear();

	for (std::list<Cgroups>::iterator it_mygroup = mygroup_list_.begin();it_mygroup != mygroup_list_.end(); it_mygroup++) //me recorro mis groups.
	{
		unsigned int id_group=it_mygroup->get_id_group();
		double group_width=it_mygroup->get_group_width(); // Ancho del grupo
		double group_length=it_mygroup->get_group_length(); // largo del grupo
		SpointV_cov central_group=it_mygroup->get_central_group();

		exitGroup Group(id_group, group_width, group_length, central_group);
		exitGroups_.push_back(Group);

		//std::cout << "\n exitGroups_.back().print(); "<< std::endl;
		//exitGroups_.back().print_group();
	}

	//std::cout << "\n salgo de groups out. " << std::endl;

}

void  Cmht::update_groups(){
	restart_groups();
	distances_for_groups_mht();
	make_groups_mht();
	remove_groups();
	central_point_groups();
	publish_only_one_track_in_groups();
	groups_out();
	//publish_only_one_track_in_clusters(); //FUNCTION ONLY FOR CARGO ANTS
}


/*** For cargo ANTS, only ***/

void Cmht::mht_filter_by_velocity()
{
	// FOR CARGO ANTS ONLY, Find the track with less track_id, this track is the only track that is send out.
//TODO: IMPORTANTE! filtrar para cargo ANTS, si el vehiculo esta parado. velocidad de odometria ==0. Entonces, filtrar objetos con velocidad =0; SI se mueve, NO filtrar esos!!!
	//std::cout<< " !!!!FILTER by VELOCITY!!!! " << std::endl;
	double vehicle_vx; //OJO!!! ha de ser -Vxcoche!!!!
	double vehicle_vy; //OJO!!! ha de ser -Vycoche!!!!
	double velocity_margin;

	vehicle_vx=-vehicle_pose_.vx;
	vehicle_vy=-vehicle_pose_.vy;
	velocity_margin=0.1;
	//std::cout<< " vehicle_vx " <<vehicle_vx<< std::endl;
	//std::cout<< " vehicle_vy " <<vehicle_vy<< std::endl;
	//std::cout<< "-vehicle_vx-(30/100)*vehicle_vx= "<<vehicle_vx-1.6<< std::endl;
	//std::cout<< "-vehicle_vx+(20/100)*vehicle_vx= "<<vehicle_vx+0.09<< std::endl;
	//std::cout<< "-vehicle_vy-(30/100)*vehicle_vy= "<<vehicle_vy-1.6<< std::endl;
	//std::cout<< "-vehicle_vy+(20/100)*vehicle_vy= "<<vehicle_vy+0.09<< std::endl;

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		int id = it_myTrack->get_idTracker();
		//std::cout<< "\n ID Kalman correct to propagation track " << id << std::endl;
		SpointV_cov kalman_state = it_myTrack->get_kalman_update();
		//if( (((vehicle_vx-((130/100)*vehicle_vx))<kalman_state.vx)&&((vehicle_vx+((120/100)*vehicle_vx))>kalman_state.vx))||(((vehicle_vy+((120/100)*vehicle_vy))>kalman_state.vy)&&((vehicle_vy-((130/100)*vehicle_vy))<kalman_state.vy))||(kalman_state.vy>0.3)||(kalman_state.vx>0.3)){
		if( (((vehicle_vx-1.6)<kalman_state.vx)&&((vehicle_vx+0.04)>kalman_state.vx))||(((vehicle_vy+0.04)>kalman_state.vy)&&((vehicle_vy-1.6)<kalman_state.vy))){

			if(((kalman_state.vx<0.06)&&(kalman_state.vx>-0.06))||((kalman_state.vy<0.05)&&(kalman_state.vy>-0.06))){

			}else{
				exit_track_cargo_ANTS_[id] = false;
			}

		}
		/*if(kalman_state.vy>0.3){
			exit_track_cargo_ANTS_[id] = false;
		}
		if(kalman_state.vx>0.3){
			exit_track_cargo_ANTS_[id] = false;
		}
		if(kalman_state.vy<-0.3){
			exit_track_cargo_ANTS_[id] = false;
		}
		if(kalman_state.vx<-0.3){
			exit_track_cargo_ANTS_[id] = false;
		}

		if(kalman_state.x<0.3){
					exit_track_cargo_ANTS_[id] = false;
		}
		if(id==6){
							exit_track_cargo_ANTS_[id] = false;
		}
		if(id==12){
									exit_track_cargo_ANTS_[id] = false;
				}*/

		//std::cout<< " track id= "<<id<< std::endl;
		//std::cout<< " track_vx " <<kalman_state.vx<< std::endl;
		//std::cout<< " track_vy "<< kalman_state.vy<< std::endl;

	}
}


/*** Needed functions but not for one class ***/

std::vector<unsigned int> Cmht::arrange_tracks(std::vector<unsigned int> tracks)
{

	std::vector<unsigned int> tracks_out = tracks;
	for (unsigned int mi = 1; mi < tracks_out.size(); mi++)
	{
		for (unsigned int mj = 0; mj < tracks_out.size() - mi; mj++)
		{
			if (tracks_out[mj] > tracks_out[mj + 1])
			{
				unsigned int k;

				k = tracks_out[mj + 1];
				tracks_out[mj + 1] = tracks_out[mj];
				tracks_out[mj] = k;
			}
		}
	}
	return tracks_out;
}


bool Cmht::arrange_tracks_of_one_cluster(std::vector<unsigned int> tracks_mht,std::vector<unsigned int> tracks)
{
	// arrange the tracks in the 2 clusters for make good comparison.
	// return true if the track vectors are the same.

	bool same_tracks_in_clusters = false;
	std::vector<unsigned int> tracks_mht_act = tracks_mht;
	std::vector<unsigned int> tracks_act = tracks;

	// arrange the tracks_mht
	for (unsigned int mi = 1; mi < tracks_mht.size(); mi++)
	{
		for (unsigned int mj = 0; mj < tracks_mht.size() - mi; mj++)
		{
			if (tracks_mht_act[mj] > tracks_mht_act[mj + 1])
			{
				unsigned int k;

				k = tracks_mht_act[mj + 1];
				tracks_mht_act[mj + 1] = tracks_mht_act[mj];
				tracks_mht_act[mj] = k;
			}
		}
	}
	// arrange the tracks
	for (unsigned int mi = 1; mi < tracks.size(); mi++)
	{
		for (unsigned int mj = 0; mj < tracks.size() - mi; mj++)
		{
			if (tracks_act[mj] > tracks_act[mj + 1])
			{
				unsigned int k;

				k = tracks_act[mj + 1];
				tracks_act[mj + 1] = tracks_act[mj];
				tracks_act[mj] = k;
			}
		}
	}

	if(debug_cruces_fallos){
		for (unsigned int mi = 0; mi < tracks_act.size(); mi++)
		{
			std::cout << " \n tracks_act[" << mi << "]:" << tracks_act[mi]<< std::endl;
		}
	}

	if (tracks_mht_act == tracks_act)
	{
		same_tracks_in_clusters = true;
	}

	return same_tracks_in_clusters;
}

// FUNCTIONS ONLY FOR CARGO ANTS
void Cmht::publish_only_one_track_in_clusters()
{

	for (std::list<Ccluster>::iterator it_myCluster = mycluster_list_.begin();it_myCluster != mycluster_list_.end(); it_myCluster++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		if (cargo_ANTS)
		{
			// FOR CARGO ANTS ONLY, Find the track with greatest Pd
			// obtengo las Probabilidades de los tracks del cluster
			std::vector<unsigned int> tracks_cluster_mht =it_myCluster->get_tracks_in_cluster_mht();
			std::vector<double> Pd_tracks_in_cluster;
			for (unsigned int it_t_clu = 0;it_t_clu < tracks_cluster_mht.size(); it_t_clu++)
			{
				unsigned int id_track_clus_act = tracks_cluster_mht[it_t_clu];

				for (std::list<Ctrack>::iterator it_myTrack =mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
				{
					if (id_track_clus_act == it_myTrack->get_idTracker())
					{
						Pd_tracks_in_cluster.push_back(it_myTrack->get_probability());
					}
				}

			}
			// Busco la probabilidad maxima y por tanto el track que tiene la probabilidad maxima.
			//double max_probabilidad=Pd_tracks_in_cluster[0];
			unsigned int id_track_id_menor = tracks_cluster_mht[0];
			unsigned int num_Probabilidades = Pd_tracks_in_cluster.size();
			for (unsigned int j = 0; j < num_Probabilidades; j++)
			{
				if (tracks_cluster_mht[j] < id_track_id_menor)
				{
					id_track_id_menor = tracks_cluster_mht[j];
				}
			}
			// Todos los tracks del cluster excepto el de la probabilidad maxima no los saco fuera.
			// solo publico el track con Pd maxima de los del cluster.

			for (unsigned int iter = 0; iter < tracks_cluster_mht.size();iter++)
			{
				if (tracks_cluster_mht[iter] == id_track_id_menor)
				{
					exit_track_cargo_ANTS_[tracks_cluster_mht[iter]] = true;
				}
				else
				{
					exit_track_cargo_ANTS_[tracks_cluster_mht[iter]] = false;
				}
			}

		}
	}
}



/* GENERATE data base people companion */


void Cmht::eliminate_new_tracks_bad_associated()
{

	//std::cout<< " IN eliminate_new_tracks_bad_associated eliminate mytracks_list_.size()="<<mytracks_list_.size() <<  std::endl;

//This make the detection<-->track association by distances.
	//std::cout<< " 1 eliminate " <<  std::endl;
//paso1: Distance calculation!!! and made the "clusters" according to the vectors track_no_asociated_ and vector_distances_
	//for then apply the functions of track.cpp that require each cluster.
//	std::cout<< " \n \n Function --> association_by_distances:" <<  std::endl;

	double distance_laser_track = 0.0;
	int iteration_distance = 0;

	std::vector<unsigned int> associationsTracks_onetrack; //vector where I save the tracks associated to a detection.
	associationsTracks_onetrack.reserve(mytracks_list_.size());

	std::vector<double> inc_time_act_track;
	inc_time_act_track.reserve(mytracks_list_.size());
	//std::vector<int> associationsDetetections_oneTrack;
	//associationsDetetections_oneTrack.reserve(obs.size());
	Onetrack_ntracks_association_.clear();

	std::vector<double> vector_distances_det; //save the distance vector for this detection.
	vector_distances_with_tracks_.clear();
	vector_distances_with_tracks_.reserve(mytracks_list_.size());


	inc_time_with_tracks_.clear();
	inc_time_with_tracks_.reserve(mytracks_list_.size());

	//std::cout << "(1)"<< std::endl;
	//std::cout<< " 2 eliminate " <<  std::endl;
	std::vector<bool> ide_ya_en_grupo_;
	ide_ya_en_grupo_.clear();
	ide_ya_en_grupo_.reserve(count_my_tracks_);
	for(unsigned int t=0;t<count_my_tracks_;t++){
		ide_ya_en_grupo_.push_back(false);
	}
	//std::cout<< " 3 eliminate " <<  std::endl;
	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++)
	{
//		std::cout<< "\n print detection" << std::endl;
//		obs[i_detection].print();
		associationsTracks_onetrack.clear();
		inc_time_act_track.clear();
		vector_distances_det.clear();
		iteration_distance = 0;
		int id_track_actual1;
		id_track_actual1 = it_myTrack->get_idTracker();
		Cperson_abstract* person_obj1;
		//std::cout << "(2)"<< std::endl;
		//std::cout<< " 4 eliminate " <<  std::endl;
		bool first=true;
		bool first_iterator=true;

		bool id_list1_en_grupo=false;
		//std::cout<< " 5 eliminate " <<  std::endl;
		/*if(fuse_tibi_and_teo_tracks_){
			dt_=5.0;
		}*/


		if (scene_tracks_people_prediction_.find_person(id_track_actual1,&person_obj1))
		{
			//std::cout<< " 6 eliminate " <<  std::endl;
			SpointV_cov track_poseV1 = person_obj1->pointV_propagation(dt_); //calculation of the increment!. Get the track pose, by the function and the pointer to the tracks list.
			//std::cout<<"dt_" << dt_  << std::endl;
			//std::cout<< "\n track_poseV_actual=" << std::endl;
			//track_poseV.print();
			std::vector<double> track_cov1;
			track_cov1.reserve(4);
			track_cov1.resize(4, 0.0);
			track_cov1[0] = track_poseV1.cov[0] + aug_cov_prediction;
			track_cov1[1] = track_poseV1.cov[1];
			track_cov1[2] = track_poseV1.cov[4];
			track_cov1[3] = track_poseV1.cov[5] + aug_cov_prediction;

			Spoint_cov track_pose1(track_poseV1.x, track_poseV1.y,track_poseV1.time_stamp, track_cov1);
			//std::cout << "(3)"<< std::endl;

			//std::cout << "(4)"<< std::endl;
			//std::cout<< " 7 eliminate " <<  std::endl;
			for (std::list<Ctrack>::iterator it_myTrack2 =mytracks_list_.begin();it_myTrack2 != mytracks_list_.end(); it_myTrack2++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			{
				/*if(first_iterator){
					it_myTrack2=it_myTrack++;
					first_iterator=false;
				}*/
				//std::cout<< " 8 eliminate " <<  std::endl;
				int id_track_actual;
				id_track_actual = it_myTrack2->get_idTracker(); //get my id, for search this track propagated that Gonzalo passes me with the same id.
				//save the distances taking into account the order of each track id's from high to low value.
				Cperson_abstract* person_obj;
				//std::cout << "(5)"<< std::endl;
				//track_pose.print();
				if (debug_cruces_fallos)
				{
					std::cout<< "\n ASOCIATION, tracks que hay en Ctrack id_track_actual="<< id_track_actual << std::endl;
				}
				//std::cout<< " 9 eliminate " <<  std::endl;

				if (scene_tracks_people_prediction_.find_person(id_track_actual,&person_obj))
				{
					//std::cout<< " 10 eliminate " <<  std::endl;
					//person_obj->get_current_pointV().print();
					SpointV_cov track_poseV = person_obj->pointV_propagation(dt_); //calculation of the increment!. Get the track pose, by the function and the pointer to the tracks list.
					//std::cout<<"dt_" << dt_  << std::endl;
					//std::cout<< "\n track_poseV_actual=" << std::endl;
					//track_poseV.print();
					std::vector<double> track_cov;
					track_cov.reserve(4);
					track_cov.resize(4, 0.0);
					track_cov[0] = track_poseV.cov[0] + aug_cov_prediction;
					track_cov[1] = track_poseV.cov[1];
					track_cov[2] = track_poseV.cov[4];
					track_cov[3] = track_poseV.cov[5] + aug_cov_prediction;
					Spoint_cov track_pose(track_poseV.x, track_poseV.y,track_poseV.time_stamp, track_cov);

	//				std::cout<< "\n (distance) track_pose_actual="<<id_track_actual << std::endl;
	//				track_poseV.print();
					//std::cout<< " 11 eliminate " <<  std::endl;

					//				std::cout<< "\n (function distances) detection_actual=" << std::endl;
	//				detection_actual.print();
					//Spoint detection_actualV( obs[i_detection].x , obs[i_detection].y,obs[i_detection].time_stamp );

	//				double distance_laser_track_sin_cov = track_pose.distance(detection_actual); //TODO:  future (change to cov_dist)
	//				distance_laser_track = track_pose.cov_dist(detection_actual);
					//double distv;
					//double distxv;

					//distance_laser_track = track_pose.distance(detection_actual);//,&distv,&distxv);

					if (cargo_ANTS)
					{
						distance_laser_track = track_pose.cov_dist(track_pose1);
						//distance_laser_track = track_pose.distance(detection_actual);
					}
					else
					{
						distance_laser_track = track_pose.cov_dist(track_pose1); //,&distv,&distxv);
						//distance_laser_track =track_pose.cov_dist(detection_actual); //TODO: usar distancia con covarianza como en cargo_ANTS, pero hace falta reducir más la distancia de asociación (o las covarianzas=creo que es solo la distancia de asociacion, pero mirar).
					}
					//std::cout<< " 12 eliminate " <<  std::endl;
								   //std::cout<<" \n DISTANCE: con cov distance_laser_track:" <<  distance_laser_track << std::endl;
	//			    std::cout<<" \n DISTANCE: sin cov distance_laser_track:" <<  distance_laser_track_sin_cov << std::endl;
				   // std::cout<<"   del track: "<<id_track_actual<< std::endl;
				   // std::cout<<"   detetección número: "<< i_detection << std::endl;

					if (((distance_laser_track < 4) && (distance_laser_track > 0))&&(ide_ya_en_grupo_[id_track_actual1]==false))
					{ //create the cluster with this corresponding detection and all the tracks that contains. (creo ya el cluster correspondiente con la detección y los tracks que la contengan.)
						iteration_distance = iteration_distance + 1;
						//std::cout<< " 13 eliminate " <<  std::endl;
						//track_no_associated_[id_track_actual] = false; //(id_track_actual-1) porque mis id_track empiezan en el 1. track_no_asociado=0-->el track está asociado.
	//				    std::cout<< " vector_bool_tracks_no_associados size:" <<  track_no_associated_.size()<< std::endl;
	//
						if(first){
							first=false;
							associationsTracks_onetrack.push_back(id_track_actual1);
							inc_time_act_track.push_back(it_myTrack->get_Increment_time_old_track_());
							id_list1_en_grupo=true;
							//ide_ya_en_grupo_[id_track_actual1]=true;
						}
						//std::cout<< " 14 eliminate " <<  std::endl;
						inc_time_act_track.push_back(it_myTrack2->get_Increment_time_old_track_());
						//std::cout << "(6)"<< std::endl;
						associationsTracks_onetrack.push_back(id_track_actual); //vector of tracks associated with this actual detection.
						ide_ya_en_grupo_[id_track_actual]=true;
						//std::cout << "(7)"<< std::endl;
						vector_distances_det.push_back(distance_laser_track); //I save only the distance of the track that is a shorter distance for this detection.
						//std::cout<< " 15 eliminate " <<  std::endl;
	//std::cout<< "DISTANCE" << std::endl;
						//if (debug_cruces_fallos)
						if(debug_companion_akp_)
						{
							std::cout << "id_track_actual1:"<< id_track_actual1 << std::endl;
							std::cout << "person_obj1->get_Increment_time_old_track_():"<< it_myTrack->get_Increment_time_old_track_() << std::endl;
							std::cout << "id_track_actual2:" << id_track_actual<< std::endl;
							std::cout << "person_obj2->get_Increment_time_old_track_():"<< it_myTrack2->get_Increment_time_old_track_() << std::endl;

						}
	//							    std::cout<< " SIZE tracks_associados_a deteccion actual:" <<  associationsTracks_oneDetection.size()<< std::endl;
	//							    std::cout<< " tracks_associados_a deteccion actual:" <<  id_track_actual << std::endl;
	//std::cout<< "distancias de tracks_associados_a deteccion actual:" <<  vector_distances_det[iteration_distance]<< std::endl;
						//	detection_belongs_track=true;
					}
				}
			}
		}
		//std::cout<< " 16 eliminate " <<  std::endl;
		//std::cout << "(8)"<< std::endl;
		vector_distances_with_tracks_.push_back(vector_distances_det); //TODO: ojo! las detecciones tambien van de la 1 a la 0 del vector.! (esto es menos problema que lo de los tracks creo que ya iría bien de normal)
		Onetrack_ntracks_association_.push_back(associationsTracks_onetrack);
		inc_time_with_tracks_.push_back(inc_time_act_track);
		//std::cout << "(9)"<< std::endl;
		//std::cout<< " 17 eliminate " <<  std::endl;
		if(id_list1_en_grupo){
			ide_ya_en_grupo_[id_track_actual1]=true;
		}
		//std::cout<< " 18 eliminate " <<  std::endl;

	}

	//std::cout << "(10)"<< std::endl;
	//if (debug_cruces_fallos)
	bool remove=false;
	std::vector<unsigned int> index_remove_;
	for(unsigned int u = 0; u < Onetrack_ntracks_association_.size();u++){
		if(Onetrack_ntracks_association_[u].size()>2){
			remove=true;
			index_remove_.push_back(u);
		}
	}
	///std::cout<< " 19 eliminate " <<  std::endl;

	if((!Onetrack_ntracks_association_.empty())&&(remove)&&(debug_companion_akp_))
	{
		std::cout << "OneDetection_ntracks_association_ FINAL ASSOCIATION; Onetrack_ntracks_association_.empty()="<<Onetrack_ntracks_association_.empty()<< std::endl;
		std::cout << " Onetrack_ntracks_association_[u].size()="<< Onetrack_ntracks_association_[0].size()<< std::endl;

		for (unsigned int u = 0; u < Onetrack_ntracks_association_.size();u++)
		{
			for (unsigned int l = 0;l < Onetrack_ntracks_association_[u].size(); l++)
			{
				std::cout << "Onetrack_ntracks_association_[" << u << "]["<< l << "]." << Onetrack_ntracks_association_[u][l]<< std::endl;
				std::cout << "distance[" << u << "]["<< l << "]." << vector_distances_with_tracks_[u][l]<< std::endl;

			}
		}
	}
	//std::cout<< " 20 eliminate " <<  std::endl;
	//std::cout << "(11)"<< std::endl;
	// find the track with min time.
	if((!Onetrack_ntracks_association_.empty())&&(remove)&& (!inc_time_with_tracks_[0].empty()))
	{
		//std::cout<< " 21 eliminate " <<  std::endl;
		std::vector<double> min_inc_time_out;
		std::vector<unsigned int> id_track_min_inc_out;
		//std::cout<< " 22 eliminate " <<  std::endl;
		for(unsigned int e = 0;e < index_remove_.size(); e++){

			//std::cout<< " 23 eliminate " <<  std::endl;
			double min_inc_time=inc_time_with_tracks_[index_remove_[e]][0];
			unsigned int id_track_min_inc=Onetrack_ntracks_association_[index_remove_[e]][0];
			//std::cout << "(12)"<< std::endl;
			for(unsigned int l = 0;l < Onetrack_ntracks_association_[index_remove_[e]].size(); l++){

				if(inc_time_with_tracks_[0][l]<min_inc_time){
					//std::cout << "(13)"<< std::endl;
					min_inc_time=inc_time_with_tracks_[index_remove_[e]][l];
					id_track_min_inc=Onetrack_ntracks_association_[index_remove_[e]][l];
				}
			}
			//std::cout<< " 24 eliminate " <<  std::endl;std::cout<< " 1 eliminate " <<  std::endl;
			min_inc_time_out.push_back(min_inc_time);
			id_track_min_inc_out.push_back(id_track_min_inc);

		}
		//std::cout<< " 25 eliminate " <<  std::endl;
		//std::cout << "(14)"<< std::endl;
		// eliminate this track.
		std::list<Ctrack>::iterator iter_min_inc;
		bool erase_one_track=false;
		for(unsigned int l = 0;l < id_track_min_inc_out.size(); l++){
			//std::cout<< " 26 eliminate " <<  std::endl;
			for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++)
			{
				//std::cout<< " 27 eliminate " <<  std::endl;
				//std::cout << "(15)"<< std::endl;
				if(id_track_min_inc_out[l]==it_myTrack->get_idTracker()){
					//std::cout<< " 28 eliminate " <<  std::endl;
					// remove this track in track list.
					//std::cout << "(16)"<< std::endl;
					iter_min_inc=it_myTrack;
					//std::cout << "ERASE id=>it_myTrack"<< it_myTrack->get_idTracker()<< std::endl;
					erase_one_track=true;
					//std::cout<< " 29 eliminate " <<  std::endl;
				}
			}
			//std::cout<< " 32 eliminate mytracks_list_.size()="<<mytracks_list_.size() <<  std::endl;
			//std::cout<< " 30 eliminate " <<  std::endl;
			if(erase_one_track){
				bool erase_now=false;
				for (std::list<Ctrack>::iterator it_myTrack7 = mytracks_list_.begin();it_myTrack7 != mytracks_list_.end(); it_myTrack7++)
							{
								//std::cout << "(find exist id to erase==>"<< it_myTrack7->get_idTracker()<< std::endl;
								if(it_myTrack7->get_idTracker()==iter_min_inc->get_idTracker() ){
									erase_now=true;
								}
							}
					//std::cout<< " 30 iter_min_inc->id= "<<iter_min_inc->get_idTracker() <<  std::endl;
				if(erase_now){

					// TODO: INI 2 nuevo errase tracks, errase the prediction list ok, when erase tracks.
					scene_tracks_people_prediction_.remove_person(iter_min_inc->get_idTracker());
					mht_remove_clusters_with_track_removed(iter_min_inc->get_idTracker());
					// TODO: FIN 2 nuevo errase tracks, errase the prediction list ok, when erase tracks.

					mytracks_list_.erase(iter_min_inc); //vector_punteros_a_mis_tracks[i]==it_myTrack -> pointer to my track for delete.
				}


			}
			//std::cout<< " 31 eliminate " <<  std::endl;
		}

	}
	//std::cout<< " out 32 eliminate mytracks_list_.size()="<<mytracks_list_.size() <<  std::endl;
	//std::cout << "(17)"<< std::endl;
	/*std::cout<< "\n vector_distances_.size" <<  vector_distances_.size()<< std::endl;
	std::cout<< "\n empty " << vector_distances_.empty() << std::endl;
	if(vector_distances_.empty()) //entra si el vector no está vacio
	 {
	 for(unsigned int i=0; i<vector_distances_.size();i++){
	 std::cout<< " \n de detección:" <<  i << std::endl;
	 std::cout<< " \n DISTANCE: distance_laser_track:" << (double) vector_distances_[i][0] << std::endl; //NO le gusta esto!!!
	 }
	 }*/
}
