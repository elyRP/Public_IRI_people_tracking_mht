/*
 * Odometry.cpp
 *
 *  Created on: 12/02/2015
 *      Author: Ely Repiso
 */
// Copyright (C) from 2013-until now Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Ely Repiso
// All rights reserved.
/*
*
*      Created on: 2013 by Ely Repiso and published first as her TFC on 13/12/2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration) 
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

//OJO!!! iba bien con cargo_ants == false!!! Tener en cuenta! que va bien con lo de tibi y dabo, por si con cargo_ants=true va mal, por si no este arreglado, que creo que si.
#include "odometry.h"

Codometry::Codometry():
debug_odometry_(false),
cargo_ants(false),
vx_(0.0),vy_(0.0),wz_(0.0),time_stamp_(0.0),R_(0.0),thetaZ_(0.0),dt_(0.0)
{
}

Codometry::~Codometry()
{
}


void Codometry::update_odometryGood(double dx, double dy, double dtheta, double dt, double wz, double linear_vx, double linear_vy){

	//std::cout<< " \n update_odometryGood" << std::endl;
	//dt_=time_odom_act_-time_odom_ant_;
	dx_=dx_+dx; // [m]
	dy_=dy_+dy; // [m]

	Wz_=Wz_+wz;

	linear_vx_=linear_vx;
	linear_vy_=linear_vy;

	dt_=dt_+dt;

	if(cargo_ants){
	/*	dtheta_=dtheta_+dtheta; // [rad]
		R_=R_+sqrt((vx_*vx_)+(vy_*vy_));
		thetaZ_=thetaZ_+wz_;

		if(debug_odometry_){
			std::cout<< " \n tracker_mht=> dt=" <<dt<< std::endl;
			std::cout<< " \n tracker_mht=> dt_=" <<dt_<< std::endl;
			std::cout<< " \n tracker_mht=> dx_=" <<dx_<< std::endl;
			std::cout<< " \n tracker_mht=> dy_=" <<dy_<< std::endl;
			std::cout<< " \n tracker_mht=> dtheta_=" <<dtheta_<< std::endl;
			std::cout<< " \n tracker_mht=> thetaZ_=" <<thetaZ_<< std::endl;
			std::cout<< " \n tracker_mht=> R_=" <<R_<< std::endl;
		}*/
		dtheta_=dtheta_+dtheta; // [rad]
		R_=R_+sqrt((dx_*dx_)+(dy_*dy_)); // COMO TENDRIA QUE SER ESTANDO BIEN LA ODOMETRIA, PROBLEMA DEL BAG_raro_de_Cargo_ANTS.
		thetaZ_=thetaZ_+wz*dt; // => opcion que parece ir mejor!!!
		//thetaZ_=dtheta_;//dtheta_*dt_;

		if(debug_odometry_){
			std::cout<< " \n tracker_mht=> dt=" <<dt<< std::endl;
			std::cout<< " \n tracker_mht=> dt_=" <<dt_<< std::endl;
			std::cout<< " \n tracker_mht=> dx_=" <<dx_<< std::endl;
			std::cout<< " \n tracker_mht=> dy_=" <<dy_<< std::endl;
			std::cout<< " \n tracker_mht=> dtheta_=" <<dtheta_<< std::endl;
			std::cout<< " \n tracker_mht=> thetaZ_=" <<thetaZ_<< std::endl;
			std::cout<< " \n tracker_mht=> R_=" <<R_<< std::endl;
			double R_con_v=R_con_v+sqrt((linear_vx_*linear_vx_)+(linear_vy_*linear_vy_))*dt;
			double theta_con_v=theta_con_v+wz*dt;
			std::cout<< " \n R_con_v=" <<R_con_v<< std::endl;
			std::cout<< " \n theta_con_v=" <<theta_con_v<< std::endl;
		}

	}else{
		dtheta_=dtheta_+dtheta; // [rad]
		R_=R_+sqrt((dx_*dx_)+(dy_*dy_)); // COMO TENDRIA QUE SER ESTANDO BIEN LA ODOMETRIA, PROBLEMA DEL BAG_raro_de_Cargo_ANTS.
		thetaZ_=thetaZ_+wz*dt; // => opcion que parece ir mejor!!!
		//thetaZ_=dtheta_;//dtheta_*dt_;

		if(debug_odometry_){
			std::cout<< " \n tracker_mht=> dt=" <<dt<< std::endl;
			std::cout<< " \n tracker_mht=> dt_=" <<dt_<< std::endl;
			std::cout<< " \n tracker_mht=> dx_=" <<dx_<< std::endl;
			std::cout<< " \n tracker_mht=> dy_=" <<dy_<< std::endl;
			std::cout<< " \n tracker_mht=> dtheta_=" <<dtheta_<< std::endl;
			std::cout<< " \n tracker_mht=> thetaZ_=" <<thetaZ_<< std::endl;
			std::cout<< " \n tracker_mht=> R_=" <<R_<< std::endl;
			double R_con_v=R_con_v+sqrt((linear_vx_*linear_vx_)+(linear_vy_*linear_vy_))*dt;
			double theta_con_v=theta_con_v+wz*dt;
			std::cout<< " \n R_con_v=" <<R_con_v<< std::endl;
			std::cout<< " \n theta_con_v=" <<theta_con_v<< std::endl;
		}
	}

	R_ant_=R_;
	thetaZ_ant_=thetaZ_;
	
	
	////
	
	Eigen::Matrix3d global_odom_tf_act;
	global_odom_tf_act=odom_eigen_tf_act_.inverse()*odom_eigen_tf_ant_;
	vect_odom_transforms.push_back(global_odom_tf_act);
	
	//std::cout<< " \n global_odom_tf_act= \n" <<global_odom_tf_act<< std::endl;
	global_tf_odom_=odom_eigen_tf_act_.inverse()*GLOBALodom_eigen_tf_ant_;
	//global_tf_odom_=global_odom_tf_act*global_tf_odom_;
	//std::cout<< " \n global_tf_odom_= \n" <<global_tf_odom_<< std::endl;

}






void Codometry::update_odometry(double vx, double vy, double wz, double dt){
	//dt_=time_odom_act_-time_odom_ant_;
	vx_=vx;
	vy_=vy;
	wz_=wz;
	dt_=dt_+dt;


	if(cargo_ants){
		R_=R_+sqrt((vx_*vx_)+(vy_*vy_));
		thetaZ_=thetaZ_+wz_;
	}else{
		R_=R_+dt*sqrt((vx_*vx_)+(vy_*vy_));//R_+dt*sqrt((vx_*vx_)+(vy_*vy_)); // COMO TENDRIA QUE SER ESTANDO BIEN LA ODOMETRIA, PROBLEMA DEL BAG_raro_de_Cargo_ANTS.
		thetaZ_=thetaZ_+wz_*dt;//thetaZ_+dt*wz_;
		//R_=R_+sqrt((vx_*vx_)+(vy_*vy_)); // COMO TENDRIA QUE SER ESTANDO BIEN LA ODOMETRIA, PROBLEMA DEL BAG_raro_de_Cargo_ANTS.
		//thetaZ_=thetaZ_+wz_;
		//linear_vx_=linear_vx_+vx*dt;
		//linear_vy_=linear_vy_+vy*dt;
		linear_vx_=linear_vx_+vx*dt;
		linear_vy_=linear_vy_+vy*dt;
		if(debug_odometry_){
		std::cout<< " \n tracker_mht=> dt=" <<dt<< std::endl;
		std::cout<< " \n tracker_mht=> dt_=" <<dt_<< std::endl;
		std::cout<< " \n tracker_mht=> wz_=" <<wz_<< std::endl;
		std::cout<< " \n tracker_mht=> vx_=" <<vx_<< std::endl;
		std::cout<< " \n tracker_mht=> vy_=" <<vy_<< std::endl;
		std::cout<< " \n tracker_mht=> thetaZ_=" <<thetaZ_<< std::endl;
		std::cout<< " \n tracker_mht=> R_=" <<R_<< std::endl;
		}
	}
}


void Codometry::init_odometryGood(double ini_x, double ini_y, double ini_theta, double time_act){
	before_x_=ini_x; // [m]
	before_y_=ini_y; // [m]
	before_theta_=ini_theta; // [rad]
	time_stamp_=time_act;
	dt_=0.0;
	R_=0.0;
	thetaZ_=0.0;
	linear_vx_=0.0;
	linear_vy_=0.0;
	Wz_=0.0;
	vect_odom_transforms.clear();
	GLOBALodom_eigen_tf_ant_=odom_eigen_tf_ant_;
}


void Codometry::init_odometry(double vx, double vy, double wz, double time_act){
	vx_=vx;
	vy_=vy;
	wz_=wz;
	linear_vx_=vx;
	linear_vy_=vy;
	dt_=0.0;
	R_=0.0;
	thetaZ_=0.0;
	time_stamp_=time_act;
	vect_odom_transforms.clear();
}

void Codometry::reset_odometry(){
	if(debug_odometry_){
	std::cout<< " RESET ODOMETRY \n" << std::endl;
	}
	dx_=0.0;
	dy_=0.0;
	dtheta_=0.0;
	dt_=0.0;
	R_=0.0;
	thetaZ_=0.0;
	linear_vx_=0;
	linear_vy_=0;
	vect_odom_transforms.clear();
	GLOBALodom_eigen_tf_ant_=odom_eigen_tf_ant_;
}

void Codometry::print_odometry(){
	std::cout<< " Actual ODOMETRY \n" << std::endl;
	std::cout<< " vx_" <<  vx_ << std::endl;
	std::cout<< " vy_:" << vy_ << std::endl;
	std::cout<< " wz_:" << wz_ << std::endl;
	std::cout<< " dt_:" << dt_ << std::endl;
	std::cout<< " R_:" << R_ << std::endl;
	std::cout<< " thetaZ_:" << thetaZ_ << std::endl;
	std::cout<< " time_stamp_:" << time_stamp_<< std::endl;
}

/*
 * void Cmht::mht_odom_update(Codometry & odome)
{

	if(debug_odometry_){
		std::cout << " ENTRO EN odom_update (mht)" << std::endl;
	}
	double R; //Traslación del frame
	double thetaZ; // rotación del frame que te da la nueva odometria.
	R = odome.get_translation();

	if(debug_odometry_){
		std::cout << " R=" << R << std::endl;
	}

	thetaZ = odome.get_rotation();

	if(debug_odometry_){
		std::cout << " thetaZ=" << thetaZ << std::endl;
	}
	Eigen::MatrixXd homT1; // homogeneous transfor para pasar los tracks al nuevo frame de la odometria.
	Eigen::MatrixXd homT2;
	Eigen::MatrixXd J; // necesaria para introducir el aumento de la covarianza por culpa del error en R=traslacion y en thetaZ=rotación.
	Eigen::MatrixXd H_cov;
	Eigen::MatrixXd P;
	homT1.resize(3, 3);
	homT2.resize(3, 3);
	J.resize(4, 2);
	H_cov.resize(4, 4);
	P.resize(2, 2);

	homT1.row(0) << cos(thetaZ), -sin(thetaZ), R * cos(thetaZ / 2);
	homT1.row(1) << sin(thetaZ), cos(thetaZ), R * sin(thetaZ / 2);
	homT1.row(2) << 0, 0, 1;
	if (debug_odometry_)
	{
		std::cout << "homT1 = [ " << homT1(0, 0) << " , " << homT1(0, 1)
				<< " , " << homT1(0, 2) << std::endl << "               "
				<< homT1(1, 0) << " , " << homT1(1, 1) << " , " << homT1(1, 2)
				<< std::endl << "               " << homT1(2, 0) << " , "
				<< homT1(2, 1) << " , " << homT1(2, 2) << std::endl;
	}

	homT2 = homT1.inverse();

	if (debug_odometry_)
	{
		std::cout << "homT2 = [ " << homT2(0, 0) << " , " << homT2(0, 1)
				<< " , " << homT2(0, 2) << std::endl << "               "
				<< homT2(1, 0) << " , " << homT2(1, 1) << " , " << homT2(1, 2)
				<< std::endl << "               " << homT2(2, 0) << " , "
				<< homT2(2, 1) << " , " << homT2(2, 2) << std::endl;
	}

	H_cov.row(0) << cos(thetaZ), sin(thetaZ), 0, 0;
	H_cov.row(1) << -sin(thetaZ), cos(thetaZ), 0, 0;
	H_cov.row(2) << 0, 0, cos(thetaZ), sin(thetaZ);
	H_cov.row(3) << 0, 0, -sin(thetaZ), cos(thetaZ);

	P.row(0) << 10 * R / 100, 0; //[G_R^2       0    ]
	P.row(1) << 0, 10 * thetaZ / 100; //[  0    G_thetaZ^2]

	for (std::list<Ctrack>::iterator it_myTrack = mytracks_list_.begin();it_myTrack != mytracks_list_.end(); it_myTrack++) //me recorro mis tracks.
	{
		SpointV_cov track_ant;
		SpointV_cov track_act;
		Eigen::MatrixXd pos_ant;
		Eigen::MatrixXd pos_act;
		Eigen::MatrixXd vel_ant;
		Eigen::MatrixXd vel_act;
		Eigen::MatrixXd cov_ant;
		Eigen::MatrixXd cov_act;
		pos_ant.resize(3, 1);
		pos_act.resize(3, 1);
		vel_ant.resize(3, 1);
		vel_act.resize(3, 1);
		cov_ant.resize(4, 4);
		cov_act.resize(4, 4);

		track_ant = it_myTrack->get_kalman_update();

		if (debug_odometry_)
		{
			std::cout << " TRACK ID:" << it_myTrack->get_idTracker()<< std::endl;
			std::cout << " track_ant" << std::endl;
			track_ant.print();
		}

		double x = track_ant.x;
		double y = track_ant.y;

		if (debug_odometry_)
		{
			std::cout << " track_ant.x" << x << std::endl;
			std::cout << " track_ant.y" << y << std::endl;
		}
		pos_ant(0, 0) = x;
		pos_ant(1, 0) = y;
		pos_ant(2, 0) = 1;

		if (debug_odometry_)
		{
			std::cout << " pos_ant.x" << pos_ant(0, 0) << std::endl;
			std::cout << " pos_ant.y" << pos_ant(1, 0) << std::endl;
			std::cout << " pos_ant.1" << pos_ant(2, 0) << std::endl;
		}

		pos_act = homT2 * pos_ant;

		if (debug_odometry_)
		{
			std::cout << " pos_act.x" << pos_act(0, 0) << std::endl;
			std::cout << " pos_act.y" << pos_act(1, 0) << std::endl;
			std::cout << " pos_act.1" << pos_act(2, 0) << std::endl;
		}

		vel_ant.row(0) << track_ant.vx;
		vel_ant.row(1) << track_ant.vy;
		vel_ant.row(2) << 0;

		if (debug_odometry_)
		{
			std::cout << " vel_ant.x" << vel_ant(0, 0) << std::endl;
			std::cout << " vel_ant.y" << vel_ant(1, 0) << std::endl;
			std::cout << " vel_ant.1" << vel_ant(2, 0) << std::endl;
		}

		vel_act = homT2 * vel_ant;

		if (debug_odometry_)
		{
			std::cout << " vel_act.x" << vel_act(0, 0) << std::endl;
			std::cout << " vel_act.y" << vel_act(1, 0) << std::endl;
			std::cout << " vel_act.1" << vel_act(2, 0) << std::endl;
		}

		J.row(0) << -cos(thetaZ), -sin(thetaZ) * track_ant.x
				+ cos(thetaZ) * track_ant.y + R * sin(thetaZ / 2);
		J.row(1) << sin(thetaZ), -cos(thetaZ) * track_ant.x
				- sin(thetaZ) * track_ant.y + R * cos(thetaZ / 2);
		J.row(2) << 0, -sin(thetaZ) * track_ant.vx + cos(thetaZ) * track_ant.vy;
		J.row(3) << 0, -cos(thetaZ) * track_ant.vx - sin(thetaZ) * track_ant.vy;

		cov_ant.row(0) << track_ant.cov[0], track_ant.cov[1], track_ant.cov[2], track_ant.cov[3];
		cov_ant.row(1) << track_ant.cov[4], track_ant.cov[5], track_ant.cov[6], track_ant.cov[7];
		cov_ant.row(2) << track_ant.cov[8], track_ant.cov[9], track_ant.cov[10], track_ant.cov[11];
		cov_ant.row(3) << track_ant.cov[12], track_ant.cov[13], track_ant.cov[14], track_ant.cov[15];

		cov_act = H_cov * cov_ant * H_cov.transpose() + J * P * J.transpose();
		if (debug_odometry_)
		{
			std::cout << "cov_ant = [ " << cov_ant(0, 0) << " , "
					<< cov_ant(0, 1) << " , " << cov_ant(0, 2) << " , "
					<< cov_ant(0, 3) << std::endl << "               "
					<< cov_ant(1, 0) << " , " << cov_ant(1, 1) << " , "
					<< cov_ant(1, 2) << " , " << cov_ant(1, 3) << std::endl
					<< "               " << cov_ant(2, 0) << " , "
					<< cov_ant(2, 1) << " , " << cov_ant(2, 2) << " , "
					<< cov_ant(2, 3) << std::endl << "               "
					<< cov_ant(3, 0) << " , " << cov_ant(3, 1) << " , "
					<< cov_ant(3, 2) << " , " << cov_ant(3, 3) << std::endl;

			std::cout << "cov_act = [ " << cov_act(0, 0) << " , "
					<< cov_act(0, 1) << " , " << cov_act(0, 2) << " , "
					<< cov_act(0, 3) << std::endl << "               "
					<< cov_act(1, 0) << " , " << cov_act(1, 1) << " , "
					<< cov_act(1, 2) << " , " << cov_act(1, 3) << std::endl
					<< "               " << cov_act(2, 0) << " , "
					<< cov_act(2, 1) << " , " << cov_act(2, 2) << " , "
					<< cov_act(2, 3) << std::endl << "               "
					<< cov_act(3, 0) << " , " << cov_act(3, 1) << " , "
					<< cov_act(3, 2) << " , " << cov_act(3, 3) << std::endl;
		}

		std::vector<double> vect_cov_act;
		vect_cov_act.reserve(16);
		vect_cov_act.resize(16, 0.0);
		vect_cov_act[0] = (double) cov_act(0, 0); // cov_xx
		vect_cov_act[1] = (double) cov_act(0, 1); //cov_xy
		vect_cov_act[2] = (double) cov_act(0, 2); // cov_xvx
		vect_cov_act[3] = (double) cov_act(0, 3); //cov_xvy
		vect_cov_act[4] = (double) cov_act(1, 0); //cov_yx
		vect_cov_act[5] = (double) cov_act(1, 1); //cov_yy
		vect_cov_act[6] = (double) cov_act(1, 2); //cov_yvx
		vect_cov_act[7] = (double) cov_act(1, 3); //cov_yvy
		vect_cov_act[8] = (double) cov_act(2, 0); // cov_vxvx
		vect_cov_act[9] = (double) cov_act(2, 1); //cov_vxvy
		vect_cov_act[10] = (double) cov_act(2, 2); // cov_vxx
		vect_cov_act[11] = (double) cov_act(2, 3); //cov_vxy
		vect_cov_act[12] = (double) cov_act(3, 0); //cov_vyvx
		vect_cov_act[13] = (double) cov_act(3, 1); //cov_vyvy
		vect_cov_act[14] = (double) cov_act(3, 2); //cov_vyx
		vect_cov_act[15] = (double) cov_act(3, 3); //cov_vyy

		if (debug_odometry_)
		{
			std::cout << " track_ant" << std::endl;
			track_ant.print();
		}

		track_act = SpointV_cov(pos_act(0, 0), pos_act(1, 0),track_ant.time_stamp, vel_act(0, 0), vel_act(1, 0),vect_cov_act);

		if (debug_odometry_)
		{
			std::cout << " track_act" << std::endl;
			track_act.print();
		}

		it_myTrack->set_kalman_update(track_act);

		odome.reset_odometry();

	}
}
 *
 */

