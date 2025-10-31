/*
 * Odometry.h
 *
 *  Created on: 12/02/2015
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
*      License (for other authors that will not be the original one): CC BY-NC-ND 4.0 
*      (Attribution-NonCommercial-#NoDerivatives 4.0 International)
*       https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
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

#include <eigen3/Eigen/Dense> //Library for matrix. Being here, not needed in cpp. (Al estar aquí, no hacen falta en el cpp).
#include <iostream> //Library for screen messages.
#include <math.h> //library for sen cos sqrt etc.
#include <vector>

class Codometry
{
public:

	Codometry();
	~Codometry();
	void update_odometryGood(double dx, double dy, double dtheta, double dt, double wz, double linear_vx, double linear_vy);
	void update_odometry(double vx, double vy, double wz, double dt);
	void init_odometryGood(double ini_x, double ini_y, double ini_theta, double time_act);
	void init_odometry(double vx, double vy, double wz, double time_act);
	void reset_odometry();
	void print_odometry();
	double get_timeStamp(){ return time_stamp_; };
	void set_timeStamp(double timeStamp){ time_stamp_=timeStamp; };
	double get_translation(){ return R_; };
	double get_rotation(){ return thetaZ_; };
	double get_ant_translation(){ return R_ant_; };
	double get_ant_rotation(){ return thetaZ_ant_; };
	double get_vx(){return vx_;};
	double get_vy(){return vy_;};
	double get_wz(){return wz_;};
	double get_dt(){return dt_;};
	double get_linear_vx(){return linear_vx_;};
	double get_linear_vy(){return linear_vy_;};

	double get_before_x(){
		//std::cout<< " \n get_before_x_" << std::endl;
		return before_x_;
	};
	double get_before_y(){
		//std::cout<< " \n get_before_y_" << std::endl;
		return before_y_;
	};
	double get_before_theta(){
		//std::cout<< " \n  get_before_theta" << std::endl;
		return before_theta_;
	};
	void set_before_x(double before_x){
		//std::cout<< " \n set_before_x" << std::endl;
		before_x_=before_x;
	};
	void set_before_y(double before_y){
		//std::cout<< " \n set_before_y" << std::endl;
		before_y_=before_y;
	};
	void set_before_theta(double before_theta){
		//std::cout<< " \n set_before_theta" << std::endl;
		before_theta_=before_theta;
	};

	double get_dx_(){
			//std::cout<< " \n get_dx_" << std::endl;
			return dx_;
	};
	double get_dy_(){
			//std::cout<< " \n get_dy_" << std::endl;
			return dy_;
	};

	double get_actual_velocity_Wz_(){
				//std::cout<< " \n get_Wz_" << std::endl;
				return Wz_;
		};

	void set_act_odomTFmatrix(Eigen::Matrix3d odom_eigen_tf_act){
		//std::cout<< " \n set_before_theta" << std::endl;
		odom_eigen_tf_act_=odom_eigen_tf_act;
	};
	void set_ant_odomTFmatrix(Eigen::Matrix3d odom_eigen_tf_ant){
		//std::cout<< " \n set_before_theta" << std::endl;
		odom_eigen_tf_ant_=odom_eigen_tf_ant;
	};

	Eigen::Matrix3d get_global_tf_odom(){
		//std::cout<< " \n set_before_theta" << std::endl;
		return global_tf_odom_;
	};


private:
	bool debug_odometry_;
	bool cargo_ants;
	double vx_; // [m/s]
	double vy_; // [m/s]
	double wz_; // [rad/s]
	double Wz_; // velocidad de rotación, para compensarla en el track.

	double dx_; // [m]
	double dy_; // [m]
	double dtheta_; // [rad]

	double before_x_; // [m]
	double before_y_; // [m]
	double before_theta_; // [rad]


	double time_stamp_; // [s]
	double R_; // [m] translation module.
	double thetaZ_; // [rad] rotation angle.
	double dt_; // [s] increment of time= time_odom_act_-time_odom_ant_

	double linear_vx_;
	double linear_vy_;

	double R_ant_; // [m] translation module.
	double thetaZ_ant_; // [rad] rotation angle.
		
	Eigen::Matrix3d GLOBALodom_eigen_tf_ant_;

	Eigen::Matrix3d odom_eigen_tf_ant_;
	
	//Eigen::Matrix3d inc_odom_eigen_tf_ant_;
	Eigen::Matrix3d odom_eigen_tf_act_;
	
	Eigen::Matrix3d global_tf_odom_;
	
	std::vector<Eigen::Matrix3d> vect_odom_transforms;

};

