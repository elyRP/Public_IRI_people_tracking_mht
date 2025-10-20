/*
 * Odometry.h
 *
 *  Created on: 12/02/2015
 *      Author: erepiso
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

