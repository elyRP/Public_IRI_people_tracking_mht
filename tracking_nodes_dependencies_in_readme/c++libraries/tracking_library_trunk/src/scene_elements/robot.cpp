/*
 * robot.cpp
 *
 *  Created on: Jul 31, 2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration).
 *      Author: gferrer
 *      Author: Ely Repiso (Modifications for people tracking mht)
 *
 *      license (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
 *      This license does not allow other authors to modify or to take profit from these works. 
 *      Then, for modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
 *   collaborations (for journals with other researchers, formal collaborations between UPC and companies, and so on).
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
 */
#include "scene_elements/robot.h"
#include <math.h>
#include <iostream>

Crobot::Crobot(  unsigned int id , robot_type robot_type,
		Cperson_abstract::force_type person_force_type, double _time_window) :
	Cperson_abstract( id, Cperson_abstract::Robot, person_force_type),
	v_max_(1.0), w_max_(3.0), a_v_max_(2.0), a_v_break_(2.0),a_w_max_(5.0),
	platform_radii_(1.0), platform_radii_2_(1.0),
	debug_antes_subgoals_entre_AKP_goals_(false),
	test_aw_max_(0.0),test_w_max_(0.0),
	debug_velocity_file2_(true),
	debug_velocity_cout2_(true),
	debug_robot_propagation_(false),
	high_vel_dampening_parameter_(2.6),
	normal_vel_dampening_parameter_(1.6),
	slow_vel_dampening_parameter_(0.7),
	limit_linear_vel_for_dampening_parameter_(0.2),
	limit_angular_vel_for_dampening_parameter_(0.5),
	augment_initial_v_(false),
	initial_v_robot_needed_(0.17)
{
}

Crobot::~Crobot()
{

}

void Crobot::add_pose( Spose  observation )
{
	//TODO take into account covariances
	diff_pose_ = observation - current_pose_;
	current_pose_ = observation;
	SpointV_cov current_point = SpointV_cov( observation.x, observation.y,observation.time_stamp,
			observation.v*cos(observation.theta), observation.v*sin(observation.theta) );
	diff_pointV_ = current_point - current_pointV_ ;
	current_pointV_ = current_point;
}

void Crobot::prediction( double min_v_to_predict)
{

}

void Crobot::reset()
{
	current_pointV_ = SpointV_cov();
	current_pose_ = Spose();
	diff_pointV_ = SpointV_cov();
	best_destination_ = Sdestination();
	destinations_.clear();
	planning_trajectory_.clear();
	planning_SpointV_trajectory_.clear();
	rnd_goal_ = Spoint();
    planning_v_.clear();
    planning_w_.clear();
    planning_x_.clear();
    planning_y_.clear();
    planning_theta_.clear();
    planning_before_av_.clear();
    planning_before_aw_.clear();
    planning_after_av_.clear();
    planning_after_aw_.clear();
    planning_damping_aw_.clear();
}

void Crobot::reset_before_destination_prob(){
	best_destination_ = Sdestination();
	destinations_.clear();
}
void Crobot::clear_planning_trajectory()
{
    planning_trajectory_.clear();
    planning_trajectory_.push_back(current_pose_);
    planning_SpointV_trajectory_.clear();
    planning_SpointV_trajectory_.push_back( current_pointV_);
    planning_v_.clear();
    planning_w_.clear();
    planning_x_.clear();
    planning_y_.clear();
    planning_theta_.clear();
    planning_before_av_.clear();
    planning_before_aw_.clear();
    planning_after_av_.clear();
    planning_after_aw_.clear();
    planning_damping_aw_.clear();



    time_stamp_plan_.clear();
    for(unsigned int g=0;g<500;g++){
    	 time_stamp_plan_.push_back(0.0);
    }



}

void Crobot::erase_last_planning_propagation()
{
    planning_trajectory_.pop_back();
    planning_SpointV_trajectory_.pop_back();
}
/*
void Crobot::robot_propagation(double dt , unsigned int index, double v, double w)
{
	//Kinematic unicycle TODO propagate covariances
	//std::cout << "robot_propagation 1 " <<  std::endl;
	Spose propagated_pose,ini_pose;
	ini_pose = planning_trajectory_.at(index);
	if( v > v_max_) v=v_max_;
	if( v < -v_max_) v=-v_max_;
	if( w > w_max_) w=w_max_;
	if( w < -w_max_) w=-w_max_;
	propagated_pose.x = ini_pose.x + v*cos(ini_pose.theta)*dt;
	propagated_pose.y = ini_pose.y + v*sin(ini_pose.theta)*dt;
	propagated_pose.theta = ini_pose.theta + w*dt;
	propagated_pose.time_stamp = ini_pose.time_stamp + dt;
	propagated_pose.v = v;
	propagated_pose.w = w;
	//update
	planning_trajectory_.push_back(propagated_pose);
	planning_SpointV_trajectory_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
				propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
				propagated_pose.v*sin(propagated_pose.theta)) );


}*/
/*
void Crobot::robot_propagation_Zanlungo(double dt , unsigned int index, const Sforce &f ,SpointV_cov pose_of_the_robot , double before_angular_velocity, bool debug_force_vel) // con fuera entro en esta!!!
{


	//std::cout << " Zanlungo 2; robot_->get_planning_trajectory( ).size()="<<robot_->get_planning_trajectory( )->size()<<std::endl;
	Vector2D robot_pose(pose_of_the_robot.vx,pose_of_the_robot.vy,atan(pose_of_the_robot.vy/pose_of_the_robot.vx)); // theta en rads, chec if is ok.
	//std::cout << "(robot_propagation_Zanlungo) v_max_="<<v_max_<<std::endl;


	Spose propagated_pose,ini_pose;
	ini_pose = planning_trajectory_.at(index);



	prob_clas_Francesco_model_.set_Self(pose_of_the_robot.x,pose_of_the_robot.y,pose_of_the_robot.vx,pose_of_the_robot.vy,robot_pose.th);

	Vector2D force_act(f.fx,f.fy);
	double radius;



	if(index<0){
			std::cout << " index="<<index<< " IMPORTANTE!!!!  (Before/Antes scale dt) force_act.m="<<force_act.m<<"; force_act.x"<<force_act.x<<"; force_act.y"<<force_act.y<<"; a_v_max_="<<a_v_max_<<std::endl;

		}


	//if(force_act.m>a_v_max_){
	//	force_act.Scale(a_v_max_/force_act.m);
	//}

	if(index<0){
			std::cout << " index="<<index<< " IMPORTANTE!!!!  (After/despues scale dt) force_act.m="<<force_act.m<<"; force_act.x"<<force_act.x<<"; force_act.y"<<force_act.y<<std::endl;

		}


	double vlin;
	Vector2D nextv=force_act;
	if(index<0){
	std::cout << " index="<<index<< " IMPORTANTE!!!!  (solo fuerza; f.fx y f.fy) nextv.m="<<nextv.m<<"; nextv.x"<<nextv.x<<"; nextv.y"<<nextv.y<<"; nextv.th"<<nextv.th<<std::endl;
	std::cout << " index="<<index<< " IMPORTANTE!!!!  Self.v.m="<<prob_clas_Francesco_model_.Self.v.m<<"; self.x="<<prob_clas_Francesco_model_.Self.v.x<<"; self.y="<<prob_clas_Francesco_model_.Self.v.y<<"; self.th="<<prob_clas_Francesco_model_.Self.v.th<<std::endl;
	}
	double omega;
	nextv.Scale(dt);
	if(index<0){
		std::cout << " index="<<index<< " IMPORTANTE!!!!  (After scale dt) nextv.m="<<nextv.m<<"; nextv.x"<<nextv.x<<"; nextv.y"<<nextv.y<<"; nextv.th"<<nextv.th<<std::endl;

	}







	// todo: add selft velocity
	nextv.Add(prob_clas_Francesco_model_.Self.v);
	//if(index<0){
		//std::cout << " index="<<index<< " IMPORTANTE!!!!  (After add selft.v) nextv.m="<<nextv.m<<"; nextv.x"<<nextv.x<<"; nextv.y"<<nextv.y<<"; nextv.th"<<nextv.th	<<std::endl;

		//std::cout <<"; prob_clas_Francesco_model_.Self.v.x="<<prob_clas_Francesco_model_.Self.v.x<<"; prob_clas_Francesco_model_.Self.v.y="<<prob_clas_Francesco_model_.Self.v.y<<std::endl;

		//}

	if(!prob_clas_genome_.rotating) //if finished rotating moves with vlin (linear velocity) given by vstart
	{
		vlin=prob_clas_genome_.vstart;
		omega=0;
	}
	else //else no change in position and omega, while  goes on rotating
	{
		prob_clas_genome_.rotating--;
		 vlin=0;
	}

	   double omega_final;



	   if(augment_initial_v_){
		   std::cout << " in ... augment_initial_v_; nextv.m="<<nextv.m<<std::endl;

		   if((nextv.m>0.07)&&(nextv.m<initial_v_robot_needed_)){
			   std::cout << " in ...  if((nextv.m>0.07)&&(nextv.m<0.15)) ... augment_initial_v_"<<std::endl;
			   nextv.Vector2D_v2(initial_v_robot_needed_,nextv.th);

			   std::cout << " out ...  if((nextv.m>0.07)&&(nextv.m<0.15)) ... augment_initial_v_; nextv.x="<<nextv.x<<"; nextv.y="<<nextv.y<<"; nextv.m="<<nextv.m<<std::endl;
		   }
	   }



	// limit
	if(nextv.m>v_max_){
		nextv.Scale(v_max_/nextv.m);
	}
	if(index<0){
		std::cout << " index="<<index<< " IMPORTANTE!!!!  (After scale max) nextv.m="<<nextv.m<<"; nextv.x"<<nextv.x<<"; nextv.y"<<nextv.y<<"; nextv.th"<<nextv.th<<"; v_max_="<<v_max_<<std::endl;

	}

	double aw;
							// Take into account outside the robot_pose.x=is set as vx and robot_pose.y=is set as vy
	if(prob_clas_Francesco_model_.Scalar(prob_clas_Francesco_model_.Self.v,nextv)>=0)//if the next velocity is directed as the current
	  {

		aw = -sin(atan(pose_of_the_robot.vy/pose_of_the_robot.vx))*f.fx + cos(atan(pose_of_the_robot.vy/pose_of_the_robot.vx))*f.fy;//cross product theta x f
		if(index<0){
			std::cout << " ZANLUNGO good omenga;  aw1="<<aw<<std::endl;
		}

		aw -= normal_vel_dampening_parameter_*ini_pose.w;
		// I=m*rad^2 I*aw=taw-2*m*r*(inc_r/inc_t)*w
		//

				if(index<0){
							std::cout << " ZANLUNGO good omenga;  aw2="<<aw<<"; a_v_max="<<a_v_max_<<std::endl;
						}

		if( aw > a_w_max_) aw=a_w_max_;
		if( aw < -a_w_max_) aw=-a_w_max_;

	    double rad_acc=prob_clas_Francesco_model_.Clock_Ort(force_act,prob_clas_Francesco_model_.Self.v);   //first computes radial acceleration (ICSR 2017)

	    if( rad_acc > a_w_max_) rad_acc=a_w_max_;
	    if( rad_acc < -a_w_max_) rad_acc=-a_w_max_;

	    force_act_x_.push_back(f.fx);
	    force_act_y_.push_back(f.fy);
	    nextv_.push_back(nextv.m);

	    selft_v_x_.push_back(prob_clas_Francesco_model_.Self.v.x);
	    selft_v_y_.push_back(prob_clas_Francesco_model_.Self.v.y);
	    rad_acc_.push_back(rad_acc);

	    if(nextv.m){

	    	//if(sqrt(f.fy*f.fy)<0.1){
	    	//	omega=0;
	    	//}else{
	    		omega=-rad_acc/nextv.m;//-nextv.m/rad_acc;//rad_acc;//;  //and from it derives v,omega
	    	//}
	    	//omega=nextv.m/rad_acc;

	    	if(index<0){
		    	std::cout << " ZANLUNGO good omenga; f.fx="<<f.fx<<"; f.fy"<<f.fy <<"; aw3="<<aw<<"; aw3*dt="<<aw*dt<<std::endl;
		    	std::cout << " nextv.m="<<nextv.m<<"; omega=-rad_acc/nextv.m= "<<-rad_acc/nextv.m<<"; rad_acc="<<rad_acc<<"; nextv.m="<<nextv.m<<std::endl;

	    	}

	    }else{
	    	omega=0;
	    }


		double increment_angular_velocity=sqrt((before_angular_velocity-omega)*(before_angular_velocity-omega));

		radius=fabs(nextv.m/omega);
		//omega_final=(w_max_*(nextv.m/omega))/radius;

	    if(omega>w_max_) omega=w_max_;
	    else if(omega<-w_max_) omega=-w_max_;

	    vlin=nextv.m;

	  //  std::cout << " nextv="<<nextv.m<<"; omega=-rad_acc/nextv.m= "<<-rad_acc/nextv.m<<"; rad_acc="<<rad_acc<<"; nextv.m="<<nextv.m<<std::endl;



	  }
	else  //but if negative
	  {

		// Check_th es Vector2D class. mirar
	    double dtheta=force_act.Check_th(prob_clas_Francesco_model_.Self.v.th-nextv.th);  //checks rotation angles
	    prob_clas_genome_.rotating=int((fabs(dtheta)/w_max_)/dt);//and how long it takes
	    if((prob_clas_genome_.rotating*w_max_*dt)!=fabs(dtheta)) prob_clas_genome_.rotating++;
	    omega=dtheta/(prob_clas_genome_.rotating*dt);//and the required omega

	    if(index<0){
	    	 std::cout << "  ZANLUNGO rotational omenga; dtheta="<<dtheta<<"; omega=dtheta/(prob_clas_genome_.rotating*dt)= "<<omega<<"; prob_clas_genome_.rotating="<<prob_clas_genome_.rotating<<"; dt="<<dt<<std::endl;

	    }


	    vlin=0;
	    prob_clas_genome_.rotating--;
	  }







	//std::cout <<" ini_pose.x="<<ini_pose.x<<"; ini_pose.y"<<ini_pose.y<<"; ini_pose.v="<<ini_pose.v<<"; ini_pose.w="<<ini_pose.w<<"; dt="<<dt<<  std::endl;

	// update velocities
		propagated_pose.v =nextv.m; //ini_pose.v + av*dt;

		propagated_pose.w =omega;//omega_final;//omega;//omega_final;//omega; //ini_pose.w + aw*dt;
	    omega_.push_back(omega);

		 if(index<0){
				std::cout << " MYA good omenga;  propagated_pose.w="<<ini_pose.w + aw*dt<<"; ini_pose.w="<<ini_pose.w<<"; omega="<<omega<<std::endl;

		 }


		propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose.theta)*dt
				+ dt*dt/2.0*cos(ini_pose.theta)*force_act.m;
		propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose.theta)*dt
				+ dt*dt/2.0*sin(ini_pose.theta)*force_act.m;
		propagated_pose.theta = ini_pose.theta + ini_pose.w*dt;
				+ dt*dt/2.0*((omega-ini_pose.w)/dt);
		propagated_pose.time_stamp = ini_pose.time_stamp + dt;

		//update
		//std::cout <<"propagated_pose person companion"<<  std::endl;
		//propagated_pose.print();
	   planning_before_av_.push_back(force_act.m);
	   planning_before_aw_.push_back(((omega-ini_pose.w)/dt));
	   planning_v_.push_back(propagated_pose.v);
	   planning_w_.push_back(propagated_pose.w);
	   planning_x_.push_back(propagated_pose.x);
	   planning_y_.push_back(propagated_pose.y);
	   planning_theta_.push_back(propagated_pose.theta);
	   planning_after_av_.push_back(force_act.m);
	   planning_after_aw_.push_back(((omega-ini_pose.w)/dt));

	   planning_damping_aw_.push_back(10000.0);


		planning_trajectory_.push_back(propagated_pose);
		planning_SpointV_trajectory_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
					propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
					propagated_pose.v*sin(propagated_pose.theta)) );


		if(debug_robot_propagation_){
			//std::cout <<" planning_trajectory_.size="<<planning_trajectory_.size()<<" size="<<planning_trajectory_.size()-1<<  std::endl;
			//if(index<10){
			planning_SpointV_trajectory_.back().print();
		}

		//std::cout <<" propagated_pose.x="<<propagated_pose.x<<"; propagated_pose.y"<<propagated_pose.y<<"; ini_pose.v="<<propagated_pose.v<<"; propagated_pose.w="<<propagated_pose.w<<"; planning_SpointV_trajectory_.size()="<<planning_SpointV_trajectory_.size()<<  std::endl;


}*/


void Crobot::robot_propagation(double dt , unsigned int index, const Sforce &f , bool debug_force_vel) // con fuera entro en esta!!!
{
	//kinodynamic unicycle
	if(debug_robot_propagation_){
	//if(index<10){
		std::cout << "AKP-propagation IN robot_propagation (propagate pose); v_max_="<<v_max_<<  std::endl;
	}
	//std::cout <<"(out function: robot_propagation); v_max_="<<v_max_<<  std::endl;

	//}
	Spose propagated_pose,ini_pose;
	ini_pose = planning_trajectory_.at(index);
	//std::cout << "ini_pose.theta="<<ini_pose.theta<<  std::endl;
	//std::cout << "ini_pose.v="<<ini_pose.v<<  std::endl;

	//convert 2D forces into robot forces with non-holonomic contraints
	double av = cos(ini_pose.theta)*f.fx + sin(ini_pose.theta)*f.fy;//projection to robot pose (dot product)

	double aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy;//cross product theta x f


    planning_before_av_.push_back(av);
    planning_before_aw_.push_back(aw);

	//if(debug_robot_propagation_){
	if(index<0){
		std::cout << "f.fx="<<f.fx<<"cos(ini_pose.theta)"<<cos(ini_pose.theta)<<"; f.fy="<<f.fy<<"sin(ini_pose.theta)="<<sin(ini_pose.theta)<< std::endl;
		std::cout <<"ini_pose.v="<<ini_pose.v<<"; av="<<av<<"; av*dt="<<av*dt<<  std::endl;
		std::cout <<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;
	}
	//std::cout <<"(before damping) f.fx="<<f.fx<<"; f.fy="<<f.fy<<"; sin(ini_pose.theta)="<<sin(ini_pose.theta)<<"; cos(ini_pose.theta)="<<cos(ini_pose.theta)<<  std::endl;

	//std::cout <<"(before damping) ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;

	// calculate the proportional torque thau = k (0 - w) similar to the steering force
	/*if(sqrt(ini_pose.v*ini_pose.v)<limit_linear_vel_for_dampening_parameter_){ // linear_velocity_gir_limit_=0.2 initial limit_linear_vel_for_dampening_parameter_=0.2
		aw -= slow_vel_dampening_parameter_*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly
		//std::cout <<"case1 slow_vel_dampening_parameter_="<<slow_vel_dampening_parameter_<<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;
	}
	else if((sqrt(ini_pose.v*ini_pose.v)<limit_angular_vel_for_dampening_parameter_)&&(sqrt(ini_pose.v*ini_pose.v)>limit_linear_vel_for_dampening_parameter_)){ //angular_velocity_gir_limit_=0.5
		aw -= normal_vel_dampening_parameter_*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly
		//std::cout <<"case2 normal_vel_dampening_parameter_="<<normal_vel_dampening_parameter_<<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;

	}else if((sqrt(ini_pose.v*ini_pose.v)>limit_angular_vel_for_dampening_parameter_)){//||(sqrt(ini_pose.v*ini_pose.v)>limit_linear_vel_for_dampening_parameter_)
		aw -= high_vel_dampening_parameter_*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly
		//std::cout <<"case3 high_vel_dampening_parameter_="<<high_vel_dampening_parameter_<<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;

	}else{
		aw -= normal_vel_dampening_parameter_*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly
		//std::cout <<"case4 normal_vel_dampening_parameter_="<<high_vel_dampening_parameter_<<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;

	}*/
	aw -= normal_vel_dampening_parameter_*ini_pose.w;
	// girar mas rapido==> disminuir parametro a 1. Girar mas suave, lento, subir parametro por encima del original que es 1.6.
	if(index<0){
		std::cout <<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;
	}
	//std::cout <<"(after damping) ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;
    planning_damping_aw_.push_back(normal_vel_dampening_parameter_*ini_pose.w);

    //std::cout << "v_max_="<<v_max_<<"; w_max_="<<w_max_<<"; desired_velocity_="<<desired_velocity_<<  std::endl;

	//if(debug_robot_propagation_){
	if(index<0){
		std::cout <<"(after) aw="<<aw<<  std::endl;
		std::cout << "a_v_max_="<<a_v_max_<<"; a_w_max_="<<a_w_max_<<"; a_v_break_="<<a_v_break_<<  std::endl;
		std::cout << "v_max_="<<v_max_<<"; w_max_="<<w_max_<<  std::endl;
	}

	double a_v_max_loc=a_v_max_;
	double a_v_breack_loc=a_v_break_;
	bool caso_frenar=false;
	//if(global_dist_to_obstacle_in_front>1.5m){ o 2m
		if(av<0.0){ // intentamos frenar.
			a_v_max_loc=a_v_max_negativa_;  // para frenado más suave.
			a_v_breack_loc=a_v_max_negativa_;
			//std::cout << "CASO FRENAAAAARRRRR; a_v_max_negativa_="<<a_v_max_negativa_<<  std::endl;
			caso_frenar=true;
		}
	//}

	// check for forces validity
	if( av*ini_pose.v < 0.001 )//breaking configuration if different sign v and a
	{
		if( av > a_v_breack_loc) av=a_v_breack_loc;
		if( av < -a_v_breack_loc) av=-a_v_breack_loc;
		if(caso_frenar){
			//std::cout << "CASO FRENAAAAARRRRR; a_v_break_="<<a_v_breack_loc<<  std::endl;
		}
	}
	else // not breaking or 0.0
	{
		if( av > a_v_max_loc) av=a_v_max_loc;
		if( av < -a_v_max_loc) av=-a_v_max_loc;
		if(caso_frenar){
			//std::cout << "CASO FRENAAAAARRRRR; a_v_max_="<<a_v_max_loc<<  std::endl;
		}
	}
	if( aw > a_w_max_) aw=a_w_max_;
	if( aw < -a_w_max_) aw=-a_w_max_;

	//if(debug_robot_propagation_){


	// update velocities
	propagated_pose.v = ini_pose.v + av*dt;
	//if(debug_robot_propagation_){
	if(index<0){
			std::cout <<"(after if) av="<<av<<"; av_max="<<a_v_max_<<"; propagated_pose.v="<<propagated_pose.v<<  std::endl;
			std::cout <<"(after if) aw="<<aw<<"; a_w_max_="<<a_w_max_<<  std::endl;
		}
	propagated_pose.w = ini_pose.w + aw*dt;

	if( propagated_pose.v > v_max_) propagated_pose.v=v_max_;
	if( propagated_pose.v < -v_max_) propagated_pose.v=-v_max_;
	if( propagated_pose.w > w_max_) propagated_pose.w=w_max_;
	if( propagated_pose.w < -w_max_) propagated_pose.w=-w_max_;
	propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose.theta)*dt
			+ dt*dt/2.0*cos(ini_pose.theta)*av;
	propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose.theta)*dt
			+ dt*dt/2.0*sin(ini_pose.theta)*av;
	propagated_pose.theta = ini_pose.theta + ini_pose.w*dt
			+ dt*dt/2.0*aw;
	propagated_pose.time_stamp = ini_pose.time_stamp + dt;

	time_stamp_plan_.push_back(dt+time_stamp_plan_.at(index));

	if(index<0){
		double dist=sqrt((propagated_pose.x -ini_pose.x)*(propagated_pose.x -ini_pose.x)+(propagated_pose.y -ini_pose.y)*(propagated_pose.y -ini_pose.y));
		std::cout <<"; dist="<<dist<< "; propagated_pose.x -ini_pose.x="<<propagated_pose.x -ini_pose.x<<"; propagated_pose.y -ini_pose.y="<<propagated_pose.y -ini_pose.y<<  std::endl;
	}

	if(index<0){
		std::cout << "propagated_pose.v=(ini_pose.v + av*dt)="<<propagated_pose.v<<"propagated_pose.v=(ini_pose.w + aw*dt)="<<propagated_pose.w<<  std::endl;
		std::cout <<" propagated_pose.x="<<propagated_pose.x<<"; propagated_pose.y="<<propagated_pose.y<<"; v_max_="<<v_max_<<" dt="<<dt<<  std::endl;
	}

	//update
	//std::cout <<"time_stamp_plan_.at(index)="<<time_stamp_plan_.at(index)<<"; index="<<index<<"; dt="<<dt<<  std::endl;
	//propagated_pose.print();

   planning_v_.push_back(propagated_pose.v);
   planning_w_.push_back(propagated_pose.w);
   planning_x_.push_back(propagated_pose.x);
   planning_y_.push_back(propagated_pose.y);
   planning_theta_.push_back(propagated_pose.theta);
   planning_after_av_.push_back(av);
   planning_after_aw_.push_back(aw);




	planning_trajectory_.push_back(propagated_pose);
	planning_SpointV_trajectory_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
				propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
				propagated_pose.v*sin(propagated_pose.theta)) );

	if(debug_robot_propagation_){
		std::cout <<"(out function: robot_propagation) planning_trajectory_.size="<<planning_trajectory_.size()<<  std::endl;
		planning_SpointV_trajectory_.back().print();
		std::cout <<" FINAL aw="<<aw<<"; final_propagated_pose="<<propagated_pose.w<<  std::endl;
	}

}

Sbehavior* Crobot::find_behavior_estimation( unsigned int id )
{
	std::list<Sbehavior>::iterator iit = expected_behavior_list_.begin();
	Sbehavior *behavior;
	if ( !expected_behavior_list_.empty() )
	{
		for( ; iit != expected_behavior_list_.end(); iit++ )
		{
			if ( iit->related_person_id == id )
			{
				behavior = &(*iit);
				assert( behavior != NULL );
				return  behavior;
			}
			if ( iit->related_person_id > id )
				break;
		}
	}
	//behavior not found for person id, then a new behavior is set
	expected_behavior_list_.insert( iit, Sbehavior( id ) );
	iit--;//element inserted is before iit, so we want the pointer to the inserted element
	behavior = &(*iit);
	assert( behavior != NULL );
	return  behavior;
}

Cperson_abstract::behavior_type
Crobot::get_best_behavior_to_person( unsigned int interacting_person ) const
{
	Cperson_abstract::behavior_type result = Cperson_abstract::Balanced;
	const Sbehavior * behavior = NULL;
	std::list<Sbehavior>::const_iterator iit = expected_behavior_list_.begin();
	if ( !expected_behavior_list_.empty() )
	{
		for( ; iit != expected_behavior_list_.end(); iit++ )
		{
			if ( iit->related_person_id == interacting_person )
			{
				behavior = &(*iit);
				assert( behavior != NULL );
			}
			if ( iit->related_person_id > interacting_person )
				break;//person not found, so result is a balanced behavior
		}
	}
	if ( behavior == NULL ) return result;
	assert( behavior != NULL );
	double best_expectation =  behavior->expectation[0];
	for ( unsigned int i = 1; i < behavior->expectation.size(); ++i  )
	{

		if( best_expectation < behavior->expectation[i]  )
		{
			best_expectation = behavior->expectation[i];
			result = (Cperson_abstract::behavior_type)i;
		}
	}
	//behavior->print();
	return result;
}


void Crobot::correct_state_to_delay( Spose last_control_cmd, double now, double delay )
{
	current_pose_.w = last_control_cmd.w + (current_pose_.w - last_control_cmd.w ) * exp( - fabs(now - current_pose_.time_stamp + delay) / 0.05 );
	current_pose_.v = last_control_cmd.v + (current_pose_.v - last_control_cmd.v ) * exp( - fabs(now - current_pose_.time_stamp + delay) / 0.05 );
	//current_pointV_ not necessary for dynamic constraints, only for comparisions
	//TODO propagate state {x,y,theta}, not implemented yet
}

Spose Crobot::robot_propagation_companion_position(double dt, const Sforce& f ,Spose in_ini_pose, bool coments){ // made by (ely) for robot companion.
	//kinodynamic unicycle
	if(debug_antes_subgoals_entre_AKP_goals_){
		std::cout << " IN robot_propagation_companion_position(); dt="<<dt << std::endl;
	}

	//std::cout << "(in) dt="<<dt<< std::endl;

	Spose propagated_pose,ini_pose;
	ini_pose = in_ini_pose;

	//if(coments){
	//	std::cout << " dt="<<dt << std::endl;
	//	std::cout << "f.fx="<<f.fx<<"; f.fy="<<f.fy << std::endl;
	//}

	double ini_pose_theta=ini_pose.theta;
	if(ini_pose.theta<0){
		ini_pose_theta=(2*3.14)+ini_pose.theta;
	}

	if(debug_antes_subgoals_entre_AKP_goals_){
		std::cout << "ini_pose.print();!!!!!"<< std::endl;
		ini_pose.print();
	}

	//convert 2D forces into robot forces with non-holonomic contraints
	if(debug_antes_subgoals_entre_AKP_goals_){
		std::cout << "ini_pose.theta"<<ini_pose.theta*(180/3.14) << std::endl;
	}

	//double av = cos(ini_pose.theta)*f.fx + sin(ini_pose.theta)*f.fy;//projection to robot pose (dot product)
	//double aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy;//cross product theta x f
	double av = cos(ini_pose_theta)*f.fx + sin(ini_pose_theta)*f.fy;//projection to robot pose (dot product)
	double aw = -sin(ini_pose_theta)*f.fx + cos(ini_pose_theta)*f.fy;//cross product theta x f

	//std::cout << " (Initial) av ="<<av  << std::endl;
	//std::cout << " (Initial) aw ="<<aw  << std::endl;

	if(debug_antes_subgoals_entre_AKP_goals_){
	//	if(ini_pose.theta<0){
		//	std::cout << " av ="<<cos((2*3.14)+ini_pose.theta)*f.fx + sin((2*3.14)+ini_pose.theta)*f.fy << "cos(ini_pose.theta)="<<cos((2*3.14)+ini_pose.theta)<< std::endl;
		//	std::cout << " aw ="<<-sin((2*3.14)+ini_pose.theta)*f.fx + cos((2*3.14)+ini_pose.theta)*f.fy <<"sin(ini_pose.theta)="<< sin((2*3.14)+ini_pose.theta)<< std::endl;

	//	}
	}

	if(debug_velocity_file2_){
		//std::cout << " av ="<<cos(-ini_pose.theta)*f.fx + sin(-ini_pose.theta)*f.fy << "cos(ini_pose.theta)="<<cos(-ini_pose.theta)<< std::endl;
		//std::cout << " aw ="<<-sin(-ini_pose.theta)*f.fx + cos(-ini_pose.theta)*f.fy <<"sin(ini_pose.theta)="<< sin(-ini_pose.theta)<< std::endl;
		//std::cout << " aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy"<<aw << std::endl;
		//std::cout << " av="<<av << std::endl;
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		fileMatlab2 << " av ="<<cos(-ini_pose.theta)*f.fx + sin(-ini_pose.theta)*f.fy << "cos(ini_pose.theta)="<<cos(-ini_pose.theta)<< "\n";
		fileMatlab2 << " aw ="<<-sin(-ini_pose.theta)*f.fx + cos(-ini_pose.theta)*f.fy <<"sin(ini_pose.theta)="<< sin(-ini_pose.theta)<< "\n";
		fileMatlab2 << " aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy"<<aw <<  "\n";
		fileMatlab2 << " av="<<av <<  "\n";
		fileMatlab2.close();

	}
	// calculate the proportional torque thau = k (0 - w) similar to the steering force
	 aw = aw -1.6*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly
	// TODO: ojo! cambie esto, pq entonces no usa la fuerza, para girar y no gira bien!
	 //aw = aw -2.0*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly

		//std::cout << " (2 Initial) av ="<<av  << std::endl;
		//std::cout << " (2 Initial) aw ="<<aw  << std::endl;


	if(debug_velocity_file2_){
		//std::cout << " aw -= 1.6*ini_pose.w"<<aw << std::endl;
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		fileMatlab2 << " aw -= 1.6*ini_pose.w"<<aw << "\n";
		fileMatlab2.close();
	//	std::cout << " (2) aw ="<<aw  << std::endl;
	}


	double a_v_max_loc=a_v_max_;
	double a_v_breack_loc=a_v_break_;
	bool caso_frenar=false;
	//if(global_dist_to_obstacle_in_front>1.5m){ o 2m
		if(av<0.0){ // intentamos frenar.
			a_v_max_loc=a_v_max_negativa_;  // para frenado más suave.
			a_v_breack_loc=a_v_max_negativa_;
			// debug_sideBySide2019_
			//std::cout << "CASO FRENAAAAARRRRR; a_v_max_negativa_="<<a_v_max_negativa_<<  std::endl;
			caso_frenar=true;
		}
	//}

	// check for forces validity
	if( av*ini_pose.v < 0.001 )//breaking configuration if different sign v and a
	{
		//std::cout << "if av*ini_pose.v < 0.001 ; av="<<av << std::endl;

		if(debug_velocity_file2_){
			//std::cout << " caso 1" << std::endl;
			std::ofstream fileMatlab2;
			fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
			fileMatlab2<< " caso 1" << "\n";
			fileMatlab2.close();
		}

		if( av > a_v_breack_loc){
			av=a_v_breack_loc;
			//std::cout << " av > a_v_break_ ; av= a_v_break_="<<av << std::endl;

		}
		if( av < -a_v_breack_loc){

			av=-a_v_breack_loc;
			//std::cout << " av < -a_v_break_ ; av= -a_v_break_="<<av << std::endl;
		}

		if(caso_frenar){
			//std::cout << "CASO FRENAAAAARRRRR; a_v_break_="<<a_v_breack_loc<<  std::endl;
		}

	}
	else // not breaking or 0.0
	{
		//std::cout << " else av*ini_pose.v < 0.001 ; av="<<av << std::endl;
		if(debug_velocity_file2_){
			//std::cout << " caso 2" << std::endl;
			std::ofstream fileMatlab2;
			fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
			fileMatlab2<< " caso 2" << "\n";
			fileMatlab2.close();
		}

		if( av > a_v_max_loc){
			av=a_v_max_loc;
			//std::cout << " av > a_v_max_ ; av=a_v_max_="<<av << std::endl;
		}
		if( av < -a_v_max_loc){
			av=-a_v_max_loc;
			//std::cout << " av <- a_v_max_ ; av=-a_v_max_="<<av << std::endl;
		}

		if(caso_frenar){
			//std::cout << "CASO FRENAAAAARRRRR; a_v_max_="<<a_v_max_loc<<  std::endl;
		}
	}
	if( aw > a_w_max_){



		if(debug_velocity_file2_){
			//std::cout << " caso 3" << std::endl;
			std::ofstream fileMatlab2;
			fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
			fileMatlab2<< " caso 3" << "\n";
			fileMatlab2.close();
		}

		aw=a_w_max_;
		//std::cout << " aw > a_w_max_ ; aw=a_w_max_="<<aw << std::endl;
	}
	if( aw < -a_w_max_){

		if(debug_velocity_file2_){
			//std::cout << " caso 4" << std::endl;
			std::ofstream fileMatlab2;
			fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
			fileMatlab2<< " caso 4" << "\n";
			fileMatlab2.close();
		}

		aw=-a_w_max_;
		//std::cout << " aw < -a_w_max_ ; aw=-a_w_max_="<<aw << std::endl;
	}
//debug_sideBySide2019_
	//std::cout << " IMPORTANTE COMPROAVAR AV_MAX (after if's) av ="<<av <<"; a_w_max_="<<a_w_max_ << std::endl;
	//std::cout << " (after if's) aw ="<<aw  << std::endl;

	if(debug_velocity_file2_){

		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		fileMatlab2<<" (after if's) av ="<<av  << "\n";
		fileMatlab2<<" (after if's) aw ="<<aw  << "\n";
		fileMatlab2.close();
	}
	// update velocities
	propagated_pose.v = ini_pose.v + av*dt;

	if(debug_velocity_file2_){
		//std::cout << " propagated_pose.v ="<<propagated_pose.v <<"; ini_pose.v="<<ini_pose.v<< "; av*dt="<<av*dt<<std::endl;
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		fileMatlab2<<" propagated_pose.v ="<<propagated_pose.v <<"; ini_pose.v="<<ini_pose.v<< "; av*dt="<<av*dt<< "\n";
		fileMatlab2.close();

	}

	propagated_pose.w = ini_pose.w + aw*dt;

	if(debug_velocity_file2_){
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		//std::cout << " propagated_pose.w ="<<propagated_pose.w <<"; ini_pose.w="<<ini_pose.w<< "; aw*dt="<<aw*dt<<std::endl;
		fileMatlab2 << " propagated_pose.w ="<<propagated_pose.w <<"; ini_pose.w="<<ini_pose.w<< "; aw*dt="<<aw*dt<<"\n";
		fileMatlab2.close();
	}

	if( propagated_pose.v > v_max_){
		propagated_pose.v=v_max_;
	}
	if( propagated_pose.v < -v_max_){
		propagated_pose.v=-v_max_;
	}
	if( propagated_pose.w > w_max_) {
		propagated_pose.w=w_max_;
	}
	if( propagated_pose.w < -w_max_) {
		propagated_pose.w=-w_max_;
	}

	//propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose.theta)*dt
	//		+ dt*dt/2.0*cos(ini_pose.theta)*av;

	propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose_theta)*dt
						+ dt*dt/2.0*cos(ini_pose_theta)*av;
	if(debug_velocity_file2_){
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		//std::cout << " propagated_pose.x ="<<propagated_pose.x <<"; ini_pose.x="<<ini_pose.x<< "; ini_pose.v*cos(ini_pose.theta)*dt="<<ini_pose.v*cos(ini_pose.theta)*dt<< "; dt*dt/2.0*cos(ini_pose.theta)*av="<<dt*dt/2.0*cos(ini_pose.theta)*av<<std::endl;
		fileMatlab2 << " propagated_pose.x ="<<propagated_pose.x <<"; ini_pose.x="<<ini_pose.x<< "; ini_pose.v*cos(ini_pose.theta)*dt="<<ini_pose.v*cos(ini_pose.theta)*dt<< "; dt*dt/2.0*cos(ini_pose.theta)*av="<<dt*dt/2.0*cos(ini_pose.theta)*av<<"\n";
		fileMatlab2.close();
	}

	//propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose.theta)*dt
	//+ dt*dt/2.0*sin(ini_pose.theta)*av;

	propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose_theta)*dt
			+ dt*dt/2.0*sin(ini_pose_theta)*av;



	if(debug_velocity_file2_){
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		//std::cout << " propagated_pose.y ="<<propagated_pose.y<< "; ini_pose.y="<<ini_pose.y <<"; ini_pose.v*sin(ini_pose.theta)*dt="<<ini_pose.v*sin(ini_pose.theta)*dt<<"; dt*dt/2.0*sin(ini_pose.theta)*av="<<dt*dt/2.0*sin(ini_pose.theta)*av<< std::endl;
		fileMatlab2 << " propagated_pose.y ="<<propagated_pose.y<< "; ini_pose.y="<<ini_pose.y <<"; ini_pose.v*sin(ini_pose.theta)*dt="<<ini_pose.v*sin(ini_pose.theta)*dt<<"; dt*dt/2.0*sin(ini_pose.theta)*av="<<dt*dt/2.0*sin(ini_pose.theta)*av<< "\n";
		fileMatlab2.close();
	}

	//propagated_pose.theta = ini_pose.theta + ini_pose.w*dt
	//			+ dt*dt/2.0*aw;

	propagated_pose.theta = ini_pose_theta + ini_pose.w*dt
			+ dt*dt/2.0*aw;
	if(debug_velocity_file2_){
		std::ofstream fileMatlab2;
		fileMatlab2.open (debug_file_.c_str(), std::ofstream::out | std::ofstream::app);
		//std::cout << " propagated_pose.theta="<<propagated_pose.theta<< "; ini_pose.theta="<<ini_pose.theta <<"; ini_pose.w*dt="<<ini_pose.w*dt<<";  dt*dt/2.0*aw="<< dt*dt/2.0*aw<< std::endl;
		fileMatlab2 << " propagated_pose.theta="<<propagated_pose.theta<< "; ini_pose.theta="<<ini_pose.theta <<"; ini_pose.w*dt="<<ini_pose.w*dt<<";  dt*dt/2.0*aw="<< dt*dt/2.0*aw<< "\n";
		fileMatlab2.close();
	}

	//propagated_pose.time_stamp = ini_pose.time_stamp + dt;

	/*std::cout << " propagated_pose.x ="<<propagated_pose.x  << std::endl;
	std::cout << " propagated_pose.y ="<<propagated_pose.y  << std::endl;
	std::cout << " propagated_pose.v ="<<propagated_pose.v  << std::endl;
	std::cout << " propagated_pose.w ="<<propagated_pose.w  << std::endl;
	std::cout << " propagated_pose.theta ="<<propagated_pose.theta  << std::endl;
	std::cout << " propagated_pose.time_stamp="<<propagated_pose.time_stamp << std::endl;
*/

	if(debug_antes_subgoals_entre_AKP_goals_){
		std::cout << " propagated_pose.x ="<<propagated_pose.x  << std::endl;
		std::cout << " propagated_pose.y ="<<propagated_pose.y  << std::endl;
		std::cout << " propagated_pose.v ="<<propagated_pose.v  << std::endl;
		std::cout << " propagated_pose.w ="<<propagated_pose.w  << std::endl;
		std::cout << " propagated_pose.theta ="<<propagated_pose.theta  << std::endl;
		std::cout << " propagated_pose.time_stamp="<<propagated_pose.time_stamp << std::endl;
		if(ini_pose.theta<0){

			std::cout << " INCREMENTO GIRO ANGULO ROBOT ="<< (180/3.14)*(propagated_pose.theta - (360+ini_pose.theta))   << std::endl;
		}else{
			std::cout << " INCREMENTO GIRO ANGULO ROBOT ="<< (180/3.14)*(propagated_pose.theta - ini_pose.theta)   << std::endl;
		}
		std::cout << "  propagated_pose.theta ="<<  (180/3.14)*propagated_pose.theta<<";  ini_pose.theta" <<(180/3.14)*ini_pose.theta << std::endl;
		std::cout << "  propagated_pose.w ="<<  propagated_pose.w<<";  ini_pose.w" <<ini_pose.w  << std::endl;
		std::cout << " propagated_pose.print();!!!!!!!!!!!"<< std::endl;
		propagated_pose.print();
	}

	if(test_w_max_<propagated_pose.w){
		test_w_max_=propagated_pose.w;
	}
	if(test_aw_max_<aw){
		test_aw_max_=aw;
	}

	if(debug_antes_subgoals_entre_AKP_goals_){
	std::cout << " test_w_max_="<<test_w_max_<<"; test_aw_max_"<<test_aw_max_<< std::endl;
	}

	return propagated_pose;
	//update
	//planning_trajectory_.push_back(propagated_pose);
	//planning_SpointV_trajectory_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),propagated_pose.v*sin(propagated_pose.theta)) );
}


void Crobot::prediction_propagation( double dt , unsigned int index, const Sforce &f , double max_person_vel )
{
	Spose propagated_pose,ini_pose;
	SpointV actual_group_point2;

	ini_pose = prediction_trajectory_.at(index);

	actual_group_point2=prediction_trajectory_SpointV_.at(index);//SpointV(current_pose_.x,current_pose_.y,current_pose_.time_stamp,current_pose_.v*cos(current_pose_.theta),current_pose_.v*sin(current_pose_.theta));//robot_->get_current_pointV();


	//std::cout << "3333 desired_velocity_="<<desired_velocity_ <<"; max_person_vel="<<max_person_vel<<  std::endl;

	//SpointV propagated_robot=actual_group_point2.propagate(dt,f,desired_velocity_);

	SpointV propagated_robot=actual_group_point2.propagate(dt,f,max_person_vel);

	propagated_pose=Spose(propagated_robot.x,propagated_robot.y,propagated_robot.time_stamp,propagated_robot.orientation(),propagated_robot.v());

	//kinodynamic unicycle
	/*if(debug_robot_propagation_){
		std::cout << "IN robot_propagation (propagate pose)" <<  std::endl;
	}
	Spose propagated_pose,ini_pose;
	ini_pose = prediction_trajectory_.at(index);
	//std::cout << "ini_pose.theta="<<ini_pose.theta<<  std::endl;
	//std::cout << "ini_pose.v="<<ini_pose.v<<  std::endl;

	//convert 2D forces into robot forces with non-holonomic contraints
	double av = cos(ini_pose.theta)*f.fx + sin(ini_pose.theta)*f.fy;//projection to robot pose (dot product)
	double aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy;//cross product theta x f

	if(debug_robot_propagation_){
		std::cout << "f.fx="<<f.fx<<"cos(ini_pose.theta)"<<cos(ini_pose.theta)<<"; f.fy="<<f.fy<<"sin(ini_pose.theta)="<<sin(ini_pose.theta)<< std::endl;
		std::cout <<"ini_pose.v="<<ini_pose.v<<"; av="<<av<<"; av*dt="<<av*dt<<  std::endl;
		std::cout <<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;
	}

	// calculate the proportional torque thau = k (0 - w) similar to the steering force
	aw -= 1.6*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly

	if(debug_robot_propagation_){
		std::cout <<"(after) aw="<<aw<<  std::endl;
		std::cout << "a_v_max_="<<a_v_max_<<"; a_w_max_="<<a_w_max_<<"; a_v_break_="<<a_v_break_<<  std::endl;
		std::cout << "v_max_="<<v_max_<<"; w_max_="<<w_max_<<  std::endl;
	}
	// check for forces validity
	if( av*ini_pose.v < 0.001 )//breaking configuration if different sign v and a
	{
		if( av > a_v_break_) av=a_v_break_;
		if( av < -a_v_break_) av=-a_v_break_;
	}
	else // not breaking or 0.0
	{
		if( av > a_v_max_) av=a_v_max_;
		if( av < -a_v_max_) av=-a_v_max_;
	}
	if( aw > a_w_max_) aw=a_w_max_;
	if( aw < -a_w_max_) aw=-a_w_max_;

	if(debug_robot_propagation_){
		std::cout <<"(after if) av="<<av<<  std::endl;
		std::cout <<"(after if) aw="<<aw<<  std::endl;
	}

	// update velocities
	propagated_pose.v = ini_pose.v + av*dt;
	if(debug_robot_propagation_){
		std::cout << "propagated_pose.v=(ini_pose.v + av*dt)="<<propagated_pose.v<<  std::endl;
	}
	propagated_pose.w = ini_pose.w + aw*dt;
	if( propagated_pose.v > v_max_) propagated_pose.v=v_max_;
	if( propagated_pose.v < -v_max_) propagated_pose.v=-v_max_;
	if( propagated_pose.w > w_max_) propagated_pose.w=w_max_;
	if( propagated_pose.w < -w_max_) propagated_pose.w=-w_max_;
	propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose.theta)*dt
			+ dt*dt/2.0*cos(ini_pose.theta)*av;
	propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose.theta)*dt
			+ dt*dt/2.0*sin(ini_pose.theta)*av;
	propagated_pose.theta = ini_pose.theta + ini_pose.w*dt
			+ dt*dt/2.0*aw;;
	propagated_pose.time_stamp = ini_pose.time_stamp + dt;

	//update
	//std::cout <<"propagated_pose person companion"<<  std::endl;
	//propagated_pose.print();
*/

	prediction_trajectory_.push_back(propagated_pose);
	prediction_trajectory_SpointV_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
				propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
				propagated_pose.v*sin(propagated_pose.theta)) );
	if(debug_robot_propagation_){
		std::cout << " ROBOT pose and Spoint prediction trajectory:  "<< std::endl;

		prediction_trajectory_.back().print();
		prediction_trajectory_SpointV_.back().print();
	}





	/*if(debug_prediction_propagation_){
		std::cout << "IN Cperson_behavior::prediction_propagation prediction_trajectory_.emty()"<<prediction_trajectory_.empty()<< std::endl;
		std::cout <<" dt="<<dt<<";desired_velocity_="<<desired_velocity_<<"; force.fx"<<force.fx<<"; force.fy="<<force.fy<<"; prediction_trajectory_.at(index).print():"<< std::endl;
		prediction_trajectory_.at(index).print();
	}

	prediction_trajectory_.push_back( prediction_trajectory_.at(index).propagate(dt,force,desired_velocity_) );*/
}



void Crobot::prediction_propagation2_only_vel( double dt , unsigned int index, const Sforce &f , double distance_person_t_x,double distance_person_t_y,double vel_person_x,double vel_person_y , double max_person_vel)
{
	Spose propagated_pose,ini_pose;
	SpointV actual_group_point2;
	SpointV actual_group_point3;

	ini_pose = prediction_trajectory_only_vel_.at(index);

	actual_group_point2=prediction_trajectory_only_vel_SpointV_.at(index);//SpointV(current_pose_.x,current_pose_.y,current_pose_.time_stamp,current_pose_.v*cos(current_pose_.theta),current_pose_.v*sin(current_pose_.theta));//robot_->get_current_pointV();
	actual_group_point3=prediction_trajectory_SpointV_.at(index);

	//std::cout << "actual_group_point2.x="<<actual_group_point2.x<<"; actual_group_point2.y="<<actual_group_point2.y<<  std::endl;

	//std::cout << "actual_group_point3.x="<<actual_group_point3.x<<"; actual_group_point3.y="<<actual_group_point3.y<<  std::endl;

	//SpointV propagated_robot=actual_group_point2.propagate2_comp_only_vel(dt,f,desired_velocity_,distance_person_t_x,distance_person_t_y,vel_person_x,vel_person_y);
	SpointV propagated_robot;
	if(max_person_vel!=0.0){
		propagated_robot=actual_group_point2.propagate2_comp_only_vel(dt,f,max_person_vel,distance_person_t_x,distance_person_t_y,vel_person_x,vel_person_y);
	}else{
		propagated_robot=actual_group_point2.propagate2_comp_only_vel(dt,f,desired_velocity_,distance_person_t_x,distance_person_t_y,vel_person_x,vel_person_y);

	}

	propagated_pose=Spose(propagated_robot.x,propagated_robot.y,propagated_robot.time_stamp,propagated_robot.orientation(),propagated_robot.v());

	//kinodynamic unicycle
	/*if(debug_robot_propagation_){
		std::cout << "IN robot_propagation (propagate pose)" <<  std::endl;
	}
	Spose propagated_pose,ini_pose;
	ini_pose = prediction_trajectory_.at(index);
	//std::cout << "ini_pose.theta="<<ini_pose.theta<<  std::endl;
	//std::cout << "ini_pose.v="<<ini_pose.v<<  std::endl;

	//convert 2D forces into robot forces with non-holonomic contraints
	double av = cos(ini_pose.theta)*f.fx + sin(ini_pose.theta)*f.fy;//projection to robot pose (dot product)
	double aw = -sin(ini_pose.theta)*f.fx + cos(ini_pose.theta)*f.fy;//cross product theta x f

	if(debug_robot_propagation_){
		std::cout << "f.fx="<<f.fx<<"cos(ini_pose.theta)"<<cos(ini_pose.theta)<<"; f.fy="<<f.fy<<"sin(ini_pose.theta)="<<sin(ini_pose.theta)<< std::endl;
		std::cout <<"ini_pose.v="<<ini_pose.v<<"; av="<<av<<"; av*dt="<<av*dt<<  std::endl;
		std::cout <<"ini_pose.w="<<ini_pose.w<<"; aw="<<aw<<"; aw*dt="<<aw*dt<<  std::endl;
	}

	// calculate the proportional torque thau = k (0 - w) similar to the steering force
	aw -= 1.6*ini_pose.w;//TODO this dampening parameter depends on the velocity v, w! careful if it changes significantly

	if(debug_robot_propagation_){
		std::cout <<"(after) aw="<<aw<<  std::endl;
		std::cout << "a_v_max_="<<a_v_max_<<"; a_w_max_="<<a_w_max_<<"; a_v_break_="<<a_v_break_<<  std::endl;
		std::cout << "v_max_="<<v_max_<<"; w_max_="<<w_max_<<  std::endl;
	}
	// check for forces validity
	if( av*ini_pose.v < 0.001 )//breaking configuration if different sign v and a
	{
		if( av > a_v_break_) av=a_v_break_;
		if( av < -a_v_break_) av=-a_v_break_;
	}
	else // not breaking or 0.0
	{
		if( av > a_v_max_) av=a_v_max_;
		if( av < -a_v_max_) av=-a_v_max_;
	}
	if( aw > a_w_max_) aw=a_w_max_;
	if( aw < -a_w_max_) aw=-a_w_max_;

	if(debug_robot_propagation_){
		std::cout <<"(after if) av="<<av<<  std::endl;
		std::cout <<"(after if) aw="<<aw<<  std::endl;
	}

	// update velocities
	propagated_pose.v = ini_pose.v + av*dt;
	if(debug_robot_propagation_){
		std::cout << "propagated_pose.v=(ini_pose.v + av*dt)="<<propagated_pose.v<<  std::endl;
	}
	propagated_pose.w = ini_pose.w + aw*dt;
	if( propagated_pose.v > v_max_) propagated_pose.v=v_max_;
	if( propagated_pose.v < -v_max_) propagated_pose.v=-v_max_;
	if( propagated_pose.w > w_max_) propagated_pose.w=w_max_;
	if( propagated_pose.w < -w_max_) propagated_pose.w=-w_max_;
	propagated_pose.x = ini_pose.x + ini_pose.v*cos(ini_pose.theta)*dt
			+ dt*dt/2.0*cos(ini_pose.theta)*av;
	propagated_pose.y = ini_pose.y + ini_pose.v*sin(ini_pose.theta)*dt
			+ dt*dt/2.0*sin(ini_pose.theta)*av;
	propagated_pose.theta = ini_pose.theta + ini_pose.w*dt
			+ dt*dt/2.0*aw;;
	propagated_pose.time_stamp = ini_pose.time_stamp + dt;

	//update
	//std::cout <<"propagated_pose person companion"<<  std::endl;
	//propagated_pose.print();
*/

	prediction_trajectory_only_vel_.push_back(propagated_pose);
	prediction_trajectory_only_vel_SpointV_.push_back( SpointV_cov(propagated_pose.x,propagated_pose.y,
				propagated_pose.time_stamp, propagated_pose.v*cos(propagated_pose.theta),
				propagated_pose.v*sin(propagated_pose.theta)) );
	if(debug_robot_propagation_){
		std::cout << " ROBOT pose and Spoint prediction trajectory:  "<< std::endl;

		prediction_trajectory_only_vel_.back().print();
		prediction_trajectory_only_vel_SpointV_.back().print();
	}


}



void Crobot::clear_prediction_trajectory()
{
	prediction_trajectory_.clear();
	prediction_trajectory_.push_back( current_pose_ );

	prediction_trajectory_SpointV_.clear();
	prediction_trajectory_SpointV_.push_back( current_pointV_ );



	//has_copied_propagation_ = false;
}


void Crobot::clear_prediction_trajectory_onlyV()
{
	prediction_trajectory_only_vel_.clear();
	prediction_trajectory_only_vel_.push_back( current_pose_ );

	prediction_trajectory_only_vel_SpointV_.clear();
	prediction_trajectory_only_vel_SpointV_.push_back( current_pointV_ );
	//has_copied_propagation_ = false;
}
/*
void Crobot::clear_prediction_pose_zanlungo_vec()
{
	omega_.clear();
	force_act_x_.clear();
		    force_act_y_.clear();
		    nextv_.clear();

		    selft_v_x_.clear();
		    selft_v_y_.clear();
		    rad_acc_.clear();
}*/


