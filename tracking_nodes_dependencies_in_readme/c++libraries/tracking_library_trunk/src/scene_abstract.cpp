/*
 * scene_abstract.cpp
 *
 *  Created on: Jul 9, 2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration).
 *      Author: gferrer
 *      Author: Ely Repiso (Modifications for people tracking mht)
 *
 *      License (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
 *
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
 
#include "scene_abstract.h"
#include <stdio.h>
#include <iostream>
#include <math.h>


Cscene_abstract::Cscene_abstract( Cperson_abstract::filtering_method filter,
		Cperson_abstract::update_person_method update_method,
		Cperson_abstract::force_type type):
robot_(NULL), dt_(0.1) , now_(0.0) , read_laser_obstacle_success_(false), read_force_map_success_(false), read_destination_map_success_(false),
scene_force_type_(type), filtering_method_(filter), update_method_(update_method),
robot_in_the_scene_(false),
sim_bool_(false),
max_d_to_detect_laser_obs_(20), // TODO: ojo, devolver al 20.
max_asos_point_to_person_(100),
debug_update_person_companion_(false),
debug_find_person_(false)
{

	switch( scene_force_type_ )
	{
	case Cperson_abstract::Collision_Prediction:
		// Zanlungo collision prediction parameters , {k,lambda,A,B,d}
		social_forces_param_to_person_ = {1.52, 0.29,1.13,0.71,0.0};
		//Person-Robot Spherical parameters obtained using our optimization method
		social_forces_param_to_robot_ = {1.52, 0.29,1.13,0.71,0.0};
		//set_social_force_parameters_person_robot( force_params_to_vector(2.3, 0.59,2.66,0.79,0.4) );
		//Obstacle spherical parameters obtained using our optimization method
		social_forces_param_to_obs_ = {2.3, 1.0,10.0,0.1,0.2};
		break;
	case Cperson_abstract::Elliptical:
		//Default Zanlungo Elliptical parameters
		social_forces_param_to_person_ = {1.19, 0.08,1.33,0.34,1.78};
		//Person-Robot Spherical parameters obtained using our optimization method
		social_forces_param_to_robot_ = {2.3, 0.59,2.66,0.79,0.4};
		//Obstacle spherical parameters obtained using our optimization method
		social_forces_param_to_obs_ = {2.3, 1.0,10.0,0.1,0.2};
		break;
	case Cperson_abstract::Spherical:
		//TODO set as default the paramters calculated in ICRA'2104 (Balanced behavior)
		//Default Zanlungo Spherical parameters (2.3, 0.08,1.33,0.64,0.16)
		social_forces_param_to_person_ = {4.9, 1.0,10.0,0.64,0.16};//B=0.34, changed to 0.64
		//Person-Robot Spherical parameters obtained using our optimization method
		social_forces_param_to_robot_ = {2.3, 0.59,2.66,0.79,0.4};
		//Obstacle spherical parameters obtained using our optimization method
		social_forces_param_to_obs_ = {2.3, 1.0,10.0,0.6,0.2};
		break;
	}



}

Cscene_abstract::~Cscene_abstract()
{

}

void Cscene_abstract::set_destinations( std::vector<Sdestination>& dest )
{
	destinations_ = dest;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		(*iit)->set_destinations( dest );
	}
	// robot destinations are not required, are set if planned
}

Cperson_abstract* Cscene_abstract::add_person( unsigned int id )
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}

	return add_person_container(id,iit);//virtual function adding the specific person container
}

void Cscene_abstract::update_other_person_companion( unsigned int id, SpointV_cov in_other_person_companion_point )
{
	/*std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if ( (*iit)->get_id()  == id ) //interrupts if the id is greater for ordering the list
			{
				// update person and breack! => creo que iria mejor un for de la lista normal...

				break;
			}
			iit++;
		}
	}*/

	 for( auto iit: person_list_ )
	{

		if(iit->get_id()==id){
			iit->set_current_pointV(in_other_person_companion_point);
			// hay que cambiarle el punto actual con el que predice y etc, por el punto que le entra de la odometria del otro robot.
		}
	}


	//return add_person_container(id,iit);//virtual function adding the specific person container
}

void Cscene_abstract::remove_person( unsigned int id )
{
	std::list<Cperson_abstract*>::iterator iit;
	Cperson_abstract* person;
	if (find_person( id,  &person, iit ))
	{
		delete person;
		person_list_.erase(  iit );

	}
}

void Cscene_abstract::update_robot(Spose observation, bool case_zanlungo)
{

	now_ = observation.time_stamp;
	//std::cout << ";IMPORTANTE!!! 7777 333 => observation.time_stamp="<<observation.time_stamp<<"; now_="<<now_<<"; robot_in_the_scene_="<<robot_in_the_scene_<<"; case_zanlungo="<<case_zanlungo<< std::endl;
	if ( robot_in_the_scene_ )
	{
		robot_->add_pose( observation );

		if(case_zanlungo){
			// if Zanlungo.

			robot_->set_current_pose(observation);
			//std::cout << ";robot_->get_current_pose.Time_stamp="<<robot_->get_current_pose().time_stamp<< std::endl;
			robot_->set_current_pointV(SpointV(observation.x,observation.y,observation.time_stamp,observation.v*cos(observation.theta),observation.v*sin(observation.theta)));
		}

	}
	else
	{
		robot_in_the_scene_ = true;
		robot_ = new Crobot(0,Crobot::Differential,scene_force_type_);
		robot_->add_pose( observation );
		if(case_zanlungo){
			// if Zanlungo.
			robot_->set_current_pose(observation);
			robot_->set_current_pointV(SpointV(observation.x,observation.y,observation.time_stamp,observation.v*cos(observation.theta),observation.v*sin(observation.theta)));

		}

	}

	//std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!111 update_robot !!!!!!!!!!!!!!!!!!1111"<< std::endl;
	//std::cout << "observation.w="<<observation.w<<"; observation.v="<<observation.v<<"; robot.w="<<robot_->get_current_pose().w<<"; robot.v="<<robot_->get_current_pose().v<< std::endl;
}

void Cscene_abstract::update_person_companion(Spose observation, SpointV_cov obs_spoint)
{
	if(debug_update_person_companion_){
		std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!111 update_person_companion !!!!!!!!!!!!!!!!!!1111"<< std::endl;
	}

	//now_ = observation.time_stamp;
	if ( person_companion_in_the_scene_ )
	{
		if(debug_update_person_companion_){
			std::cout << "(IN update_person_companion) update person companion "<< std::endl;
		}

		person_companion_->add_pose( observation );
		person_companion_->set_current_pointV(obs_spoint);
		person_companion_->set_current_pose(observation);
	}
	else
	{
		if(debug_update_person_companion_){
			std::cout << "(IN update_person_companion) new person companion "<< std::endl;
		}
		person_companion_in_the_scene_ = true;
		person_companion_ = new Crobot(0,Crobot::Differential,scene_force_type_);
		person_companion_->add_pose( observation );
		person_companion_->set_current_pointV(obs_spoint);
		person_companion_->set_current_pose(observation);
	}
}

bool Cscene_abstract::find_person(unsigned int id)
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			// only returns persons (robot wont be returned if sought)
			if( ((*iit)->get_id()  == id) && (((*iit)->get_person_type() == Cperson_abstract::Person)||((*iit)->get_person_type() == Cperson_abstract::Person_companion)) )
			{
				return true;
			}
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}
	return false;
}

bool Cscene_abstract::find_person(unsigned int id , Cperson_abstract** person) // (ely) suelo entrar en esta find person!!!
{
	if(debug_find_person_){
	 std::cout <<" (entro en find person 2) person_list_.empty()="<<person_list_.empty() << std::endl;
	}

	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();

	if(debug_find_person_){
	 std::cout <<" (entro en find person 2) person_list_.empty()="<<person_list_.empty() << std::endl;
	}

	// std::cout <<" person_list_.empty()="<<person_list_.empty() << std::endl;
	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if( ((*iit)->get_id()  == id) && (((*iit)->get_person_type() == Cperson_abstract::Person)||((*iit)->get_person_type() == Cperson_abstract::Person_companion)))
			{
				*person = *iit;
				return true;
			}
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}
	if(debug_find_person_){
	 std::cout <<" (salgo de find person 2) person_list_.empty()="<<person_list_.empty() << std::endl;
	}
	return false;
}

bool Cscene_abstract::find_person(unsigned int id , Cperson_abstract** person,
		std::list<Cperson_abstract*>::iterator& it)
{
	std::list<Cperson_abstract*>::iterator iit = person_list_.begin();


	if ( !person_list_.empty())
	{
		while( iit != person_list_.end())
		{
			if( ((*iit)->get_id()  == id) && (((*iit)->get_person_type() == Cperson_abstract::Person)||((*iit)->get_person_type() == Cperson_abstract::Person_companion)))
			{
				*person = *iit;
				it = iit;
				return true;
			}
			if ( (*iit)->get_id()  > id ) //interrupts if the id is greater for ordering the list
			{
				break;
			}
			iit++;
		}
	}

	return false;
}

void Cscene_abstract::print()
{
	std::cout << "Cscene_abstract::print() person_list_.empty="<<person_list_.empty()<< std::endl;

	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		(*iit)->print_dest();
		(*iit)->print();
	}
}

void Cscene_abstract::read_laser_scan( const std::vector<Spoint>&  laser_scan , bool person_simulation)
{

	//std::cout << "Entro en read_laser_scan !!!" << std::endl;
	/*if(sim_){
		std::cout << "sim true !!!" << std::endl;
	}else{
		std::cout << "sim false !!!" << std::endl;
	}
	if(sim_bool_){
			std::cout << "sim_bool_ true !!!" << std::endl;
		}else{
			std::cout << "sim_bool_ false !!!" << std::endl;
		}*/

	Spose act_robot_pose;
	double distance_robot_obstacle;

	if(!sim_bool_){
		act_robot_pose= robot_->get_current_pose();
	}
	//std::cout << "act_robot_pose.x="<<act_robot_pose.x << std::endl;
	//std::cout << "act_robot_pose.y="<<act_robot_pose.y << std::endl;

	//TODO take into account both back and front laser scans
	laser_obstacle_list_.clear();
	Spoint before_point;
	unsigned int num_of_near_static_obst_=0;
	bool first_time=true;

	for ( Spoint p : laser_scan )
	{

		if(!sim_bool_){
			distance_robot_obstacle=sqrt(pow(act_robot_pose.x-p.x,2)+pow(act_robot_pose.y-p.y,2));
		//	std::cout <<"p.x="<<p.x<<"; p.y ="<<p.y<< "; distance_ robot-laser_point ="<<distance_robot_obstacle << std::endl;
		}
		//filter laser points corresponding to persons and obstacles
		if ( !point_corresponds_person(p) && !point_belongs_to_laser_obstacle(p,person_simulation) )
		{
			if(!sim_bool_){
				if(distance_robot_obstacle<max_d_to_detect_laser_obs_){
					//std::cout <<"insert point!"<< std::endl;
				//new cluster scale-invariant laser points is created
					laser_obstacle_list_.push_back( p );
					if(first_time){
						first_time=false;
						num_of_near_static_obst_=1;
					}
				}
			}else{
				if(first_time){
					first_time=false;
					num_of_near_static_obst_=1;
				}
				laser_obstacle_list_.push_back( p );
			}
		}


		///// INI TO estimate distance between contiguous laser points:
		double dro = p.distance2( robot_->get_current_pointV()  );//distance to robot
		double dist_near_laser_points;
				if( dro < 1.0  )//calculates basic distance to robot, near distance, the thr is lower
				{
					//dist_thr = 0.36;//0.1 [m]=sqrt(0.01) = 0.1[m] is considered obstacle o sqrt(0.04) = 0.2 [m]
					//dist_thr = 0.09;//sqrt(0.09) = 0.3[m] is considered obstacle (Lo que tenia antes Gonzalo)
					if(sim_){
						dist_near_laser_points = 0.01;
						//std::cout << "SIM CASE if(d2 < 1.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
					}else{ // real robot case.
						dist_near_laser_points = 0.2;//sqrt(0.01) = 0.1[m] is considered obstacle antes 0.09 == 0.3 m
						//std::cout << "REAL CASE if(d2 < 1.0) => dist_thr = 0.09=sqrt(0.09) = 0.3[m] is considered obstacle"<< std::endl;
					}

				}
				else if( dro < 16.0 )//4 [m]
				{
					//dist_thr = 0.36;//0.1 [m]=sqrt(0.01) = 0.1[m] is considered obstacle o sqrt(0.04) = 0.2 [m]
					//dist_thr = 0.09;

					if(sim_){
						dist_near_laser_points = 0.01; //
						//std::cout << "SIM CASE if(d2 < 16.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
					}else{ // real robot case.
						dist_near_laser_points = 0.2;//sqrt(0.1) = 0.32[m] is considered obstacle
						//std::cout << "REAL CASE if(d2 < 16.0) => dist_thr = 0.2=sqrt(0.2) = 0.45[m] is considered obstacle"<< std::endl;
					}
					//dist_thr = 0.2;//4 [m]=sqrt(0.16) = 0.45[m] is considered obstacle

				}
				else
				{
					//dist_thr = 0.36;//sqrt(0.09) = 0.6[m] is considered obstacle
					//dist_thr = 0.09;

					if(sim_){
						dist_near_laser_points = 0.01;
						//std::cout << "SIM CASE if(d2 > 16.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
					}else{ // real robot case.
						dist_near_laser_points = 0.36;//sqrt(0.09) = 0.6[m] is considered obstacle
						//std::cout << "REAL CASE if(d2 > 16.0) => dist_thr = 0.36=sqrt(0.09) = 0.6[m] is considered obstacle"<< std::endl;
					}
					//std::cout << "else => dist_thr =  0.36=sqrt(0.09) = 0.6[m] is considered obstacle"<< std::endl;
				}

		////////////// Fin TO estimate distance between contiguous laser points:

		if(!first_time){

			if(before_point.distance2(p)>dist_near_laser_points){
				num_of_near_static_obst_++;
			}

		}

		before_point=p;

	}


	//std::cout << "Best approximation of num_of_near_static_obst_="<<num_of_near_static_obst_<< std::endl;
	//std::cout <<"sim_bool_="<<sim_bool_<<"; max_d_to_detect_laser_obs_="<<max_d_to_detect_laser_obs_<<"; laser_obstacle_list_.size()="<<laser_obstacle_list_.size()<< std::endl;

	/*if(!laser_obstacle_list_.empty()){
		std::cout << "laser_obstacle_list_[0]:"<< std::endl;
		laser_obstacle_list_[0].print();
	}*/
	//std::cout << "max_d_to_detect_laser_obs_="<<max_d_to_detect_laser_obs_<< std::endl;
	//std::cout << "laser_obstacle_list_.size() ="<<laser_obstacle_list_.size()<< std::endl;

	read_laser_obstacle_success_ = true;//flag to indicate that indeed we can make use of laser information regarding obstacles.
}

void Cscene_abstract::read_laser_scan_person_companion_akp_planner( const std::vector<Spoint>&  laser_scan , bool person_simulation)
{

	//std::cout << "Entro en read_laser_scan !!!" << std::endl;
	Spose act_robot_pose;
	double distance_robot_obstacle;

	if(!sim_bool_){
		act_robot_pose= person_companion_->get_current_pose();
	}else{
		act_robot_pose= person_companion_->get_current_pose();
	}
	//std::cout << "act_robot_pose.x="<<act_robot_pose.x << std::endl;
	//std::cout << "act_robot_pose.y="<<act_robot_pose.y << std::endl;

	//TODO take into account both back and front laser scans
	laser_obstacle_list_.clear();
	for ( Spoint p : laser_scan )
	{

		if(!sim_bool_){
			distance_robot_obstacle=sqrt(pow(act_robot_pose.x-p.x,2)+pow(act_robot_pose.y-p.y,2));
		//	std::cout <<"p.x="<<p.x<<"; p.y ="<<p.y<< "; distance_ robot-laser_point ="<<distance_robot_obstacle << std::endl;
		}else{
			distance_robot_obstacle=sqrt(pow(act_robot_pose.x-p.x,2)+pow(act_robot_pose.y-p.y,2));
		}
		//filter laser points corresponding to persons and obstacles
		if ( !point_corresponds_person(p) && !point_belongs_to_laser_obstacle(p,person_simulation) )
		{
			if(!sim_bool_){
				if(distance_robot_obstacle<max_d_to_detect_laser_obs_){
					//std::cout <<"insert point!"<< std::endl;
				//new cluster scale-invariant laser points is created
					laser_obstacle_list_.push_back( p );
				}
			}else{
				if(distance_robot_obstacle<max_d_to_detect_laser_obs_){
					laser_obstacle_list_.push_back( p );
				}
			}
		}
	}

	/*if(!laser_obstacle_list_.empty()){
		std::cout << "laser_obstacle_list_[0]:"<< std::endl;
		laser_obstacle_list_[0].print();
	}*/
	//std::cout << "max_d_to_detect_laser_obs_="<<max_d_to_detect_laser_obs_<< std::endl;
	//std::cout << "laser_obstacle_list_.size() ="<<laser_obstacle_list_.size()<< std::endl;
	//std::cout <<"[PERSON COMPANION] sim_bool_="<<sim_bool_<<"; max_d_to_detect_laser_obs_="<<max_d_to_detect_laser_obs_<<"; laser_obstacle_list_.size()="<<laser_obstacle_list_.size()<< std::endl;

	read_laser_obstacle_success_ = true;//flag to indicate that indeed we can make use of laser information regarding obstacles.
}



void Cscene_abstract::read_laser_scan_companion( const std::vector<Spoint>&  laser_scan , bool person_simulation)
{

	//std::cout << "Entro en read_laser_scan !!!" << std::endl;
	Spose act_robot_pose;
	double distance_robot_obstacle;
	std::vector<Spoint> laser_obstacle_list_act;//list of obstacles seen by laser
	laser_obstacle_list_act.clear();

	if(!sim_bool_){
		act_robot_pose= robot_->get_current_pose();
	}
	//std::cout << "act_robot_pose.x="<<act_robot_pose.x << std::endl;
	//std::cout << "act_robot_pose.y="<<act_robot_pose.y << std::endl;

	//TODO take into account both back and front laser scans
	laser_obstacle_list_.clear();
	real_person_list_.clear();

	for( Cperson_abstract* iit : person_list_ )
	{
		unsigned int count_act_person=0;

		for ( Spoint p : laser_scan ){

			if( p.distance( iit->get_current_pointV() ) < 1 )//sqrt(0.3) = 0.6[m] is considered person (for companion Ely, pq asociaba mal personas en obstaculos)
			{
				count_act_person++;
			}

		}

		if(count_act_person<max_asos_point_to_person_){
			real_person_list_.push_back(iit);
			//std::cout << "real person id="<<iit->get_id() << std::endl;
		}

	}


	for ( Spoint p : laser_scan )
	{

		if(!sim_bool_){
			distance_robot_obstacle=sqrt(pow(act_robot_pose.x-p.x,2)+pow(act_robot_pose.y-p.y,2));
			//	std::cout <<"p.x="<<p.x<<"; p.y ="<<p.y<< "; distance_ robot-laser_point ="<<distance_robot_obstacle << std::endl;
		}
			//filter laser points corresponding to persons and obstacles
		if ( !point_corresponds_person_companion(p) && !point_belongs_to_laser_obstacle(p,person_simulation) )
		{
			if(!sim_bool_){
				if(distance_robot_obstacle<max_d_to_detect_laser_obs_){
						//std::cout <<"insert point!"<< std::endl;
					//new cluster scale-invariant laser points is created
					laser_obstacle_list_.push_back( p );
				}
			}else{
				laser_obstacle_list_.push_back( p );
			}
		}
	}

	//std::cout << "max_d_to_detect_laser_obs_="<<max_d_to_detect_laser_obs_<< std::endl;
		//std::cout << "laser_obstacle_list_.size() ="<<laser_obstacle_list_.size()<< std::endl;

	read_laser_obstacle_success_ = true;//flag to indicate that indeed we can make use of laser information regarding obstacles.
}


bool Cscene_abstract::point_corresponds_person( Spoint laser_point )
{
	for( Cperson_abstract* iit : person_list_ )
	{

		//std::cout << "; person_list_.id="<<iit->get_id()<<"; person_list.size()="<<person_list_.size()<<"; laser_point.distance( iit->get_current_pointV() ="<<laser_point.distance( iit->get_current_pointV() )<< std::endl;
		//if( laser_point.distance2( iit->get_current_pointV() ) < 1.0 )//sqrt(1) = 1[m] is considered person
		if( laser_point.distance( iit->get_current_pointV() ) < 1.5 )//sqrt(0.3) = 0.6[m] is considered person (for companion Ely, pq asociaba mal personas en obstaculos)
		{  // en simulacion es < 0.5!!!

			//for(Spoint iit2 : laser_obstacle_list_){
			//	double df=iit2.distance(iit->get_current_pointV());
			//	if(df< 0.5){
			//		return false;
			//	}else{
					return true;
			//	}
			//}
		}
	}
	return false;
}


bool Cscene_abstract::point_corresponds_person_companion( Spoint laser_point )
{

	for( Cperson_abstract* iit : real_person_list_ )
		{
			//if( laser_point.distance2( iit->get_current_pointV() ) < 1.0 )//sqrt(1) = 1[m] is considered person
			if( laser_point.distance( iit->get_current_pointV() ) < 0.8 )//sqrt(0.3) = 0.6[m] is considered person (for companion Ely, pq asociaba mal personas en obstaculos)
			{ // en simulacion es < 0.5!!!

				//for(Spoint iit2 : laser_obstacle_list_){
				//	double df=iit2.distance(iit->get_current_pointV());
				//	if(df< 0.5){
				//		return false;
				//	}else{
						return true;
				//	}
				//}
			}
		}
		return false;



}


bool Cscene_abstract::point_belongs_to_laser_obstacle( Spoint p, bool person_simulation)
{

	double dist_thr,d2;
	if(!person_simulation){
		// CASE obstacles to robot.
		d2 = p.distance2( robot_->get_current_pointV()  );//distance to robot
		if( d2 < 1.0  )//calculates basic distance to robot, near distance, the thr is lower
		{
			//dist_thr = 0.36;//0.1 [m]=sqrt(0.01) = 0.1[m] is considered obstacle o sqrt(0.04) = 0.2 [m]
			//dist_thr = 0.09;//sqrt(0.09) = 0.3[m] is considered obstacle (Lo que tenia antes Gonzalo)
			if(sim_){
				//dist_thr = 0.01; sim
				dist_thr=0.09;
				//std::cout << "SIM CASE if(d2 < 1.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
			}else{ // real robot case.
				dist_thr = 0.09;//sqrt(0.09) = 0.3[m] is considered obstacle
				//std::cout << "REAL CASE if(d2 < 1.0) => dist_thr = 0.09=sqrt(0.09) = 0.3[m] is considered obstacle"<< std::endl;
			}

		}
		else if( d2 < 16.0 )//4 [m]
		{
			//dist_thr = 0.36;//0.1 [m]=sqrt(0.01) = 0.1[m] is considered obstacle o sqrt(0.04) = 0.2 [m]
			//dist_thr = 0.09;

			if(sim_){
				//dist_thr = 0.01; // sim
				dist_thr = 0.2;
				//std::cout << "SIM CASE if(d2 < 16.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
			}else{ // real robot case.
				dist_thr = 0.2;//sqrt(0.16) = 0.45[m] is considered obstacle
				//std::cout << "REAL CASE if(d2 < 16.0) => dist_thr = 0.2=sqrt(0.2) = 0.45[m] is considered obstacle"<< std::endl;
			}
			//dist_thr = 0.2;//4 [m]=sqrt(0.16) = 0.45[m] is considered obstacle

		}
		else
		{
			//dist_thr = 0.36;//sqrt(0.09) = 0.6[m] is considered obstacle
			//dist_thr = 0.09;


			if(sim_){
				//dist_thr = 0.01;sim
				dist_thr = 0.36;
				//std::cout << "SIM CASE if(d2 > 16.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
			}else{ // real robot case.
				dist_thr = 0.36;//sqrt(0.09) = 0.6[m] is considered obstacle
				//std::cout << "REAL CASE if(d2 > 16.0) => dist_thr = 0.36=sqrt(0.09) = 0.6[m] is considered obstacle"<< std::endl;
			}
			//std::cout << "else => dist_thr =  0.36=sqrt(0.09) = 0.6[m] is considered obstacle"<< std::endl;
		}
		//std::cout << "entro en IF dist_thr="<<dist_thr<< std::endl;
	}else{
		// CASE obstacles to person companion simulated.
		//std::cout << "entro en else dist_thr="<<dist_thr<< std::endl;


		if(sim_){
			//dist_thr = 0.01; // sim
			dist_thr = 0.09;
			//std::cout << "Person_companion SIM CASE if(d2 > 16.0) => dist_thr = 0.01=sqrt(0.01) = 0.1[m] is considered obstacle"<< std::endl;
		}else{ // real robot case. // NEVER enter in this case!
			dist_thr = 0.09;//sqrt(0.09) = 0.3[m] is considered obstacle
		}

		dist_thr = 0.36; // for zanlungo model.
	}
	for( Spoint o : laser_obstacle_list_ )
	{
		if ( o.distance2( p )  <  dist_thr )
			return true;
	}
	return false;
}

Sforce Cscene_abstract::force_objects_laser_int( Cperson_abstract* person)
{
	Sforce force_res;
	for( Spoint iit : laser_obstacle_list_)
	{
		//there is a list for persons and another for robot(s). This function is for obstacles
		if( person->get_current_pointV().distance2( iit ) < 25.0)//square distance 5^2
		{
			force_res += person->force_sphe( iit , this->get_sfm_int_params( person ) );
		}
	}
	return force_res;

}

Sforce Cscene_abstract::force_persons_int( Cperson_abstract* center , bool if_robot_is_person_companion )
{
	//std::cout << " (1) !!! force_persons_int" << std::endl;

	Sforce force_res;

	if(if_robot_is_person_companion){
		//std::cout << " (1) !!! if_robot_is_person_companion" << std::endl;
		for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
		{
			//std::cout << " (2) !!! force_persons_int" << std::endl;
			//there is a list for persons and another for robot(s). This function is for persons
			if( *center != *(*iit) && center->get_current_pointV().distance( (*iit)->get_current_pointV() ) < 15.0)
			{
				//std::cout << " (3) !!! force_persons_int; center.type="<<center->get_person_type()<<"; *iit.type="<<(*iit)->get_person_type() <<"; (*iit)->get_id()="<<(*iit)->get_id()<< std::endl;
				if((*iit)->get_id()!=1){ // todo, poner el id de la person companion desde el planner.
					force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit) );
				}
				//std::cout << " (4) !!! force_persons_int" << std::endl;
			}
		}
	}else{
		//std::cout << " (1) !!! else =>if_robot_is_person_companion" << std::endl;
		for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
		{
			//std::cout << " (2) !!! force_persons_int" << std::endl;
			//there is a list for persons and another for robot(s). This function is for persons
			if( *center != *(*iit) && center->get_current_pointV().distance( (*iit)->get_current_pointV() ) < 15.0)
			{
				//std::cout << " (3) !!! force_persons_int; center.type="<<center->get_person_type()<<"; *iit.type="<<(*iit)->get_person_type() << std::endl;
				force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit) );
				//std::cout << " (4) !!! force_persons_int" << std::endl;
			}
		}
	}

	//std::cout << " (out) !!! force_persons_int" << std::endl;



	return force_res;

}

Sforce Cscene_abstract::force_persons_int_companion( Cperson_abstract* center , unsigned int id_person_companion, bool person_reactive, Cperson_abstract::companion_reactive type_rec_akp)
{
	Sforce force_res;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		//there is a list for persons and another for robot(s). This function is for persons
		if( *center != *(*iit) && center->get_current_pointV().distance( (*iit)->get_current_pointV() ) < 15.0)
		{
			if((*iit)->get_id()!=id_person_companion){
				force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit) );
			}else if(((*iit)->get_id()==id_person_companion)&&person_reactive){
				force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit),&robot_->get_current_pointV(),type_rec_akp);
			}
		}
	}
	return force_res;

}

Sforce Cscene_abstract::force_persons_int2( Cperson_abstract* center, std::vector<int> ids_of_persons_in_group ) // made by ely (companion, no take into accound the person companion of the group force.)
{
	// para tener grupos de personas en las personas simuladas.

	Sforce force_res;

	std::vector<bool> person_ids_in_group_bool;
	//person_ids_in_group_bool.reserve(person_list_.size());
	for(unsigned int p=0; p<person_list_.size();p++){ //initialize boolean vector
		person_ids_in_group_bool.push_back(false);
	}

	for(unsigned int p=0; p<ids_of_persons_in_group.size();p++){
		person_ids_in_group_bool[ids_of_persons_in_group[p]]=true;
	}

	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		//there is a list for persons and another for robot(s). This function is for persons
		if( *center != *(*iit) && center->get_current_pointV().distance( (*iit)->get_current_pointV() ) < 15.0)
		{
			//if(person_ids_in_group_bool[(*iit)->get_id()]==false){
				force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit) );
			//}else{

			//}

		}
	}
	return force_res;

}

Sforce Cscene_abstract::force_persons_int3( Cperson_abstract* center ) // made by ely (companion, no take into accound the person companion of the group force.)
{
	Sforce force_res;

	/*std::vector<bool> person_ids_in_group_bool;
	//person_ids_in_group_bool.reserve(person_list_.size());
	for(unsigned int p=0; p<person_list_.size();p++){ //initialize boolean vector
		person_ids_in_group_bool.push_back(false);
	}

	for(unsigned int p=0; p<ids_of_persons_in_group.size();p++){
		person_ids_in_group_bool[ids_of_persons_in_group[p]]=true;
	}*/

	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		//there is a list for persons and another for robot(s). This function is for persons
		if( *center != *(*iit) && center->get_current_pointV().distance( (*iit)->get_current_pointV() ) < 15.0)
		{
				force_res += center->force( (*iit)->get_current_pointV() ,  this->get_sfm_int_params( center , *iit) );
		}
	}
	return force_res;

}

const std::vector<double>* Cscene_abstract::get_sfm_params( const Cperson_abstract * center_person)
{
	assert( center_person != NULL );
	switch( center_person->get_person_type() )
	{
	case Cperson_abstract::Person_companion :
		return &social_forces_param_to_person_;
	case Cperson_abstract::Person :
		return &social_forces_param_to_person_;
	case Cperson_abstract::Robot :
		return &social_forces_param_to_robot_;
	case Cperson_abstract::Obstacle :
	default:
		return &social_forces_param_to_obs_;
	}
}




std::vector<double> Cscene_abstract::new_get_sfm_params( const Cperson_abstract * center_person)
{
	assert( center_person != NULL );
	switch( center_person->get_person_type() )
	{
	case Cperson_abstract::Person_companion :
		return social_forces_param_to_person_;
	case Cperson_abstract::Person :
		return social_forces_param_to_person_;
	case Cperson_abstract::Robot :
		return social_forces_param_to_robot_;
	case Cperson_abstract::Obstacle :
	default:
		return social_forces_param_to_obs_;
	}
}

const std::vector<double>* Cscene_abstract::get_sfm_int_params( const Cperson_abstract * center_person, const Cperson_abstract * interacting_person)
{
	//std::cout << " IIINNNNNNNNN Cscene_abstract::get_sfm_int_params " << std::endl;

	assert( center_person != NULL );
	Cperson_abstract::target_type interacting_type;
	if ( interacting_person == NULL )
	{
		interacting_type = Cperson_abstract::Obstacle;
	}
	else
	{
		interacting_type = interacting_person->get_person_type();
	}

	//return the corresponding SFM parameters
	if( center_person->get_person_type() == Cperson_abstract::Person
			&& interacting_type != Cperson_abstract::Obstacle )
		//	&& interacting_type == Cperson_abstract::Person ) //TODO cambio para ver si funciona la relacion asimetrica
	{
		return &social_forces_param_to_person_;
	}if(center_person->get_person_type() == Cperson_abstract::Person_companion
			&& interacting_type != Cperson_abstract::Obstacle ){
		return &social_forces_param_to_person_; // TODO: in future could be that the person_companion respect to the robot need to use the same parameters than the robot respect to the person companion
	}
	else if( center_person->get_person_type() == Cperson_abstract::Robot &&
			(interacting_type == Cperson_abstract::Person))
	{
		return &social_forces_param_to_robot_;
	}else if( center_person->get_person_type() == Cperson_abstract::Robot &&
					(interacting_type == Cperson_abstract::Robot)) // probisional, simulation person_companion=> repulsion with real robot == social_forces_param_to_person_companion_ (the same as the repulsion of the robot to the person companion)
	{
		//provisional:
		//std::cout << " social_forces_param_to_person_companion_.size()="<<social_forces_param_to_person_companion_.size() << std::endl;

			//	if(!social_forces_param_to_person_companion_.empty()){
			//		std::cout << " social_forces_param_to_person_companion_[0]="<<social_forces_param_to_person_companion_[0]<<"; [1]="<< social_forces_param_to_person_companion_[1]<<"; [2]="<<social_forces_param_to_person_companion_[2]<<"; [3]="<<social_forces_param_to_person_companion_[3]<<"; [4]="<<social_forces_param_to_person_companion_[4]<< std::endl;
			//	}
			return &social_forces_param_to_person_companion_;
	}else if(center_person->get_person_type() == Cperson_abstract::Robot &&  interacting_type ==Cperson_abstract::Person_companion){

		//std::cout << " social_forces_param_to_person_companion_.size()="<<social_forces_param_to_person_companion_.size() << std::endl;

		//if(!social_forces_param_to_person_companion_.empty()){
		//	std::cout << " social_forces_param_to_person_companion_[0]="<<social_forces_param_to_person_companion_[0]<<"; [1]="<< social_forces_param_to_person_companion_[1]<<"; [2]="<<social_forces_param_to_person_companion_[2]<<"; [3]="<<social_forces_param_to_person_companion_[3]<<"; [4]="<<social_forces_param_to_person_companion_[4]<< std::endl;
		//}

		//std::cout << " IIINNNNNNNNN Cscene_abstract::get_sfm_int_params Cperson_abstract::Robot && Cperson_abstract::Person_companion ;" << std::endl;

		return &social_forces_param_to_person_companion_;
	}
	else
	{
		return &social_forces_param_to_obs_;
	}


	return NULL;
}

bool Cscene_abstract::read_force_map( const char * path )
{
	FILE * fid;
	float fx,fy;
	int obstacle;
	fid = fopen( path , "r");
	if (fid==NULL ) return read_force_map_success_;//false at ini
	//fscanf(fid,"%f",&fx);
	int fs =fscanf(fid,"%f",&min_x_);
	fs = fscanf(fid,"\n%d",&map_number_x_);
	fs =fscanf(fid,"\n%f",&min_y_);
	fs =fscanf(fid,"\n%d",&map_number_y_);
	fs = fscanf(fid,"\n%f",&map_resolution_);
	max_x_ = min_x_+  map_number_x_ * map_resolution_;
	max_y_ = min_y_+  map_number_y_ * map_resolution_;

	if( map_number_x_ * map_number_y_ <= 0 ) return read_force_map_success_;//false empty
	//std::cout << "number of elements (x,y ) = " << map_number_x_ << " , " << map_number_y_ << std::endl;
	force_map_.reserve( map_number_x_ * map_number_y_ );
	obstacle_map_.reserve( map_number_x_ * map_number_y_ );
	for( unsigned int i = 0; i< map_number_y_*map_number_x_; ++i)
	{
		//fscanf(fid,"\n%f %f %d",&fx,&fy,&obstacle);
		fs = fscanf(fid,"\n%f",&fx);
		fs = fscanf(fid," %f",&fy);
		fs = fscanf(fid," %d",&obstacle);
		force_map_.push_back( Sforce(fx,fy) );
		obstacle_map_.push_back( (bool)obstacle );
	}
	fclose(fid);
	fs &= 0;//to avoid warning
	//cout << "map_number_x_" << map_number_x_ << " map_number_rows = " << map_number_y_ <<
	//		"  and the vector map has a total of elements = " << force_map_.size() << endl;
	read_force_map_success_ = true;
	return read_force_map_success_;
}

Sforce Cscene_abstract::get_force_map( double x, double y )
{
	if ( read_force_map_success_ && x >= min_x_ && x <= max_x_ &&  y >= min_y_ && y <= max_y_ )
	{
		//by construction in the .m, the vector is read in columns
		// iit = floor( (x-vv(1)) / vv(5))   + floor((y-vv(3)) / vv(5))*M +1;
		//cout << "iteration" << int( (x-min_x_) / map_resolution_) * map_number_y_  + int ((y-min_y_) / map_resolution_) << endl;
		return force_map_[ int( (x-min_x_) / map_resolution_) + int ((y-min_y_) / map_resolution_) * map_number_x_ ];
	}
	else
		return Sforce();
}

bool Cscene_abstract::is_cell_clear_map( double x, double y )
{
	//returns true if cell is clear (or outside the map) and false if there is an obstacle
	if ( read_force_map_success_ && x >= min_x_ && x <= max_x_ &&  y >= min_y_ && y <= max_y_ )
	{
		//by construction in the .m, the vector is read in columns
		// iit = floor( (x-vv(1)) / vv(5))   + floor((y-vv(3)) / vv(5))*M +1;
		//cout << "iteration" << int( (x-min_x_) / map_resolution_) * map_number_y_  + int ((y-min_y_) / map_resolution_) << endl;
		return obstacle_map_[ int ( (x-min_x_) / map_resolution_) + int ((y-min_y_) / map_resolution_) * map_number_x_ ];
	}
	else
		return true;
}

void Cscene_abstract::get_map_params(float &min_x, float &max_x, float &min_y, float &max_y, float &resolution,
		unsigned int &map_number_x, unsigned int &map_number_y)
{
	min_x = min_x_;
	max_x = max_x_;
	min_y = min_y_;
	max_y = max_y_;
	resolution = map_resolution_;
	map_number_x = map_number_x_;
	map_number_y = map_number_y_;
}

unsigned int Cscene_abstract::xy_to_m(double x, double y)
{
	if ( read_force_map_success_ && x >= min_x_ && x <= max_x_ &&  y >= min_y_ && y <= max_y_ )
		{
			return int ( (x-min_x_) / map_resolution_) + int ((y-min_y_) / map_resolution_) * map_number_x_ ;
		}
	return 0;
}
Spoint Cscene_abstract::m_to_xy( unsigned int m)
{
	double x = min_x_+(m - m/map_number_x_ * map_number_x_ ) * map_resolution_;
	double y = min_y_ + m/map_number_x_*map_resolution_;
	return Spoint( x,y );
}

bool Cscene_abstract::read_destination_map( const char * path )
{
	/* * how to use this function: you need a destination's file
	number of destinations in the scene
	dest_id_1 , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
	dest_id_2 , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
	2
	1 0.0 0.0 0.5 1 2
	2 10.0 10.0 0.5 1 1
	*/
	std::cout << "Enters in Cscene_abstract::read_destination_map( const char * path ) " << std::endl;

	FILE * fid;
	fid = fopen( path , "r");
	if (fid==NULL) return read_destination_map_success_;//false at ini
	//fscanf(fid,"%f",&fx);
	int n_dest;
	int fs = fscanf(fid,"%d",&n_dest);
	//n_dest=4; // de momento hard core, por error al leer.
	destinations_.clear();
	//std::cout << "n_dest " <<n_dest << std::endl;
	destinations_.reserve( n_dest );
	int id, nn, n;
	float x,y,pr;
	std::vector<int> neigh;
	for( unsigned int i = 0; i< (unsigned) n_dest; ++i)
	{
		//dest_ id , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
		fs = fscanf(fid,"\n%d, %f, %f, %f, %d",&id,&x,&y,&pr,&nn);
		//nn=1;  // de momento hard core, por error al leer.
		neigh.clear();
		neigh.reserve(nn);
		//std::cout << "nn= " <<nn << std::endl;
		for(unsigned int j = 0; j < (unsigned) nn; ++j)
		{
			fs = fscanf(fid,", %d" , &n);
			neigh.push_back(n);
		}
		destinations_.push_back( Sdestination(id,x,y,pr, Sdestination::Map_goal , neigh) );
		destinations_.back().print();
	}
	fclose(fid);
	fs *= 0;
	//check that there are destinations
	if( destinations_.empty() ) return read_destination_map_success_;
	read_destination_map_success_ = true;
	//update robot and people destinations
	this->set_destinations( destinations_ );
	return read_destination_map_success_;
}

bool Cscene_abstract::read_destination_map2( const char * path )  // made by (ely), to make people groups in the simulation.
{
	/* * how to use this function: you need a destination's file
	number of destinations in the scene
	dest_id_1 , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
	dest_id_2 , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
	2
	1 0.0 0.0 0.5 1 2
	2 10.0 10.0 0.5 1 1
	*/
	std::cout << "(ELY) Enters in Cscene_abstract::read_destination_map( const char * path ) " << std::endl;

	FILE * fid;
	//std::cout << "1 "<< std::endl;
	fid = fopen( path , "r");
	//std::cout << "2"<< std::endl;
	if (fid==NULL) return read_destination_map_success_;//false at ini
	//fscanf(fid,"%f",&fx);
	//std::cout << "3 "<< std::endl;
	int n_dest;
	//std::cout << "4 "<< std::endl;
	int fs = fscanf(fid,"%d",&n_dest);
	//std::cout << "5 "<< std::endl;
	//n_dest=4; // de momento hard core, por error al leer.
	destinations_.clear();
	//std::cout << "n_dest " <<n_dest << std::endl;
	destinations_.reserve( n_dest );
	int id, nn, n;
	float x,y,pr;
	std::vector<int> neigh;
	for( unsigned int i = 0; i< (unsigned) n_dest; ++i)
	{
		//dest_ id , x , y , pr, n_neighbours , neighbour1, neighbour2, ...
		fs = fscanf(fid,"\n%d, %f, %f, %f, %d",&id,&x,&y,&pr,&nn);
		//nn=1;  // de momento hard core, por error al leer.
		neigh.clear();
		neigh.reserve(nn);
		std::cout << "nn= " <<nn << std::endl;
		for(unsigned int j = 0; j < (unsigned) nn; ++j)
		{
			fs = fscanf(fid,", %d" , &n);
			neigh.push_back(n);
		}
		destinations_.push_back( Sdestination(id,x,y,pr, Sdestination::Map_goal , neigh) );
		destinations_.back().print();
	}
	fclose(fid);
	fs *= 0;
	//check that there are destinations
	if( destinations_.empty() ) return read_destination_map_success_;
	read_destination_map_success_ = true;
	//update robot and people destinations
	this->set_destinations( destinations_ );
	std::cout << "destinations_.size()="<<destinations_.size()<< std::endl;

	return read_destination_map_success_;
}
