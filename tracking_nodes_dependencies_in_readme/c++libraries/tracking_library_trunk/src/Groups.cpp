/*
 * Groups.cpp
 *
 *  Created on: 3/03/2015
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

#include "Groups.h"

Cgroups::Cgroups(unsigned int id, std::vector<unsigned int> tracks_group, std::vector<SpointV_cov> SpointV_cov_tracks_group):
cargo_ants(false),
id_group_(id),
group_width_(3.0),
group_length_(3.0),
update_group_(false),
tracks_group_(tracks_group),
SpointV_cov_tracks_group_(SpointV_cov_tracks_group)
{
}

Cgroups::~Cgroups()
{
}

void Cgroups::generation_group_track_pose(){

	//std::cout<< " Entro en generation_group_track_pose()"<< std::endl;

	unsigned int num_tracks_group=SpointV_cov_tracks_group_.size();
	double vx_group=0.0;
	double vy_group=0.0;
	std::vector<double> cov_t;
	std::vector<double> cov_t_final_vector;
	cov_t_final_vector.reserve(16);
	cov_t_final_vector.resize(16,0.0);
	Eigen::MatrixXd cov_t2;
	cov_t2.resize(0,0);
	cov_t2.resize(4,4);
	Eigen::MatrixXd cov_tfinal;
	cov_tfinal.resize(0,0);
	cov_tfinal.resize(4,4);

	for(unsigned int y=0; y<num_tracks_group;y++){

		SpointV_cov SpointVcov_act=SpointV_cov_tracks_group_[y];
		cov_t=SpointVcov_act.cov;

		cov_t2.row ( 0 ) << cov_t[0] , cov_t[1], cov_t[2],cov_t[3];
		cov_t2.row ( 1 ) << cov_t[4] , cov_t[5], cov_t[6],cov_t[7];
		cov_t2.row ( 2 ) << cov_t[8] , cov_t[9], cov_t[10],cov_t[11];
		cov_t2.row ( 3 ) << cov_t[12] , cov_t[13], cov_t[14],cov_t[15];

		vx_group=vx_group+SpointVcov_act.vx;
		vy_group=vy_group+SpointVcov_act.vy;

		if(y==0){
			cov_tfinal=cov_t2;
		}else{
			cov_tfinal=cov_tfinal+cov_t2;
		}
	}

	vx_group=vx_group/num_tracks_group;
	vy_group=vy_group/num_tracks_group;
	cov_tfinal=cov_tfinal/num_tracks_group;

	cov_t_final_vector[0]=(double) cov_tfinal(0,0);
	cov_t_final_vector[1]=(double) cov_tfinal(0,1);
	cov_t_final_vector[2]=(double) cov_tfinal(0,2);
	cov_t_final_vector[3]=(double) cov_tfinal(0,3);
	cov_t_final_vector[4]=(double) cov_tfinal(1,0);
	cov_t_final_vector[5]=(double) cov_tfinal(1,1);
	cov_t_final_vector[6]=(double) cov_tfinal(1,2);
	cov_t_final_vector[7]=(double) cov_tfinal(1,3);
	cov_t_final_vector[8]=(double) cov_tfinal(2,0);
	cov_t_final_vector[9]=(double) cov_tfinal(2,1);
	cov_t_final_vector[10]=(double) cov_tfinal(2,2);
	cov_t_final_vector[11]=(double) cov_tfinal(2,3);
	cov_t_final_vector[12]=(double) cov_tfinal(3,0);
	cov_t_final_vector[13]=(double) cov_tfinal(3,1);
	cov_t_final_vector[14]=(double) cov_tfinal(3,2);
	cov_t_final_vector[15]=(double) cov_tfinal(3,3);

	SpointV_cov Central_group(track_with_less_id_in_group_.x, track_with_less_id_in_group_.y, SpointV_cov_tracks_group_[0].time_stamp, vx_group, vy_group,cov_t_final_vector);
	central_group_=Central_group;
	//central_group_.print();

}

void Cgroups::print_groups(){
	std::cout<< "\n id_group="<< id_group_<< std::endl;
	std::cout<< " update_group_="<<update_group_ << std::endl;

	std::cout<< " id tracks_in_group:" << std::endl;
	for(unsigned int g=0;g<tracks_group_.size();g++){
		std::cout<< " track_id=" <<  tracks_group_[g]<< std::endl;
	}

	std::cout<< " SpointV_cov tracks_in_group:" << std::endl;
	for(unsigned int g=0;g<SpointV_cov_tracks_group_.size();g++){
		SpointV_cov_tracks_group_[g].print();
	}

	std::cout<< " Group track POSE:" << std::endl;

	std::cout<< " central_group_.x (X_group)="<<central_group_.x << std::endl;
	std::cout<< " central_group_.y (Y_group)="<<central_group_.y << std::endl;
	std::cout<< " central_group_.vx (vx_group)="<<central_group_.vx<< std::endl;
	std::cout<< " central_group_.vy (vy_group)="<<central_group_.vy << std::endl;

}


exitGroup::exitGroup(unsigned int id, double width,double length, SpointV_cov central_group):
ex_id_group_(id),
ex_group_width_(width),
ex_group_length_(length),
ex_central_group_(central_group)
{
}

exitGroup::~exitGroup()
{
}

void exitGroup::print_group(){
	std::cout<< "\n id_group="<<ex_id_group_<< std::endl;
	std::cout<< " Group width:" <<ex_group_width_<< std::endl;
	std::cout<< " Group length:"<<ex_group_length_<< std::endl;
	std::cout<< " Group track POSE:" << std::endl;
	std::cout<< " central_group_.x (X_group)="<<ex_central_group_.x << std::endl;
	std::cout<< " central_group_.y (Y_group)="<<ex_central_group_.y << std::endl;
	std::cout<< " central_group_.vx (vx_group)="<<ex_central_group_.vx<< std::endl;
	std::cout<< " central_group_.vy (vy_group)="<<ex_central_group_.vy << std::endl;

}

