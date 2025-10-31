/*
 * mhtStructs.cpp
 *
 *  Created on: 07/07/2013 (aprox)
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

#include "mhtStructs.h"

#include <math.h>
#include <iostream>

/***Sdetection functions***/

Sdetection::Sdetection():
	Spoint_cov(),type(),probability()
{
}


Sdetection::Sdetection(double x_ , double y_ , double time_stamp_,const std::vector<double>& cov_ ,int type_, double prob_):
	Spoint_cov(x_ , y_,time_stamp_, cov_ ),type(type_),probability(prob_)
{
}


Spoint  Sdetection::getSpointDetection () //TODO: tendría que ser get, no return.
{
	return Spoint( x , y , time_stamp);
}

Spoint_cov  Sdetection::getSpoint_cov ()
{
	return Spoint_cov( x , y , time_stamp, cov);
}
double  Sdetection::get_detector_probability ()
{
	return probability;
}

Sdetection::~Sdetection() {}

void Sdetection::print_spoint_DET()
{
	std::cout << "(x , y ) = (" << x << " , " << y  << " )" << std::endl;
}

/***Strack functions***/

Strack::Strack():
SpointV_cov(),id(),probability_of_detection()
{
}

Strack::Strack(double x_, double y_, double time_stamp_, double vx_, double vy_,
		 const std::vector<double>& cov_, unsigned int id_, double probability_of_detection_):
 id(id_), probability_of_detection(probability_of_detection_),
SpointV_cov( x_ , y_ , time_stamp_ , vx_, vy_, cov_) //SpointV_cov_Strack_( x_ , y_ , time_stamp_ , vx_, vy_, cov_)
{
	//std::cout<< "IN generate track=>"<<id << std::endl;
	//SpointV_cov_Strack_=SpointV_cov( x_ , y_ , time_stamp_ , vx_, vy_, cov_);
	//std::cout<< "end generate track=>"<<id << std::endl;
}

/*Strack::Strack( Strack& track_in ):
		SpointV_cov(track_in.getSpointV_cov()), id(track_in.returnId ()), probability_of_detection(track_in.returnProbability ()){

}*/


Spoint Strack::getSpointTrack()
{
 	return Spoint( x , y );
	//return Spoint( SpointV_cov_Strack_.x , SpointV_cov_Strack_.y );
}

Spoint_cov Strack::getSpoint_cov()
{
 	return Spoint_cov( x , y , time_stamp, cov );
	//return Spoint_cov( SpointV_cov_Strack_.x , SpointV_cov_Strack_.y , SpointV_cov_Strack_.time_stamp, SpointV_cov_Strack_.cov );

 		//cov is a vector that represent a matrix =[ (0) sigma_xx, (1) sigma_xy, (2) sigma_xVx, (3) sigma_xVy,
 												//   (4) sigma_yx, (5) sigma_yy, (6) sigma_yVx, (7) sigma_yVy,
 												//   (8) sigma_Vxx, (9) sigma_Vxy, (10) sigma_VxVx, (11) sigma_VxVy
 												//   (12) sigma_Vyx, (13) sigma_Vyy, (14) sigma_VyVx, (15) sigma_VyVy,
}

SpointV_cov Strack::getSpointV_cov()
{
	return SpointV_cov( x , y , time_stamp, vx , vy , cov );
	//return SpointV_cov( SpointV_cov_Strack_.x , SpointV_cov_Strack_.y , SpointV_cov_Strack_.time_stamp, SpointV_cov_Strack_.vx , SpointV_cov_Strack_.vy , SpointV_cov_Strack_.cov );
 		//cov is a vector that represent a matrix =[ (0) sigma_xx, (1) sigma_xy, (2) sigma_xVx, (3) sigma_xVy,
 												//   (4) sigma_yx, (5) sigma_yy, (6) sigma_yVx, (7) sigma_yVy,
 												//   (8) sigma_Vxx, (9) sigma_Vxy, (10) sigma_VxVx, (11) sigma_VxVy
 												//   (12) sigma_Vyx, (13) sigma_Vyy, (14) sigma_VyVx, (15) sigma_VyVy,
}

int  Strack::returnId ()
{
	return id;
}

double  Strack::returnProbability ()
{
	return probability_of_detection;
}

Strack Strack::Create_Strack_with_SpointV_cov(SpointV_cov track_pose, unsigned int identf, double probabil)
{
	return Strack(track_pose.x, track_pose.y,track_pose.time_stamp,track_pose.vx,track_pose.vy, track_pose.cov, identf, probabil);
}

void Strack::fill_Strack_with_SpointV_cov( SpointV_cov track_pose, unsigned int identf, double probabil ){
	Strack(track_pose.x, track_pose.y,track_pose.time_stamp,track_pose.vx,track_pose.vy, track_pose.cov, identf, probabil);
}

Strack::~Strack() {}

void Strack::print(){
	std::cout<< " In strack print (funcion_cambiada) " << std::endl;
	std::cout<< " id="<<id << std::endl;
	std::cout<< " probability_of_detection=" <<  probability_of_detection << std::endl;
	std::cout<< " (funcion_cambiada) SpointV_cov del Strack:" << std::endl;
	//SpointV_cov act_track=getSpointV_cov();
	std::cout<< " act_track.x" <<x<< std::endl;
	std::cout<< " act_track.y" <<y<< std::endl;
	std::cout<< " act_track.time_stamp" <<time_stamp<< std::endl;
	std::cout<< " act_track.vx" <<vx<< std::endl;
	std::cout<< " act_track.vy" <<vy<< std::endl;
	std::cout<< " cov[0]=" <<cov[0]<< std::endl;
	std::cout<< " cov[1]=" <<cov[1]<< std::endl;
	std::cout<< " cov[6]=" <<cov[6]<< std::endl;
	std::cout<< " cov[7]=" <<cov[7]<< std::endl;
	//SpointV_cov act_track=getSpointV_cov();

	//std::cout<< " SpointV_cov_Strak_.print():" <<x<< std::endl;
	//SpointV_cov_Strack_.print();
	std::cout<< " Fin strack print (funcion_cambiada) " << std::endl;
}

/***Scluster functions***/

Scluster::Scluster():
id_cluster_(),tracks_(),detections_()
{
}


Scluster::Scluster(unsigned int id_cluster, const std::vector<unsigned int>& tracks , const std::vector<unsigned int>& detections,const std::vector<std::vector<double>>& association_distances,const std::vector<std::vector<unsigned int>> tracks_associated, double Pd_cluster):
id_cluster_(id_cluster),tracks_(tracks),detections_(detections),association_distances_(association_distances),tracks_associated_(tracks_associated),Pd_cluster_(Pd_cluster)
{
}

unsigned int Scluster::get_idCluster(){
	return id_cluster_;
}

void Scluster::set_idCluster(unsigned int id_actual){
	id_cluster_=id_actual;
}

std::vector<unsigned int> Scluster::get_tracks_in_cluster(){
	return tracks_;
}

std::vector<unsigned int> Scluster::get_detections_in_cluster(){
	return detections_;
}


void Scluster::set_detections_in_cluster(unsigned int detection){
	detections_.push_back(detection);
}

void Scluster::set_tracks_in_cluster(unsigned int track){
	tracks_.push_back(track);
}

double Scluster::get_Pd_cluster(){
	return Pd_cluster_;
}

void Scluster::set_Pd_cluster(double Pd_actual){
	Pd_cluster_=Pd_actual;
}

std::vector<std::vector<double>> Scluster::get_association_distances_Scluster(){
	return association_distances_;
}

void Scluster::print_cluster(){
	   std::cout<< " Clusters actual! \n" << std::endl;
	   std::cout<< " id_cluster actual:" <<  id_cluster_ << std::endl;
	   std::cout<< " cluster_life actual:" << cluster_life_ << std::endl;
	   std::cout<< " Pd_cluster actual:" << Pd_cluster_ << std::endl;
	  // std::cout<< " tracks_separados actual: (0=false) (1=true)" << tracks_separados_ << std::endl;

	   std::cout<< " tracks in cluster actual:"<< std::endl;
	   std::cout<< " tracks_.size():"<<tracks_.size()<< std::endl;
	   for(unsigned int t; t<tracks_.size();t++ ){
		   std::cout<< " tracks("<< t <<")="<< tracks_[t] << std::endl;
		   std::cout<< " tracks_separados actual: (0=false) (1=true)" << tracks_separados_[t] << std::endl;
	   }

	   std::cout<< " detections in cluster actual:"<< std::endl;
	   std::cout<< " detections_.size():"<<detections_.size()<< std::endl;
	   for(unsigned int d; d<detections_.size();d++ ){
		   std::cout<< " detections("<< d <<")="<< detections_[d] << std::endl;
	   }

	   ////////////////
		std::cout<< " tracks_associated_ actual:"<< std::endl;
			std::cout<< " SIZE association tracks="<<tracks_associated_.size()<< std::endl;
			for(unsigned int a=0; a<detections_.size() ;a++ ){
				for(unsigned int t=0; t<tracks_associated_[detections_[a]].size() ;t++ ){
					std::cout<< "  tracks_associated_(detection="<< detections_[a] <<")="<< tracks_associated_[detections_[a]][t] << std::endl;

				}
			}
			std::cout<< " association_distances_ actual:"<< std::endl;
			std::cout<< " SIZE association distances="<<association_distances_.size()<< std::endl;
			for(unsigned int d=0; d<detections_.size() ;d++ ){
				for(unsigned int t=0; t<association_distances_[detections_[d]].size() ;t++ ){
					std::cout<< "  association_distances_[detection="<< detections_[d] <<"][track="<< tracks_associated_[detections_[d]][t] <<"]="<< association_distances_[detections_[d]][t] << std::endl;
				}
			}
}

Scluster::~Scluster() {}

/*** N_hypothesis functions***/

N_hypothesis::N_hypothesis():
	N_det_(),N_tgt_(),N_fa_(), N_new_(),N_cluster_no_det_(),N_cluster_no_confirm_(),distances_sub_hypothesis_(),
	delta_sub_hypothesis_()
{
}

N_hypothesis::N_hypothesis(int  N_det, int N_tgt, int N_fa, int N_new, int N_cluster_no_det, int N_cluster_no_confirm, std::vector<double>& distances_sub_hypothesis, std::vector<double>& delta_sub_hypothesis,std::vector<double>& laser_beta_track_no_confirmado, double laser_beta_cluster_no_confirmado, double Pd_orientation ):
	N_det_(N_det),N_tgt_(N_tgt),N_fa_(N_fa), N_new_(N_new),N_cluster_no_det_(N_cluster_no_det),
	N_cluster_no_confirm_(N_cluster_no_confirm),distances_sub_hypothesis_(distances_sub_hypothesis),
	delta_sub_hypothesis_(delta_sub_hypothesis),laser_beta_track_no_confirmado_(laser_beta_track_no_confirmado),
	laser_beta_cluster_no_confirmado_(laser_beta_cluster_no_confirmado),Pd_orientation_(Pd_orientation)
{
}

N_hypothesis::~N_hypothesis() {}

void N_hypothesis::print_N_hypothesis(){
	 std::cout<<"N_det=" << N_det_ << std::endl;
	 std::cout<<"N_tgt_=" << N_tgt_ << std::endl;
	 std::cout<<"N_fa_=" << N_fa_ << std::endl;
	 std::cout<<"N_new_=" << N_new_ << std::endl;
	 std::cout<<"N_cluster_no_det_=" << N_cluster_no_det_ << std::endl;
	 std::cout<<"N_cluster_no_confirm_=" << N_cluster_no_confirm_ << std::endl;

	 std::cout<< "distances_sub_hypothesis_: (d=columna hipotessis, deteccion x)"<< std::endl;
	 for(unsigned int d=0; d<distances_sub_hypothesis_.size();d++ ){
			   std::cout<< " distances_sub_hypothesis_("<< d <<")="<< distances_sub_hypothesis_[d] << std::endl;
	 }
	 std::cout<< "delta_sub_hypothesis_: (d=columna hipotessis, deteccion x)"<< std::endl;
	 for(unsigned int d=0; d<delta_sub_hypothesis_.size();d++ ){
		std::cout<< "delta_sub_hypothesis_(columna,deteccion="<< d <<")="<< delta_sub_hypothesis_[d] << std::endl;
	 }
	std::cout<< "laser_beta_track_no_confirmado_: (d=columna hipotessis, deteccion x)"<< std::endl;
	for(unsigned int d=0; d<laser_beta_track_no_confirmado_.size();d++ ){
		std::cout<< " laser_beta_track_no_confirmado_("<< d <<")="<< laser_beta_track_no_confirmado_[d] << std::endl;
	}

	std::cout<< "(for all det<-->track in cluster) laser_beta_cluster_no_confirmado_: "<<laser_beta_cluster_no_confirmado_<< std::endl;
}

Exitcluster::Exitcluster():
	id_cluster_(),tracks_(),position_tracks_(),centroid_cluster_(),centroid_cluster_with_cluster_area_(),Pd_cluster_(){
}

Exitcluster::Exitcluster(unsigned int id_cluster, const std::vector<unsigned int>& tracks , const std::vector<SpointV_cov>& position_tracks,Spoint centroid_cluster, Spoint_cov centroid_cluster_with_cluster_area, double Pd_cluster,double mediumVcluster,bool equal_velocity_tracks,bool equal_orientation_tracks):
	id_cluster_(id_cluster),tracks_(tracks),position_tracks_(position_tracks),centroid_cluster_(centroid_cluster),centroid_cluster_with_cluster_area_(centroid_cluster_with_cluster_area),Pd_cluster_(Pd_cluster),mediumVcluster_(mediumVcluster),equal_velocity_tracks_(equal_velocity_tracks),equal_orientation_tracks_(equal_orientation_tracks){
}

Exitcluster::~Exitcluster() {}

GroupDataBaseFusion::GroupDataBaseFusion():
		Strack(),
		//id_group_(0),
		threshold_dist_assos_group_(0.5),
		debug_groups_(false)
		//track_group_probability_(0.0),
		//track_group_x_(0.0),
		//track_group_y_(0.0),
		//track_group_vx_(0.0),
		//track_group_vy_(0.0),
		//track_group_time_stamp_(0.0)
{ //,track_group_(Strack())
	//track_group_.setSpointV_cov_Strack(SpointV_cov());
	/*cov.reserve(16);
	cov.resize(16,0.0);
	cov[0] = 0.5;//0.4;
	cov[5] = 0.5;
	cov[10] = 0.1; //0.1
	cov[15] = 0.1;*/

	Stracks_tracks_tibi_=std::vector<Strack>();
	//std::cout<< " Stracks_tracks_tibi_.EMPTY()"<<Stracks_tracks_tibi_.empty()<<"; Stracks_tracks_tibi_.size()"<<Stracks_tracks_tibi_.size()<< std::endl;

	Stracks_tracks_teo_=std::vector<Strack>();
	//std::cout<< " Stracks_tracks_teo_.EMPTY()"<<Stracks_tracks_teo_.empty()<<"; Stracks_tracks_teo_.size()"<<Stracks_tracks_teo_.size()<< std::endl;
}


GroupDataBaseFusion::GroupDataBaseFusion(unsigned int in_id_group,std::vector<Strack> in_Stracks_tracks_tibi,std::vector<Strack> in_Stracks_tracks_teo, Strack in_track_group):
		Strack(),threshold_dist_assos_group_(0.5),debug_groups_(false) //id_group_(in_id_group)
{ //track_group_(in_track_group)

	//std::cout<< " IN GroupDataBaseFusion"<< std::endl;


	// fill tibi list
	//unsigned int coun_tracks_tibi_=0;

	for(unsigned int i=0;i<in_Stracks_tracks_tibi.size();i++){

		Strack tibi=in_Stracks_tracks_tibi[i];

		/*std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
		it_insert_track = mytracks_list_tibi_.begin();

		if (!mytracks_list_tibi_.empty())
		{
			while (it_insert_track != mytracks_list_tibi_.end())
			{
				it_insert_track++;
				if (it_insert_track->returnId() > coun_tracks_tibi_)
				{
					break;
				}

			}
		}
		mytracks_list_tibi_.insert(it_insert_track, tibi);*/
		Stracks_tracks_tibi_.push_back(tibi);

	}

	// fill teo list
	//unsigned int coun_tracks_teo_=0;

	for(unsigned int i=0;i<in_Stracks_tracks_teo.size();i++){

		Strack teo=in_Stracks_tracks_teo[i];

		/*std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
		it_insert_track = mytracks_list_teo_.begin();

		if (!mytracks_list_teo_.empty())
		{
			while (it_insert_track != mytracks_list_teo_.end())
			{
				it_insert_track++;
				if (it_insert_track->returnId() > coun_tracks_teo_)
				{
					break;
				}

			}
		}
		mytracks_list_teo_.insert(it_insert_track, teo);*/
		Stracks_tracks_teo_.push_back(teo);



	}

}

GroupDataBaseFusion::~GroupDataBaseFusion(){}


void GroupDataBaseFusion::print_GroupDataBaseFusion(){
// prin de la clase



	 std::cout<<"(Strack interno) id=" << id << std::endl;
	 std::cout<<"(Strack interno) probability_of_detection="<<probability_of_detection << std::endl;

	 std::cout<<"(Spoint) track_group_:" << std::endl;
//	 track_group_.print();
	 std::cout<<"(Strack interno) x="<<x << std::endl;
	 std::cout<<"(Strack interno) y="<<y << std::endl;
	 std::cout<<"(Strack interno) vx="<<vx << std::endl;
	 std::cout<<"(Strack interno) vy="<<vy<< std::endl;
	 std::cout<<"(Strack interno) time_stamp="<<time_stamp<< std::endl;
	 std::cout<<"cov:"<< std::endl;
	 std::cout << "[ cov[0]=" << cov[0] <<"; cov[1]="<<cov[1]<<"; cov[2]="<<cov[2]<<"; cov[3]="<<cov[3]<<"]" << std::endl;
	 std::cout << "[ cov[4]=" << cov[4] <<"; cov[5]="<<cov[5]<<"; cov[6]="<<cov[6]<<"; cov[7]="<<cov[7]<<"]" << std::endl;
	 std::cout << "[ cov[8]=" << cov[8] <<"; cov[9]="<<cov[9]<<"; cov[10]="<<cov[10]<<"; cov[11]="<<cov[11]<<"]" << std::endl;
	 std::cout << "[ cov[12]=" << cov[12] <<"; cov[13]="<<cov[13]<<"; cov[14]="<<cov[14]<<"; cov[15]="<<cov[15]<<"]" << std::endl;


	 std::cout<< "tracks_list_tibi_:"<< std::endl;
	 //for (std::list<Strack>::iterator it_myTrack =mytracks_list_tibi_.begin(); it_myTrack != mytracks_list_tibi_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	 for(unsigned int d=0; d<Stracks_tracks_tibi_.size();d++){
		 Stracks_tracks_tibi_[d].print();

		 //it_myTrack->print();
	 }
	 std::cout<< "tracks_list_teo_:"<< std::endl;
	 //for (std::list<Strack>::iterator it_myTrack =mytracks_list_teo_.begin(); it_myTrack != mytracks_list_teo_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	 for(unsigned int d=0; d<Stracks_tracks_teo_.size();d++){
		 Stracks_tracks_teo_[d].print();
			// it_myTrack->print();
	 }

}

Strack GroupDataBaseFusion::ini_group_tibi(unsigned int in_track_tibi, Strack tibi, unsigned int id_group_count){
// initialize the group track, only with a track of tibi. (In the ROS node we have only tracks of tibi in this iteration)
// initialice the intern Strack, and the Strack vector of tibi tracks.
	/////////////////
	//std::cout << " ini_group_tibi (1): tibi.print()" << std::endl;
	//tibi.print();
	//std::cout << " ini_group_tibi (2)" << std::endl;

	SpointV_cov tibi_spoint_cov=tibi.getSpointV_cov();

	//std::cout << " ini_group_tibi (3)" << std::endl;
	std::vector<double> cov_c=tibi_spoint_cov.cov;

	//std::cout << "cov_c:\n" << std::endl;
	//for (auto c : cov_c)
	//	std::cout << c << std::endl;


	//std::cout << " ini_group_tibi (4)" << std::endl;
	Strack new_t(tibi_spoint_cov.x,tibi_spoint_cov.y,tibi_spoint_cov.time_stamp,tibi_spoint_cov.vx,tibi_spoint_cov.vy,cov_c, tibi.id, tibi.probability_of_detection);
	//std::cout << " ini_group_tibi (5); ini new_t.print():" << std::endl;
	//new_t.print();
	//std::cout << " ini_group_tibi (5); fin new_t.print():" << std::endl;
	//id_group_=id_group_count;
	id=id_group_count;
	//std::cout << " ini_group_tibi (6) id="<<id << std::endl;
	//track_group_probability_=tibi.probability_of_detection;
	probability_of_detection=tibi.probability_of_detection;
	//std::cout << " ini_group_tibi (7) probability_of_detection="<<probability_of_detection << std::endl;
	//track_group_x_=tibi_spoint_cov.x;
	x=tibi_spoint_cov.x;
	//std::cout << " ini_group_tibi (8) (x of group) x="<<x << std::endl;
	//track_group_y_=tibi_spoint_cov.y;
	y=tibi_spoint_cov.y;
	//std::cout << " ini_group_tibi (9) (y of group) y="<<y << std::endl;
	//track_group_vx_=tibi_spoint_cov.vx;
	vx=tibi_spoint_cov.vx;
	//std::cout << " ini_group_tibi (10) (of group) vx="<<vx << std::endl;
	//track_group_vy_=tibi_spoint_cov.vy;
	vy=tibi_spoint_cov.vy;
	//std::cout << " ini_group_tibi (11) (of group) vy="<<vy << std::endl;
	//track_group_time_stamp_=tibi_spoint_cov.time_stamp;
	time_stamp=tibi_spoint_cov.time_stamp;
	//std::cout << " ini_group_tibi (12) (of group) time_stamp="<<time_stamp << std::endl;


	//std::cout << "cov_c.size()="<<cov_c.size()<< std::endl;

//	std::cout << "cov:\n" << std::endl;
//	for (auto c : cov)
//		std::cout << c << std::endl;
//
//	cov_.reserve(cov_c.size());

	try
	{
		//std::cout << "1" << std::endl;
		cov = tibi_spoint_cov.cov;
		//std::cout << "2" << std::endl;
	}
	catch (std::exception& e)
	{
		std::cout << "exception caught: " << e.what() << '\n';
	}

	/*cov.reserve(16);
	std::cout << " ini_group_tibi (13)"<< std::endl;
	cov.resize(16,0.0);
	std::cout << " ini_group_tibi (14) "<< std::endl;
	double cov_0=cov_c[0];
	std::cout << " ini_group_tibi (14.1) "<< std::endl;
	cov.push_back(cov_0);
	std::cout << " ini_group_tibi (14.2) "<< std::endl;
	double cov_1=cov_c[1];
	cov[1]=cov_1;
	std::cout << " ini_group_tibi (14.3) "<< std::endl;
	double cov_2=cov_c[2];
	cov[2]=cov_2;
	std::cout << " ini_group_tibi (14.4) "<< std::endl;
	double cov_3=cov_c[3];
	cov[3]=cov_3;
	std::cout << " ini_group_tibi (14.5) "<< std::endl;
	double cov_4=cov_c[4];
	cov[4]=cov_4;
	double cov_5=cov_c[5];
	cov[5]=cov_5;
	double cov_6=cov_c[6];
	cov[6]=cov_6;
	double cov_7=cov_c[7];
	cov[7]=cov_7;
	double cov_8=cov_c[8];
	cov[8]=cov_8;
	double cov_9=cov_c[9];
	cov[9]=cov_9;
	double cov_10=cov_c[10];
	cov[10]=cov_10;
	double cov_11=cov_c[11];
	cov[11]=cov_11;
	double cov_12=cov_c[12];
	cov[12]=cov_12;
	double cov_13=cov_c[13];
	cov[13]=cov_13;
	double cov_14=cov_c[14];
	cov[14]=cov_14;
	std::cout << " ini_group_tibi (14.7) "<< std::endl;
	double cov_15=cov_c[15];
	track_grtry
	{
		cov=cov_c;
	}
	catch (std::exception& e)
	{
		std::cerr << "exception caught: " << e.what() << '\n';
	}oup_cov_[15]=cov_15;
	std::cout << " ini_group_tibi (15) cov[0]="<<cov[0]<<"; cov[1]="<<cov[1]<<"; cov[6]="<<cov[6]<<"; cov[7]="<<cov[7] << std::endl;
*/

	//////////////////

	//Stracks_tracks_tibi_.clear();

	//mytracks_list_tibi_.clear();



	// Put the tibi tracks as initial tracks.

	//track_group_=tibi;
	//track_group_.id=id_group_count;

	//tibi_spoint_cov.print();

		//std::cout << " ini_group_tibi (2.1) new_t" << std::endl;
	//new_t.print();
	//std::cout << " ini_group_tibi (2.1) new_t 2" << std::endl;
	//track_group_.Strack();
	/*track_group_.x=tibi_spoint_cov.x;
	//std::cout << " ini_group_tibi (2.1) new_t 2.1.1; x="<< track_group_.x<< std::endl;
	track_group_.y=tibi_spoint_cov.y;
	//std::cout << " ini_group_tibi (2.1) new_t 2.1.2; y="<<track_group_.y << std::endl;
	track_group_.time_stamp=tibi_spoint_cov.time_stamp;
	//std::cout << " ini_group_tibi (2.1) new_t 2.1.3" << std::endl;
	track_group_.vx=tibi_spoint_cov.vx;
	//std::cout << " ini_group_tibi (2.1) new_t 2.1.3" <<track_group_.vx<< std::endl;
	track_group_.vy=tibi_spoint_cov.vy;
	//std::cout << " ini_group_tibi (2.1) new_t 2.1.4"<<track_group_.vy << std::endl;
	track_group_.cov=cov_c;*/

	/*std::cout << " ini_group_tibi (2.1) new_t 2.1.5" << std::endl;
	track_group_.SpointV_cov_Strack_.x=tibi_spoint_cov.x;
	std::cout << " ini_group_tibi (2.1) new_t 2.1.6; x=" <<track_group_.SpointV_cov_Strack_.x<< std::endl;
	track_group_.SpointV_cov_Strack_.y=tibi_spoint_cov.y;
	std::cout << " ini_group_tibi (2.1) new_t 2.1.7; y="<<track_group_.SpointV_cov_Strack_.y << std::endl;
	track_group_.SpointV_cov_Strack_.time_stamp=tibi_spoint_cov.time_stamp;
	std::cout << " ini_group_tibi (2.1) new_t 2.1.8" << std::endl;
	track_group_.SpointV_cov_Strack_.vx=tibi_spoint_cov.vx;
	std::cout << " ini_group_tibi (2.1) new_t 2.1.9; vx=" <<track_group_.SpointV_cov_Strack_.vx<< std::endl;
	track_group_.SpointV_cov_Strack_.vy=tibi_spoint_cov.vy;
	std::cout << " ini_group_tibi (2.1) new_t 2.1.10; vy="<<track_group_.SpointV_cov_Strack_.vy << std::endl;*/
	/*track_group_.SpointV_cov_Strack_.cov[0]=cov_c[0];
	std::cout << " ini_group_tibi (2.1) new_t 2.1.10.1" << std::endl;
	track_group_.SpointV_cov_Strack_.cov[1]=cov_c[1];
	std::cout << " ini_group_tibi (2.1) new_t 2.1.10.2" << std::endl;
	track_group_.SpointV_cov_Strack_.cov[6]=cov_c[6];
	std::cout << " ini_group_tibi (2.1) new_t 2.1.10.3" << std::endl;
	track_group_.SpointV_cov_Strack_.cov[7]=cov_c[7];
	std::cout << " ini_group_tibi (2.1) new_t 2.1.11" << std::endl;*/
	//track_group_.id=tibi.id;
	//std::cout << " ini_group_tibi (2.1) new_t 2.1.12" << std::endl;
	//track_group_.probability_of_detection=tibi.probability_of_detection;


	//track_group_.setSpointV_cov_Strack(tibi_spoint_cov);
	///std::cout << " ini_group_tibi (2.1) new_t 2.1" << std::endl;
	/*track_group_.set_id(tibi.id);
	//std::cout << " ini_group_tibi (2.1) new_t 2.2" << std::endl;
	track_group_.set_probability_of_detection(tibi.probability_of_detection);
	std::cout << " ini_group_tibi (2.1) new_t 3 [print track group]" << std::endl;
	track_group_.print();
	std::cout << " ini_group_tibi (2.1) new_t 3 [fin print track group]" << std::endl;*/
	//track_group_(teo_spoint_cov.x,teo_spoint_cov.y,teo_spoint_cov.time_stamp,teo_spoint_cov.vx,teo_spoint_cov.vy,cov_c, teo.id, teo.probability_of_detection);
	//track_group_.fill_Strack_with_SpointV_cov(teo_spoint_cov, teo.id, teo.probability_of_detection);
	//std::cout << " ini_group_teo (2.2)" << std::endl;
	//track_group_.print();
	//SpointV_cov_track_group_( teo_spoint_cov.x,  teo_spoint_cov.y,  teo_spoint_cov.time_stamp,  teo_spoint_cov.vx, teo_spoint_cov.vy,cov_c);
	/*SpointV_cov_track_group_.x=tibi_spoint_cov.x;//tibi.getSpointV_cov();
	std::cout << " ini_group_tibi (2.1) new_t 3.1" << std::endl;
	SpointV_cov_track_group_.y=tibi_spoint_cov.y;
	std::cout << " ini_group_tibi (2.1) new_t 3.2" << std::endl;
	SpointV_cov_track_group_.vx=tibi_spoint_cov.vx;
	std::cout << " ini_group_tibi (2.1) new_t 3.3" << std::endl;
	SpointV_cov_track_group_.vy=tibi_spoint_cov.vy;
	std::cout << " ini_group_tibi (2.1) new_t 3.4" << std::endl;
	SpointV_cov_track_group_.time_stamp=tibi_spoint_cov.time_stamp;
	std::cout << " ini_group_tibi (2.1) new_t 3.5" << std::endl;
	SpointV_cov_track_group_.cov[0]=cov_c[0];
	std::cout << " ini_group_tibi (2.1) new_t 3.6" << std::endl;
	SpointV_cov_track_group_.cov[1]=cov_c[1];
	std::cout << " ini_group_tibi (2.1) new_t 3.7" << std::endl;
	SpointV_cov_track_group_.cov[6]=cov_c[6];
	std::cout << " ini_group_tibi (2.1) new_t 3.8" << std::endl;
	SpointV_cov_track_group_.cov[7]=cov_c[7];
*/
	//std::cout << " ini_group_tibi (2.1) new_t 4 [print SpointV_cov_track_group_]" << std::endl;
	//SpointV_cov_track_group_.print();
	//track_group_probability_=tibi.probability_of_detection;

	//std::cout << " ini_group_tibi (3)" << std::endl;
	//track_group_.id=id_group_count;


//////

	//unsigned int coun_tracks_tibi_=1;



	/*std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
	it_insert_track = mytracks_list_tibi_.begin();

	std::cout << " ini_group_tibi (4)" << std::endl;
	std::cout << "1 mytracks_list_tibi_.empty()="<<mytracks_list_tibi_.empty() << std::endl;
	if (!mytracks_list_tibi_.empty())
	{ std::cout << "2 mytracks_list_tibi_.empty()="<<mytracks_list_tibi_.empty() << std::endl;
		std::cout << " ini_group_tibi (5)" << std::endl;
		while (it_insert_track != mytracks_list_tibi_.end())
		{
			std::cout << " ini_group_tibi (6)" << std::endl;
			it_insert_track++;
			std::cout << " ini_group_tibi (7)" << std::endl;
			if (it_insert_track->returnId() > coun_tracks_tibi_)
			{
				std::cout << " ini_group_tibi (8)" << std::endl;
				break;
			}

		}
	}
	std::cout << " ini_group_tibi (9)" << std::endl;
	mytracks_list_tibi_.insert(it_insert_track, tibi);
	std::cout << " ini_group_tibi (10)" << std::endl;*/

	//std::cout << "4" << std::endl;
	//Stracks_tracks_tibi_.clear();
	Stracks_tracks_tibi_.resize(0);
	//std::cout << "4.2 Stracks_tracks_tibi_.empty()"<<Stracks_tracks_tibi_.empty() <<"; Stracks_tracks_tibi_.size()="<<Stracks_tracks_tibi_.size() << std::endl;
	Stracks_tracks_tibi_.push_back(tibi);
	//std::cout << "5" << std::endl;

	return new_t;
}

Strack GroupDataBaseFusion::ini_group_teo(unsigned int in_track_teo, Strack teo, unsigned int id_group_count){
// initialize the group track, only with a track of teo. (In the ROS node we have only tracks of teo in this iteration)
// initialice the intern Strack, and the Strack vector of teo tracks.

	//std::cout << " ini_group_teo (1): teo.print()" << std::endl;
	//teo.print();
	//std::cout << " ini_group_teo (2)" << std::endl;

	SpointV_cov teo_spoint_cov=teo.getSpointV_cov();
	//std::cout << " ini_group_teo (3)" << std::endl;

	std::vector<double> cov_c=teo_spoint_cov.cov;
	//std::cout << "cov_c:\n" << std::endl;
	//for (auto c : cov_c)
	//	std::cout << c << std::endl;

	//std::cout << " ini_group_teo (4)" << std::endl;
	Strack new_t(teo_spoint_cov.x,teo_spoint_cov.y,teo_spoint_cov.time_stamp,teo_spoint_cov.vx,teo_spoint_cov.vy,cov_c, teo.id, teo.probability_of_detection);
	//std::cout << " ini_group_teo (5); ini new_t.print()" << std::endl;
	//new_t.print();
	//std::cout << " ini_group_teo (5); fin new_t.print()" << std::endl;
	//id_group_=id_group_count;
	id=id_group_count;
	//std::cout << " ini_group_teo (6) (of group) id="<<id << std::endl;
	//track_group_probability_=teo.probability_of_detection;
	probability_of_detection=teo.probability_of_detection;
	//std::cout << " ini_group_teo (7) (of group) probability_of_detection="<<probability_of_detection << std::endl;
	//track_group_x_=teo_spoint_cov.x;
	x=teo_spoint_cov.x;
	//std::cout << " ini_group_teo (8) (of group) x="<<x << std::endl;
	//track_group_y_=teo_spoint_cov.y;
	y=teo_spoint_cov.y;
	//std::cout << " ini_group_teo (9) (of group) y="<<y << std::endl;
	//track_group_vx_=teo_spoint_cov.vx;
	vx=teo_spoint_cov.vx;
	//std::cout << " ini_group_teo (10) (of group) vx="<<vx << std::endl;
	//track_group_vy_=teo_spoint_cov.vy;
	vy=teo_spoint_cov.vy;
	//std::cout << " ini_group_teo (11) (of group) vy="<<vy << std::endl;
	//track_group_time_stamp_=teo_spoint_cov.time_stamp;
	time_stamp=teo_spoint_cov.time_stamp;
	//std::cout << " ini_group_teo (12) (of group) time_stamp="<<time_stamp << std::endl;


	//std::cout << "cov:\n" << std::endl;
	/*for (auto c : cov)
		std::cout << c << std::endl;

	cov.reserve(cov_c.size());*/
	try
	{
		//std::cout << "1" << std::endl;
		cov = teo_spoint_cov.cov;
		//std::cout << "2" << std::endl;
	}
	catch (std::exception& e)
	{
		std::cout << "exception caught: " << e.what() << '\n';
	}
	//std::cout << " ini_group_teo (13) "<< std::endl;
	//cov.reserve(16);
	/*std::cout << " ini_group_teo (13) "<< std::endl;
	cov.resize(16,0.0);
	std::cout << " ini_group_teo (13.1) "<< std::endl;
	double cov_0=cov_c[0];
	std::cout << " ini_group_teo (13.2) "<< std::endl;
		cov.push_back(cov_0);
		std::cout << " ini_group_teo (14.2) "<< std::endl;
		double cov_1=cov_c[1];
		cov[1]=cov_1;
		std::cout << " ini_group_teo (14.3) "<< std::endl;
		double cov_2=cov_c[2];
		cov[2]=cov_2;
		std::cout << " ini_group_teo (14.4) "<< std::endl;
		double cov_3=cov_c[3];
		cov[3]=cov_3;
		std::cout << " ini_group_teo (14.5) "<< std::endl;
		double cov_4=cov_c[4];
		cov[4]=cov_4;
		double cov_5=cov_c[5];
		cov[5]=cov_5;
		double cov_6=cov_c[6];
		cov[6]=cov_6;
		double cov_7=cov_c[7];
		cov[7]=cov_7;
		double cov_8=cov_c[8];
		cov[8]=cov_8;
		double cov_9=cov_c[9];
		cov[9]=cov_9;
		double cov_10=cov_c[10];
		cov[10]=cov_10;
		double cov_11=cov_c[11];
		cov[11]=cov_11;
		double cov_12=cov_c[12];
		cov[12]=cov_12;
		double cov_13=cov_c[13];
		cov[13]=cov_13;
		double cov_14=cov_c[14];
		cov[14]=cov_14;
		std::cout << " ini_group_teo (14.7) "<< std::endl;
		double cov_15=cov_c[15];
		cov[15]=cov_15;*/

	//std::cout << " ini_group_teo (15) cov[0]="<<cov[0]<<"; cov[1]="<<cov[1]<<"; cov[6]="<<cov[6]<<"; cov[7]="<<cov[7] << std::endl;

	//Stracks_tracks_teo_.clear();
	//mytracks_list_teo_.clear();


	//std::cout << " ini_group_teo (1); in_track_teo="<<in_track_teo<<"; id_group_count="<<id_group_count << std::endl;
	//new_t.print();
	//std::cout << " ini_group_teo (2.1) new_t 2" << std::endl;
	//track_group_.Strack();
	//track_group_.setSpointV_cov_Strack(teo_spoint_cov);
	//track_group_(teo_spoint_cov.x,teo_spoint_cov.y,teo_spoint_cov.time_stamp,teo_spoint_cov.vx,teo_spoint_cov.vy,cov_c, teo.id, teo.probability_of_detection);
	/*track_group_.x=teo_spoint_cov.x;
	std::cout << " ini_group_teo (2.1) new_t 2.1.1" << std::endl;
	track_group_.y=teo_spoint_cov.y;
	std::cout << " ini_group_teo (2.1) new_t 2.1.2" << std::endl;
	track_group_.time_stamp=teo_spoint_cov.time_stamp;
	std::cout << " ini_group_teo (2.1) new_t 2.1.3" << std::endl;
	track_group_.vx=teo_spoint_cov.vx;
	std::cout << " ini_group_teo (2.1) new_t 2.1.4" << std::endl;
	track_group_.vy=teo_spoint_cov.vy;
	std::cout << " ini_group_teo (2.1) new_t 2.1.5; cov_c[0]="<<cov_c[0]<< std::endl;
	track_group_.cov[0]=cov_c[0];
	std::cout << " ini_group_teo (2.1) new_t 2.1.5.1" << std::endl;
	track_group_.cov[1]=cov_c[1];
	track_group_.cov[6]=cov_c[6];
	track_group_.cov[7]=cov_c[7];
	std::cout << " ini_group_teo (2.1) new_t 2.1.6" << std::endl;*/


	/*track_group_.SpointV_cov_Strack_.x=teo_spoint_cov.x;
	std::cout << " ini_group_teo (2.1) new_t 2.1.7" << std::endl;
	track_group_.SpointV_cov_Strack_.y=teo_spoint_cov.y;
	std::cout << " ini_group_teo (2.1) new_t 2.1.8" << std::endl;
	track_group_.SpointV_cov_Strack_.time_stamp=teo_spoint_cov.time_stamp;
	std::cout << " ini_group_teo (2.1) new_t 2.1.9" << std::endl;
	track_group_.SpointV_cov_Strack_.vx=teo_spoint_cov.vx;
	std::cout << " ini_group_teo (2.1) new_t 2.1.10" << std::endl;
	track_group_.SpointV_cov_Strack_.vy=teo_spoint_cov.vy;
	std::cout << " ini_group_teo (2.1) new_t 2.1.11" << std::endl;*/
	/*track_group_.SpointV_cov_Strack_.cov[0]=cov_c[0];
	std::cout << " ini_group_teo (2.1) new_t 2.1.11.1" << std::endl;
	track_group_.SpointV_cov_Strack_.cov[1]=cov_c[1];
	std::cout << " ini_group_teo (2.1) new_t 2.1.11.2" << std::endl;
	track_group_.SpointV_cov_Strack_.cov[6]=cov_c[6];
	std::cout << " ini_group_teo (2.1) new_t 2.1.11.3" << std::endl;
	track_group_.SpointV_cov_Strack_.cov[7]=cov_c[7];
	std::cout << " ini_group_teo (2.1) new_t 2.1.12" << std::endl;
	track_group_.id=teo.id;
	//std::cout << " ini_group_teo (2.1) new_t 2.1.13" << std::endl;
	track_group_.probability_of_detection=teo.probability_of_detection;*/

	//std::cout << " ini_group_teo (2.1) new_t 2.1" << std::endl;
	/*track_group_.set_id(teo.id);
	//std::cout << " ini_group_teo (2.1) new_t 2.2" << std::endl;
	track_group_.set_probability_of_detection(teo.probability_of_detection);
	std::cout << " ini_group_teo (2.1) new_t 3 [print track group]" << std::endl;
	track_group_.print();
	std::cout << " ini_group_teo (2.1) new_t 3 [fin print track group]" << std::endl;*/
		//track_group_.fill_Strack_with_SpointV_cov(teo_spoint_cov, teo.id, teo.probability_of_detection);
	//std::cout << " ini_group_teo (2.2)" << std::endl;
	//track_group_.print();
	//SpointV_cov_track_group_( teo_spoint_cov.x,  teo_spoint_cov.y,  teo_spoint_cov.time_stamp,  teo_spoint_cov.vx, teo_spoint_cov.vy,cov_c);
	/*SpointV_cov_track_group_.x=teo_spoint_cov.x;//teo.getSpointV_cov();
	std::cout << " ini_group_teo (2.1) new_t 3.1" << std::endl;
	SpointV_cov_track_group_.y=teo_spoint_cov.y;
	std::cout << " ini_group_teo (2.1) new_t 3.2" << std::endl;
	SpointV_cov_track_group_.vx=teo_spoint_cov.vx;
	std::cout << " ini_group_teo (2.1) new_t 3.3" << std::endl;
	SpointV_cov_track_group_.vy=teo_spoint_cov.vy;
	std::cout << " ini_group_teo (2.1) new_t 3.4" << std::endl;
	SpointV_cov_track_group_.time_stamp=teo_spoint_cov.time_stamp;
	std::cout << " ini_group_teo (2.1) new_t 3.5" << std::endl;
	SpointV_cov_track_group_.cov[0]=cov_c[0];
	std::cout << " ini_group_teo (2.1) new_t 3.6" << std::endl;
	SpointV_cov_track_group_.cov[1]=cov_c[1];
	std::cout << " ini_group_teo (2.1) new_t 3.7" << std::endl;
	SpointV_cov_track_group_.cov[6]=cov_c[6];
	std::cout << " ini_group_teo (2.1) new_t 3.8" << std::endl;
	SpointV_cov_track_group_.cov[7]=cov_c[7];
*/
	//std::cout << " ini_group_teo (2.1) new_t 4 [print SpointV_cov_track_group_]" << std::endl;
	//SpointV_cov_track_group_.print();
	//track_group_probability_=teo.probability_of_detection;



	//std::cout << " ini_group_teo (3)" << std::endl;
	//track_group_.id=id_group_count;



	/*unsigned int coun_tracks_teo_=1;

	std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
	it_insert_track = mytracks_list_teo_.begin();

	std::cout << " 1 mytracks_list_teo_.empty()="<<mytracks_list_teo_.empty() << std::endl;

	if (!mytracks_list_teo_.empty())
	{	std::cout << "2  mytracks_list_teo_.empty()="<<mytracks_list_teo_.empty() << std::endl;
		while (it_insert_track != mytracks_list_teo_.end())
		{	std::cout << " ini_group_teo (4.4)" << std::endl;
			it_insert_track++;
			std::cout << " ini_group_teo (4.5)" << std::endl;
			if (it_insert_track->returnId() > coun_tracks_teo_)
			{	std::cout << " ini_group_teo (4.6)" << std::endl;
				break;
			}

		}
	}
	std::cout << " ini_group_teo (4.7)" << std::endl;
	mytracks_list_teo_.insert(it_insert_track, teo);
	std::cout << " ini_group_teo (5) out" << std::endl;*/
	//std::cout << " ini_group_teo (4)" << std::endl;
	//Stracks_tracks_teo_.clear();
	Stracks_tracks_teo_.resize(0);
	//std::cout << "4.2 Stracks_tracks_tibi_.empty()"<<Stracks_tracks_teo_.empty()<<"Stracks_tracks_teo_.size()"<<Stracks_tracks_teo_.size() << std::endl;
	Stracks_tracks_teo_.push_back(teo);
	//std::cout << " ini_group_teo (5)" << std::endl;


	return new_t;

}


Strack  GroupDataBaseFusion::ini_group_tibiAndteo(unsigned int in_track_tibi, Strack tibi,unsigned int in_track_teo, Strack teo,Strack combined_tibiAndteo, unsigned int id_group_count){
// initialize the group track, with two tracks (joined) of tibi and teo. Tracks of the same person.
//(In the ROS node we have tracks of tibi and teo in this iteration)
// initialice the intern Strack, and the Strack vector of tibi and teo tracks.

	//std::cout << " [ini_group_tibiAndteo]  (1); combined_tibiAndteo:" << std::endl;

	///
	//combined_tibiAndteo.print();
	//std::cout << " [ini_group_tibiAndteo] (2)" << std::endl;

	SpointV_cov teoAndTibi_spoint_cov=combined_tibiAndteo.getSpointV_cov();

	//std::cout << " [ini_group_tibiAndteo] (3)" << std::endl;
	std::vector<double> cov_c=teoAndTibi_spoint_cov.cov;
	//std::cout << " [ini_group_tibiAndteo] (4)" << std::endl;
	Strack new_t(teoAndTibi_spoint_cov.x,teoAndTibi_spoint_cov.y,teoAndTibi_spoint_cov.time_stamp,teoAndTibi_spoint_cov.vx,teoAndTibi_spoint_cov.vy,cov_c, combined_tibiAndteo.id, combined_tibiAndteo.probability_of_detection);
	//std::cout << " [ini_group_tibiAndteo] (5.1) Ini new_t.print():" << std::endl;
	//new_t.print();
	//std::cout << " [ini_group_tibiAndteo] (5.2) fin new_t.print():" << std::endl;
	//id_group_=id_group_count;
	id=id_group_count;
	//std::cout << " [ini_group_tibiAndteo] (6) (of group) id="<<id << std::endl;

	probability_of_detection=combined_tibiAndteo.probability_of_detection;
	//track_group_probability_=combined_tibiAndteo.probability_of_detection;
	//std::cout << " [ini_group_tibiAndteo] (7) (of group) probability_of_detection="<<probability_of_detection << std::endl;
	//track_group_x_=teoAndTibi_spoint_cov.x;
	x=teoAndTibi_spoint_cov.x;
	//std::cout << " [ini_group_tibiAndteo] (8) (of group) x="<<x << std::endl;
	y=teoAndTibi_spoint_cov.y;
	//track_group_y_=teoAndTibi_spoint_cov.y;
	//std::cout << " [ini_group_tibiAndteo] (9) (of group) y="<<y << std::endl;
	//track_group_vx_=teoAndTibi_spoint_cov.vx;
	vx=teoAndTibi_spoint_cov.vx;
	//std::cout << " [ini_group_tibiAndteo] (10) (of group) vx="<<vx << std::endl;
	//track_group_vy_=teoAndTibi_spoint_cov.vy;
	vy=teoAndTibi_spoint_cov.vy;
	//std::cout << " [ini_group_tibiAndteo] (11) (of group) vy="<<vy<< std::endl;
	//track_group_time_stamp_=teoAndTibi_spoint_cov.time_stamp;
	time_stamp=teoAndTibi_spoint_cov.time_stamp;
	//std::cout << " [ini_group_tibiAndteo] (12) (of group) time_stamp="<<time_stamp << std::endl;

	cov.assign(cov_c.begin(), cov_c.end());

	/*cov.reserve(16);
	std::cout << " [ini_group_tibiAndteo] (13) "<< std::endl;
	cov.resize(16,0.0);
	std::cout << " [ini_group_tibiAndteo] (13.1) "<< std::endl;
	double cov_0=cov_c[0];
	std::cout << " [ini_group_tibiAndteo] (13.2) "<< std::endl;
		cov.push_back(cov_0);
		std::cout << " [ini_group_tibiAndteo] (14.2) "<< std::endl;
		double cov_1=cov_c[1];
		cov[1]=cov_1;
		std::cout << " [ini_group_tibiAndteo] (14.3) "<< std::endl;
		double cov_2=cov_c[2];
		cov[2]=cov_2;
		std::cout << " [ini_group_tibiAndteo] (14.4) "<< std::endl;
		double cov_3=cov_c[3];
		cov[3]=cov_3;
		std::cout << " [ini_group_tibiAndteo] (14.5) "<< std::endl;
		double cov_4=cov_c[4];
		cov[4]=cov_4;
		double cov_5=cov_c[5];
		cov[5]=cov_5;
		double cov_6=cov_c[6];
		cov[6]=cov_6;
		double cov_7=cov_c[7];
		cov[7]=cov_7;
		double cov_8=cov_c[8];
		cov[8]=cov_8;
		double cov_9=cov_c[9];
		cov[9]=cov_9;
		double cov_10=cov_c[10];
		cov[10]=cov_10;
		double cov_11=cov_c[11];
		cov[11]=cov_11;
		double cov_12=cov_c[12];
		cov[12]=cov_12;
		double cov_13=cov_c[13];
		cov[13]=cov_13;
		double cov_14=cov_c[14];
		cov[14]=cov_14;
		std::cout << " [ini_group_tibiAndteo] (14.7) "<< std::endl;
		double cov_15=cov_c[15];
		cov[15]=cov_15;*/
	//std::cout << " [ini_group_tibiAndteo] (15) cov[0]="<<cov[0]<<"; cov[1]="<<cov[1]<<"; cov[6]="<<cov[6]<<"; cov[7]="<<cov[7] << std::endl;



	///

	//Stracks_tracks_tibi_.clear();
	///Stracks_tracks_teo_.clear();

	//mytracks_list_tibi_.clear();
	//mytracks_list_teo_.clear();


	//track_group_=combined_tibiAndteo;
	//track_group_.id=id_group_count;


	//////////////////////////



	//track_group_.Strack();
	//track_group_.setSpointV_cov_Strack(teo_spoint_cov);
	//track_group_(teo_spoint_cov.x,teo_spoint_cov.y,teo_spoint_cov.time_stamp,teo_spoint_cov.vx,teo_spoint_cov.vy,cov_c, teo.id, teo.probability_of_detection);
	/*track_group_.x=teoAndTibi_spoint_cov.x;
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.1" << std::endl;
	track_group_.y=teoAndTibi_spoint_cov.y;
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.2" << std::endl;
	track_group_.time_stamp=teoAndTibi_spoint_cov.time_stamp;
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.3" << std::endl;
	track_group_.vx=teoAndTibi_spoint_cov.vx;
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.4" << std::endl;
	track_group_.vy=teoAndTibi_spoint_cov.vy;
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.5" << std::endl;
	track_group_.cov=cov_c;
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.6 (track_group_.print:)" << std::endl;
	track_group_.print();
	std::cout << " ini_group_tibiAndteo (2.1) new_t 2.1.6 ( fin track_group_.print:)" << std::endl;*/
	////////////////////////////

	/*unsigned int coun_tracks_tibi_=1;

	std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
	it_insert_track = mytracks_list_tibi_.begin();

	std::cout << "1 mytracks_list_tibi_.empty()="<<mytracks_list_tibi_.empty() << std::endl;
	if (!mytracks_list_tibi_.empty())
	{std::cout << "2 mytracks_list_tibi_.empty()="<<mytracks_list_tibi_.empty() << std::endl;
		while (it_insert_track != mytracks_list_tibi_.end())
		{
			it_insert_track++;
			if (it_insert_track->returnId() > coun_tracks_tibi_)
			{
				break;
			}

		}
	}
	mytracks_list_tibi_.insert(it_insert_track, tibi);

	unsigned int coun_tracks_teo_=0;

	//std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
	it_insert_track = mytracks_list_teo_.begin();

	if (!mytracks_list_teo_.empty())
	{
		while (it_insert_track != mytracks_list_teo_.end())
		{
			it_insert_track++;
			if (it_insert_track->returnId() > coun_tracks_teo_)
			{
				break;
			}

		}
	}
	mytracks_list_teo_.insert(it_insert_track, teo);*/

	//std::cout << "[ini_group_tibiAndteo] 6" << std::endl;
	Stracks_tracks_tibi_.resize(0);
	Stracks_tracks_tibi_.push_back(tibi);
	Stracks_tracks_teo_.resize(0);
	Stracks_tracks_teo_.push_back(teo);

	//std::cout << " fin ini_group_tibiAndteo" << std::endl;

	return 	new_t;

}


Strack GroupDataBaseFusion::update_group(std::vector<Strack> in_all_tracks_tibi,std::vector<Strack> in_all_tracks_teo, SpointV_cov track_pose_group_predicted){

// update the grup. We enter all the tibi and teo tracks + the prediction of the track of the group.
	// then we associate by distance the tibi and teo tracks that are near for the prediction in this iteration.
	// first we fount the tracks to be associated with the group.
	// Then with a ponderated mean we obtain the real group track position, velocity, covariance and probability.
	//std::cout << " ini update_group" << std::endl;
	ids_tracks_group_tibi_.clear();
	ids_tracks_group_teo_.clear();

	for(unsigned int t=0;t<in_all_tracks_tibi.size();t++){
		ids_tracks_group_tibi_.push_back(false);
	}
	for(unsigned int t=0;t<in_all_tracks_teo.size();t++){
		ids_tracks_group_teo_.push_back(false);
	}



	std::vector<Strack> tracks_to_fuse_;

	//std::vector<bool> track_tibi_finded_;
	//track_tibi_finded_.resize(mytracks_list_tibi_.size());

	double time_tibi=0;
	double time_teo=0;

	if(!in_all_tracks_tibi.empty()){
		Strack tibi_track_act=in_all_tracks_tibi[0];
		SpointV_cov tibi_track_act2=tibi_track_act.getSpointV_cov();
		time_tibi=tibi_track_act2.time_stamp;
		// find tracks tibi del grupo. (no se si buscar lo de los id's que tengo, o si coger los de tentro de la prediccion o si usar ambas...=> AMBAS)
		for(unsigned int d=0; d<in_all_tracks_tibi.size();d++){
			// find tracks of tibi inside track group prediction. (add new tracks with different id)
			Spoint track_tibi_actual=in_all_tracks_tibi[d].getSpointTrack();
			double distance_act=track_pose_group_predicted.distance(track_tibi_actual);

			bool now_inseted=false;
			//for (std::list<Strack>::iterator it_myTrack =mytracks_list_tibi_.begin(); it_myTrack != mytracks_list_tibi_.end();it_myTrack++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			for(unsigned int t=0; t<Stracks_tracks_tibi_.size();t++)
			{
				if(in_all_tracks_tibi[d].id==Stracks_tracks_tibi_[t].id){
					//it_myTrack->Strack( in_all_tracks_tibi[d] );
					tracks_to_fuse_.push_back(in_all_tracks_tibi[d]);
					Stracks_tracks_tibi_[t]=in_all_tracks_tibi[d];
					now_inseted=true;
				}
			}


			if((distance_act<threshold_dist_assos_group_)&&(!now_inseted)){

			//	std::cout << " (tibi) distance_act="<<distance_act<<"; threshold_dist_assos_group_="<<threshold_dist_assos_group_ << std::endl;

				tracks_to_fuse_.push_back(in_all_tracks_tibi[d]); // + falta añadir a la lista.

				// insert track. add this track to the tibi tracks list
				/*unsigned int coun_tracks_tibi_=get_max_id_list(mytracks_list_tibi_); // todo: need to know the máx id of the tracks of the list.

				std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
				it_insert_track = mytracks_list_tibi_.begin();

				if (!mytracks_list_tibi_.empty())
				{
					while (it_insert_track != mytracks_list_tibi_.end())
					{
						it_insert_track++;
						if (it_insert_track->returnId() > coun_tracks_tibi_)
						{
							break;
						}

					}
				}
				mytracks_list_tibi_.insert(it_insert_track, in_all_tracks_tibi[d]);*/
				Stracks_tracks_tibi_.push_back(in_all_tracks_tibi[d]);

				ids_tracks_group_tibi_[in_all_tracks_tibi[d].id]=true;


			}
		}
	}

	if(!in_all_tracks_teo.empty()){
		Strack teo_track_act=in_all_tracks_teo[0];
		SpointV_cov teo_track_act2=teo_track_act.getSpointV_cov();
		time_teo=teo_track_act2.time_stamp;
		// fin tracks teo
		for(unsigned int d=0; d<in_all_tracks_teo.size();d++){
			Spoint track_teo_actual=in_all_tracks_teo[d].getSpointTrack();
			double distance_act=track_pose_group_predicted.distance(track_teo_actual);
			bool now_inseted=false;
			//for (std::list<Strack>::iterator it_myTrack =mytracks_list_teo_.begin(); it_myTrack != mytracks_list_teo_.end();it_myTrack++){ //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
			for(unsigned int t=0; t<Stracks_tracks_teo_.size();t++){

				if(in_all_tracks_teo[d].id==Stracks_tracks_teo_[t].id){
					//it_myTrack->Strack( in_all_tracks_teo[d] );
					tracks_to_fuse_.push_back(in_all_tracks_teo[d]);
					Stracks_tracks_teo_[t]=in_all_tracks_teo[d];
					now_inseted=true;
				}
			}

			// find tracks of teo inside track group prediction. (add new tracks with different id)
			if((distance_act<threshold_dist_assos_group_)&&(!now_inseted)){
				tracks_to_fuse_.push_back(in_all_tracks_teo[d]); // + falta añadir a la lista.
				//std::cout << " (teo) distance_act="<<distance_act<<"; threshold_dist_assos_group_="<<threshold_dist_assos_group_ << std::endl;

				// insert track. add this track to the tibi tracks list
				/*unsigned int coun_tracks_teo_=get_max_id_list(mytracks_list_teo_); // todo: need to know the máx id of the tracks of the list.

				std::list<Strack>::iterator it_insert_track; //I search in my track list where I have to put this track for maintain the track list in order.
				it_insert_track = mytracks_list_teo_.begin();

				if (!mytracks_list_teo_.empty())
				{
					while (it_insert_track != mytracks_list_teo_.end())
					{
						it_insert_track++;
						if (it_insert_track->returnId() > coun_tracks_teo_)
						{
							break;
						}

					}
				}
				mytracks_list_teo_.insert(it_insert_track, in_all_tracks_teo[d]);*/

				Stracks_tracks_teo_.push_back(in_all_tracks_teo[d]);

				ids_tracks_group_teo_[in_all_tracks_teo[d].id]=true;

			}

		}
	}

	double actual_time=0;

	if(time_tibi>=time_teo){
		actual_time=time_tibi;
	}else{
		actual_time=time_teo;
	}

	// update track group. Generate track group with the tracks to fuse (conservamos id, solo update x, y, vx, vy, probabilidad, covarianzas)
	/*double x;
	double y;
	double vx;
	double vy;
	double prob;*/
	double cov_0;
	double cov_1;
	double cov_6;
	double cov_7;
	/*std::vector<double> cov;
	cov.reserve(16);
	cov.resize(16,0.0);*/

	for(unsigned int t=0;t<tracks_to_fuse_.size();t++){  //ojo. puede que sea x_ etc.
		SpointV_cov track_act=tracks_to_fuse_[t].getSpointV_cov();
		x=x+track_act.x;
		y=y+track_act.y;
		vx=vx+track_act.vx;
		vy=vy+track_act.vy;
		probability_of_detection=probability_of_detection+tracks_to_fuse_[t].probability_of_detection;
		cov_0=cov_0+track_act.cov[0];
		cov_1=cov_1+track_act.cov[1];
		cov_6=cov_6+track_act.cov[6];
		cov_7=cov_7+track_act.cov[7];
	}

	int size;

	if(debug_groups_){
	std::cout << " tracks_to_fuse_.size()="<<tracks_to_fuse_.size() << std::endl;
	std::cout << " x="<<x << std::endl;
	std::cout << " y="<<y << std::endl;
	std::cout << " vx="<<vx << std::endl;
	std::cout << " vy="<<vy << std::endl;
	}

	if(tracks_to_fuse_.size()==0){
		size=1;
	}else{
		size=tracks_to_fuse_.size();
	}


	x=x/size;
	y=y/size;
	vx=vx/size;
	vy=vy/size;
	probability_of_detection=probability_of_detection/size;
	cov_0=cov_0/size;
	cov_1=cov_1/size;
	cov_6=cov_6/size;
	cov_7=cov_7/size;
	cov[0]=cov_0;
	cov[1]=cov_1;
	cov[6]=cov_6;
	cov[7]=cov_7;
	time_stamp=actual_time;

	Strack new_t(x,y,actual_time,vx,vy,cov,id,probability_of_detection);

	if(debug_groups_){
	std::cout << " final Strack" << std::endl;
	print();
	std::cout << " fin update_group" << std::endl;
	}

	/*track_group_x_=x;
	track_group_y_=y;
	track_group_vx_=vx;
	track_group_vy_=vy;
	track_group_probability_=prob;*/


	return new_t;
}


unsigned int GroupDataBaseFusion::get_max_id_list(std::list<Strack> in_list){
// Ya no se usa! solo se usaba cuando usaba lista interna!
	unsigned int max_id=0;

	for (std::list<Strack>::iterator it_myTrack2 =in_list.begin(); it_myTrack2 != in_list.end();it_myTrack2++) //me recorro mis tracks, aunque uso las spose_cov de los tracks que me dan a mi entrada. (los que me predice su posición gonzalo)
	{
		if(max_id<it_myTrack2->returnId()){
			max_id=it_myTrack2->returnId();
		}
	}

	return max_id;

}



