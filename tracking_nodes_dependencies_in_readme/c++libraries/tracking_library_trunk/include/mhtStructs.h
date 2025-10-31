/*
 * mhtStructs.h
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

#ifndef mhtStructs_H
#define mhtStructs_H

#include "iri_geometry.h"
#include "scene_elements/person_abstract.h"

class Sdetection : public Spoint_cov //TODO: inheritance from SpointCovaraince
{
	public:
		int type;												// type of detection = laser, vision, etc.
		double probability;										// probability of the detector of this type of detection.
		Sdetection();
		Sdetection( double x_ , double y_ ,double time_stamp_=0.0, const std::vector<double>& cov_=std::vector<double>(), int type_=0, double prob_=0.0); 		// my parameters are type_ and prob_.
		~Sdetection();
		Spoint getSpointDetection ();		//get the Spoint + time stamp of the detection.
		Spoint_cov getSpoint_cov (); 			//get the Spoint_cov (point + covariance + time stamp )
		void print_spoint_DET();
		double get_detector_probability();  //para obtener la probabilidad del detector.
};


class Strack : public SpointV_cov
{
	public:
		unsigned int id;								//id, number that identifies this track.
		double probability_of_detection;	//probability of association for this track.
		Strack();
		Strack( double x_ , double y_ , double time_stamp_=0, double vx_=0.0, double vy_=0.0,
				const std::vector<double>& cov_= std::vector<double>(), unsigned int id_=0, double probability_of_detection_=0.0 );
		//Strack( Strack& track_in ); 		// funcion para cambiar Strack por otro. (para database companion)
							// my parameters are id_ and probability_of_detection_.
		//SpointV_cov SpointV_cov_Strack_;
		~Strack();

		Spoint getSpointTrack(); //return the only the position of my track.
		Spoint_cov getSpoint_cov();
		SpointV_cov getSpointV_cov(); //return the position with covariances, time stamp and velocities of my track.
		/* void setSpointV_cov_Strack(SpointV_cov in_SpointV_cov){
			// SpointV_cov(in_SpointV_cov.x,in_SpointV_cov.y,in_SpointV_cov.time_stamp,in_SpointV_cov.vx,in_SpointV_cov.vy,in_SpointV_cov.cov);
			 SpointV_cov_Strack_=SpointV_cov( in_SpointV_cov.x , in_SpointV_cov.y , in_SpointV_cov.time_stamp , in_SpointV_cov.vx, in_SpointV_cov.vy, in_SpointV_cov.cov);

		 };*/

		/* SpointV_cov getSpointV_cov_Strack(){
				return SpointV_cov_Strack_;

			 };*/

		int returnId ();  					//return the Id of the track.
		double returnProbability ();		//return the probability that I associate at this track.
		Strack Create_Strack_with_SpointV_cov( SpointV_cov track_pose, unsigned int identf, double probabil );
		void fill_Strack_with_SpointV_cov( SpointV_cov track_pose, unsigned int identf, double probabil );
		void set_id(unsigned int in_id){
			id=in_id;
		};
		void set_probability_of_detection(double in_probability_of_detection){
			probability_of_detection=in_probability_of_detection;
		};

		void print();
};



//class for clusters

class Scluster
{
	public:
	unsigned int id_cluster_;
	std::vector<unsigned int> tracks_;
	std::vector<unsigned int> detections_;
	std::vector<std::vector<double>> association_distances_;
	std::vector<std::vector<unsigned int>> tracks_associated_;
	double Pd_cluster_;  //valor inicial y por defecto ==1. TODO: quitar!!! aquí no hacen falta!!!
	std::vector<bool> tracks_separados_; // true if tracks are separated. TODO: mirar si hace falta, creo que es más en la ineterna del mht que hace falta...
	double cluster_life_; // time that the cluster is alive after the tracks are separated. ODO: mirar si hace falta, creo que es más en la ineterna del mht que hace falta...
	//unsigned int id_cluster_;
	//std::vector<unsigned int> tracks_;
	//std::vector<unsigned int> current_detections_;
	// std::list<Ctracks*> cluster_tracks_;  //smart pointers. if needed


	Scluster();
	Scluster(unsigned int id_cluster, const std::vector<unsigned int>& tracks= std::vector<unsigned int>() , const std::vector<unsigned int>& detections= std::vector<unsigned int>(),const std::vector<std::vector<double>>& association_distances= std::vector<std::vector<double>>(),const std::vector<std::vector<unsigned int>> tracks_associated= std::vector<std::vector<unsigned int>>(), double Pd_cluster=1.0 );
	//Scluster( unsigned int id_cluster, const std::vector<unsigned int>& tracks= std::vector<unsigned int>() , const std::vector<unsigned int>& detections= std::vector<unsigned int>(), double Pd_cluster=1.0 );
	~Scluster();
	unsigned int get_idCluster();
	void set_idCluster(unsigned int id_actual);
	//const std::vector<unsigned int>* get_tracks_in_cluster() const;
	std::vector<unsigned int> get_tracks_in_cluster();
	//const std::vector<unsigned int>* get_detections_in_cluster() const;
	std::vector<unsigned int> get_detections_in_cluster();
	void set_detections_in_cluster( unsigned int detection_ );
	void set_tracks_in_cluster( unsigned int track_ );
	double get_Pd_cluster();
	void set_Pd_cluster(double Pd_actual);
	std::vector<std::vector<double>> get_association_distances_Scluster();
	void print_cluster();
};

class N_hypothesis
{
	public:
	int N_det_;
	int N_tgt_;
	int N_fa_;
	int N_new_;
	int N_cluster_no_det_;
	int N_cluster_no_confirm_;
	std::vector<double> distances_sub_hypothesis_;
	std::vector<double> delta_sub_hypothesis_;
	std::vector<double> laser_beta_track_no_confirmado_;
	double laser_beta_cluster_no_confirmado_;
	double Pd_orientation_;

	N_hypothesis();
	N_hypothesis(int  N_det, int N_tgt, int N_fa, int N_new, int N_cluster_no_det, int N_cluster_no_confirm, std::vector<double>& distances_sub_hypothesis, std::vector<double>& delta_sub_hypothesis, std::vector<double>& laser_beta_track_no_confirmado, double laser_beta_cluster_no_confirmado, double Pd_orientation);
	~N_hypothesis();
	void print_N_hypothesis();

};

class Exitcluster
{
	public:
	unsigned int id_cluster_;
	std::vector<unsigned int> tracks_; //TODO: necesito la posicion de los tracks.
	//std::vector<unsigned int> detections_;
	//std::vector<std::vector<unsigned int>> tracks_associated_;
	//std::vector<std::vector<double>> association_distances_;
	std::vector<SpointV_cov> position_tracks_; // position for the tracks before the correction step. (correction Kalman)
	Spoint centroid_cluster_; // centroid for the cluster, calculate de median distance with the positions of all tracks!
	Spoint_cov centroid_cluster_with_cluster_area_;
	double Pd_cluster_;  //valor inicial y por defecto ==1. TODO: quitar!!! aquí no hacen falta!!!
	double mediumVcluster_;
	bool equal_velocity_tracks_;
	bool equal_orientation_tracks_;
	std::vector<bool> tracks_separados_; // true if tracks are separated. TODO: mirar si hace falta, creo que es más en la ineterna del mht que hace falta...
	double cluster_life_; // time that the cluster is alive after the tracks are separated. ODO: mirar si hace falta, creo que es más en la ineterna del mht que hace falta...



	Exitcluster();
	Exitcluster(unsigned int id_cluster, const std::vector<unsigned int>& tracks= std::vector<unsigned int>() , const std::vector<SpointV_cov>& position_tracks= std::vector<SpointV_cov>(),Spoint centroid_cluster= Spoint(), Spoint_cov centroid_cluster_with_cluster_area= Spoint_cov(), double Pd_cluster=0.0,double mediumVcluster=0.0, bool equal_velocity_tracks=false, bool equal_orientation_tracks=false);
		//Scluster( unsigned int id_cluster, const std::vector<unsigned int>& tracks= std::vector<unsigned int>() , const std::vector<unsigned int>& detections= std::vector<unsigned int>(), double Pd_cluster=1.0 );
	~Exitcluster();
};


class GroupDataBaseFusion : public Strack
{

	// funcion que sirve de container para los grupos. + junta los tracks de tibi y teo en el grupo que le pertoque.
	// Si un track de tibi o teo cae a cierta distancia de la prediccion del grupo, ese track sirve para hacer el update de la posicion de este grupo.

	public:


	// tiene interno un Strack! que es la posicion del grupo actual!!!

	std::vector<Strack> Stracks_tracks_tibi_;

	//std::list<Strack> mytracks_list_tibi_;

	std::vector<Strack> Stracks_tracks_teo_;
	//std::list<Strack> mytracks_list_teo_;

	//Strack track_group_;

	/*double track_group_x_; // estas ya no se usan!!!
	double track_group_y_;
	double track_group_vx_;
	double track_group_vy_;
	double track_group_time_stamp_;
	//double track_group_probability_;
	 * //unsigned int id_group_;
	 */


	//SpointV_cov SpointV_cov_track_group_;


	double threshold_dist_assos_group_;

	std::vector<bool> ids_tracks_group_tibi_;
	std::vector<bool> ids_tracks_group_teo_;


	// clase functions.
	GroupDataBaseFusion();
	GroupDataBaseFusion(unsigned int in_id_group,std::vector<Strack> in_Stracks_tracks_tibi,std::vector<Strack> in_Stracks_tracks_teo, Strack in_track_group);
	// this has to be used in case of initial iteration with tibi and teo tracks.
	~GroupDataBaseFusion();
	void print_GroupDataBaseFusion();
	Strack update_group(std::vector<Strack> in_all_tracks_tibi,std::vector<Strack> in_all_tracks_teo, SpointV_cov track_pose_group_predicted);
	Strack ini_group_tibi(unsigned int in_track_tibi, Strack tibi, unsigned int id_group_count);
	Strack ini_group_teo(unsigned int in_track_teo, Strack teo, unsigned int id_group_count);
	Strack ini_group_tibiAndteo(unsigned int in_track_tibi, Strack tibi,unsigned int in_track_teo, Strack teo,Strack combined_tibiAndteo, unsigned int id_group_count);

	Strack get_Strack_track_group(){
		//return track_group_;
		Strack new_t(x,y,time_stamp,vx,vy,cov,id,probability_of_detection);
		return new_t;
	};
	unsigned int get_group_id(){
		return id;
	};
	unsigned int get_max_id_list(std::list<Strack> in_list);

	void set_threshold_dist_assos_group(double in_threshold_dist_assos_group){
		threshold_dist_assos_group_=in_threshold_dist_assos_group;
	};


	std::vector<bool> get_ids_tracks_group_tibi(){
		return ids_tracks_group_tibi_;
	}
	std::vector<bool> get_ids_tracks_group_teo(){
		return ids_tracks_group_teo_;
	}

	bool debug_groups_;
};

#endif

