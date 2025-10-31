/*
 * mht.h
 *
 *  Created on: 2013
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

#ifndef _PEOPLE_mhtING_MHT_H
#define _PEOPLE_mhtING_MHT_H
#include "prediction_bhmip.h"

#include "track.h"
#include "mhtStructs.h"
#include "Clusters.h"
#include "odometry.h"
#include "total_velocity.h"
#include "Groups.h"

#include <list>
#include <iostream>   //for structs
#include <eigen3/Eigen/Dense>

//

#include <ctime>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>


class Cmht
{
  public:
    Cmht();
    ~Cmht();
    void get_total_velocity(Cvelocity velocity);
    void update_total_velocity();
    void mht_odom_update( Codometry& odome );
    void mht_update( std::vector<Sdetection>& obs , double external_time_stamp=0.0);
    void mht_Cscene_se_in_tracks();
    std::vector<Strack>&  get_persons_mht()  { return exit_params_mht_;}
    std::vector<Strack>&  get_propagations_mht()  { return exit_propagations_mht_;}
    std::vector<Exitcluster>&  get_clusters_mht()  { return exit_clusters_mht_;}
    double get_dt(){ return dt_; };
    double get_time_stamp(){ return time_stamp_; };
    void set_dt(double dt_in){ dt_=dt_in; };

    //my parameters to change outside of the code in ROS.


    double get_threshold_distance(){ return mht_threshold_distance_; };  //association distance.
    void set_threshold_distance(double threshold_distance_in);

    double get_threshold_confirmation_track(){ return mht_threshold_confirmation_track_; }; //confirmation_threshold.
    void set_threshold_confirmation_track(double threshold_confirmation_track_in);

    double get_threshold_no_deteccion(){ return threshold_no_deteccion_; }; //elimination_threshold.
    void set_threshold_no_deteccion(double threshold_no_deteccion_in){threshold_no_deteccion_=threshold_no_deteccion_in;};

    double get_laser_Pd(){ return mht_laser_Pd_; }; //detection probability laser
    void set_laser_Pd(double laser_Pd_in);

    double get_laser_beta_ft(){ return mht_laser_beta_ft_; }; //beta falsa alarma Laser.
    void set_laser_beta_ft(double laser_beta_ft_in);

    double get_laser_beta_nt(){ return mht_laser_beta_nt_; }; //beta new person Laser.
    void set_laser_beta_nt(double laser_beta_nt_in);

    double get_laser_beta_no_detection(){ return mht_laser_beta_no_detection_; }; //beta no detection Laser (for eliminate tracks)
    void set_laser_beta_no_detection(double laser_beta_no_detection_in); // beta_no_Det=0.95 (increase)==remain more the track.
    																	// beta_no_Det=0.95 (decrease)==eliminate faster the track.

    double get_laser_beta_track_no_confirmed(){ return mht_laser_beta_track_no_confirmed_; };
    void set_laser_beta_track_no_confirmed(double laser_beta_track_no_confirmed_in);

    double get_increment_iteration_track_no_confirmed(){ return mht_increment_iteration_track_no_confirmed_; }; //beta increment no confirmed Laser (for confirm tracks)
    void set_increment_iteration_track_no_confirmed(double increment_iteration_track_no_confirmed_in);
    											// beta_increment_no_confirmed=0.003 (increase)==confirm faster the track.
    											// beta_increment_no_confirmed=0.003 (decrease)==confirm more slow the track.


    void set_generate_database_companion(bool in_generate_database_companion){
    	generate_database_companion_=in_generate_database_companion;
    };
    void set_fuse_tibi_and_teo_tracks(bool in_fuse_tibi_and_teo_tracks){
    	fuse_tibi_and_teo_tracks_=in_fuse_tibi_and_teo_tracks;
    };
    void set_debug_fuse_tibi_and_teo_tracks(bool in_debug_fuse_tibi_and_teo_tracks){
    	debug_fuse_tibi_and_teo_tracks_=in_debug_fuse_tibi_and_teo_tracks;
    };


    void set_mht_velocity_margin(double in_mht_velocity_margin){
    	mht_velocity_margin_=in_mht_velocity_margin;

    }

    void set_augment_covariance_track(double in_aug_cov_prediction ){
    	aug_cov_prediction=in_aug_cov_prediction;
    }

    void mht_print_results();
    std::vector<exitGroup>  get_groups_mht(){return exitGroups_;};

    void set_covariance_no_track(double in_covariance_no_track){
    	covariance_no_track_=in_covariance_no_track;
    }

  private:

    void new_matlab_file();

    void initialization_param_of_associations(std::vector<Sdetection>& obs);
    void association_by_distances( std::vector<Sdetection>& obs ); // void calculate_distances();
    void mht_inicialization_tracks( std::vector<Sdetection>& obs);
    void mht_association_det_with_tracks( std::vector<Sdetection>& obs);
    void mht_continuation_tracks( std::vector<Sdetection>& obs );
    void mht_remove_tracks_by_probability();
    void mht_confirm_tracks_by_bool();
    void mht_update_tracks( std::vector<Sdetection>& obs );
    void mht_propagation_tracks();

    //functions two detections in one track

    void mht_look_tracks_associaten_in_one_detection( );
    void mht_find_two_dets_in_one_track();

    // clusters functions

   void mht_insert_cluster( const std::vector<unsigned int>& tracks= std::vector<unsigned int>() , const std::vector<unsigned int>& detections= std::vector<unsigned int>(),const std::vector<std::vector<double>>& distances= std::vector<std::vector<double>>(),const std::vector<std::vector<unsigned int>>& tracks_associated= std::vector<std::vector<unsigned int>>() );
   //void mht_merge_split_cluster( const std::vector<unsigned int>& tracks , const std::vector<unsigned int>& detections );
   void mht_made_clusters( std::vector<Sdetection>& obs );
   void find_Ctrack_in_cluster(std::vector<unsigned int> tracks);
   void mht_insert_cluster_in_mht_cluster_list( Ccluster cluster_new );
   bool mht_compare_clusters(std::vector<unsigned int> actual_tracks,std::vector<unsigned int> cluster_mht_tracks);
   void mht_update_mht_cluster_list(std::vector<Sdetection>& obs);
   void mht_update_cluster_tracks(std::vector<Sdetection>& obs);
   void mht_cluster_update_with_many_tracks(std::list<Ccluster>::iterator it_myCluster1);
   void mht_remove_clusters_by_probability();
   void mht_remove_clusters_with_track_removed(unsigned int id);
   //void mht_remove_clusters_with_only_one_track();
   void mht_remove_repeated_clusters();
   void mht_update_tracks_with_winner_Pd_cluster();
   void print_clusters_mht();
   void mht_exit_clusters();
   std::vector<unsigned int> mht_find_tracks_act_in_cluster_mht_for_update_cluster_list_mht(const std::vector<unsigned int>& tracks_act, const std::vector<unsigned int>& tracks_mht);
   std::vector<bool> mht_find_tracks_mht_in_cluster_act_for_update_separate_clusters(const std::vector<unsigned int>& tracks_act, const std::vector<unsigned int>& tracks_mht, std::vector<unsigned int>& tracks_in_mht_but_not_in_actual_cluster);
//my parameters to change outside of the code in ROS.

   //cost to go: distance between poses
   double cost_to_go( SpointV target1, const Spoint& target2 , const std::vector<double>* params);
   bool arrange_tracks_of_one_cluster(std::vector<unsigned int> tracks_mht, std::vector<unsigned int> tracks);



   // FUNCTIONS FOR Groups
   void distances_for_groups_mht();
   void make_groups_mht();
   bool find_group(std::vector<unsigned int> tracks_group_act, unsigned int& id_group);
   void restart_groups();
   void remove_groups();

   void central_point_groups();
   void groups_out();
   void update_groups();

   // FUNCTIONS ONLY FOR CARGO ANTS
   	  void publish_only_one_track_in_groups();
      void publish_only_one_track_in_clusters();
      void mht_filter_by_velocity();

   // function to generate data base robot companion
   void eliminate_new_tracks_bad_associated();
   void save_tracks_in_txt_tibi_and_teo_companion_database();
   ///////////
   std::vector<unsigned int> arrange_tracks(std::vector<unsigned int> tracks);

   //enum detection_type { Laser=0 , PointCloud , Vision, ZigBee};
   bool cargo_ANTS;
   double aug_cov_prediction;

   	bool debug_cruces_fallos;
   	bool debug_odometry_;
   	bool debug_cruces_v_;
   	bool debug_cruces_2015_;
   	bool debug_groups_;
   	bool pruebas_C_;


    double mht_threshold_distance_;
    double mht_threshold_confirmation_track_;
    double mht_laser_Pd_;
    double mht_laser_beta_ft_;
    double mht_laser_beta_nt_;
    double mht_laser_beta_no_detection_;
    double mht_laser_beta_track_no_confirmed_;
    double mht_increment_iteration_track_no_confirmed_;


    int counter_;
    double dt_;
    double total_dt_;
    double time_stamp_;
    double time_stamp_anterior_;

    std::list<Ctrack> mytracks_list_;
    std::vector<Strack> exit_params_mht_; //TODO: use only one list!!!tracks, probability and id, that tracker returned by the exit.
    std::vector<Strack> exit_propagations_mht_;
    std::vector<SdetectionObservation> kalman_tracks_out_;
    Cprediction_bhmip scene_tracks_people_prediction_;//TODO: Gonzalo. De momento usamos low pass filter.
    double threshold_distance_;
    double threshold_distance_groups_;
    double threshold_no_deteccion_;
    double covariance_no_track_; // if the covariance of the track surpass this covariance, the track is removed.

    unsigned int count_my_tracks_; //my tracks counter.
    unsigned int count_my_clusters_actual_;
    unsigned int count_my_clusters_; //my cluster counter.=> Cclusters, los que guardas en tiempo!!!

    std::vector< std::vector<unsigned int> > OneDetection_ntracks_association_; //es un vector, que cada posición es una detección, en orden=1,2,etc
    																//y en cada posición correspondiente a 1 detección,
    						//contiene un vector que indica los id de los tracks a los que se puede asociar esta detección.
    std::vector<bool> track_no_associated_; //cada posición del vector se corresponde con el trac id=1,2,etc.
    	//y si es true=1=track_no_asociado, si es false=0=track asociado. (buscaremos los tracks no asociados.)
    std::vector< std::vector<double> > vector_distances_;  //pos [1][n] para la detección 1 guardo sus n distancias a los n tracks diferentes.

    //two detections in one Track

    std::vector<unsigned int> tracks_first_det_;
    std::vector<unsigned int> tracks_other_det_for_compare_;

    //cluster variables (cruces personas!)

    std::list<Ccluster> mycluster_list_;
    std::list<Ccluster>::iterator it_insert_cluster_;
    std::list<Scluster> mycluster_list_actual_;
    std::list<Scluster>::iterator it_insert_cluster_actual_;
    std::vector<unsigned int> tracks_other_detections_for_compare_;
    std::vector<unsigned int> tracks_associated_this_detection_;
    std::vector<bool> detection_in_cluster_;
    std::vector<std::list<Ctrack>::iterator> it_Ctracks_cluster_mht_;
    double threshold_remove_cluster_;
    unsigned int num_det_iter_;
    std::vector<Exitcluster> exit_clusters_mht_;
    std::vector<bool> no_new_track_; // when have to detections inside one cluster, one detection is associated to the cluster and the other is
    					// a lost detection!. (is good for not change id's).
    std::vector<bool> track_no_asociated_in_cross_situation_;
    unsigned int max_num_tracks_in_cluster_allowed_; //más numero de tracks permitidos en el cluster, para hacer un update cluster con probabilidades.
    													// max_num_tracks_in_cluster_allowed_= 4; //con 6tracks, ya le cuesta!!! salen infinitas combinaciones de probabilidades!.
    // for filter detections taking in to account the distance around the robot

    double perimeter_distance_;

    // provisional variable for crossing persons by velocity. Clusters.
    std::vector<SpointV_cov> initial_crossing_tracks;
    std::vector<unsigned int> initial_id_crossing_tracks;


    // variables for vehicle velocity estimation.
    SpointV_cov vehicle_pose_;
    Cprediction_bhmip vehicle_prediction_;
	double total_px_; // [m/s]
	double total_py_; // [m/s]
	double total_oz_; // [rad]
	double time_act_veh_;
	bool first_time_;

	//variables for cargo ANTS tracks.
	 std::vector<bool> exit_track_cargo_ANTS_;

	 // variables for groups
	 std::vector< std::vector<double> > vect_dist_between_tracks_; // [ vec_dist track1 to the other tracks if distance is less than a threshold] [ vec_dist track2 to the other tracks if distance is less than a threshold] ... [ vec_dist trackn to the other tracks if distance is less than a threshold];
	 std::vector< std::vector<unsigned int> > groups_onetrack_ntracks_associate_; // [vect of idTracks grouped with track1][vect of idTracks grouped with track2] ... [vect of idTracks grouped with trackn]
	 std::vector< std::vector<unsigned int> > track1_associated_mht_;
	 std::vector< std::vector<SpointV_cov> > SpointVcov_tracks_associated_mht_;
	 std::vector<bool> tracks_en_grupo_;
	 std::list<Cgroups> mygroup_list_;
	 unsigned int count_my_groups_; //my groups counter.
	 std::list<Cgroups>::iterator it_insert_group_;
	 std::vector<exitGroup> exitGroups_;

	 // variables for odometry
	 bool first_odom_;
	 double actual_wz_;
	 double linear_vx_;
	 double linear_vy_;

	 bool debug_companion_akp_;

	 SpointV_cov Spoint_ini_track_for_covariance_in_predictions_;


	 // variables generate data base companion (tibi and teo)
	 bool generate_database_companion_;

	 std::vector<std::vector<unsigned int>> Onetrack_ntracks_association_;
	 std::vector<std::vector<double>> vector_distances_with_tracks_;
	 std::vector<std::vector<double>> inc_time_with_tracks_;

	 bool fuse_tibi_and_teo_tracks_;
	 bool debug_fuse_tibi_and_teo_tracks_;
	 std::string results_file_;
	 unsigned int iteration_;

	 float float_time_stamp_;
	 // if(debug_fuse_tibi_and_teo_tracks_){

	 double mht_velocity_margin_;
};


#endif

