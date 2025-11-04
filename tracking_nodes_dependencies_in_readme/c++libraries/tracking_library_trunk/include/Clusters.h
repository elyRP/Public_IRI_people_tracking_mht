/*
 * Clusters.h
 *
 *  Created on: 03/03/2014 (aprox)
 *      Author: ely repiso
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

#ifndef _PEOPLE_TRACKING_MHT2_H
#define _PEOPLE_TRACKING_MHT2_H

#include "mhtStructs.h"
#include "track.h"

#include <eigen3/Eigen/Dense> //Library for matrix. Being here, not needed in cpp. (Al estar aquí, no hacen falta en el cpp).
#include <iostream> //Library for screen messages.
#include <math.h> //library for sen cos sqrt etc.
#include <list>

class Ccluster
{
  public: //put here the variables and functions to be used for other people

	// functions for cluster container.
    Ccluster( int id, std::vector<unsigned int>& tracks_mht, std::vector<unsigned int>& detections_mht, std::vector<std::vector<double>>& association_distances_mht, std::vector<std::vector<unsigned int>>& tracks_associated_mht, std::vector<std::list<Ctrack>::iterator> it_Ctracks_cluster_mht, std::vector<bool>& tracks_separados_mht, std::vector<double>& initial_Velocities_x, std::vector<double>& initial_Velocities_y, std::vector<double>& initial_orientarions, std::vector<SpointV_cov>& initial_tracks, std::vector<unsigned int>& initial_id_crossing_tracks, std::vector<bool>& vx_vy_ori_to_use_in_cluster, std::vector<double>& initial_v, std::vector<double>& vector_distance_between_tracks, std::vector<SpointV_cov>& actual_tracks, double Pd_cluster_mht=1.0, double cluster_life_mht=0.0);
    ~Ccluster();
    bool get_cluster_confirmed_mht();
    std::vector<unsigned int> get_tracks_in_cluster_mht();
    unsigned int get_id_cluster_mht();
    double get_Pd_cluster_mht();
    Eigen::MatrixXi get_winner_hypothesis_cluster_mht();
    std::vector<unsigned int> get_detections_in_cluster_mht();
    bool get_bool_cluster_update_mht();
    std::vector<std::vector<double>> get_association_distances_mht();
    std::vector<std::vector<unsigned int>> get_tracks_associated_mht();
    std::vector<bool> get_tracks_separados_mht();
    double get_cluster_life_mht();
    std::vector<double> get_initial_velocities_x_mht();
    std::vector<double> get_initial_velocities_y_mht();
    std::vector<double> get_initial_orientations_mht();
    std::vector<double> get_initial_velocity_mht();
    std::vector<SpointV_cov> get_initial_tracks_mht();

    void set_detections_in_cluster_mht( std::vector<unsigned int> dets );
    void set_tracks_in_cluster_mht(std::vector<unsigned int> tracks);
    void set_distances_in_cluster_mht(std::vector<std::vector<double>> association_dist);
    void set_tracks_associated_in_cluster_mht(std::vector<std::vector<unsigned int>> tracks_assoc);
    void set_temp_count_my_tracks_mht(unsigned int temp_count_my_tracks);
    void set_temp_count_my_clusters_mht(unsigned int temp_count_my_clusters);
    void set_Ctracks_cluster_mht(std::vector<std::list<Ctrack>::iterator> it_Ctracks_cluster_mht);
    void set_bool_cluster_update_mht(bool bool_cluster_update);
    void set_updated_actual_tracks_in_cluster_mht(std::vector<unsigned int>& new_act_tracks);
    void set_updated_separate_tracks_in_cluster_mht(std::vector<bool>& separate_tracks);
    void set_new_dets_in_cluster_mht(unsigned int det);
    void set_new_association_tracks_in_cluster_mht(unsigned int det, std::vector<unsigned int>& vector_association_tracks);
    void set_new_association_distances_in_cluster_mht(unsigned int det, std::vector<double>& vector_association_distances);
    void set_distances_between_tracks_in_cluster_mht( std::vector<double>& vector_distance_between_tracks); // set actual distances between tracks in clusters, for orientations in clusters.
    void set_actual_tracks_in_cluster_mht( std::vector<SpointV_cov>& actual_tracks); // set actual tracks in clusters, for orientations in clusters.

    void print_Ccluster();

    // functions for mht cluster.
    void cluster_make_matrix_hipotesis();//==crea_matriz_hipotesis_y_parametros_para_probabilidades(); //en todas faltan las variables!!!
    std::vector<int> cluster_ordenar_vector_hypothesis_a_borrar(std::vector<int> fila_a_borrar_h);
    void cluster_remove_imposible_hypotesis();
    void cluster_parameters_to_calculate_probabilities_of_hypotesis(std::vector<Sdetection>& obs);
    void cluster_make_probabilities();//==crear_probabilidades(); //use the laser_Q and etc, to recalculate the distances, o calculate directly the distances if I change it directly.
    void cluster_prune_probabilities();//==podado_probabilidades(); //requires only the probability vector created in the previous function!
    void update_cluster(std::vector<Sdetection>& obs);
    void update_no_cluster(); //update cluster without the existence of this cluster, actually. For remove slow the clusters.

    //variables
  private:

    // cluster variables.
    bool debug_fallos_;
    bool debug_probabilities_;
    bool debug_cruces_fallosc;
    bool debug_cru_vel_;
    unsigned int id_cluster_mht_;
    std::vector<unsigned int> tracks_mht_;
    std::vector<unsigned int> detections_mht_;
    std::vector<std::vector<double>> association_distances_mht_;
    std::vector<std::vector<unsigned int>> tracks_associated_mht_;
    std::vector<std::list<Ctrack>::iterator> it_Ctracks_cluster_mht_;
    std::vector<bool> tracks_separados_mht_; // true if tracks are separated.


    std::vector<double> initial_Velocities_x_;
    std::vector<double> initial_Velocities_y_;
    std::vector<double> initial_orientarions_;
    std::vector<SpointV_cov> initial_tracks_;
    std::vector<unsigned int> initial_id_crossing_tracks_;
    std::vector<bool> vx_vy_ori_to_use_in_cluster_; // vector that contain 1=true if this characteristic is sufficient differentiative for this cross situation.
    std::vector<double> initial_v_;

    std::vector<double> vector_distance_between_tracks_;
    std::vector<SpointV_cov> actual_tracks_;

    double Pd_cluster_mht_;  //valor inicial y por defecto ==1

    double cluster_life_mht_; // time that the cluster is alive after the tracks are separated.

    bool cluster_update_; // variable for know if in this iterration we have this cluster and need to make the cluster update. If not, we have to do the cluster_no_update for this cluster.
   // if cluster_update_=true, have to do the update!!!.
    // mht matrix_probabilities and probabilities (tracker output variables, probability and matrix/vector hypothesis).

    unsigned int temp_count_my_tracks_;
     int initial_temp_count_my_tracks_;
    unsigned int temp_count_my_clusters_;

    Eigen::MatrixXd H_;

    // parameters for hypothesis
	double cluster_Probability_before_;
	Eigen::MatrixXi cluster_matrix_hipotesis_; // matrix variable change depending on the number of detections and tracks associated. (have the possible association hypothesis)
	Eigen::MatrixXi winner_hypothesis_;
	std::vector<N_hypothesis> vector_N_hypothesis_;
	double Pi_anterior_; // En tracks-> Pi_zero_;

	// threshold probabilities.
	double threshold_distance_;
	double threshold_probability_; //=0.001. for the prune of hypothesis by probabilities and not stay only with a the high probability. (no quedarse solo con la máxima)
	double threshold_confirmation_cluster_;

	// Parameters probabilities.
    double laser_Pd_;
    double laser_beta_ft_;
    double laser_beta_nt_;
    double laser_beta_cluster_no_detection_; //oclusion in matlab, becomes no_detection. that is truly what is this variable.
    double laser_beta_cluster_no_confirmed_;
    double increment_iteration_cluster_no_confirmed_;
    std::vector<double> laser_beta_track_no_confirmado_;

    bool tracks_in_cluster_confirmeds_;
    bool cluster_confirmed_;

    //variables for  calculation and pruned probabilities.
    Eigen::MatrixXd vector_probability_before_;
    Eigen::MatrixXd vector_probability_actual_;
    double Pi_zero_;

    unsigned int max_num_tracks_in_cluster_allowed_; //más numero de tracks permitidos en el cluster, para hacer un update cluster con probabilidades.
        													// max_num_tracks_in_cluster_allowed_= 4; //con 6tracks, ya le cuesta!!! salen infinitas combinaciones de probabilidades!.
};

#endif

