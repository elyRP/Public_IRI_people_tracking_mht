/*
 * Clusters.cpp
 *
 *  Created on: 03/03/2014 (aprox, puede que algo antes o despues)
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
*	   License (for other authors that will not be the original one): CC BY-NC-ND 4.0 
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

#include "Clusters.h"
#include "mhtStructs.h"
#include <math.h> //for exponentials and squares of the function make probabilities. (tendrá que ir dentro de esa sub funcion)
#include <vector>
#include <iostream> //for screen messages, for debug!
#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <time.h>
//#define PI 3.14159265358979323846

Ccluster::Ccluster( int id, std::vector<unsigned int>& tracks_mht, std::vector<unsigned int>& detections_mht, std::vector<std::vector<double>>& association_distances_mht, std::vector<std::vector<unsigned int>>& tracks_associated_mht, std::vector<std::list<Ctrack>::iterator> it_Ctracks_cluster_mht, std::vector<bool>& tracks_separados_mht, std::vector<double>& initial_Velocities_x, std::vector<double>& initial_Velocities_y, std::vector<double>& initial_orientarions, std::vector<SpointV_cov>& initial_tracks, std::vector<unsigned int>& initial_id_crossing_tracks, std::vector<bool>& vx_vy_ori_to_use_in_cluster, std::vector<double>& initial_v, std::vector<double>& vector_distance_between_tracks,std::vector<SpointV_cov>& actual_tracks, double Pd_cluster_mht, double cluster_life_mht):
debug_fallos_(false),
debug_probabilities_(false),
debug_cruces_fallosc(false),
debug_cru_vel_(0),
id_cluster_mht_(id),tracks_mht_(tracks_mht),detections_mht_(detections_mht),association_distances_mht_(association_distances_mht),tracks_associated_mht_(tracks_associated_mht),it_Ctracks_cluster_mht_(it_Ctracks_cluster_mht),tracks_separados_mht_(tracks_separados_mht),initial_Velocities_x_(initial_Velocities_x),initial_Velocities_y_(initial_Velocities_y),initial_orientarions_(initial_orientarions),initial_tracks_(initial_tracks),initial_id_crossing_tracks_(initial_id_crossing_tracks),vx_vy_ori_to_use_in_cluster_(vx_vy_ori_to_use_in_cluster),initial_v_(initial_v),vector_distance_between_tracks_(vector_distance_between_tracks),actual_tracks_(actual_tracks),Pd_cluster_mht_(Pd_cluster_mht),cluster_life_mht_(cluster_life_mht),
cluster_update_(true),
threshold_distance_(1), //4=(2)^2
threshold_confirmation_cluster_(0.8),  //con threshold a 0.9 deja pasar solo tracks con Pd>0.5. lo he bajado para las pruebas, para que se confirmen antes las detecciones.
laser_Pd_(0.9),
laser_beta_ft_(0.1),
laser_beta_nt_(0.11),
laser_beta_cluster_no_detection_(0.9), //valor bueno matlab 0.9 TIBI DABO, 0.5 CARGO ANTS.
laser_beta_cluster_no_confirmed_(0.02), // 0.02 o 0.015, valor inicial bueno en la realidad! luego se va augmentando. ver si va así o no.
increment_iteration_cluster_no_confirmed_(0.027), // valor incremento rápido=0.02, si lo vas bajando, cada vez sube más lento. 0.01-->0.005-->0.001-->0.0005, etc.
max_num_tracks_in_cluster_allowed_(6)
{
	H_.resize(2,2);
    H_(0,0)=1;
    H_(0,1)=0;
    H_(1,0)=0;
    H_(1,1)=1;

}
 
Ccluster::~Ccluster()
{
}

// funtions for cluster containers.
bool Ccluster::get_cluster_confirmed_mht(){
	return cluster_confirmed_;
}



std::vector<unsigned int> Ccluster::get_tracks_in_cluster_mht(){
	return tracks_mht_;
}

unsigned int Ccluster::get_id_cluster_mht(){
	return id_cluster_mht_;
}

double Ccluster::get_Pd_cluster_mht(){
	return cluster_Probability_before_;
}

Eigen::MatrixXi Ccluster::get_winner_hypothesis_cluster_mht(){
	return winner_hypothesis_;
}

std::vector<unsigned int> Ccluster::get_detections_in_cluster_mht(){
	return detections_mht_;
}

bool Ccluster::get_bool_cluster_update_mht(){
	return cluster_update_;
}

std::vector<std::vector<double>> Ccluster::get_association_distances_mht(){
	return  association_distances_mht_;
}

std::vector<std::vector<unsigned int>> Ccluster::get_tracks_associated_mht(){
	return  tracks_associated_mht_;
}

std::vector<bool> Ccluster::get_tracks_separados_mht(){
	return  tracks_separados_mht_;
}

double Ccluster::get_cluster_life_mht(){
	return  cluster_life_mht_;
}

std::vector<double> Ccluster::get_initial_velocities_x_mht(){
	return  initial_Velocities_x_;
}

std::vector<double> Ccluster::get_initial_velocities_y_mht(){
	return  initial_Velocities_y_;
}

std::vector<double> Ccluster::get_initial_orientations_mht(){
	return  initial_orientarions_;
}

std::vector<double> Ccluster::get_initial_velocity_mht(){
	return  initial_v_;
}

std::vector<SpointV_cov> Ccluster::get_initial_tracks_mht(){
	return  initial_tracks_;
}



void Ccluster::set_detections_in_cluster_mht(std::vector<unsigned int> dets){
	detections_mht_=dets;
}

void Ccluster::set_tracks_in_cluster_mht(std::vector<unsigned int> tracks){
	tracks_mht_=tracks;
}

void Ccluster::set_distances_in_cluster_mht(std::vector<std::vector<double>> association_dist){
	association_distances_mht_=association_dist;
}

void Ccluster::set_tracks_associated_in_cluster_mht(std::vector<std::vector<unsigned int>> tracks_assoc){
	tracks_associated_mht_=tracks_assoc;
}
void Ccluster::set_temp_count_my_tracks_mht(unsigned int temp_count_my_tracks){
	temp_count_my_tracks_=temp_count_my_tracks;
}

void Ccluster::set_temp_count_my_clusters_mht(unsigned int temp_count_my_clusters){
	temp_count_my_clusters_=temp_count_my_clusters;
}

void Ccluster::set_Ctracks_cluster_mht(std::vector<std::list<Ctrack>::iterator> it_Ctracks_cluster_mht){
	it_Ctracks_cluster_mht_=it_Ctracks_cluster_mht;
}

void Ccluster::set_bool_cluster_update_mht(bool bool_cluster_update){
	cluster_update_=bool_cluster_update;
}


void Ccluster::set_updated_actual_tracks_in_cluster_mht(std::vector<unsigned int>& new_act_tracks){
	// this function set the actual tracks that not have the cluster in the cluster.
	for(unsigned int t_act=0;t_act<new_act_tracks.size();t_act++){
		tracks_mht_.push_back(new_act_tracks[t_act]);
	}
}

void Ccluster::set_updated_separate_tracks_in_cluster_mht(std::vector<bool>& separate_tracks){
	tracks_separados_mht_=separate_tracks; // associados con las posiciones de los tracks_mht_.
}

void Ccluster::set_new_dets_in_cluster_mht(unsigned int det){
	detections_mht_.push_back(det); // associados con las posiciones de los tracks_mht_.
}

void Ccluster::set_new_association_tracks_in_cluster_mht(unsigned int det, std::vector<unsigned int>& vector_association_tracks){
	tracks_associated_mht_[det].clear();
	for(unsigned int it_as_t=0; it_as_t< vector_association_tracks.size();it_as_t++){
		tracks_associated_mht_[det].push_back(vector_association_tracks[it_as_t]); // associados con las posiciones de los tracks_mht_.
	}
}

void Ccluster::set_new_association_distances_in_cluster_mht(unsigned int det, std::vector<double>& vector_association_distances){
	association_distances_mht_[det].clear();
		for(unsigned int it_as_d=0; it_as_d< vector_association_distances.size();it_as_d++){
			association_distances_mht_[det].push_back(vector_association_distances[it_as_d]); // associados con las posiciones de los tracks_mht_.
		}
}

void Ccluster::set_distances_between_tracks_in_cluster_mht( std::vector<double>& vector_distance_between_tracks){
	vector_distance_between_tracks_.clear();
	vector_distance_between_tracks_=vector_distance_between_tracks;
}

void Ccluster::set_actual_tracks_in_cluster_mht( std::vector<SpointV_cov>& actual_tracks){
	actual_tracks_.clear();
	actual_tracks_=actual_tracks;
}

void Ccluster::print_Ccluster(){

	std::cout<< " Clusters mht!!! \n" << std::endl;
	std::cout<< " id_cluster_mht_:" <<  id_cluster_mht_ << std::endl;
	std::cout<< " cluster_life_mht_:" << cluster_life_mht_ << std::endl;
	std::cout<< " Pd_cluster_mht_:" << Pd_cluster_mht_ << std::endl;
	std::cout<< " winner_hypothesis_:"<< winner_hypothesis_ << std::endl;
	//std::cout<< " tracks_separados_mht_: (0=false) (1=true)" << tracks_separados_mht_ << std::endl;

	std::cout<< " tracks in cluster mht:"<< std::endl;
	for(unsigned int t=0; t<tracks_mht_.size();t++ ){
		std::cout<< " tracks_mht_("<< t <<")="<< tracks_mht_[t] << std::endl;
		std::cout<< " tracks_separados_mht_: (0=false) (1=true)" << tracks_separados_mht_[t] << std::endl;
	}

	std::cout<< " detections in cluster mht:"<< std::endl;
	for(unsigned int d=0; d<detections_mht_.size();d++ ){
		std::cout<< " detections_mht_("<< d <<")="<< detections_mht_[d] << std::endl;
	}

	std::cout<< " association tracks:"<< std::endl;
	std::cout<< " SIZE association tracks="<<tracks_associated_mht_.size()<< std::endl;
	for(unsigned int a=0; a<detections_mht_.size() ;a++ ){
		for(unsigned int t=0; t<tracks_associated_mht_[detections_mht_[a]].size() ;t++ ){
			std::cout<< "  tracks_associated_mht_(detection="<< detections_mht_[a] <<")="<< tracks_associated_mht_[detections_mht_[a]][t] << std::endl;

		}
	}

	std::cout<< " association distances:"<< std::endl;
	std::cout<< " SIZE association distances="<<association_distances_mht_.size()<< std::endl;
	for(unsigned int d=0; d<detections_mht_.size() ;d++ ){
		for(unsigned int t=0; t<association_distances_mht_[detections_mht_[d]].size() ;t++ ){
			std::cout<< "  association_distances_mht_[detection="<< detections_mht_[d] <<"][track="<< tracks_associated_mht_[detections_mht_[d]][t] <<"]="<< association_distances_mht_[detections_mht_[d]][t] << std::endl;

		}
	}

	std::cout<< " initial_Velocities_x_:"<< std::endl;
	for(unsigned int d=0; d<initial_Velocities_x_.size() ;d++ ){
		std::cout<< "  initial_Velocities_x_[track"<<tracks_mht_[d]<<"]="<<initial_Velocities_x_[d]<< std::endl;
	}

	std::cout<< " initial_Velocities_y_:"<< std::endl;
	for(unsigned int d=0; d<initial_Velocities_y_.size() ;d++ ){
		std::cout<< "  initial_Velocities_y_[track"<<tracks_mht_[d]<<"]="<<initial_Velocities_y_[d]<< std::endl;
	}

	std::cout<< " initial_orientarions_:"<< std::endl;
		for(unsigned int d=0; d<initial_orientarions_.size() ;d++ ){
				std::cout<< "  initial_orientarions_[track"<<tracks_mht_[d]<<"]="<<initial_orientarions_[d]<< std::endl;
		}
}

void Ccluster::cluster_make_matrix_hipotesis()
{
	//crear matriz hipotesis tracker_fusion.
	if(debug_fallos_){
		std::cout<< " inicio!!! cluster_make_matrix_hipotesis. ID_cluster "<<id_cluster_mht_<< std::endl;
	}

		unsigned int num_detections2=detections_mht_.size();
		cluster_matrix_hipotesis_.resize(0,0);
		cluster_matrix_hipotesis_.resize(tracks_associated_mht_[detections_mht_[0]].size()+2,1); //cluster_matrizes_h(indice_clusters).matriz_hipotesis=[];
		initial_temp_count_my_tracks_=temp_count_my_tracks_; // save the actual number of real tracks!

		for (unsigned int d = 0; d < num_detections2 ; d++ ) //columnas=detecciones
		{
	        if(d==0)
			{
	        	unsigned int actual_detection=detections_mht_[d];
	        //	std::cout<< " actual_detection"<<actual_detection<< std::endl;
	        	//std::cout<< "cluster_matrix_hipotesis_"<<cluster_matrix_hipotesis_<< std::endl;
	        	//std::cout<< "cluster_matrix_hipotesis_.row(0)"<<cluster_matrix_hipotesis_.row(0)<< std::endl;
	        	//caso deteccion asociada a tracks existentes.
	        	cluster_matrix_hipotesis_.row(0)<<0;
	   //     	std::cout<< " cluster_matrix_hipotesis_.row(0)"<<cluster_matrix_hipotesis_.row(0)<<std::endl;
	            unsigned int num_tracks_associated_to_this_dect=tracks_associated_mht_[actual_detection].size();

	          //  std::cout<< " num_tracks_associated_to_this_dect"<<num_tracks_associated_to_this_dect<< std::endl;

	            for( unsigned int x=0; x<num_tracks_associated_to_this_dect; x++)
				{
	             	//cluster_matrizes_h(indice_clusters).matriz_hipotesis=[cluster_matrizes_h(indice_clusters).matriz_hipotesis;struct_parametros(numero_matrizes_hipotesis+1).cluster(indice_clusters).struct_distancias(d).track_menor_distancia(1,x)];
	           // 	std::cout<< " rows for associated tracks"<<d+x+1<< std::endl;
	            	cluster_matrix_hipotesis_.row(d+x+1)<<tracks_associated_mht_[actual_detection][x]; // track en la pos, [detection][0],etc.
	            }
	          //  std::cout<< " row for new tracks"<<num_tracks_associated_to_this_dect<< std::endl;
	            cluster_matrix_hipotesis_.row(num_tracks_associated_to_this_dect+1)<<temp_count_my_tracks_+1;
	            //cluster_matrizes_h(indice_clusters).matriz_hipotesis=[cluster_matrizes_h(indice_clusters).matriz_hipotesis;num_tracks_inicial+1];
	            //num_tracks_inicial=num_tracks_inicial+1;
	       //     std::cout<< " ANT temp_count_my_tracks_ "<<temp_count_my_tracks_<<std::endl;
	            temp_count_my_tracks_++;
	           // std::cout<< " DESP temp_count_my_tracks_"<<temp_count_my_tracks_<<std::endl;
	            //std::cout<< " ini (d=0) cluster_matrix_hipotesis_ \n"<<cluster_matrix_hipotesis_<< std::endl;
			}
	        else if(d>0)
			{
	        	//cojo la dimension de la matriz hipotesis anterior.
	        		unsigned int actual_detection=detections_mht_[d];
	        	//	std::cout<< " actual_detection"<<actual_detection<< std::endl;

	        		Eigen::MatrixXi matrix_hip_ant=cluster_matrix_hipotesis_; // guardo a parte la matriz anterior.

	        		unsigned int n_hipotesis_rows= cluster_matrix_hipotesis_.rows(); // numero de filas de una matrix Eigen


	                unsigned int num_tracks_associated_to_this_dect=tracks_associated_mht_[actual_detection].size();

	        //        std::cout<< " num_tracks_associated_to_this_dect"<<num_tracks_associated_to_this_dect<< std::endl;

	        		unsigned int num_hipotesis_this_det=num_tracks_associated_to_this_dect+2;

	        		//   std::cout<< " !!!! num_hipotesis_this_det"<<num_hipotesis_this_det<< std::endl;

	        		//std::cout<< " n_hipotesis_rows"<<n_hipotesis_rows<< std::endl;
	        		cluster_matrix_hipotesis_.resize(0,0);
	        		cluster_matrix_hipotesis_.resize(num_hipotesis_this_det*n_hipotesis_rows,d+1);

	        	//	std::cout<< " resize matrix \n"<<cluster_matrix_hipotesis_<< std::endl;

	        		for(unsigned int h_new=0; h_new<num_hipotesis_this_det; h_new++){
	        			for(unsigned int h_row=0; h_row<n_hipotesis_rows;h_row++){
	        				Eigen::MatrixXi matrix_row;
	        				matrix_row.resize(1,d+1);
	        				for(unsigned int r=0;r<d;r++){
	        					matrix_row(0,r)=matrix_hip_ant(h_row,r);
	        				}
	        					if(h_new==0){
	        						matrix_row(0,d)=0;
	        						cluster_matrix_hipotesis_.row(n_hipotesis_rows*h_new+h_row)=matrix_row;
	        					}
	        					if((h_new>0)&&(h_new<num_hipotesis_this_det-1)){
	        						matrix_row(0,d)=tracks_associated_mht_[actual_detection][h_new-1];
	        						cluster_matrix_hipotesis_.row(n_hipotesis_rows*h_new+h_row)=matrix_row;
	        					}
	        					if(h_new==(num_hipotesis_this_det-1)){
	        						matrix_row(0,d)=temp_count_my_tracks_+1;
	        						cluster_matrix_hipotesis_.row(n_hipotesis_rows*h_new+h_row)=matrix_row;
	        					}
	        				//	std::cout<< " matrix_row\n"<<matrix_row<< std::endl;
	        			}
	        		}
	        		temp_count_my_tracks_++;
	        	//	std::cout<< " (d="<<d<<") cluster_matrix_hipotesis_ \n"<<cluster_matrix_hipotesis_<< std::endl;
			}
		}

		//std::cout<< " 1- cluster_matrix_hipotesis_.row(): \n "<<cluster_matrix_hipotesis_.rows()<< std::endl;
		//std::cout<< " 1- cluster_matrix_hipotesis_.cols(): \n "<<cluster_matrix_hipotesis_.cols()<< std::endl;
		//std::cout<< " FINAL cluster_matrix_hipotesis_ : \n "<<cluster_matrix_hipotesis_<< std::endl;

}

std::vector<int> Ccluster::cluster_ordenar_vector_hypothesis_a_borrar(std::vector<int> fila_a_borrar_h)
{
	if(debug_fallos_){
		std::cout<<"\n cluster_ordenar_vector_hypothesis_a_borrar"<< std::endl;
	}
	for (unsigned int i=1; i<fila_a_borrar_h.size(); i++){
		for (unsigned int j=0; j<fila_a_borrar_h.size() - i; j++){
	    	if (fila_a_borrar_h[j] > fila_a_borrar_h[j+1])
	        {
	        	unsigned int temp = fila_a_borrar_h[j];
	            fila_a_borrar_h[j] = fila_a_borrar_h[j+1];
	            fila_a_borrar_h[j+1] = temp;
	         }
	    }
	 }
	return fila_a_borrar_h;
}

void Ccluster::cluster_remove_imposible_hypotesis()
{	// This function remove the impossible hypothesis generated before. impossible hypothesis == hypothesis that have 2 detections associated to the same track.
	if(debug_cruces_fallosc){
		std::cout<< "INICIAL cluster_matrix_hipotesis_ : \n"<<cluster_matrix_hipotesis_<< std::endl;
	}

	int n_hipotesis_fila=cluster_matrix_hipotesis_.rows();
	unsigned int n_hipotesis_cols=cluster_matrix_hipotesis_.cols();
	unsigned int num_filas_a_borrar=0; //numero de filas a borrar.
	std::vector<int> index_row_remain_in_matrix_hipotesis; //indice de filas a borrar, si es -1, es una fila a borrar.

	// Busco las filas a borrar, y las diferencio con las filas de la matriz de hypothesis que me he de quedar.
	for( int h=0; h<n_hipotesis_fila; h++) //filas=tracks
	{
		bool borrar_fila=false;
		Eigen::MatrixXi vector_hipotesis_podado; // vect_hypotesis_actual
		vector_hipotesis_podado.resize(1,n_hipotesis_cols);

		//me guardo los valores del vector de esa hypothesis, para comparar asociaciones de diferentes detecciones al mismo track.
	    for (unsigned int d=0;d<n_hipotesis_cols;d++)
	    {
	        vector_hipotesis_podado(0,d)=cluster_matrix_hipotesis_(h,d);//=[vector_hipotesis_podado,cluster_matrizes_h(indice_clusters).matriz_hipotesis(h,d2)]; // me quedo con una fila (hipotesis)
	    }

	    //busco si hay dos detecciones asociadas a un mismo track en esta hypothesis.
	   for( unsigned int d1=0;d1<n_hipotesis_cols; d1++ )
	   {
		   for(unsigned int d2=0; d2<n_hipotesis_cols; d2++) //columnas=detecciones
		   {//bucle filas matriz!
			   if((d1!=d2)&&(vector_hipotesis_podado(0,d1)==vector_hipotesis_podado(0,d2))&&(vector_hipotesis_podado(0,d1)!=0)&&(vector_hipotesis_podado(0,d2)!=0))
			   {
	            if(borrar_fila==false) //para que solo cuente una vez, cada fila que ha de borrar, por si encuentra más de 1 coincidencia. caso 222
	            {
	                borrar_fila=true;
	                num_filas_a_borrar++;
	            }
			   }
		   }
	   }

	   if( borrar_fila==false){
		   index_row_remain_in_matrix_hipotesis.push_back(h);
	   }else{
		   index_row_remain_in_matrix_hipotesis.push_back(-1);
	   }
	}

//	Me quedo solo con las hypothesis posibles de la matriz de hypothesis.
	Eigen::MatrixXi real_hypothesis_matrix;
	real_hypothesis_matrix.resize(n_hipotesis_fila-num_filas_a_borrar,n_hipotesis_cols);
	unsigned int real_hypothesis_index=0;
	for(unsigned int f=0;f<index_row_remain_in_matrix_hipotesis.size();f++){
		if(index_row_remain_in_matrix_hipotesis[f]!=-1){
			real_hypothesis_matrix.row(real_hypothesis_index)=cluster_matrix_hipotesis_.row(f);
			real_hypothesis_index++;
		}
	}

	cluster_matrix_hipotesis_.resize(0,0);
	cluster_matrix_hipotesis_=real_hypothesis_matrix;  // cambio la matriz de hypothesis, por la matriz sin hypotesis imposibles.

	   if(debug_cruces_fallosc){
		   std::cout<< "FINAL cluster_matrix_hipotesis_  (borrada): \n"<<cluster_matrix_hipotesis_<< std::endl;
	   }
}

void Ccluster::cluster_parameters_to_calculate_probabilities_of_hypotesis(std::vector<Sdetection>& obs)
{
	if(debug_fallos_){
		std::cout<< " INICIO cluster_parameters_to_calculate_probabilities_of_hypotesis "  << std::endl;
	}
	vector_N_hypothesis_.clear();
	double Pd_orientation_actual=1.0;  // Introduction of orientations in cross situations.
	tracks_in_cluster_confirmeds_=true;

	int N_ft=0;
	int N_nt=0;
	int N_dt=0;
	int N_tgt=0;
	int N_cluster_no_detection=0; // In this cases I detect the cluster, and N_cluster_no_detection is always 0!
	int N_cluster_NO_confirmed=0;

	unsigned int n_hipotesis_rows=cluster_matrix_hipotesis_.rows();
	unsigned int n_hipotesis_cols=cluster_matrix_hipotesis_.cols();
	unsigned int num_tracks2=tracks_mht_.size();
	    N_tgt=num_tracks2; //assigno los N_tgt!
	// cluster no confirmado.
	if(cluster_confirmed_==false){
		N_cluster_NO_confirmed=1;
	}

	 //clock_t tStart = clock();
	    for(unsigned int h=0;h<n_hipotesis_rows;h++) //filas matriz hipotesis.
	    {

	    	std::vector<double> deltas_detection_tracks; //vector de deltas, correspondientes a esa hipotesis, hay un vector deltas para cada detección que haya en esa hipótesis.
	    	deltas_detection_tracks.clear();									//para cada detección, ponemos un flag para cada track. flag=0 la detección No se asocia a ese track,flag=1la detección se asocia a ese track.
	    	std::vector<double> vector_distancias_deteccion_tracks;
	    	vector_distancias_deteccion_tracks.clear();
	    	std::vector<double> laser_beta_track_no_confirmado;
	    	laser_beta_track_no_confirmado.clear(); //std::vector<double> laser_beta_cluster_no_confirmado;
	    	laser_beta_track_no_confirmado_.clear();
	    	for(unsigned int x=0;x<detections_mht_.size();x++ ){
	    		vector_distancias_deteccion_tracks.push_back(100.0);  //TODO: ha de ser 100
	    	}
	    	Pd_orientation_actual=1.0;

	        for(unsigned int d=0;d<n_hipotesis_cols;d++) //columnas matriz hipotesis.==detecciones!!!
	        {
	        	laser_beta_track_no_confirmado_.push_back(1); // OJO! quitar si se incluye la no confirmación de los tracks.
	            if(cluster_matrix_hipotesis_(h,d)==0)
	            {
	            	N_ft=N_ft+1;
	            	deltas_detection_tracks.push_back(0.0);
	            	vector_distancias_deteccion_tracks[d]=100.0;  //TODO: ha de ser 100
	            	Pd_orientation_actual=1.0;
	            }
	            else if(cluster_matrix_hipotesis_(h,d)<=initial_temp_count_my_tracks_)
	            {
	            	N_dt=N_dt+1;
	            	unsigned int track=cluster_matrix_hipotesis_(h,d);
	            	unsigned int pos_track=0;
	            	for( unsigned int get_posTrack=0 ;get_posTrack< tracks_associated_mht_[detections_mht_[d]].size();get_posTrack++){
	            		if(tracks_associated_mht_[detections_mht_[d]][get_posTrack]==track){
	            			pos_track=get_posTrack;
	            		}
	            	}
	                deltas_detection_tracks.push_back(1.0);
														   //     std::cout<< " deltas_detection_tracks="<< deltas_detection_tracks[d]<< std::endl;
															//std::cout<< "association_distances_mht_[detection="<<detections_mht_[d]<<"][track="<<tracks_associated_mht_[detections_mht_[d]][pos_track]<<"]="<< association_distances_mht_[detections_mht_[d]][pos_track] << std::endl;

	                double distance=association_distances_mht_[detections_mht_[d]][pos_track];
	                vector_distancias_deteccion_tracks[d]=distance;

	                // INICI Calculate Pd_Orientation for this part of the hypothesis. (Cross situation with velocity)
	                bool cross_with_angle=false;
	                bool moving_track=false;

	                for(unsigned int dist_betw_tracks=0; dist_betw_tracks<vector_distance_between_tracks_.size();dist_betw_tracks++){
	                	if(debug_cru_vel_){
	                		std::cout<< " distancias["<<dist_betw_tracks<<"]: "<<fabs(vector_distance_between_tracks_[dist_betw_tracks])<< std::endl;
	                	}
	                	if(fabs(vector_distance_between_tracks_[dist_betw_tracks])<0.50){
	                		cross_with_angle=true;
	                	}
	                }
	                for(unsigned int v_tracks=0; v_tracks<actual_tracks_.size();v_tracks++){
	               		if(fabs(actual_tracks_[v_tracks].v())>0.25){
	               			moving_track=true;
	               		}
	               	}

	                if(debug_cru_vel_){
	                	std::cout<< "cross_with_angle="<<cross_with_angle<< std::endl;
	                	std::cout<< "moving_track="<<moving_track<< std::endl;
	                }

	                if((cross_with_angle==true) && (moving_track==true)){  // si d_entre_tracks<0.30 && V.track>0.25 (ojo, lo de las V, puede que haya que quitarlo)
	                	if(debug_cru_vel_){
	                		std::cout<< "ENTRO en probabilidades orientacion!"<< std::endl;
	                	}
	                	Spoint detection_actual=obs[d].getSpointDetection();
	                	unsigned int id_initial_track=(unsigned int)cluster_matrix_hipotesis_(h,d);
	                	int id_pos_track=-1;
	                	for(unsigned int i_id_t=0; i_id_t<initial_id_crossing_tracks_.size(); i_id_t++){
	                		if(initial_id_crossing_tracks_[i_id_t]==id_initial_track){
	                			id_pos_track=i_id_t;
	                		}
	                	}

	                	if(id_pos_track!=-1){
	                		SpointV_cov inicial_track=initial_tracks_[id_pos_track];
	                		double diferential_angle=inicial_track.angle_heading_point(detection_actual);

	                		if(debug_cru_vel_){
	                			std::cout<<" hipotesys[fila_h="<<h<<"][columna_det="<<d<<"],diferential_angle="<<diferential_angle<< std::endl;
	                		}

	                		double Pd_dif_angle_act= 1.0;
	                		double Pd_dif_angle_act_mod=1.0;

	                		if( ( diferential_angle > -0.0001) && ( diferential_angle < 0.0001) )
	                		{
	                			Pd_dif_angle_act = log(fabs(0.0001))/log(3);
	                			Pd_dif_angle_act_mod = fabs(Pd_dif_angle_act);
	                			if(debug_cru_vel_){
	                				std::cout<< " 0 diferential_angle: "<<diferential_angle<< std::endl;
	                				std::cout<< " 0 Pd_dif_angle_act_mod: "<<Pd_dif_angle_act_mod<< std::endl;
	                			}
	                		}else if ( ( diferential_angle > -0.4) && ( diferential_angle < 0.4) ){
	                			Pd_dif_angle_act = log(fabs(diferential_angle))/log(3);
	                			Pd_dif_angle_act_mod = fabs(Pd_dif_angle_act);

	                			if(debug_cru_vel_){
	                				std::cout<< " 1 diferential_angle: "<<diferential_angle<< std::endl;
	                				std::cout<< " Pd_dif_angle_act: "<<Pd_dif_angle_act<<"\n"<< std::endl;
	                				std::cout<< " 1 Pd_dif_angle_act_mod: "<<Pd_dif_angle_act_mod<< std::endl;
	                			}
	                		}else if ( (diferential_angle < - 0.4) && (diferential_angle > -0.8)){     // exponencial que empieze en 1 y decrezca hacia la izquierda.

	                			Pd_dif_angle_act_mod = 0.337/fabs(diferential_angle); // con el parametro (exp)^(x-s_ini): exp, haces más pronunciada la caida o subida de la exponencial.
	                			if(debug_cru_vel_){
	                				std::cout<< " 2 diferential_angle: "<<diferential_angle<< std::endl;
	                				std::cout<< " 2Pd_dif_angle_act_mod: "<<Pd_dif_angle_act_mod<< std::endl;
	                			}
	                		}else if((diferential_angle > 0.4) && (diferential_angle < 0.8)){   // exponencial que empieze en 1 y decrezca hacia la derecha.
	                			Pd_dif_angle_act_mod = 0.337/fabs(diferential_angle);
	                			if(debug_cru_vel_){
	                				std::cout<< " 3 diferential_angle: "<<diferential_angle<< std::endl;
	                				std::cout<< " 3Pd_dif_angle_act_mod: "<<Pd_dif_angle_act_mod<< std::endl;
	                			}

	                		} else if(diferential_angle < -0.8){

	                			Pd_dif_angle_act_mod = pow(3, -fabs(diferential_angle));
	                			if(debug_cru_vel_){
	                				std::cout<< " 4 diferential_angle: "<<diferential_angle<< std::endl;
	                				std::cout<< " 4Pd_dif_angle_act_mod: "<<Pd_dif_angle_act_mod<< std::endl;
	                			}

		   					} else if(diferential_angle > 0.8){

		   						Pd_dif_angle_act_mod = pow(3, -fabs(diferential_angle));
		   						if(debug_cru_vel_){
		   							std::cout<< " 5 diferential_angle: "<<diferential_angle<< std::endl;
		   							std::cout<< "5 Pd_dif_angle_act_mod: "<<Pd_dif_angle_act_mod<< std::endl;
		   						}
		   					}
	                		Pd_orientation_actual=Pd_orientation_actual*Pd_dif_angle_act_mod;
	                	}
	                	// FI Calculate Pd_Orientation for this part of the hypothesis.
	                }else{
	                	Pd_orientation_actual=Pd_orientation_actual*1.0;  // No tengo en cuenta orientación de la velocidad en el cruce.
	                	if(debug_cru_vel_){
	                		std::cout<< " else Pd_orientation_actual:"<< Pd_orientation_actual << std::endl;
	                	}
	                }
	            }else if(cluster_matrix_hipotesis_(h,d)>initial_temp_count_my_tracks_){
	            	N_nt=N_nt+1; // new tracks. id_new track.
	            	deltas_detection_tracks.push_back(0.0);
	            											// std::cout<< " deltas_detection_tracks="<< deltas_detection_tracks[d]<< std::endl;
	            	vector_distancias_deteccion_tracks[d]=100.0; //TODO: ha de ser 100
	            	Pd_orientation_actual=1.0;
	            	if(debug_cru_vel_){
	            		std::cout<< "else2 Pd_orientation_actual:"<< Pd_orientation_actual << std::endl;
	            	}
	            }

	            if(debug_cru_vel_){
	            	std::cout<<" hipotesys[fila_h="<<h<<"][columna_det="<<d<<"],Pd_orientation_actual="<<Pd_orientation_actual<< std::endl;
	            }
	        }

	        if(debug_cru_vel_){
	        	std::cout<<"Pd_orientation_actual"<<Pd_orientation_actual<< std::endl;
	        }

	        N_hypothesis Ns_hypotesis_actual(N_dt,N_tgt,N_ft,N_nt,N_cluster_no_detection,N_cluster_NO_confirmed,vector_distancias_deteccion_tracks,deltas_detection_tracks,laser_beta_track_no_confirmado_,laser_beta_cluster_no_confirmed_,Pd_orientation_actual);
	        //Ns_hypotesis_actual.print_N_hypothesis();
	        vector_N_hypothesis_.push_back(Ns_hypotesis_actual);
	        N_ft=0;
	        N_nt=0;
	        N_dt=0;
	    }
	 //  printf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
	   //if(((double)(clock() - tStart)/CLOCKS_PER_SEC)>1.0){
		//   std::cout<< "vector_N_hypothesis_.size() \n"<<vector_N_hypothesis_.size()<< std::endl;
	   //}
	   // std::cout<< "vector_N_hypothesis_.size() \n"<<vector_N_hypothesis_.size()<< std::endl;
	   // std::cout<< " FINAL cluster_parameters_to_calculate_probabilities_of_hypotesis "  << std::endl;
}

void Ccluster::cluster_make_probabilities(){

	//creation probabilities of hypothesis
	if(debug_fallos_){
		std::cout<< " CLUSTER make_probabilities " << std::endl;
		std::cout<< " CLUSTER ID:"<<id_cluster_mht_ << std::endl;
	}

   vector_probability_actual_.resize(0,0); //Pi_vector=[];       // vector of probabilities of this iteration, se pone a 0 el: Eigen::MatrixXd vector_probability_actual_;.(es un vector en vertical, en vez de horizontal, para corresponderse con la posición de las hipotesis en la matriz de hipotesis)
//    std::cout<< "\n vector_probability_before_\n" << vector_probability_before_<< std::endl;
//    printf("\n \n index_number_of_hypothesis_=%d",index_number_of_hypothesis_);

    unsigned int num_Probabilidades=cluster_matrix_hipotesis_.rows();
//    std::cout<< "\n num_Probabilidades : " << num_Probabilidades << std::endl;

    Eigen::MatrixXd Pi(num_Probabilidades,1);
    cluster_Probability_before_= Pd_cluster_mht_;
    double C_calculada=0;

//    std::cout<< "\n Pi\n" << Pi<< std::endl;

	for( unsigned int j=0 ; j<num_Probabilidades ; j++ ) //filas Pi's=num hypothesis. Pi=probabilities of each hypothesis.
    {

    	int c=1;
        double Pi_laser_dist=1;
        double Pi_laser_beta_track_no_confirmed_=1;

        for(int det=0; det<cluster_matrix_hipotesis_.cols(); det++){
        	double laser_dist=vector_N_hypothesis_[j].distances_sub_hypothesis_[det] ;//gausiana=gausiana*(exp(-(1/2)*distancias_detection(1,d).distancia_tracks(1,t)))^deltas_detection(1,d).detla_tracks(1,t);
        	double delta=vector_N_hypothesis_[j].delta_sub_hypothesis_[det];
        	Pi_laser_dist=Pi_laser_dist*(pow(exp(-(sqrt(laser_dist)/2)),delta));  //TODO: I change the laser_distance for sqrt(laser_distance) because we give me the square distances!!! Si va mal, mirar esto!
        	Pi_laser_beta_track_no_confirmed_=Pi_laser_beta_track_no_confirmed_*pow(vector_N_hypothesis_[j].laser_beta_track_no_confirmado_[det],1);
        }

        	/*double Pi_laser_beta_track_no_confirmed_=1;
        	for( int det=0; det<cluster_matrix_hipotesis_.cols(); det++){
        		Pi_laser_beta_track_no_confirmed_=Pi_laser_beta_track_no_confirmed_*pow(vector_N_hypothesis_[j].laser_beta_track_no_confirmado_[det],1);
        	}*/

    		if(j==0)
    		{
    			// Case false alarm.
    			Pi_zero_=(1/c)*pow(laser_Pd_,vector_N_hypothesis_[j].N_det_)*
    					pow(1-laser_Pd_,vector_N_hypothesis_[j].N_tgt_-vector_N_hypothesis_[j].N_det_)*
    					pow(laser_beta_cluster_no_detection_,vector_N_hypothesis_[j].N_cluster_no_det_)*
    					pow(laser_beta_ft_,vector_N_hypothesis_[j].N_fa_)*
    					pow(laser_beta_nt_,vector_N_hypothesis_[j].N_new_)*
    					Pi_laser_dist*1*1*cluster_Probability_before_*vector_N_hypothesis_[j].Pd_orientation_;
    			/*if(Pi_zero_<0.005){
    				Pi_zero_=0;
    			}*/
  //  			std::cout<< " \n  pi_fa:" << Pi_zero_<< std::endl;

    		}

    	/*	if(tracks_in_cluster_confirmeds_==false){

    			// Case track NO confirmed
    			std::cout<< " \n  entro en tracks_in_cluster_confirmeds_ NO" << std::endl;
    			Pi(j,0)=(double)(1/c)*pow(Pi_laser_beta_track_no_confirmed_,1)*
    					pow(laser_Pd_,vector_N_hypothesis_[j].N_det_)*
    					pow(1-laser_Pd_,vector_N_hypothesis_[j].N_tgt_-vector_N_hypothesis_[j].N_det_)*
    					pow(laser_beta_cluster_no_detection_,vector_N_hypothesis_[j].N_cluster_no_det_)*
    					pow(laser_beta_ft_,vector_N_hypothesis_[j].N_fa_)*
    					pow(laser_beta_nt_,vector_N_hypothesis_[j].N_new_)*
    					Pi_laser_dist*1*1*cluster_Probability_before_;
  //  			std::cout<< " \n  laser_beta_track_no_confirmed_:" << laser_N_dt_[j]<< std::endl;
   // 			std::cout<< " \n  laser_beta_track_no_confirmed_:" << laser_beta_track_no_confirmed_<< std::endl;
   // 			std::cout<< " \n  pi_track no confirmado:" << Pi(j,0)<< std::endl;

    		} else*/
    		if(cluster_confirmed_==false){
    			//std::cout<< " \n  Entro en cluster_confirmed_ NO" << std::endl;

    			Pi(j,0)=(double)(1/c)*pow(laser_beta_cluster_no_confirmed_,vector_N_hypothesis_[j].N_cluster_no_confirm_)*
    			    	pow(laser_Pd_,vector_N_hypothesis_[j].N_det_)*
    			    	pow(1-laser_Pd_,vector_N_hypothesis_[j].N_tgt_-vector_N_hypothesis_[j].N_det_)*
    			    	pow(laser_beta_cluster_no_detection_,vector_N_hypothesis_[j].N_cluster_no_det_)*
    			    	pow(laser_beta_ft_,vector_N_hypothesis_[j].N_fa_)*
    			    	pow(laser_beta_nt_,vector_N_hypothesis_[j].N_new_)*
    			    	Pi_laser_dist*1*1*cluster_Probability_before_*vector_N_hypothesis_[j].Pd_orientation_;

    			/*if(Pi(j,0)<0.005){
    				Pi(j,0)=0;
    			}*/
    		}
    		else
    		{
    			// Case track confirmed
    			//std::cout<< " \n  Entro en cluster_confirmed_ SI" << std::endl;

    			Pi(j,0)=(double)(1/c)*pow(laser_Pd_,vector_N_hypothesis_[j].N_det_)*
    			    	pow(1-laser_Pd_,vector_N_hypothesis_[j].N_tgt_-vector_N_hypothesis_[j].N_det_)*
    			    	pow(laser_beta_cluster_no_detection_,vector_N_hypothesis_[j].N_cluster_no_det_)*
    			    	pow(laser_beta_ft_,vector_N_hypothesis_[j].N_fa_)*
    			    	pow(laser_beta_nt_,vector_N_hypothesis_[j].N_new_)*
    			    	Pi_laser_dist*1*1*cluster_Probability_before_*vector_N_hypothesis_[j].Pd_orientation_;
    			/*if(Pi(j,0)<0.005){
    				Pi(j,0)=0;
    			}*/

    		}
    		 C_calculada=C_calculada+Pi(j,0);  // calculate and then apply the normalization parameter.

    		if(debug_probabilities_){
				//check that the function goes well!!!
    			double pd=pow(laser_Pd_,vector_N_hypothesis_[j].N_det_);
				printf("\n \nlaser_pd%f \n",pd);
				double tgt=pow(1-laser_Pd_,vector_N_hypothesis_[j].N_tgt_-vector_N_hypothesis_[j].N_det_);
				printf("laser_tgt%f \n",tgt);
				double ft=pow(laser_beta_ft_,vector_N_hypothesis_[j].N_fa_);
				printf("laser_ft%f \n",ft);
				double nt=pow(laser_beta_nt_,vector_N_hypothesis_[j].N_new_);
				printf("laser_nt%f \n",nt);
				double no_det=pow(laser_beta_cluster_no_detection_,vector_N_hypothesis_[j].N_cluster_no_det_);
				printf("N_cluster_no_det_ %f \n",no_det);
				double cluster_Probability_before=cluster_Probability_before_;
				printf("cluster_Probability_before__ %f \n",cluster_Probability_before);
				if(cluster_confirmed_==false){
					double cluster_no_confirm=pow(laser_beta_cluster_no_confirmed_,vector_N_hypothesis_[j].N_cluster_no_confirm_);
					printf("cluster_no_confirm %f \n",cluster_no_confirm);
				}
				std::cout<<"Pi_laser_dist"<<Pi_laser_dist<< std::endl;
				std::cout<<"vector_N_hypothesis_[j].Pd_orientation_"<<vector_N_hypothesis_[j].Pd_orientation_<< std::endl;
				std::cout<<"   	Pd_cluster_mht_"<<Pd_cluster_mht_<< std::endl;
				std::cout<< "\n Pi("<<j<<","<<0<<"): " << Pi(j,0) << std::endl;
				std::cout<< "\n C_calculada " << C_calculada << std::endl;
    		}
    }

	vector_probability_actual_.resize(num_Probabilidades,1); //Initialize the vector of actual probabilities.

    for( unsigned int j=0 ; j<num_Probabilidades ; j++) //filas Pi's=number of hypothesis
	{
        vector_probability_actual_(j,0)=Pi(j,0)/C_calculada; //Pi_vector=[Pi_vector;Pi(j)/C_calculada];
    }
    Pi_zero_=(double) vector_probability_actual_(0,0);

    if(debug_cru_vel_){
    	for( unsigned int j=0 ; j<num_Probabilidades ; j++) //filas Pi's=number of hypothesis
    	{
    	 std::cout<< " vector_probability_actual_(hypothesis="<<j<<",0)=" <<vector_probability_actual_(j,0)<< std::endl;
    	}
    }

    if(debug_probabilities_){
      std::cout<< "\n \n C_calculada:"<<C_calculada<< std::endl;
      std::cout<< "\n\n vector_probability_actual_:\n" << vector_probability_actual_<< std::endl;
      std::cout<< "\n Pi:\n" << Pi<< std::endl;
      std::cout<<"\n \n Pi_zero_:"<<Pi_zero_<< std::endl;
      std::cout<<"\n \n"<< std::endl;
    }

}

void Ccluster::cluster_prune_probabilities()
{
	// Confirmation and prune of the hypothesis. Again, I remain only with the two initial hypothesis
		//  0=falsa_alarma o 1=detección_asociada_a_track
	 if(debug_cruces_fallosc){
		 std::cout<< " Entro en Cluster prune_probabilities" << std::endl;
	 }

	 double max_probabilidad;
	 max_probabilidad=Pi_zero_;//vector_probability_actual_(0,0);
	 int k_max_one=0;
	 unsigned int num_Probabilidades=cluster_matrix_hipotesis_.rows();

	 winner_hypothesis_.resize(0,0);
	 for( unsigned int j=0 ; j<num_Probabilidades ; j++ )//filas Prob
	 {
	     if( vector_probability_actual_(j,0)>max_probabilidad )
	     {
	     	max_probabilidad=vector_probability_actual_(j,0);
	        k_max_one=j;
	     }

	 }

	 // aumentar pd's:
	 double nu=0.0;
	 nu=nu+Pi_zero_;
	 nu=nu+max_probabilidad;

	 double new_max_pd=max_probabilidad/nu;
	 max_probabilidad=new_max_pd;

	 cluster_Probability_before_=pow(max_probabilidad,(double) detections_mht_.size()/10); //struct_Pi_vectors(numero_matrizes_hipotesis+1).vector=max_probabilidad; //luego será un vector de Pi's anteriores para cada track(propagacion) que tenga.
	 Pd_cluster_mht_=pow(max_probabilidad,(double) detections_mht_.size()/10);
	 winner_hypothesis_=cluster_matrix_hipotesis_.row(k_max_one);


	 //change track no confirmed.
	 if(max_probabilidad>threshold_confirmation_cluster_)
	 {
	    	cluster_confirmed_=true;
	 }
	 else
	 {
	  	if(cluster_confirmed_==false)
	    {
	  		laser_beta_cluster_no_confirmed_=laser_beta_cluster_no_confirmed_+increment_iteration_cluster_no_confirmed_;//for some orientation =>increment_iteration_track_no_confirmed=0.02;
	    }
	 }

	 if(debug_probabilities_){
		std::cout<< " detections_mht_.size()/10"<< (double) detections_mht_.size()/10 << std::endl;
		std::cout<< " detections_mht_.size()"<< detections_mht_.size() << std::endl;
		std::cout<< " vector_probability_actual_.rows()"<< vector_probability_actual_.rows() << std::endl;
		std::cout<< " CLUSTER ID:"<<id_cluster_mht_ << std::endl;
		print_Ccluster();
		std::cout<< " (cluster winner Pi) cluster_Probability_before_"<<cluster_Probability_before_<< std::endl;
		std::cout<< " winner_hypothesis_"<< winner_hypothesis_ << std::endl;
		print_Ccluster();
		std::cout<<" \n  (máx_probabilidad) :"<< max_probabilidad<< std::endl;
		std::cout<<" \n \n " << std::endl;
	 }

}

void Ccluster::update_no_cluster() //not have this cluster.
{
	// I remain the cluster during few iterations, because our hypothesis remains that we have this cluster although
	// we don't have update of this cluster at this moment.
	// Reduce the last cluster probability for the slow elimination of the cluster.

	 if(debug_cruces_fallosc){
		 std::cout<< "\n (NO cluster) Entro en update_no_cluster" << std::endl;
	 }

	vector_N_hypothesis_.clear();
	double Pi;
	int c=1;

	Pi=(1/c)*pow(laser_Pd_,0)*
	    	 pow(1-laser_Pd_,0)*
	    	 pow(laser_beta_ft_,0)*
	    	 pow(laser_beta_cluster_no_detection_,1)*
	    	 pow(laser_beta_nt_,0)*1*cluster_Probability_before_;

	cluster_Probability_before_=Pi;
	Pd_cluster_mht_=Pi;

	 if(debug_cruces_fallosc){
		 // remain with the last winner hypothesis.
		 std::cout<< " \n last and actual winner_hypothesis_= "<<winner_hypothesis_<< std::endl;
		 std::cout<< " \n actual Pd_cluster_mht_= "<<Pd_cluster_mht_<< std::endl;
		 std::cout<< " \n update NO cluster (propagate cluster, and decrease it's probability), probability: "<<Pd_cluster_mht_<< std::endl;
		 print_Ccluster();
	 }
}

void Ccluster::update_cluster(std::vector<Sdetection>& obs)
{
	if(cluster_update_==true){
		// have cluster
    	if(debug_fallos_){
    		std::cout << "(DESPUES)cluster_prune_probabilities();"<< std::endl;
    		std::cout<< " \n cluster id= "<<id_cluster_mht_<< std::endl;
    		std::cout<< " \n cluster update! "<< std::endl;
    	}

    	cluster_make_matrix_hipotesis();

    	if(debug_fallos_){
    		std::cout << "(Despues) cluster_make_matrix_hipotesis();"<< std::endl;
    	}

    	cluster_remove_imposible_hypotesis();

    	if(debug_fallos_){
    		std::cout << "(Despues) cluster_remove_imposible_hypotesis();;"<< std::endl;
    	}

		//creation probabilities of the hypothesis
    	cluster_parameters_to_calculate_probabilities_of_hypotesis(obs);

    	if(debug_fallos_){
    		std::cout << "(Despues) cluster_parameters_to_calculate_probabilities_of_hypotesis(obs);"<< std::endl;
    	}
    	cluster_make_probabilities();

    	if(debug_fallos_){
    		std::cout << "(DESPUES)cluster_make_probabilities();"<< std::endl;
    	}

    	cluster_prune_probabilities();

    	if(debug_fallos_){
    		std::cout << "(DESPUES)cluster_prune_probabilities();"<< std::endl;
    	}

	}else{

		if(debug_fallos_){
			std::cout<< " \n cluster id= "<<id_cluster_mht_<< std::endl;
			std::cout<< " \n cluster NO update! "<< std::endl;
		}

		update_no_cluster();

	}
}

