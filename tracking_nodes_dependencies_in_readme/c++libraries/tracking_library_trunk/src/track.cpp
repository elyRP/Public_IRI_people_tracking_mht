/*
 * track.cpp
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
#include "track.h"
#include <math.h> //for exponentials and squares of the function make probabilities. (tendrá que ir dentro de esa sub funcion)
#include <vector>
#include <iostream> //for screen messages, for debug!
#include <stdio.h>

//#define PI 3.14159265358979323846

Ctrack::Ctrack(int id) :
debug_clusters_fallos_t(false),
inicialization_laser_(false),
threshold_distance_(2), //4=(2)^2
threshold_confirmation_track_(0.9),  //con threshold a 0.9 deja pasar solo tracks con Pd>0.5. lo he bajado para las pruebas, para que se confirmen antes las detecciones.
laser_Pd_(0.9),
laser_beta_ft_(0.1),
laser_beta_nt_(0.11),
laser_beta_no_detection_(0.99), //valor bueno matlab 0.99, pero aquí tarda mucho en eliminarse así que lo baje.
laser_beta_track_no_confirmed_(0.02), // 0.02 o 0.015, valor inicial bueno en la realidad! luego se va augmentando. ver si va así o no.
increment_iteration_track_no_confirmed_(0.01), // valor incremento rápido=0.02, si lo vas bajando, cada vez sube más lento. 0.01-->0.005-->0.001-->0.0005, etc.
velocity_margin_(0.1),
track_no_associated_by_change_abrut_velocity_(false),
track_in_coss_situatuion_(false),
time_window_( 0.5 ),
NO_track_cargo_ANTS(false),
cargo_ANTS(false),
debug_companion_akp_(false) // a true para debug database companion
{
	H_.resize(2,2);
    H_(0,0)=1;
    H_(0,1)=0;
    H_(1,0)=0;
    H_(1,1)=1;

    id_tracker_=id;
}
 
Ctrack::~Ctrack()
{
}

void Ctrack::set_tracks_and_detections(Spoint_cov detection, SpointV_cov track, detection_type type)
{
	//function that get ALL the new (entrantes) laser points and tracks and put this in my variables.
/*	std::cout<< " Entro en set_tracks_and_detections deteccion + track " << std::endl;
	std::cout<< "\n   Pd_laser:" <<laser_Pd_<< std::endl;
	std::cout<< "\n   threshold_distance_" <<threshold_distance_<< std::endl;
	std::cout<< "\n   threshold_confirmation_track_" <<threshold_confirmation_track_<< std::endl;
	std::cout<< "\n   laser_beta_ft_" <<laser_beta_ft_<< std::endl;
	std::cout<< "\n   laser_beta_nt_" <<laser_beta_nt_<< std::endl;
	std::cout<< "\n   laser_beta_no_detection_" <<laser_beta_no_detection_<< std::endl;
	std::cout<< "\n   laser_beta_track_no_confirmed_" <<laser_beta_track_no_confirmed_<< std::endl;
	std::cout<< "\n   increment iteration no confirmed:" <<increment_iteration_track_no_confirmed_<< std::endl;
	std::cout<< " Detecction que me entra " << std::endl;
	detection.print(); */
//std::cout<< " Track que me entra " << std::endl;
//track.print();
	//the detection have always the same covariance matrix!, the covariance that augments are the covariance of the tracks. the prediction!==.



	have_detection_laser_=false;  //TODO: the observation/detections that get inside have type, afeter I have to get it!!! (now I don't get this types!)
	//the detection that get this function have the type of this detection=laser,vision, etc.

	laser_detection_Zm_.resize(0,0);
	laser_detection_Zm_.resize(2,1);
	laser_mean_track_.resize(0,0);
	laser_mean_track_.resize(2,1);
	laser_R_detection_.resize(0,0);
	laser_R_detection_.resize(2,2);
	laser_P_track_.resize(0,0);
	laser_P_track_.resize(2,2);

	no_hay_deteccion_track_propagacion_=false;
	if(type==Laser){
		deteccion_guardada_=detection;
		//std::cout<< " IN deteccion_guardada_ "<< std::endl;
		//deteccion_guardada_.print();
		trackV_guardado_=track;

		// Inici. create and maintain the trajectory_position window for each track. (needed for crosses)
		window_before_trackV_.push_back(track);
		while(  window_before_trackV_.back().time_stamp  - window_before_trackV_.front().time_stamp >  time_window_ )
		{
			window_before_trackV_.pop_front();
		}
		//Fi. create and maintain the trajectory_position window for each track.  (needed for crosses)

		//std::cout<< " IN trackV_guardado_ "<< std::endl;
		//trackV_guardado_.print();
		have_detection_laser_=true; //have detection laser! ==type_laser inside of my function!!!

		laser_detection_Zm_(0,0)= detection.x; // detection_laser_gaussian_detection_=p;
		laser_detection_Zm_(1,0)= detection.y;

		laser_mean_track_(0,0)= track.x; //detection_laser_gaussian_prediction_= is a track!
		laser_mean_track_(1,0)= track.y;

		Eigen::MatrixXd SIGMA_detect;
		//std::cout<< " SIGMA_detect " << SIGMA_detect << std::endl;

		SIGMA_detect.resize(0,0);
		SIGMA_detect.resize(2,2);

		double covar_detect_xx=detection.cov[0];
		//std::cout<< " covar_detect_xx " << covar_detect_xx << std::endl;
		double covar_detect_xy=detection.cov[1];
		//std::cout<< " covar_detect_xy " << covar_detect_xy << std::endl;
		double covar_detect_yx=detection.cov[2];
		//std::cout<< " covar_detect_yx " << covar_detect_yx << std::endl;
		double covar_detect_yy=detection.cov[3];
		//std::cout<< " covar_detect_yy " << covar_detect_yy << std::endl;

		SIGMA_detect.row ( 0 ) << covar_detect_xx , covar_detect_xy;
		//std::cout<< "SIGMA_detect.row ( 0 )" << SIGMA_detect<< std::endl;
		SIGMA_detect.row ( 1 ) << covar_detect_yx , covar_detect_yy;
		//std::cout<< "SIGMA_detect.row ( 1 )" << SIGMA_detect<< std::endl;

		laser_R_detection_=SIGMA_detect;

		Eigen::MatrixXd SIGMA_track;

//		std::cout<< " SIGMA_track " << SIGMA_track << std::endl;

		SIGMA_track.resize(0,0);
		SIGMA_track.resize(2,2);

		double covar_track_xx=track.cov[0]; //euclidean distance , then covar_detect=1. (distance with covariances covar_detect=0.4)
		//std::cout<< " covar_track_xx " << covar_track_xx << std::endl;
		double covar_track_xy=track.cov[1];   //TODO: Mirar como cambian las covarianzas de las propagaciones con SpointV_cov
		//std::cout<< " covar_track_xy " << covar_track_xy << std::endl;
		double covar_track_yx=track.cov[4];
		//std::cout<< " covar_track_yx " << covar_track_yx << std::endl;
		double covar_track_yy=track.cov[5];
		//std::cout<< " covar_track_yy " << covar_track_yy << std::endl;

		SIGMA_track.row ( 0 ) << covar_track_xx , covar_track_xy; //I change the covariances for the covariances that pass me the node people_prediction.
		//std::cout<< "SIGMA_track.row ( 0 )" << SIGMA_track<< std::endl;
		SIGMA_track.row ( 1 ) << covar_track_yx , covar_track_yy;
		//std::cout<< "SIGMA_track.row ( 1 )" << SIGMA_track<< std::endl;

		laser_P_track_=SIGMA_track;

		//check the function!!!
/*		std::cout<< "have_detection_laser_=" << have_detection_laser_ << std::endl;
		std::cout<< " laser_P_SIGMA_track_:\n" << laser_P_track_ << std::endl; //(in matlab) track laser laser_P_SIGMA_track_==detection_laser_SIGMA_track:
		std::cout<< " laser_R_SIGMA_detection_:\n" << laser_R_detection_ << std::endl; //(in matlab)track laser laser_R_SIGMA_detection_==detection_laser_SIGMA_detection
		std::cout<< "\n laser_detection_Zm_:\n" << laser_detection_Zm_ << std::endl; // (In matlab) detection laser laser_detection_Zm_==detection_laser_gaussian_detection:
		std::cout<< "laser_mean_track_:\n" << laser_mean_track_ << std::endl; // (in matlab)detection laser laser_mean_track_==detection_laser_gaussian_track:
*/
		if((laser_detection_Zm_(0,0)==0)&&(laser_detection_Zm_(1,0)==0)){
			no_hay_deteccion_track_propagacion_=true;
			have_detection_laser_=false;
		}

	}

}

void Ctrack::set_tracks_and_detections(Spoint_cov detection, detection_type type)
{
	//function that get ALL the new (entrantes) laser points and tracks and put this in my variables. In the case without track, only detection.
	// the detection always have the same covariance matrix!, the only covariances that increase are the track covariances ( this covariances increase when the detection don't associate with the track). the prediction!==.

	//	std::cout<< " Entro en set_tracks_and_detections solo deteccion " << std::endl;

	have_detection_laser_=false;
	laser_detection_Zm_.resize(2,1);
	laser_R_detection_.resize(2,2);



	if(type==Laser){
		deteccion_guardada_=detection;

		have_detection_laser_=true; //have detection laser! ==type_laser inside my function!!!

		laser_detection_Zm_(0,0)= detection.x; // detection_laser_gaussian_detection_=p;
		laser_detection_Zm_(1,0)= detection.y;

		Eigen::MatrixXd SIGMA_detect (2,2);

		double covar_detect_xx=detection.cov[0];
		double covar_detect_xy=detection.cov[1];
		double covar_detect_yx=detection.cov[2];
		double covar_detect_yy=detection.cov[3];

		SIGMA_detect.row ( 0 ) << covar_detect_xx , covar_detect_xy;
		SIGMA_detect.row ( 1 ) << covar_detect_yx , covar_detect_yy;

		laser_R_detection_=SIGMA_detect;

		//check the function!!!
		//std::cout<< " deteccion_guardada_ "<< std::endl;
		//deteccion_guardada_.print();
		//SIGMA_detect.row ( 0 ) << 1 , 0;
		//SIGMA_detect.row ( 1 ) << 0 , 1;
//		std::cout<< "have_detection_laser_=" << have_detection_laser_ << std::endl;
//		std::cout<< "track laser laser_Q_SIGMA_track_==detection_laser_SIGMA_track:\n" << laser_Q_SIGMA_track_ << std::endl;
//		std::cout<< " laser_R_SIGMA_detection_==detection_laser_SIGMA_detection:\n" << laser_R_detection_ << std::endl;
//		std::cout<< "\n  laser_detection_Zm_==detection_laser_gaussian_detection:\n" << laser_detection_Zm_ << std::endl;
//		std::cout<< "detection laser laser_mean_track_==detection_laser_gaussian_track:\n" << laser_mean_track_ << std::endl;

	}

}



void Ctrack::ini_track(Spoint_cov detection, detection_type type, double actual_increment)
{
	//std::cout<< " Entro en ini_track (creation initial track) " << std::endl;

	Increment_time_old_track_=Increment_time_old_track_+actual_increment;

	//std::cout<< "(1) ini_track " << std::endl;

	no_detection_time=0.0;
	set_tracks_and_detections(detection,type);

	//std::cout<< "(2) ini_track " << std::endl;

	track_confirmed_=false; //initialize the track! is the first time that this track appears, therefore this track is not confirmed, yet!
	//std::cout<< "(3) ini_track " << std::endl;

	int Prob_anterior=1;
	int c=1;
	//std::cout<< "(4) ini_track " << std::endl;
	laser_N_ft_.push_back(1); //fill this vector position!!!
	laser_N_dt_.push_back(0);
	laser_N_nt_.push_back(0);
	laser_N_no_detection_.push_back(0);
	laser_N_tgt_.push_back(0);
	//std::cout<< "(5) ini_track " << std::endl;
	double Pi_fa=double((1/c)*(pow(laser_Pd_ , laser_N_dt_[0]))*(pow((1-laser_Pd_),(laser_N_tgt_[0]-laser_N_dt_[0])))*(pow(laser_beta_ft_,laser_N_ft_[0]))*(pow(laser_beta_no_detection_,laser_N_no_detection_[0]))*(pow(laser_beta_nt_,laser_N_nt_[0])))*1*Prob_anterior;
	//std::cout<< "(6) ini_track " << std::endl;
	laser_N_ft_.push_back(0);
	laser_N_dt_.push_back(0);
	laser_N_nt_.push_back(1);
	laser_N_no_detection_.push_back(0);
	laser_N_tgt_.push_back(0);
	//std::cout<< "(7) ini_track " << std::endl;
	double Pi_det=(1/c)*(pow(laser_Pd_,laser_N_dt_[1]))*(pow((1-laser_Pd_),(laser_N_tgt_[1]-laser_N_dt_[1])))*(pow(laser_beta_ft_,laser_N_ft_[1]))*(pow(laser_beta_no_detection_,laser_N_no_detection_[1]))*(pow(laser_beta_nt_,laser_N_nt_[1]))*1*Prob_anterior;


	// for check that the function goes well:
/*		printf( "           laser_N_ft_[0]: %d            ", laser_N_ft_[0] );
		printf( "laser_N_ft_[1]: %d", laser_N_ft_[1] );
		printf( "\n            laser_N_dt[0]: %d             ", laser_N_dt_[0] );
		printf( "laser_N_dt[1]: %d", laser_N_dt_[1] );
		printf( "\n           laser_N_nt_[0]: %d           ", laser_N_nt_[0] );
		printf( " laser_N_nt_[1]: %d", laser_N_nt_[1] );
		printf( "\n laser_N_no_detection_[0]: %d ", laser_N_no_detection_[0] );
		printf( " laser_N_no_detection_[1]: %d", laser_N_no_detection_[1] );
		printf( "\n          laser_N_tgt_[0]: %d          ", laser_N_tgt_[0] );
		printf( " laser_N_tgt_[1]: %d", laser_N_tgt_[1] );
		std::cout<< "\n \n Pi_fa:" << Pi_fa << std::endl;
		std::cout<< "Pi_det:" << Pi_det << std::endl;
*/
	double Prob_sum=Pi_fa+Pi_det;

	    Pi_fa=Pi_fa/Prob_sum;
	    Pi_det=Pi_det/Prob_sum;

	    //check that the function goes well:
/*	    std::cout<< "\n Prob_sum :" << Prob_sum << std::endl;
	    std::cout<< " Pi_fa_norm :" << Pi_fa << std::endl;
	    std::cout<< " Pi_det_norm :" << Pi_det << std::endl;
*/
	    vector_probability_actual_.resize(2,1);
	    vector_probability_actual_(0,0)=Pi_fa;
	    vector_probability_actual_(1,0)=Pi_det;

	    //check that the function goes well:
//	    std::cout<< "\n vector_probability_actual_:\n" << vector_probability_actual_ << std::endl;


	    unsigned int num_hipotesis=2;
	    unsigned int num_detecciones=1;

	    matrix_hipotesis_actual_.resize(num_hipotesis,num_detecciones);

	    matrix_hipotesis_actual_(0,0)=0; //caso falsa alarma.
	    matrix_hipotesis_actual_(1,0)=1; //caso detección.

	    //check that the function goes well:

	   unsigned int num_Probabilidades=vector_probability_actual_.outerSize();
//	   std::cout<< "\n  num_Probabilidades :" <<  num_Probabilidades<< std::endl;
	        double max_probabilidad=vector_probability_actual_(0,0);
	         int k_max_one=1;

	        for ( unsigned int j=0 ; j<=num_Probabilidades ; j++ ) //filas Prob
	        {

	            if( vector_probability_actual_(j,0)>max_probabilidad )
	            {
	                max_probabilidad=vector_probability_actual_(j,0);

	 //               std::cout<< " \n INICIALIZACION (máx_probabilidad) :" << max_probabilidad<< std::endl;
	                k_max_one=j;
	            }

	        }
//       std::cout<< "\n \n vector_probability_actual_:\n" <<  vector_probability_actual_ << std::endl;
	        //check that the function goes well:


	        matrix_hipotesis_anterior_.resize(1,1);
	        probability_of_detection_=max_probabilidad; //probability of detection o probability before.
	        matrix_hipotesis_anterior_(0,0)=matrix_hipotesis_actual_(k_max_one,0);
	        hipotesis_anterior=matrix_hipotesis_actual_(k_max_one,0);

	        //paso 5: update kalman estates.
	        laser_distance_=100; //first detection, this detection is not associated to one track.
	        //std::cout<< "(6) ini_track " << std::endl;
	        kalman_states();
	        //std::cout<< "(7) ini_track " << std::endl;

//printf("hipotesis_anterior:%d",hipotesis_anterior);
	        //check that the function goes well:
	//        std::cout<< " \n INICIALIZACION probability_of_detection_ (máx_probabilidad) :" <<  probability_of_detection_<< std::endl;
	//        std::cout<< " INICIALIZACION hipotesis_actual :" <<  matrix_hipotesis_anterior_<< std::endl;


}




void Ctrack::make_matrix_hipotesis_and_parameters_of_probabilities()
{
//	std::cout<< " entro en make_matrix_hipotesis_and_parameters_of_probabilities " << std::endl;
	//Start creating matrix of hypotheses and creating parameters to calculate probabilities.

//	std::cout<< " \n distancia laser: " <<laser_distance_<< std::endl;


	//check that the function goes well:
//	printf("have_detection_laser_:%d\n",have_detection_laser_);
//	printf("laser_distance_:%f\n",laser_distance_);
//	printf("threshold_d_:%f\n",threshold_d_);
//	printf("\n \n hipotesis_anterior:%d",hipotesis_anterior);
//	printf("\n probability_of_detection_:%f \n",probability_of_detection_);

	//return to initialize the N parameters!!!
	laser_N_ft_.clear();
	laser_N_nt_.clear();
	laser_N_dt_.clear();
	laser_N_tgt_.clear();
	laser_N_no_detection_.clear();

     Eigen::MatrixXi   matrix_hipotesis;
     matrix_hipotesis.resize(0,0);
     //vector_hipotesis_0=0; //mirar como va!!! hay que inicializarlo cada vez que se entra!!!

     vector_probability_before_.resize(0,0);
     vector_probability_before_.resize(2,1); //1 detection, two hypothesis. either is 0 or is 1! TODO: (luego esto hay que cambiarlo para que se reajuste al 2=num_detecciones
    //Pi_anterior=[];

    index_number_of_hypothesis_=0; //laser_pos_struct_number_of_mesurements in matlab change to -> index_number_of_hypothesis

    unsigned int number_of_hipotesis_FIL=1;//matrix_hipotesis_anterior_.outerSize(); // n_hipotesis -> number_of_hipotesis
    						//OJO! comprobar que el outerSize sea la longitud que quiero!!! y si no areglarlo.
							//mat.length = 3->obtener el número de filas de la matriz.
							//mat[0].length= 5 ->obtener el número de columnas de la fila 0.
    unsigned int number_of_hipotesis_COL=1;//matrix_hipotesis_anterior_.innerSize(); // columnas matriz==número_detecciones, en este caso 1!.
//  inline Index outerSize() const { return static_cast<Index>(m_data.size()); }
/*    std::cout<<" matrix_hipotesis_anterior_:"<<matrix_hipotesis_anterior_<< std::endl;
    std::cout<<" number_of_hipotesis_FIL:"<<number_of_hipotesis_FIL<< std::endl;
    std::cout<<" number_of_hipotesis_COL:"<<number_of_hipotesis_COL<< std::endl;
*/
    for (unsigned int l=0; l<number_of_hipotesis_FIL;l++)  //l= row (fila) of the hypothesis. fila matriz hipótesis y fila vector probabilidades de hipótesis
	{
        int zero=0;
        int uno=0;
        //int no_detection=0; // occlusion change to -> no_detection!

    	for (unsigned int c=1; c<=number_of_hipotesis_COL;c++)
    	{

        	if(hipotesis_anterior==0) // numero_matrizes_hipotesis= position matrix before iteration.
            {
            	zero=1;
            }
            if(hipotesis_anterior==1)
            {
            	uno=1;
            }

        }  // end for column

        if( ( zero == 1 ) && ( uno == 0 ) )
        {
        	if( (have_detection_laser_==1) && (laser_distance_<=threshold_distance_) )
            {
        		//case only laser. (probability before case 0)

                matrix_hipotesis.row(0)<<0;
                matrix_hipotesis.row(1)<<1;

                for( unsigned int x=0 ; x<=1 ; x++ )
                {

                	vector_probability_before_(x,0)=probability_of_detection_; //Pi_anterior=[Pi_anterior;struct_Pi_vectors(numero_matrizes_hipotesis).vector(l,1)];
                	 											//probability_of_detection== max probability of the before iteration.
                 	 if( x == 0 ) //before case=0, iteration 0
                 	 {
                     	 //case only laser.
                         laser_N_tgt_.push_back(0);
                         laser_N_ft_.push_back(1);
                         laser_N_dt_.push_back(0);
                         laser_N_nt_.push_back(0);
                         laser_N_no_detection_.push_back(0);

               index_number_of_hypothesis_=index_number_of_hypothesis_+1; //laser_pos_struct_number_of_mesurements=laser_pos_struct_number_of_mesurements+1;

                 	 }
                    else if( x == 1 ) //before case = 0, iteration 1
                 	 {
                      	 //case only laser.
                      	 laser_N_tgt_.push_back(0);
                         laser_N_ft_.push_back(0);
                         laser_N_dt_.push_back(0);
                         laser_N_nt_.push_back(1);
                         laser_N_no_detection_.push_back(0);

                      index_number_of_hypothesis_=index_number_of_hypothesis_+1;

                 	 }

                }

            }
        }
        if( uno == 1 )
		{

        	if( (have_detection_laser_==1) && (laser_distance_<=threshold_distance_) )
            {

            	//case only laser. (probability before case 1)
        		Eigen::MatrixXi matrix_hipotesis(2,1);
        		Eigen::MatrixXd vector_probability(2,1);
        		matrix_hipotesis(0,0)=0;
        		matrix_hipotesis(1,0)=1;
        		matrix_hipotesis_actual_=matrix_hipotesis;

 /*             std::cout<< "\n-- entro en caso hipotesis_anterior=1 y solo laser" << std::endl;
                std::cout<< "\n matrix_hipotesis:\n" << matrix_hipotesis << std::endl;
                std::cout<< "\n matrix_hipotesis_actual_:\n" << matrix_hipotesis_actual_ << std::endl;
*/
               	for(unsigned int x=0 ; x<=1 ; x++)
				{

               		vector_probability(x,0)=probability_of_detection_; //Pi_anterior=[Pi_anterior;struct_Pi_vectors(numero_matrizes_hipotesis).vector(l,1)];

                    if( x == 0 ) //before case =1, iteration 0
					{
                     	 //case only laser.
                         laser_N_tgt_.push_back(1);
                         laser_N_ft_.push_back(1);
                         laser_N_dt_.push_back(0);
                         laser_N_nt_.push_back(0);
                         laser_N_no_detection_.push_back(0);

                    index_number_of_hypothesis_=index_number_of_hypothesis_+1;

					}
                    else if( x == 1 ) //before case =1, iteration 1
					{
                    	//case only laser.
                        laser_N_tgt_.push_back(1);
                        laser_N_ft_.push_back(0);
                        laser_N_dt_.push_back(1);
                        laser_N_nt_.push_back(0);
                        laser_N_no_detection_.push_back(0);

                       index_number_of_hypothesis_=index_number_of_hypothesis_+1;

 					}

				}

               	vector_probability_before_=vector_probability;

//               	std::cout<< "\n vector_probability_before\n" << vector_probability_before_ << std::endl;
            }

		} //end if(uno==1) //case 1 before hypothesis

	} //end for l

//matrix_hipotesis_actual_=matrix_hipotesis;

//check that the function goes well!!!

/*    	printf( "\n\n           laser_N_ft_[0]: %d            ", laser_N_ft_[0] );
    	printf( "laser_N_ft_[1]: %d", laser_N_ft_[1] );
    	printf( "\n            laser_N_dt[0]: %d             ", laser_N_dt_[0] );
    	printf( "laser_N_dt[1]: %d", laser_N_dt_[1] );
    	printf( "\n           laser_N_nt_[0]: %d           ", laser_N_nt_[0] );
    	printf( " laser_N_nt_[1]: %d", laser_N_nt_[1] );
    	printf( "\n laser_N_no_detection_[0]: %d ", laser_N_no_detection_[0] );
    	printf( " laser_N_no_detection_[1]: %d", laser_N_no_detection_[1] );
    	printf( "\n          laser_N_tgt_[0]: %d          ", laser_N_tgt_[0] );
    	printf( " laser_N_tgt_[1]: %d", laser_N_tgt_[1] );
		std::cout<< "\n \n  matrix_hipotesis_actual_\n" << matrix_hipotesis_actual_ << std::endl;
		std::cout<< "\n vector_probability_before_\n" << vector_probability_before_<< std::endl;
*/

}

void Ctrack::make_probabilities()
{
	//paso3: creation probabilities of hypothesis
//	std::cout<< " entro en make_probabilities " << std::endl;
	//check that the function goes well!!!.
//	printf("have_detection_laser_:%d\n",have_detection_laser_);
//	printf("laser_distance_:%f\n",laser_distance_);
//	printf("threshold_d_:%f\n",threshold_d_);
//	printf("\n \n hipotesis_anterior:%d",hipotesis_anterior);
//	printf("\n \n probability_of_detection_:%f \n",probability_of_detection_);

    vector_probability_actual_.resize(0,0); //Pi_vector=[];       // vector of probabilities of this iteration, se pone a 0 el: Eigen::MatrixXd vector_probability_actual_;.(es un vector en vertical, en vez de horizontal, para corresponderse con la posición de las hipotesis en la matriz de hipotesis)
//    std::cout<< "\n matrix_hipotesis_actual_\n" << matrix_hipotesis_actual_<< std::endl;
//    std::cout<< "\n vector_probability_before_\n" << vector_probability_before_<< std::endl;
//    printf("\n \n index_number_of_hypothesis_=%d",index_number_of_hypothesis_);
    unsigned int num_Probabilidades=index_number_of_hypothesis_;
//    printf("\n \n num_Probabilidades:%d",num_Probabilidades);

    //unsigned int number_of_hipotesis_FIL=matrix_hipotesis_anterior_.outerSize(); // n_hipotesis -> number_of_hipotesis
    						//OJO! comprobar que el outerSize sea la longitud que quiero!!! y si no areglarlo...
							//mat.length = 3->obtener el número de filas de la matriz.
							//mat[0].length= 5 ->obtener el número de columnas de la fila 0.
    //unsigned int number_of_hipotesis_COL=matrix_hipotesis_anterior_.innerSize(); // columnas matriz==numer_detecciones, en este caso 1!.

    Eigen::MatrixXd Pi(num_Probabilidades,1);
    //Pi.resize(0,0);

    double C_calculada=0;

//    std::cout<< "\n Pi\n" << Pi<< std::endl;


	for( unsigned int j=0 ; j<num_Probabilidades ; j++ ) //filas Pi's=num hypothesis. Pi=probabilities of each hypothesis.
    {

    	int c=1;

//    	 std::cout<< "have_detection_laser_"<<have_detection_laser_ << std::endl;
 //   	 std::cout<< "laser_distance_"<<laser_distance_ << std::endl;
    	if( (have_detection_laser_==1) && (laser_distance_<=threshold_distance_) )
		{
        	//case only laser.

        	double Pi_laser_dist;
 //       	printf("\n \n laser_distance_%f",laser_distance_);
        	Pi_laser_dist=exp(-(sqrt(laser_distance_)/2));  //TODO: I change the laser_distance for sqrt(laser_distance) because we give me the square distances!!! Si va mal, mirar esto!
//        	printf("\n \n Pi_laser_dist%f",Pi_laser_dist);

    		if(j==0)
    		{
    			// Case false alarm.

    			Pi_zero_=(1/c)*pow(laser_Pd_,laser_N_dt_[j])*
    					pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
    					pow(laser_beta_no_detection_,laser_N_no_detection_[j])*
    					pow(laser_beta_ft_,laser_N_ft_[j])*
    					pow(laser_beta_nt_,laser_N_nt_[j])*
    					Pi_laser_dist*1*1*vector_probability_before_(j,0);
  //  			std::cout<< " \n  pi_fa:" << Pi_zero_<< std::endl;

    		}

    		if(track_confirmed_==false){

    			// Case track NO confirmed

    			Pi(j,0)=(double)(1/c)*pow(laser_beta_track_no_confirmed_,laser_N_dt_[j])*
    					pow(laser_Pd_,laser_N_dt_[j])*
    					pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
    					pow(laser_beta_no_detection_,laser_N_no_detection_[j])*
    					pow(laser_beta_ft_,laser_N_ft_[j])*
    					pow(laser_beta_nt_,laser_N_nt_[j])*
    					Pi_laser_dist*1*1*vector_probability_before_(j,0);
  //  			std::cout<< " \n  laser_beta_track_no_confirmed_:" << laser_N_dt_[j]<< std::endl;
   // 			std::cout<< " \n  laser_beta_track_no_confirmed_:" << laser_beta_track_no_confirmed_<< std::endl;
   // 			std::cout<< " \n  pi_track no confirmado:" << Pi(j,0)<< std::endl;

    		}
    		else
    		{
    			// Case track confirmed

    			Pi(j,0)=(double)(1/c)*pow(laser_Pd_,laser_N_dt_[j])*
    			    	pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
    			    	pow(laser_beta_no_detection_,laser_N_no_detection_[j])*
    			    	pow(laser_beta_ft_,laser_N_ft_[j])*
    			    	pow(laser_beta_nt_,laser_N_nt_[j])*
    			    	Pi_laser_dist*1*1*vector_probability_before_(j,0);

    		}

        	//check that the function goes well!!!
/*        	double pd=pow(laser_Pd_,laser_N_dt_[j]);
        	printf("\n \nlaser_pd%f \n",pd);
        	double tgt=pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j]);
        	printf("laser_tgt%f \n",tgt);
        	double ft=pow(laser_beta_ft_,laser_N_ft_[j]);
        	printf("laser_ft%f \n",ft);
        	double nt=pow(laser_beta_nt_,laser_N_nt_[j]);
        	printf("laser_nt%f \n",nt);
*/
		}

        C_calculada=C_calculada+Pi(j,0);  // calculate and then apply the normalization parameter.

    }

//	std::cout<< "\n Pi:\n" << Pi<< std::endl;
//	printf("\n \n Pi_zero_:%f",Pi_zero_);

	vector_probability_actual_.resize(num_Probabilidades,1); //Initialize the vector of actual probabilities.

    for( unsigned int j=0 ; j<num_Probabilidades ; j++) //filas Pi's=number of hypothesis
	{

        vector_probability_actual_(j,0)=Pi(j,0)/C_calculada; //Pi_vector=[Pi_vector;Pi(j)/C_calculada];

    }
//    std::cout<< " 11" << std::endl;
    Pi_zero_=(double) vector_probability_actual_(0,0);

 //   printf("\n \n C_calculada:%f",C_calculada);
 //   std::cout<< "\n\n vector_probability_actual_:\n" << vector_probability_actual_<< std::endl;
 //   printf("\n \nPi_zero_:%f",Pi_zero_);
//    printf("\n \n");
//    std::cout<< " salgo de make_probabilities" << std::endl;
}

void Ctrack::prune_probabilities()
{
//	std::cout<< " entro en prune_probabilities" << std::endl;
	 // paso4: confirmation and prune of the hypothesis. Again, I remain only with the two initial hypothesis
	//  0=falsa_alarma o 1=detección_asociada_a_track

	    double max_probabilidad;
	    max_probabilidad=Pi_zero_;//vector_probability_actual_(0,0);
	    int k_max_one=0;
	    unsigned int num_Probabilidades=2; //num_Probabilidades=size(Pi_vector);
//	    printf("\n \n num_Probabilidades:%d",num_Probabilidades);

	    for( unsigned int j=0 ; j<num_Probabilidades ; j++ )//filas Prob
	    {
	        if( vector_probability_actual_(j,0)>max_probabilidad )
	        {
	            max_probabilidad=vector_probability_actual_(j,0);
	            k_max_one=j;
	        }

	    }

	   probability_of_detection_=max_probabilidad; //struct_Pi_vectors(numero_matrizes_hipotesis+1).vector=max_probabilidad; //luego será un vector de Pi's anteriores para cada track(propagacion) que tenga.

//	   printf("\n \nprobability_of_detection_(max_probabilidad):%f",probability_of_detection_);

	    if(k_max_one>0) //hypothesis fa==0!!! the first probability of hypothesis that is calculated!
	    {
	        //case detection.
	        matrix_hipotesis_anterior_.resize(1,1);
	        matrix_hipotesis_anterior_(0,0)=1;
	        hipotesis_anterior=1;
	        //struct_matriz_hipotesis_anteriores(numero_matrizes_hipotesis+1).matrix=1; //get the value of the second column of the matrix that is the actual hypothesis.
	    }
	    else
	    {
	        //case false alarm.
	        matrix_hipotesis_anterior_.resize(1,1);
	        matrix_hipotesis_anterior_(0,0)=0;
	        hipotesis_anterior=0;
	        //struct_matriz_hipotesis_anteriores(numero_matrizes_hipotesis+1).matrix=0;
	    }

	    //change track no confirmed.
	    if(max_probabilidad>threshold_confirmation_track_)
	    {
	    	track_confirmed_=true;
	        // laser_beta_track_no_confirmed_=0.05; //No se si hace falta realmente.// lo inicializo, pq el track esta confirmado. Para cuando no este confirmado.
	    }
	    else
	    {
	         if(track_confirmed_==false)
	         {
	              laser_beta_track_no_confirmed_=laser_beta_track_no_confirmed_+increment_iteration_track_no_confirmed_;//increment_iteration_track_no_confirmed=0.02;
	              //std::cout<< " \n laser_beta_track_no_confirmed_="<<laser_beta_track_no_confirmed_<< std::endl;
	              //std::cout<< " id_tracker_="<< id_tracker_<< std::endl;
	              //  track_confirmado=0;
	         }
	    }
//	    std::cout<< " \n TRACK"<<id_tracker_<<" (máx_probabilidad) :" << max_probabilidad<< std::endl;
//	    printf("\n \n hipotesis_anterior:%d \n \n",hipotesis_anterior);
	    //printf("\n \n matrix_hipotesis_anterior_:%f",matrix_hipotesis_anterior_(0,0));
	    //std::cout<< "\n matrix_hipotesis_anterior_:\n" << matrix_hipotesis_anterior_(0,0)<< std::endl;
//	    printf( " \n \n " );

}

void Ctrack::kalman_states()  //change the name!!! (because the Kalman is only when the detections are associated with the tracks.
{

	if(debug_companion_akp_){
		std::cout<< " \n Entro en kalman_states" << std::endl;
	}
	//TODO:!!! change Kalman for include velocities.

//paso5: modification of new states of the tracks, by Kalman filter.
// the half part of this step is given by me for Gonzalo. (media and covariance of the before target)
        //update, of the states. correction of the target position.

//check that the function goes well!!!.
/*	std::cout<< "\n laser_detection_Zm_:\n" << laser_detection_Zm_<< std::endl;
	std::cout<< "\n laser_mean_track_:\n" << laser_mean_track_<< std::endl;
	std::cout<< "\n laser_Q_SIGMA_track_:\n" << laser_Q_SIGMA_track_<< std::endl;
	std::cout<< "\n laser_R_SIGMA_detection_:\n" << laser_R_SIGMA_detection_<< std::endl;
*/
//	printf("\n \n have_detection_laser_:%d\n",have_detection_laser_);
//	printf("laser_distance_:%f\n",laser_distance_);
//	printf("threshold_d_:%f\n",threshold_d_);
//	printf("\n \n hipotesis_anterior:%d \n",hipotesis_anterior);
//	printf("probability_of_detection_:%f \n",probability_of_detection_);

	Eigen::MatrixXd x_k(2,1);

    kalmanState_p_k_.resize(2,2); //P_k=[];
    kalmanState_x_k_.x=0;  //x_k=[];
    kalmanState_x_k_.y=0;
    SIGMA_track_kalman_.resize(2,2);//SIGMA_track_kalman=[];

    if( (have_detection_laser_==1) && (laser_distance_<=threshold_distance_) )
	{
    	if(debug_companion_akp_){
    	std::cout<< "\n  case asociated."<< std::endl;
    	}
    	track_update_=true;
        //only laser  (track<--->detection)   Track asociate with detection.
//    	printf("\n \n -- only laser \n \n");

    	K_matrix_.resize(2,2);
        //std::cout<< "\n SIGMA_track_kalman_:\n" << SIGMA_track_kalman_<< std::endl;

        Eigen::MatrixXd inverse_matrix (2,2);

//        std::cout<< "\n laser_Q_SIGMA_track_\n" << laser_Q_SIGMA_track_<< std::endl;
//        std::cout<< "\n H_\n" << H_<< std::endl;
//        std::cout<< "\n laser_R_SIGMA_detection_\n" << laser_R_SIGMA_detection_<< std::endl;
        inverse_matrix=(H_*laser_P_track_*H_.transpose()+laser_R_detection_).inverse();
//        std::cout<< "\n inverse_matrix:\n" << inverse_matrix<< std::endl;

        kalmanState_p_k_=laser_P_track_-laser_P_track_*H_.transpose()*inverse_matrix*H_*laser_P_track_;

//        std::cout<< "\n kalmanState_p_k_:\n" << kalmanState_p_k_<< std::endl;

        Eigen::MatrixXd inverse_matrix2;
        inverse_matrix2=(laser_R_detection_).inverse();
//         std::cout<< "\n inverse_matrix2:\n" << inverse_matrix2<< std::endl;


        K_matrix_=kalmanState_p_k_*H_.transpose()*inverse_matrix;//(laser_R_detection_).inverse(); //Ganancia de Kalman.

//        std::cout<< "\n K_matrix:\n" << K_matrix<< std::endl;

        //K=(double)K_matrix(0,0); //this is for check.
        //printf("K:%f \n",K);
//        std::cout<< "\n laser_detection_Zm_:\n" << laser_detection_Zm_<< std::endl;
//        std::cout<< "\n laser_mean_track_:\n" << laser_mean_track_<< std::endl;

        x_k=laser_mean_track_+K_matrix_*(laser_detection_Zm_-H_* laser_mean_track_);

//        std::cout<< "\n x_k:\n" << x_k<< std::endl;

        kalmanState_x_k_.x= (double) x_k(0,0);
		kalmanState_x_k_.y= (double) x_k(1,0);

		std::vector<double> kalman_cov;

		kalman_cov.reserve(16);
		kalman_cov.resize(16,0.0);
		kalman_cov=trackV_guardado_.cov;
		kalman_cov[0]=(double) kalmanState_p_k_(0,0); // cov_xx
		kalman_cov[1]=(double) kalmanState_p_k_(0,1); //cov_xy
		kalman_cov[4]=(double) kalmanState_p_k_(1,0); //cov_yx
		kalman_cov[5]=(double) kalmanState_p_k_(1,1); //cov_yy

//		std::cout<< " id_tracker_:"<<id_tracker_ << std::endl;
//		std::cout<< " Anterior_trackV_guardado_.x:"<<anterior_trackV_guardado_.x  << std::endl;
//		std::cout<< " Anterior_trackV_guardado_.y:"<<anterior_trackV_guardado_.y  << std::endl;
//		std::cout<< " trackV_guardado_.x:"<<trackV_guardado_.x  << std::endl;
//		std::cout<< " trackV_guardado_.y:"<<trackV_guardado_.y  << std::endl;
//		std::cout<< " Diferencia en x:"<< sqrt(pow(trackV_guardado_.x,2)) - sqrt(pow(anterior_trackV_guardado_.x,2))  << std::endl;
//		std::cout<< " Diferencia en y:"<< sqrt(pow(trackV_guardado_.y,2)) - sqrt(pow(anterior_trackV_guardado_.y,2))  << std::endl;

		if(cargo_ANTS){
			velocity_margin_=1.0;
		}

		if(debug_companion_akp_){
			std::cout<< " [kalman_states()]  =>>> velocity_margin_ = "<<velocity_margin_ << std::endl;
			std::cout<< "(ant) kalmanState_pose_: "<< std::endl;
			kalmanState_pose_.print();
		}

		if(( sqrt(pow(trackV_guardado_.x,2)) - sqrt(pow(anterior_trackV_guardado_.x,2)) >-velocity_margin_ ) && ( sqrt(pow(trackV_guardado_.x,2)) -sqrt(pow(anterior_trackV_guardado_.x,2)) < velocity_margin_ ) && ( sqrt(pow(trackV_guardado_.y,2)) - sqrt(pow(anterior_trackV_guardado_.y,2)) > -velocity_margin_  )&& ( sqrt(pow(trackV_guardado_.y,2)) - sqrt(pow(anterior_trackV_guardado_.y,2)) < velocity_margin_  ) ){
			if(debug_companion_akp_){
				std::cout<< " Entro en caso normal, creo... " << std::endl;
			}
			kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x ,kalmanState_x_k_.y , deteccion_guardada_.time_stamp ,trackV_guardado_.vx , trackV_guardado_.vy , kalman_cov); //TODO:poner time_stamp detección en el tiempo de esta.
			//track_no_associated_by_change_abrut_velocity_=false;
			//have_associated_detection_in_before_iteration=true;

		}else{
			if(debug_companion_akp_){
				std::cout<< " Entro en mala associación por velocidad!!!! " << std::endl;
			}
			// mala asociación por velocidad, caso como si no tubieras detección.
			kalmanState_x_k_.x=anterior_trackV_guardado_.x+((kalmanState_x_k_.x-anterior_trackV_guardado_.x)/4); // el centro del track propagado
			kalmanState_x_k_.y=anterior_trackV_guardado_.y+((kalmanState_x_k_.y-anterior_trackV_guardado_.y)/4);
			std::vector<double> kalman_cov;
			kalman_cov.reserve(16);
			kalman_cov.resize(16,0.0);
			kalman_cov=anterior_trackV_guardado_.cov;
			double cov_augment;
			if(cargo_ANTS){
				cov_augment=0.2;
			}else{
				cov_augment=0.15;
			}
			kalman_cov[0]=(double) kalman_cov[0]+cov_augment; // cov_xx
			kalman_cov[1]=(double) kalman_cov[1]; //cov_xy
			kalman_cov[4]=(double) kalman_cov[4]; //cov_yx
			kalman_cov[5]=(double) kalman_cov[5]+cov_augment;//cov_yy
			kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x ,kalmanState_x_k_.y , deteccion_guardada_.time_stamp ,anterior_trackV_guardado_.vx , anterior_trackV_guardado_.vy , kalman_cov);  // cuando hay associación con cambio muy brusco de velocidad, asumo que sigo con la velocidad anterior, ha sido una mala asociación.
			//probability_of_detection_=probability_of_detection_*pow(laser_beta_no_detection_,1); // reduzco probabilidad.
//			std::cout<< " kalmanState_pose_.vx:"<<kalmanState_pose_.vx  << std::endl;
//			std::cout<< " kalmanState_pose_.vy:"<<kalmanState_pose_.vy  << std::endl;
//			std::cout<< " kalmanState_pose_.x:"<<kalmanState_pose_.x  << std::endl;
//			std::cout<< " kalmanState_pose_.y:"<<kalmanState_pose_.y  << std::endl;
			//have_associated_detection_in_before_iteration=false;
			// otra opcion es marcar este track como con NO detección en este instante, has perdido una detección más. (hecha en comentario acontinuación)
			// track_no_associated_by_change_abrut_velocity_=true;
			//track_update_=false;
		}

		count_oclusions=0; // count oclusions =0, para oclusiones en cruces.
//		std::cout<< " Kalman time stamp track+deteccion"<<deteccion_guardada_.time_stamp  << std::endl;
//		std::cout<< " Association Kalman, id_track:" << id_tracker_<< std::endl;

		anterior_trackV_guardado_=kalmanState_pose_;

//		kalmanState_pose_.print();
	//Structure  of SpointV_cov( x_ , y_ , time_stamp_ , vx_, vy_, cov_)
		//cov is a vector that represent a matrix =[ (0) sigma_xx, (1) sigma_xy, (2) sigma_xVx, (3) sigma_xVy,
												//   (4) sigma_yx, (5) sigma_yy, (6) sigma_yVx, (7) sigma_yVy,
												//   (8) sigma_Vxx, (9) sigma_Vxy, (10) sigma_VxVx, (11) sigma_VxVy
												//   (12) sigma_Vyx, (13) sigma_Vyy, (14) sigma_VyVx, (15) sigma_VyVy,
	}
    else if( (have_detection_laser_==1) && (laser_distance_>threshold_distance_)  )
	{
    	if(debug_companion_akp_){
    		std::cout<< "\n  case single detection, I have to create new track."<< std::endl;
    	}

		//case single detection, I have to create new track.
    	track_update_=true;
		kalmanState_p_k_=laser_R_detection_;
		kalmanState_x_k_.x=(double) laser_detection_Zm_(0,0);
		kalmanState_x_k_.y=(double) laser_detection_Zm_(1,0);

		std::vector<double> kalman_cov;

		kalman_cov.reserve(16);
		kalman_cov.resize(16,0.0);

		kalman_cov=trackV_guardado_.cov;

		double cov_augment;
		if(cargo_ANTS){
			cov_augment=0.5;
		}else{
			cov_augment=0.1;
		}

		kalman_cov[0]=(double) laser_R_detection_(0,0)+cov_augment; // cov_xx
		kalman_cov[1]=(double) laser_R_detection_(0,1); //cov_xy
		kalman_cov[4]=(double) laser_R_detection_(1,0); //cov_yx
		kalman_cov[5]=(double) laser_R_detection_(1,1)+cov_augment; //cov_yy

		kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x, kalmanState_x_k_.y, deteccion_guardada_.time_stamp , trackV_guardado_.vx , trackV_guardado_.vy , kalman_cov); //TODO:poner time_stamp detección en el tiempo de esta.
		anterior_trackV_guardado_=kalmanState_pose_;

		//	std::cout<< " Kalman" << std::endl;
			//	std::cout<< " Kalman time stamp only detection"<<deteccion_guardada_.time_stamp  << std::endl;
		//	kalmanState_pose_.print();
	}
	else if( (have_detection_laser_==0) || (laser_distance_>threshold_distance_) )
	{
		if(debug_companion_akp_){
			std::cout<< "\n  CASO propaga track sin deteccion associada!!! (have_detection_laser_=="<<have_detection_laser_<<") || (laser_distance_="<<laser_distance_<<">threshold_distance_="<<threshold_distance_<<")"<< std::endl;
		}
		//if the track isn't have detection, the Kalman state is the track that passed me earlier.
		//track_update_=false;
		// CASO propaga track sin deteccion associada!!!

		track_update_=true;
		kalmanState_p_k_=laser_P_track_;
		kalmanState_x_k_.x=(double) laser_mean_track_(0,0);
		kalmanState_x_k_.y=(double) laser_mean_track_(1,0);
		//std::cout<< "\n (1) CASO propaga track sin deteccion associada!!! "<< std::endl;

		std::vector<double> kalman_cov;

		kalman_cov.reserve(16);
		kalman_cov.resize(16,0.0);
		double cov_augment;
		if(cargo_ANTS){
			cov_augment=0.5;
		}else{
			cov_augment=0.05; //antes estaba a  0.3 (y aumentaba demasiado, al propagar el track!) antes estaba a  0.15
		}
		//std::cout<< "\n (2) CASO propaga track sin deteccion associada!!! "<< std::endl;
		kalman_cov=trackV_guardado_.cov;
		kalman_cov[0]=(double) laser_P_track_(0,0)+cov_augment; // cov_xx
		kalman_cov[1]=(double) laser_P_track_(0,1); //cov_xy
		kalman_cov[4]=(double) laser_P_track_(1,0); //cov_yx
		kalman_cov[5]=(double) laser_P_track_(1,1)+cov_augment;//cov_yy
		//std::cout<< "\n (3) CASO propaga track sin deteccion associada!!! "<< std::endl;
	//	std::cout<< "\n MHT laser_P_track_" <<laser_P_track_<< std::endl;

		if(track_no_asociated_in_cross_situation_track_[id_tracker_]==false){
			// con anterior_trackV_guardado_.vx e vy lo que hago es usar la velocidad de la ultima vez que fue visto el track.
			//std::cout<< "\n (4) CASO propaga track sin deteccion associada!!! "<< std::endl;
			kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x, kalmanState_x_k_.y, time_stamp_ ,anterior_trackV_guardado_.vx , anterior_trackV_guardado_.vy, kalman_cov);

			//kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x, kalmanState_x_k_.y, time_stamp_ ,trackV_guardado_.vx , trackV_guardado_.vy, kalman_cov);
			if(debug_companion_akp_){
				std::cout<< "\n (CASO 1) kalmanState_pose_.x" <<kalmanState_pose_.x<<"; kalmanState_pose_.x"<<kalmanState_pose_.y<<"; kalmanState_pose_.vx="<<kalmanState_pose_.vx<<"; kalmanState_pose_.vy="<<kalmanState_pose_.vy<< std::endl;
				std::cout<< "\n (CASO 1) [Predition] kalmanState_pose_.vx" <<kalmanState_pose_.vx<<"; kalmanState_pose_.vy" <<kalmanState_pose_.vy<< std::endl;
			}
			//}else if((track_no_asociated_in_cross_situation_track_[id_tracker_]==true)&&(trackV_guardado_.v()<0.25)&&(trackV_guardado_.v()>-0.25)){
		}else if((track_no_asociated_in_cross_situation_track_[id_tracker_]==true)&&(trackV_guardado_.v()<0.25)&&(trackV_guardado_.v()>-0.25)){ // me hará que los cruces muy cercanos los haga mal!, pero lo necesito para oclusiones U_U
			//std::cout<< "\nTrack in cross situation Velocity=0.0" << std::endl;
			//std::cout<< "\n (5) CASO propaga track sin deteccion associada!!! "<< std::endl;
			kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x, kalmanState_x_k_.y, time_stamp_ ,0.0 , 0.0, kalman_cov);
			if(debug_companion_akp_){
			std::cout<< "\n (CASO 2) kalmanState_pose_.x" <<kalmanState_pose_.x<<"; kalmanState_pose_.x"<<kalmanState_pose_.y<<"; kalmanState_pose_.vx="<<kalmanState_pose_.vx<<"; kalmanState_pose_.vy="<<kalmanState_pose_.vy<< std::endl;
			}
		}else{
			//std::cout<< "\n (6) CASO propaga track sin deteccion associada!!! "<< std::endl;
			kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x, kalmanState_x_k_.y, time_stamp_ ,anterior_trackV_guardado_.vx , anterior_trackV_guardado_.vy, kalman_cov);

			//kalmanState_pose_=SpointV_cov( kalmanState_x_k_.x, kalmanState_x_k_.y, time_stamp_ ,trackV_guardado_.vx , trackV_guardado_.vy, kalman_cov);
			if(debug_companion_akp_){
			std::cout<< "\n (CASO 3) kalmanState_pose_.x" <<kalmanState_pose_.x<<"; kalmanState_pose_.x"<<kalmanState_pose_.y<<"; kalmanState_pose_.vx="<<kalmanState_pose_.vx<<"; kalmanState_pose_.vy="<<kalmanState_pose_.vy<< std::endl;
			}
		}
		//std::cout<< "\n (7) CASO propaga track sin deteccion associada!!! "<< std::endl;
		anterior_trackV_guardado_=kalmanState_pose_;
		//std::cout<< "\n (8) CASO propaga track sin deteccion associada!!! "<< std::endl;
		if(debug_companion_akp_){
				std::cout<< "MHT: Kalman time stamp only track"<<time_stamp_ << std::endl;
				std::cout<< "id_track:"<<id_tracker_ << std::endl;
				kalmanState_pose_.print();
		}
		//		std::cout<< "probability_of_detection_"<<probability_of_detection_<< std::endl;
	}
   // std::cout<< "\n (9) CASO propaga track sin deteccion associada!!! "<< std::endl;
    if(debug_companion_akp_){
    	std::cout<< "(desp) kalmanState_pose_: "<< std::endl;
    	kalmanState_pose_.print();
	}
    //std::cout<< "\n (10) CASO propaga track sin deteccion associada!!! "<< std::endl;
//    printf("\n \n kalmanState_x_k_.x:%f",kalmanState_x_k_.x);
//    printf("\n \n kalmanState_x_k_.y:%f",kalmanState_x_k_.y);

}

void Ctrack::update_no_track(double increment_time) //not have Spoint p, not have detection.
{


	// NO DETECCTION CON TIEMPO!!! exp(Inc_time/div_time); el div_time me marca cuanto quiero degradar el track.

	//function tracker without detection of any type!.
	  //I remain the track during few iterations, because our hypothesis remains that we have person although
	//we don't have a detection at this moment.

//	std::cout<< "\n Entro en update_no_track" << std::endl;
//	std::cout<< " \n NO detection in track!!!"<< std::endl;


//	std::cout<< " \n id_track= "<<get_idTracker()<< std::endl;

// initiation of generation of the new matrix hypothesis, copy of the before matrix.
	if(no_detection_time==0.0){
		probability_before_no_track_=probability_of_detection_;
	}
	no_detection_time=no_detection_time+increment_time;

	//if(track_no_asociated_in_cross_situation_track_[id_tracker_]==false){

	laser_N_ft_.clear();
	laser_N_nt_.clear();
	laser_N_dt_.clear();
	laser_N_tgt_.clear();
	laser_N_no_detection_.clear();

	matrix_hipotesis_actual_.resize(1,1);

	matrix_hipotesis_actual_(0,0)=hipotesis_anterior;  //I remain with the last before hypothesis that we had. (me quedo con la última hipótesis anterior que había/que teniamos).
																	// hypothesis =1, because we continue assuming that we have detection.

    vector_probability_actual_.resize(1,1); //Pi_vector=[];       // vector of probabilities of this iteration, se pone a 0 el: Eigen::MatrixXd vector_probability_actual_;.(es un vector en vertical, en vez de horizontal, para corresponderse con la posición de las hipótesis en la matriz de hipótesis)

//    std::cout<< " \n hipotesis_anterior:"<<hipotesis_anterior<< std::endl;

    unsigned int num_Probabilidades=matrix_hipotesis_anterior_.outerSize();

//   std::cout<< " \n matrix_hipotesis_anterior_.outerSize():"<<matrix_hipotesis_anterior_.outerSize()<< std::endl;

    vector_probability_before_.resize(1,1);
	//get the before Pi.
	vector_probability_before_(0,0)=probability_of_detection_;

//	std::cout<< " \n probability_of_detection_:"<<probability_of_detection_<< std::endl;

	double Pi;

	//calculate the probabilities of the case of no detection, decrease slowly of the detection probability.
	double multy_time;
	if(cargo_ANTS){
		multy_time=20;
	} else{
		multy_time=1;
	}


	for (unsigned int j=0; j<num_Probabilidades; j++) //filas Pi's=num hipótesis
	{
		int c=1;

	    //Nota: 0, núnca va ha poder ser, porque las falsas alarmas se eliminarán. puede
	    //ser 1, o -1 si lo cambio aquí para eliminarlo.

	    //aquí entra siempre, y hace la función de la
	    //exponencial/logaritmica (0.9)^(Num de veces sin detección), para que decrezca poco a poco la
	    //probabilidad de detección al no tener detección.
		//probabilidad en este caso!.

	    laser_N_ft_.push_back(0);
	    laser_N_nt_.push_back(0);
	    laser_N_dt_.push_back(0);
	    laser_N_tgt_.push_back(0);
	    laser_N_no_detection_.push_back(1);



	   if (cargo_ANTS){

	    Pi=(1/c)*pow(laser_Pd_,laser_N_dt_[j])*
	    	    		pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
	    	    		pow(laser_beta_ft_,laser_N_ft_[j])*
	    	    		pow(pow(laser_beta_no_detection_,multy_time*no_detection_time),laser_N_no_detection_[j])*
	    	    		pow(laser_beta_nt_,laser_N_nt_[j])*1*probability_before_no_track_;
	   }else{

		   /*Pi=(1/c)*pow(laser_Pd_,laser_N_dt_[j])*
		   	    		pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
		   	    		pow(laser_beta_ft_,laser_N_ft_[j])*
		   	    		pow(laser_beta_no_detection_,laser_N_no_detection_[j])*
		   	    		pow(laser_beta_nt_,laser_N_nt_[j])*1*vector_probability_before_(j,0);*/

		   Pi=(1/c)*pow(laser_Pd_,laser_N_dt_[j])*
		   	    	    		pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
		   	    	    		pow(laser_beta_ft_,laser_N_ft_[j])*
		   	    	    		pow(pow(laser_beta_no_detection_,multy_time*no_detection_time),laser_N_no_detection_[j])*
		   	    	    		pow(laser_beta_nt_,laser_N_nt_[j])*1*probability_before_no_track_;

	   }


	}

	vector_probability_actual_(0,0)=Pi;

	probability_of_detection_=Pi;

	//case no detection, but we continue with the assumption that no detection. (pero seguimos con la hipótesis de que hay detección.)
	matrix_hipotesis_anterior_.resize(1,1);
	matrix_hipotesis_anterior_(0,0)=hipotesis_anterior; //matrix_hipotesis_anterior_(0,0)=matrix_hipotesis_actual_(0,0); // struct_matriz_hipotesis_anteriores(numero_matrizes_hipotesis+1).matrix=matrix_hipotesis;

//	std::cout<< " \n final hipotesis_anterior :"<<hipotesis_anterior<< std::endl;
	if(debug_companion_akp_){
	std::cout<< " \n Caso propagation, no det!!!:"<< std::endl;
	}
//	std::cout<< " \n final probability_of_detection_:"<<probability_of_detection_<< std::endl;
	//sacar resultados por pantalla:

//	std::cout<< " hipotesis_actual :" <<  matrix_hipotesis_anterior_<< std::endl;
//	printf( " \n " );

	//If don't have detection, I reduce the probability of detection slowly with
	//parameter beta_no_detection_.

	//como no hay muestra=detección el paso 5 lo que hace es
	// retornar el mismo track que nos han pasado en la entrada.
	// al no haber detección no hay corrección.
	//}
	kalman_states(); //se usa la función de los estados de kalman,
							// que te devuelve el mismo estado que te entra
							//para el caso en que no hay ninguna detección de ningún detector.

}

void Ctrack::update_track(Spoint_cov detection, SpointV_cov track, detection_type type, double actual_increment)
{
	if(debug_clusters_fallos_t){
	std::cout<< " \n Entro en update_track (track + deteccion)\n" << std::endl;
	}

	if(cargo_ANTS){
		threshold_distance_=1;
		increment_iteration_track_no_confirmed_=0.01; // valor para tibi y dabo= 0.01.
	}
	Increment_time_old_track_=Increment_time_old_track_+actual_increment;

	no_detection_time=0.0;

    	set_tracks_and_detections(detection, track, type);

//    	detection.print();
//    	track.print();
    	if(debug_clusters_fallos_t){
    	std::cout<< " \n despues de set_tracks_and_detections\n" << std::endl;
    	}

		make_matrix_hipotesis_and_parameters_of_probabilities();

		//creation probabilities of the hypothesis
		if(debug_clusters_fallos_t){
		std::cout<< " \n despues de parameters_of_probabilities\n" << std::endl;
		}

		make_probabilities();

		if(debug_clusters_fallos_t){
		std::cout<< " \n despues de make_probabilities\n" << std::endl;
		}
		//confirmation and prune the hypothesis. Again, I remain only with the two inicial hypothesis
		// 0=falsa_alarma o 1=detección_asociada_a_track

		prune_probabilities();

		if(debug_clusters_fallos_t){
		std::cout<< " \n despues de prune_probabilities\n" << std::endl;
		}
		// modification of the new states of the tracks, mediante the Kalman filter
		// the half part of this step is give to me by Gonzalo. (media and covarianca of the before target)
	        //update, of the states. correction of the target position.

		kalman_states();

		if(debug_clusters_fallos_t){
		std::cout<< " \n despues de kalman_states\n" << std::endl;
		}
}

void Ctrack::update_trackn( SpointV_cov track,double increment_time, detection_type type)
{
//	std::cout<< "\n Entro en update_track (SOLO Track)\n" << std::endl;
	Increment_time_old_track_=Increment_time_old_track_+increment_time;

    set_tracks_and_detections( Spoint_cov(), track, type);
	update_no_track(increment_time);
}

//////////////////////////////////////
// Functions for crosses in tracks //
////////////////////////////////////

SpointV_cov Ctrack::get_first_window_track(){

	SpointV_cov first_track_window;
	first_track_window= window_before_trackV_.at(0);
	return first_track_window;
}

bool Ctrack::get_check_window_track_empty(){
 //true if the container size is 0, false otherwise.
	bool is_empty;
	is_empty= window_before_trackV_.empty();
	return is_empty;
}


/* BEFORE function update_no_track: (para poder desacer cambio de eiminación por tiempo si da problemas)
 * void Ctrack::update_no_track() //not have Spoint p, not have detection.
{

	//function tracker without detection of any type!.
	  //I remain the track during few iterations, because our hypothesis remains that we have person although
	//we don't have a detection at this moment.

//	std::cout<< "\n Entro en update_no_track" << std::endl;
//	std::cout<< " \n NO detection in track!!!"<< std::endl;
//	std::cout<< " \n id_track= "<<get_idTracker()<< std::endl;


	//if(track_no_asociated_in_cross_situation_track_[id_tracker_]==false){

	laser_N_ft_.clear();
	laser_N_nt_.clear();
	laser_N_dt_.clear();
	laser_N_tgt_.clear();
	laser_N_no_detection_.clear();

	matrix_hipotesis_actual_.resize(1,1);

	matrix_hipotesis_actual_(0,0)=hipotesis_anterior;  //I remain with the last before hypothesis that we had. (me quedo con la última hipótesis anterior que había/que teniamos).
																	// hypothesis =1, because we continue assuming that we have detection.

    vector_probability_actual_.resize(1,1); //Pi_vector=[];       // vector of probabilities of this iteration, se pone a 0 el: Eigen::MatrixXd vector_probability_actual_;.(es un vector en vertical, en vez de horizontal, para corresponderse con la posición de las hipótesis en la matriz de hipótesis)

//    std::cout<< " \n hipotesis_anterior:"<<hipotesis_anterior<< std::endl;

    unsigned int num_Probabilidades=matrix_hipotesis_anterior_.outerSize();

//   std::cout<< " \n matrix_hipotesis_anterior_.outerSize():"<<matrix_hipotesis_anterior_.outerSize()<< std::endl;

    vector_probability_before_.resize(1,1);
	//get the before Pi.
	vector_probability_before_(0,0)=probability_of_detection_;

//	std::cout<< " \n probability_of_detection_:"<<probability_of_detection_<< std::endl;

	double Pi;

	//calculate the probabilities of the case of no detection, decrease slowly of the detection probability.

	for (unsigned int j=0; j<num_Probabilidades; j++) //filas Pi's=num hipótesis
	{
		int c=1;

	    //Nota: 0, núnca va ha poder ser, porque las falsas alarmas se eliminarán. puede
	    //ser 1, o -1 si lo cambio aquí para eliminarlo.

	    //aquí entra siempre, y hace la función de la
	    //exponencial/logaritmica (0.9)^(Num de veces sin detección), para que decrezca poco a poco la
	    //probabilidad de detección al no tener detección.
		//probabilidad en este caso!.

	    laser_N_ft_.push_back(0);
	    laser_N_nt_.push_back(0);
	    laser_N_dt_.push_back(0);
	    laser_N_tgt_.push_back(0);
	    laser_N_no_detection_.push_back(1);



		   Pi=(1/c)*pow(laser_Pd_,laser_N_dt_[j])*
		   	    		pow(1-laser_Pd_,laser_N_tgt_[j]-laser_N_dt_[j])*
		   	    		pow(laser_beta_ft_,laser_N_ft_[j])*
		   	    		pow(laser_beta_no_detection_,laser_N_no_detection_[j])*
		   	    		pow(laser_beta_nt_,laser_N_nt_[j])*1*vector_probability_before_(j,0);

	   }


	}

	vector_probability_actual_(0,0)=Pi;

	probability_of_detection_=Pi;

	//case no detection, but we continue with the assumption that no detection. (pero seguimos con la hipótesis de que hay detección.)
	matrix_hipotesis_anterior_.resize(1,1);
	matrix_hipotesis_anterior_(0,0)=hipotesis_anterior; //matrix_hipotesis_anterior_(0,0)=matrix_hipotesis_actual_(0,0); // struct_matriz_hipotesis_anteriores(numero_matrizes_hipotesis+1).matrix=matrix_hipotesis;

//	std::cout<< " \n final hipotesis_anterior :"<<hipotesis_anterior<< std::endl;
//	std::cout<< " \n Caso propagation, no det!!!:"<< std::endl;
//	std::cout<< " \n final probability_of_detection_:"<<probability_of_detection_<< std::endl;
	//sacar resultados por pantalla:

//	std::cout<< " hipotesis_actual :" <<  matrix_hipotesis_anterior_<< std::endl;
//	printf( " \n " );


	//If don't have detection, I reduce the probability of detection slowly with
	//parameter beta_no_detection_.

	//como no hay muestra=detección el paso 5 lo que hace es
	// retornar el mismo track que nos han pasado en la entrada.
	// al no haber detección no hay corrección.
	//}
	kalman_states(); //se usa la función de los estados de kalman,
							// que te devuelve el mismo estado que te entra
							//para el caso en que no hay ninguna detección de ningún detector.

}
 */

