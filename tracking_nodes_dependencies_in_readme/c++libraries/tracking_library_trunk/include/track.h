/*
 * track.h
 *
 *  Created on: 07/07/2013 (aprox)
 *      Author: erepiso
 */

#ifndef _PEOPLE_TRACKING_MHT_H
#define _PEOPLE_TRACKING_MHT_H

#include "mhtStructs.h"

#include <eigen3/Eigen/Dense> //Library for matrix. Being here, not needed in cpp. (Al estar aquí, no hacen falta en el cpp).
#include <iostream> //Library for screen messages.
#include <math.h> //library for sen cos sqrt etc.

class Ctrack
{
  public: //put here the variables and functions to be used for other people

    //public functions:
	enum detection_type { Laser=0 , PointCloud , Vision, ZigBee};
    Ctrack(int id=0);
    ~Ctrack();
    void set_time_stamp(double time_stamp){ time_stamp_=time_stamp; };

    //Functions to change tracker parameters from outside.
    double get_threshold_distance_InTrack(){ return threshold_distance_; };
    void set_threshold_distance_InTrack(double threshold_distance_in){
    	threshold_distance_=threshold_distance_in;
        //std::cout<< " \n TRACK! set_threshold_distance_InTrack: \n \n"<< threshold_distance_ <<  std::endl;
    };
    double get_threshold_confirmation_track_InTrack(){ return threshold_confirmation_track_; };
    void set_threshold_confirmation_track_InTrack(double threshold_confirmation_track_in){
    	threshold_confirmation_track_=threshold_confirmation_track_in;
        std::cout<< " \n TRACK! set_threshold_confirmation_track_InTrack: \n \n"<<threshold_confirmation_track_ <<  std::endl;
    };
    double get_laser_Pd_InTrack(){ return laser_Pd_; };
    void set_laser_Pd_InTrack(double laser_Pd_in){
    	laser_Pd_=laser_Pd_in;
        std::cout<< " \n TRACK! set_laser_Pd_InTrack: \n \n"<< laser_Pd_<<  std::endl;
    };
    double get_laser_beta_ft_InTrack(){ return laser_beta_ft_; };
    void set_laser_beta_ft_InTrack(double laser_beta_ft_in){
    	laser_beta_ft_=laser_beta_ft_in;
        std::cout<< " \n TRACK! set_laser_beta_ft_InTrack: \n \n" <<laser_beta_ft_<<  std::endl;
    };
    double get_laser_beta_nt_InTrack(){ return laser_beta_nt_; };
    void set_laser_beta_nt_InTrack(double laser_beta_nt_in){
    	laser_beta_nt_=laser_beta_nt_in;
        std::cout<< " \n TRACK! set_laser_beta_nt_InTrack: \n \n"<<laser_beta_nt_ <<  std::endl;
    };
    double get_laser_beta_no_detection_InTrack(){ return laser_beta_no_detection_; };
    void set_laser_beta_no_detection_InTrack(double laser_beta_no_detection_in){
    	laser_beta_no_detection_=laser_beta_no_detection_in;
        std::cout<< " \n TRACK! set_laser_beta_no_detection_InTrack: \n \n"<< laser_beta_no_detection_ <<  std::endl;
    };
    double get_laser_beta_track_no_confirmed_InTrack(){ return laser_beta_track_no_confirmed_; };
    void set_laser_beta_track_no_confirmed_InTrack(double laser_beta_track_no_confirmed_in){
    	laser_beta_track_no_confirmed_=laser_beta_track_no_confirmed_in;
        std::cout<< " \n TRACK! set_laser_beta_track_no_confirmed_InTrack: \n \n" <<laser_beta_track_no_confirmed_<<  std::endl;
     };
     double get_increment_iteration_track_no_confirmed_InTrack(){ return increment_iteration_track_no_confirmed_; };
     void set_increment_iteration_track_no_confirmed_InTrack(double increment_iteration_track_no_confirmed_in){
     	increment_iteration_track_no_confirmed_=increment_iteration_track_no_confirmed_in;
        std::cout<< " \n TRACK! set_increment_iteration_track_no_confirmed_InTrack: \n \n"<<increment_iteration_track_no_confirmed_ <<  std::endl;
     };

    void set_track_no_asociated_in_cross_situation_track(std::vector<bool> track_no_asociated_in_cross_situation_track){
    	 track_no_asociated_in_cross_situation_track_=track_no_asociated_in_cross_situation_track;
        };
    void update_no_track(double increment_time); //case function tracker without detection. (NO Spoint)
    void update_track( Spoint_cov detection, SpointV_cov track, detection_type type=Laser, double actual_increment=0); //case function tracker with detection!.
    void update_trackn( SpointV_cov track, double increment_time, detection_type type=Laser);
    void set_tracks_and_detections(Spoint_cov detection, SpointV_cov track, detection_type type=Laser);	//filled the "structs"/variables of the detections and tracks that I need to make the tracker.
    void set_tracks_and_detections(Spoint_cov detection, detection_type type=Laser);
    void ini_track( Spoint_cov detection, detection_type type=Laser, double actual_increment=0); //==inicializacion_tracker_tres_detectores_bien, LOOK!!!! => In the first detection I don't have track, therefore I don't calculate distances. So, I do not need the detections = Spoint p, detection_type type=Laser

    void make_matrix_hipotesis_and_parameters_of_probabilities();//==crea_matriz_hipotesis_y_parametros_para_probabilidades(); //en todas faltan las variables!!!
    void make_probabilities();//==crear_probabilidades(); //use the laser_Q and etc, to recalculate the distances, o calculate directily the distances if I change it directly.
    void prune_probabilities();//==podado_probabilidades(); //requires only the probability vector created in the previous function!
    void kalman_states();//==modificacion_estados_mediante_kalman(); //need the laser_Q and etc, for calculate the new Kalman estates.

    bool is_initialized(){ return inicialization_laser_;}
    bool have_detection_laser(){ return have_detection_laser_;}

    double get_probability(){ return probability_of_detection_; }
    SpointV_cov get_kalman_update(){ return kalmanState_pose_; }
    void set_kalman_update(SpointV_cov kalmanState_pose_odometry_correction){kalmanState_pose_=kalmanState_pose_odometry_correction; }; // para hacer el update_odometry de mis tracks cada vez que venga un odometria nueva del robot.
    int get_hipotesis_iter(){return hipotesis_anterior;}
    double get_laser_distance(){ return laser_distance_; }
    double get_threshold_distance(){ return threshold_distance_; }
    unsigned int get_idTracker(){ return id_tracker_; }
    bool get_track_confirmed(){ return track_confirmed_; }

    void set_initialized(){ inicialization_laser_ = true; }
    void set_laser_distance(double distance_mht){ laser_distance_ = distance_mht; } //to give the laser distances from outside of the function.
    void set_idTracker(double id_tracker){ id_tracker_= id_tracker; } //to give the id's of the tracks from outside of the function.
    void set_track_in_coss_situatuion(bool track_in_coss_situatuion){track_in_coss_situatuion_=track_in_coss_situatuion;}
    SpointV_cov get_propagation_guardada(){ return trackV_guardado_; } //devuelve la propagacion de este track.

    bool get_if_track_is_update(){ return track_update_; }

    bool get_track_no_associated_by_change_abrut_velocity(){ return track_no_associated_by_change_abrut_velocity_; }
    SpointV_cov get_first_window_track();
    bool get_check_window_track_empty();
    SpointV_cov get_anterior_trackV_guardado(){return anterior_trackV_guardado_;};

    double get_Increment_time_old_track_(){return Increment_time_old_track_;};

    void set_velocity_margin(double in_velocity_margin){
    	velocity_margin_=in_velocity_margin;
    }

    //variables
  private:

    bool debug_clusters_fallos_t;
    double time_stamp_;
    SpointV_cov trackV_guardado_;  //save the in track of this time, for fill the covariance parameters of correction Kalman track.
    Spoint_cov deteccion_guardada_; //save the in detection of this time,
    SpointV_cov anterior_trackV_guardado_;
    unsigned int id_tracker_;
    bool inicialization_laser_;
    int index_number_of_hypothesis_; //counter for the Number of hypothesis!!!
    bool track_confirmed_; //habría que adaptar la parte de la inicialización al track de los tres detectores también, no solo al multitracker de solo laser!!!
    bool no_hay_deteccion_track_propagacion_;

//tracker output variables, probability and matrix/vector hypothesis.
    double probability_of_detection_; //puede que haya que hacer dos como con las matrices. una anterior y otra actual.
    Eigen::MatrixXi matrix_hipotesis_anterior_;
    Eigen::MatrixXi matrix_hipotesis_actual_; // matrix variable changeful depending on the number of detections and tracks associated.
    int hipotesis_anterior;
    double Pi_zero_;

//variables for propagation of Kalman estates. (propagación estados según kalman)
    Spoint kalmanState_x_k_;
    Eigen::MatrixXd kalmanState_p_k_; //MatrixXd. X=cuando no sabes k valor es. d=double. Porque esta matriz es variable según el número de estados.
    Eigen::MatrixXd SIGMA_track_kalman_;
    SpointV_cov kalmanState_pose_;  // This is the mht output track in the actual iteration, my tracks!!!

//variables for  calculation and  y pruned probabilities.
    Eigen::MatrixXd vector_probability_before_;
    Eigen::MatrixXd vector_probability_actual_;

// constant variables.
    double threshold_distance_;
    double threshold_probability_; //=0.001. for the prune of hypothesis by probabilities and not stay only with a the high probability. (no quedarse solo con la máxima)
    double threshold_confirmation_track_;
    Eigen::MatrixXd H_;

    double laser_Pd_;
    double laser_beta_ft_;
    double laser_beta_nt_;
    double laser_beta_no_detection_; //oclusion in matlab, becomes no_detection. that is truly what is this variable. (que es verdaderamente lo que es esta variable.)
    double laser_beta_track_no_confirmed_;
    double increment_iteration_track_no_confirmed_;

//variables distancias detección->track for each detector.
   double laser_distance_;//==variable laser_d  of matlab.

//variables detections for each detector. struct_detecciones matlab, for the people tracker of three detectors.
   int num_detections_; //num_detecciones.
   bool have_detection_laser_; // equals 1 if have detection, equals 0 if  have not detection.

//variables Kalman update.
   Eigen::MatrixXd K_matrix_;
   Eigen::MatrixXd laser_P_track_; //laser_P_track==anterior laser_Q_SIGMA_track, para correspondencias con matlab.
   Eigen::MatrixXd laser_R_detection_;//laser_R_detection==anterior laser_R_SIGMA_detection, para correspondencias con matlab.
   std::vector<double> detection_laser_gaussian_detection_; //Spoint for detección
   std::vector<double> detection_laser_gaussian_track_; //Spose for track.

//variables for the subsequent calculation of probabilities. struct_number_of_mesurements of matlab.
   std::vector<int> laser_N_tgt_;
   std::vector<int> laser_N_ft_;
   std::vector<int> laser_N_dt_;
   std::vector<int> laser_N_nt_;
   std::vector<int> laser_N_no_detection_;

//variable for use time in no detection.
   double no_detection_time;
   double probability_before_no_track_;

//matrix y vectors for the calculation of the distances.
    Eigen::MatrixXd laser_detection_Zm_; //tendrá que ir en vertical
    Eigen::MatrixXd laser_mean_track_;

    bool track_update_;

// oclusiones en cruces y oclusiones en general.
    std::vector<bool> track_no_asociated_in_cross_situation_track_;
    unsigned int count_oclusions;

// margen velocidades cambios bruscos
    double velocity_margin_; // margin for the abrupt changes in velocity
    bool track_no_associated_by_change_abrut_velocity_;
    bool have_associated_detection_in_before_iteration;
    bool track_in_coss_situatuion_;

// Crosses situation: Save the before track poses, for differentiate in cross situation. (ventana de 10 o 15 anteriores, algo así...)
    double time_window_;  // first prove with 1s
    std::deque<SpointV_cov> window_before_trackV_;

// variables cargo ANTS
    bool NO_track_cargo_ANTS;
    bool cargo_ANTS;

    //En private se ponen aquí las variables y funciones que vayan a usar solo mi tracker.
    	  //escribes como constantes del sistema todos aquellos parámetros que tienes en matlab para 1 track, por ejemplo:
    	  //vigila que algunos miembros estan en la clase Cperson
    	  //funciones guia, de Gonzalo-> people_prediction: trunk/src/peoplePredictionStructs y trunk/src/scene_elements/person


    bool debug_companion_akp_;

    double Increment_time_old_track_; // to eliminate bad tracks with we have clusters.


};


#endif

