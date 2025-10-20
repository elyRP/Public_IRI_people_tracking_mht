/*
 * personReal.h
 *
 *  Created on: Jul 9, 2013
 *      Author: gferrer
 */

#ifndef PERSON_BHMIP_H_
#define PERSON_BHMIP_H_

#include "scene_elements/person_abstract.h"

class Cperson_bhmip : public Cperson_abstract
{
  public:
	Cperson_bhmip(unsigned int id=0,
			Cperson_abstract::target_type person_target_type=Cperson_abstract::Person,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 5.0);
	virtual ~Cperson_bhmip();
	const std::deque<SpointV_cov>* get_trajectory2() {return &trajectory_;}
	//std::deque<std::vector<double> >& get_phi_prob() { return phi_prob_; }
	//std::deque<std::vector<double> >& get_phi_pose2dest() { return phi_pose2dest_; }
	virtual void reset();
	virtual void reset_before_destination_prob();
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering,  bool robot_or_person=false, double in_new_value_windowing=6.0);
	virtual void Cperson_bhmip_add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering, bool person_or_robot=false);
	virtual void refresh_person( double now );
	virtual void prediction(  double min_v_to_predict );

	virtual void rotate_and_translate_trajectory(double R, double thetaZ,
			double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug_odometry = false); // For local tracker.

	void set_debug_filter_person_velocity(bool debug_filter_person_velocity_in){
		debug_filter_person_velocity_=debug_filter_person_velocity_in;
	}

	virtual const std::deque<SpointV_cov>* get_past_trajectory(){return &trajectory_;};

	virtual void set_new_time_window(double in_new_time_window){
		time_window_=in_new_time_window;
	}

  private:
	void trajectory_windowing( );
	std::deque<SpointV_cov> trajectory_;
	double time_window_; //0.5 segs
	SpointV_cov filter_current_state_linear_regression( );
	SpointV_cov filter_current_state_linear_regression_bayes( );
	SpointV_cov low_pass_filter_current_state_linear_regression( );
	void  intention_precalculation();
	std::vector<double> posterior_destinations_prob_;
	double phi_var_;
	std::deque<std::vector<double> > phi_pose2dest_ , phi_prob_;
	double time_from_last_update_;





	// variables for local tracker
	std::deque<std::vector<double>> hom2_tf;
	std::vector<double> hom2_ant;
	std::deque<double> trajectory_local_velocity_x_;
	std::deque<double> trajectory_local_velocity_y_;

	bool debug_filter_person_velocity_; // add in addaptative companion ely
	bool debug_add_pointV_; // debug for simulation person companion.
	bool debug_Cperson_bhmip_add_pointV_; // debug for simulation person companion.
	bool debug_trajectory_windowing_;// debug for simulation person companion.
	bool debug_refresh_person_;// debug for simulation person companion.
	bool debug_prediction_;// debug for simulation person companion.
	bool debug_intention_precalculation_;// debug for simulation person companion.
	bool debug_filter_current_state_linear_regression_;

	bool person_before_moving_;
	Sdestination before_best_destination_;

};


#endif /* PERSON_BHMIP_H_ */
