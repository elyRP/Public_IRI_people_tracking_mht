 /*
 * person_bhmip.h
 *
 *  Created on: Jul 9, 2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration).
 *      Author: gferrer
 *      Author: Ely Repiso (Modifications for people tracking mht)
 *
 *      license (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
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
