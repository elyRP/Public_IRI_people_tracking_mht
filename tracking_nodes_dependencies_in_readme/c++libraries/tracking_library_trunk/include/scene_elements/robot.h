/*
 * robot.h
 *
 *  Created on: Jul 31, 2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration).
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
#ifndef ROBOT_H_
#define ROBOT_H_

#include "scene_elements/person_abstract.h"
#include <fstream>      // std::ofstream
//#include "Vector2D.h"
//#include "companion_zanlungo/CompanionFrancescoModel.h"

/**
 *
 * \brief Robot class
 *
 * The robot class inherints from Cperson_prediction and thus it can use of the
 * calculation for the interaction with other scene elements.
 *
 * Addtionally, it is specifically designed to describe different robot propagations
 * such as a differential platform, a balancing differential (segway RMP200) or others.
 * It is intended to serve as a container for the planning algorithms described in this library.
 *
*/

class Crobot : public Cperson_abstract
{
  public:
	enum robot_type{Differential, Diff_balancing, Car};
	Crobot(  unsigned int id=0, robot_type robot_type=Differential,
			Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
			double _time_window = 1.0);
	virtual ~Crobot();
	// Cperson methods for updating robot position
	virtual void add_pointV( SpointV_cov point,
			Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering, bool robot_or_person=false, double in_new_value_windowing=6.0) {}//not used in this class
	virtual void Cperson_bhmip_add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering, bool robot_or_person=false){}//not used in this class
	virtual void refresh_person( double now ) {} //this method is not used in Robot
	virtual void add_pose( Spose observation );
	Spose get_current_pose() {
		//std::cout << " IMPORTANT!!!! IN get robot pose!; current_pose_.x="<<current_pose_.x<<"; current_pose_.y="<<current_pose_.y<< std::endl;
		return current_pose_; }
	void set_current_pose(Spose in_current_pose) { current_pose_=in_current_pose; } // for change the initial pose in robot companion!
	Spose get_diff_pose() { return diff_pose_;}
	// prediction
	virtual void prediction( double min_v_to_predict );
	virtual void reset();
	virtual void reset_before_destination_prob();

	//methods for trajectory prediction, inherited form Cperson_abstract
	virtual const std::vector<SpointV_cov>* get_prediction_trajectory_with_target_person() const { return &prediction_trajectory_SpointV_;}
	virtual const std::vector<SpointV_cov>* get_propagated_pose_onlyV_trajectory_with_target_person() const { return &prediction_trajectory_only_vel_SpointV_;}
	//virtual const std::vector<SpointV_cov>* get_prediction_trajectory_Zanlungo() const { return &prediction_trajectory_;}


	virtual const std::vector<SpointV_cov>* get_prediction_trajectory() const { return &planning_SpointV_trajectory_;}
	virtual const std::vector<SpointV_cov>* get_planning_trajectory() const { return &planning_SpointV_trajectory_;}
	std::vector<SpointV_cov> get_planning_trajectory_vector()  { return planning_SpointV_trajectory_;}
	void set_planning_trajectory(std::vector<SpointV_cov> in_planning_trajectory) { planning_SpointV_trajectory_=in_planning_trajectory;}
	virtual void clear_planning_trajectory();
	virtual void reserve_planning_trajectory( unsigned int n) { planning_trajectory_.reserve(n);}

	//specific robot planning methods and data structures functions
	const std::vector<Spose>* get_robot_planning_trajectory() const {
		//std::cout <<" planning_trajectory_.size()="<<planning_trajectory_.size()<< std::endl;
		return &planning_trajectory_;}
	void erase_last_planning_propagation();
	//cinematic propagation
	//void robot_propagation(double dt, unsigned int index = 0, double v=1.0, double w=1.0 );
	//dynamic propagation
	void robot_propagation(double dt, unsigned int index = 0 , const Sforce& f = Sforce() , bool debug_force_vel=false);
	//void robot_propagation_Zanlungo(double dt , unsigned int index, const Sforce &f , SpointV_cov pose_of_the_robot, double before_angular_velocity , bool debug_force_vel=false); // con fuera entro en esta!!!

	Spose robot_propagation_companion_position(double dt, const Sforce& f = Sforce(),Spose in_ini_pose=Spose(), bool coments=false); // made by (ely) for robot companion.
	void set_rnd_local_goal( Spoint goal ) { rnd_goal_ = goal; }

	void correct_state_to_delay( Spose last_control_cmd, double now, double delay );

	//paramters
	void set_v_max( double v) { v_max_ = v; this->set_desired_velocty(v);}
	double get_v_max () { return v_max_; }
	void set_w_max( double w) { w_max_ = w;}
	double get_w_max () { return w_max_; }
	void set_a_v_max( double av) { a_v_max_ = av;}
	double get_a_v_max () { return a_v_max_; }
	void set_a_v_max_negativa( double av) { a_v_max_negativa_ = av;}
	double get_a_v_max_negativa() { return a_v_max_negativa_; }
	void set_a_v_break( double av) { a_v_break_ = av;}
	double get_a_v_break () { return a_v_break_; }
	void set_a_w_max( double aw) { a_w_max_ = aw;}
	double get_a_w_max () { return a_w_max_; }
	double get_platform_radii() { return platform_radii_; }
	double get_platform_radii_2() { return platform_radii_2_;}
	void set_platform_radii( double r ) { platform_radii_ = r; platform_radii_2_ = r*r;}

	void set_robot_debug_vel_file(bool in_debug_velocity_){
		debug_velocity_file2_=in_debug_velocity_;
	}
	void set_robot_debug_vel_cout(bool in_debug_velocity_){
		debug_velocity_cout2_=in_debug_velocity_;
	}

	void set_robot_debug_filename(std::string in_debug_file_){
		debug_file_=in_debug_file_;
	}
	//behavior estimation methods
	virtual Sbehavior * find_behavior_estimation( unsigned int id );
	virtual Cperson_abstract::behavior_type get_best_behavior_to_person( unsigned int interacting_person ) const;

	/* robot companion (ely) */
	//void set_companion_force_goal(Sdestination robot_companion_dest){robot_companion_dest_=robot_companion_dest;};
	//Sdestination get_companion_force_goal(){return robot_companion_dest_;};
	virtual const std::deque<SpointV_cov>* get_past_trajectory(){}; //not used in this class. (created by ely)

	void prediction_propagation( double dt , unsigned int index, const Sforce &f  , double max_person_vel=0.0 );
	void clear_prediction_trajectory();
	void prediction_propagation2_only_vel( double dt , unsigned int index, const Sforce &f , double distance_person_t_x,double distance_person_t_y,double vel_person_x,double vel_person_y , double max_person_vel );
	void clear_prediction_trajectory_onlyV();

	void set_high_vel_dampening_parameter(double in_dampening_parameter){
		high_vel_dampening_parameter_=in_dampening_parameter;
	}
	void set_normal_vel_dampening_parameter(double in_dampening_parameter){
		normal_vel_dampening_parameter_=in_dampening_parameter;
	}
	void set_slow_vel_dampening_parameter(double in_dampening_parameter){
		slow_vel_dampening_parameter_=in_dampening_parameter;
	}

	void set_limit_linear_vel_for_dampening_parameter(double in_limit_linear_vel){
		limit_linear_vel_for_dampening_parameter_=in_limit_linear_vel;
	}

	void set_limit_angular_vel_for_dampening_parameter(double in_limit_angular_vel){
		limit_angular_vel_for_dampening_parameter_=in_limit_angular_vel;
	}

	const  std::vector<double>* get_planning_v() const{
		return &planning_v_;
	}
	const  std::vector<double>* get_planning_w() const{
		return &planning_w_;
	}
	const  std::vector<double>* get_planning_x() const{
		return &planning_x_;
	}
	const  std::vector<double>* get_planning_y() const{
		return &planning_y_;
	}
	const std::vector<double>* get_planning_theta() const{
		return &planning_theta_;
	}
	const std::vector<double>* get_planning_before_av() const{
		return &planning_before_av_;
	}

	const std::vector<double>* get_planning_before_aw() const{
		return &planning_before_aw_;
	}
	const std::vector<double>* get_planning_after_av() const{
		return &planning_after_av_;
	}
	const std::vector<double>* get_planning_after_aw() const{
		return &planning_after_aw_;
	}
	const std::vector<double>* get_planning_damping_aw() const{
		return &planning_damping_aw_;
	}

	/*void robot_set_genome(Cgenome in_prob_clas_genome){
		prob_clas_genome_=in_prob_clas_genome;
	}*/

	std::vector<double> get_time_stamp_plan(){
		return time_stamp_plan_;
	}


    std::vector<double> get_force_act_x(){
    	return force_act_x_;
    }
    std::vector<double> get_force_act_y(){
    	return force_act_y_;
    }

    std::vector<double> get_rad_acc(){
    	return  rad_acc_;
    }

    std::vector<double> get_nextv(){
    	return nextv_;
    }
    std::vector<double> get_omega(){
    	return omega_;
    }
    std::vector<double> get_selft_v_x(){
    	return selft_v_x_;
    }
    std::vector<double> get_selft_v_y(){
    	return selft_v_y_;
    }

    //void clear_prediction_pose_zanlungo_vec();

    void set_augment_initial_v(bool in_augment_initial_v){
    	augment_initial_v_=in_augment_initial_v;
    }

    void set_initial_v_robot_needed(double in_initial_v_robot_needed){
    	initial_v_robot_needed_=in_initial_v_robot_needed;
    }

  private:
	//robot parameters
	Spose current_pose_, diff_pose_;
	double v_max_, w_max_, a_v_max_, a_v_break_, a_w_max_, a_v_max_negativa_; // a_v_max_negativa_ added by ely to stop slowly in companion.
	double platform_radii_, platform_radii_2_;
	std::vector<Spose> planning_trajectory_, prediction_trajectory_, prediction_trajectory_only_vel_;  //vector de Sposes de la trajectoria del robot.
	std::vector<SpointV_cov> planning_SpointV_trajectory_, prediction_trajectory_SpointV_, prediction_trajectory_only_vel_SpointV_;



	Spoint rnd_goal_;

	//behavior containers
	std::list<Sbehavior> expected_behavior_list_;

	/* robot companion (ely) */
	//Sdestination robot_companion_dest_;
	bool debug_antes_subgoals_entre_AKP_goals_;

	double test_w_max_;
	double test_aw_max_;

	bool debug_velocity_file2_;
	bool debug_velocity_cout2_;

	 std::string debug_file_;

	bool debug_robot_propagation_;


	// to obtain the best value when we increment the velocity, to allow the robot to does not do "S" behaviour.
	double high_vel_dampening_parameter_; // original normal dampening_parameter set by Gonzalo Ferrer is dampening_parameter_=1.6; when robot max velocity was 0.9 m/s
	double normal_vel_dampening_parameter_;
	double slow_vel_dampening_parameter_;
	double limit_linear_vel_for_dampening_parameter_;
	double limit_angular_vel_for_dampening_parameter_;

	// to limit the increment of angular acceleration depending on the linear velocity of the robot
	double threshold_linear_velocity_to_limit_aw_;
	double max_increment_aw_slow_vel_;
	double max_increment_aw_normal_vel_;
	double max_increment_aw_max_vel_;

	// to see problems on propagation:
    std::vector<double> planning_v_;
    std::vector<double> planning_w_;
    std::vector<double> planning_x_;
    std::vector<double> planning_y_;
    std::vector<double> planning_theta_;
    std::vector<double> planning_before_av_;
    std::vector<double> planning_before_aw_;
    std::vector<double> planning_after_av_;
    std::vector<double> planning_after_aw_;
    std::vector<double> planning_damping_aw_;


    std::vector<double> force_act_x_;
    std::vector<double> force_act_y_;
    std::vector<double> rad_acc_;
    std::vector<double> nextv_;
    std::vector<double> omega_;
    std::vector<double> selft_v_x_;
    std::vector<double> selft_v_y_;


    //CompanionFrancescoModel prob_clas_Francesco_model_;
   // Cgenome prob_clas_genome_;

	std::vector<double> time_stamp_plan_;
	bool augment_initial_v_;
	double initial_v_robot_needed_;

};

#endif /* PERSON_ROBOT_H_ */
