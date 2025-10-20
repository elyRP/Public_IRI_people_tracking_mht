#ifndef person_abstract_H
#define person_abstract_H


#include <deque>
#include <vector>
#include <list>
#include "iri_geometry.h"
#include <stddef.h>
#include <assert.h>
#include <iostream>
#include <fstream>      // std::ofstream

/**
 *
 * \brief Behavior Class
 *
 * The behavior class is a container class  for the different behavior paramters
 * used for a specific human motion prediction (Cperson_behavior)
 *
*/
class Sbehavior
{
  public:
	Sbehavior( unsigned int id );
	~Sbehavior();
	unsigned int related_person_id;
	std::vector<double> expectation;
	void print() const;
};

/**
 *
 * \brief Person Class 
 *
 * The person class is an abstract container class defining the interfaces
 * of the person prediction and trajectory storing (if needed)
 * 
 * different types of persons are defined: Person , Robot , Obstacle
 * depending on the prediction method or scene type
 *
*/
class Cperson_abstract
{
	public:
		enum target_type { Person=0 , Robot , Obstacle, Person_companion};
		enum force_type { Spherical=0, Elliptical, Collision_Prediction};
		enum behavior_type { Balanced=0, Aware, Unaware, Count_behaviors};
	    enum filtering_method{ No_filtering=0, Linear_regression_filtering, Bayes_filtering, Low_pass_linear_regression_filtering};
	    enum companion_reactive{ Akp_planning=0, Reactiva_repulsive=1, Reactive_atractive=2, Person_companion_akp=3};
	    //Manual remove requires an external call to eliminate a person by using the scene method ()
	    enum update_person_method { Autoremove=0, Manual_remove };
	    enum vel_per_ok{ Vel_per=0, Near_goal, Far_goal};
	    Cperson_abstract (  unsigned int id=0 ,
				Cperson_abstract::target_type target_type=Cperson_abstract::Person,
				Cperson_abstract::force_type person_force_type=Cperson_abstract::Spherical,
				double _time_window = 1.0);
		virtual ~Cperson_abstract()=0;
		unsigned int get_id() const { return person_id_; }
		void set_id( unsigned int id ) { person_id_ = id; }
		bool operator== (Cperson_abstract& p2) const;
		bool operator!= (Cperson_abstract& p2) const { return !(*this==p2); }

		virtual void add_pointV( SpointV_cov point ,
				Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering , bool person_or_robot=false, double in_new_value_windowing=6.0)= 0;

		virtual void Cperson_bhmip_add_pointV( SpointV_cov point, Cperson_abstract::filtering_method filter=Cperson_abstract::Linear_regression_filtering, bool person_or_robot=false) = 0;
		virtual void refresh_person( double now ) = 0;
		virtual void prediction( double min_v_to_predict) = 0;
		void set_best_dest(const Sdestination& dest ){
			//std::cout << "(set_best_dest) dest:"<< std::endl;
			//dest.print();
			best_destination_=dest;
			//std::cout << " best_destination_:"<< std::endl;
			//best_destination_.print();
		}
		Sdestination get_best_dest(){
			//std::cout << "(get_best_dest) best_destination_:"<< std::endl;
			//best_destination_.print();
			//std::cout << "(get_best_dest) best_destination_:"<<best_destination_.x<<"; best_destination_.y"<<best_destination_.y<< std::endl;

			return best_destination_;
		}
		void set_destinations( const std::vector<Sdestination>& dest ) { 	destinations_ = dest; }
		const std::vector<Sdestination>* get_destinations()   { return &destinations_; }
		std::vector<Sdestination> get_destinations2()   { return destinations_; }
		void set_desired_velocty(double v){
			//std::cout << "(set_desired_velocty, ant) desired_velocity_ ="<<desired_velocity_ << std::endl;
			desired_velocity_ = v;
			//std::cout << "(set_desired_velocty, act, changed) desired_velocity_ ="<<desired_velocity_ << std::endl;
		}
		double get_desired_velocity() { return desired_velocity_; }
		/**
			 * \brief point propagation
			 *
			 * this method propagates person position dt time in the future.
			 * if no destinations are set (or the best destination estimated is null)
			 * then propagates linearly with velocity as a short term predictor.
			*/
		SpointV_cov pointV_propagation( double dt , Sforce force = Sforce() );
		SpointV_cov pointV_propagation( double dt , Sforce force, SpointV_cov point );

		const SpointV_cov& get_current_pointV() const { return current_pointV_; }
		void set_current_pointV(SpointV_cov in_current_pointV) {
			//std::cout << "INNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN SET CURRENT POINT; current_pointV_.x="<<current_pointV_.x<<"; current_pointV_.y="<<current_pointV_.y<< std::endl;
			current_pointV_=in_current_pointV;
		} // made by (ely) change the initial robot position for robot companion.
		double get_time() {
			std::cout << "current_pointV_.time_stamp"<<current_pointV_.time_stamp<< std::endl;
			return current_pointV_.time_stamp; }

		//misc methods
		virtual void reset()=0;
		virtual void reset_before_destination_prob()=0;
		void print();
		void print_dest();
		void set_person_type( target_type type ) {
			//std::cout << "IN person type"<< std::endl;
			//std::cout << "type"<<type<< std::endl;
			//std::cout << " SET person tipe => type_"<<type_<<"; person id="<<person_id_<< std::endl;
			type_ = type;
			//std::cout << "out person type"<< std::endl;
		}
		target_type get_person_type () const { return type_; }

		//forces calculation
		/**
			 * \brief force_cp
			 *
			 *  Calculates the interacting forces using the collision prediction model
			 *  described by Zanlungo et al in "" , 2011.
			 *
			*/
		Sforce force( const SpointV& interacting_person , const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL ,  Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);
		Sforce forceAnticipateCollision( force_type act_person_force_type, const SpointV& interacting_person ,  const std::vector<double>* social_forces_param, const SpointV* virtual_current_point= NULL, Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning );


		Sforce force_sphe( const Spoint& interacting_person , const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL , Cperson_abstract::companion_reactive reactive=Cperson_abstract::Akp_planning);
		Sforce force_ellip( const SpointV& interacting_person ,  const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL  );
		Sforce force_cp( const SpointV& interacting_person , const std::vector<double>* social_forces_param,
				const SpointV* virtual_current_point = NULL );
		Sforce force_goal( const Sdestination& dest , const std::vector<double>* social_forces_param, const SpointV* virtual_current_point = NULL , unsigned int parent_vertex=10, bool output_mesages=false);
		Sforce force_goal_near(  const Sdestination& dest, const std::vector<double>* social_forces_param, const SpointV* virtual_current_point = NULL , double d_max_reduce_vel=1.0, bool in_go_with_vel_per=false,  double real_person_vel=0.9);
		Sforce force_goal_companion( const Sdestination& dest , const std::vector<double>* social_forces_param, const SpointV* virtual_current_point = NULL , unsigned int parent_vertex=10, bool output_mesages=false);

		// , double real_person_vel= default the max velociy of the robot,
		Sforce force_sphe_prob( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
	            const SpointV* virtual_current_point = NULL );
	    Sforce force_sphe_mahalanobis( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
	            const SpointV* virtual_current_point = NULL );
	    Sforce force_sphe_worst( const SpointV_cov& interacting_person , const std::vector<double>* social_forces_param,
	            const SpointV* virtual_current_point = NULL );
		Sforce get_force_person() { return force_to_goal_ + force_int_person_ + force_int_robot_ + force_obstacle_; }
		Sforce get_forces_person( Sforce&  force_to_goal, Sforce& force_int_person , Sforce& force_int_robot, Sforce& force_obstacle ) const;
		Sforce get_forces_person_companion( Sforce&  force_to_goal, Sforce& force_int_person , Sforce& force_int_robot, Sforce& force_obstacle, Sforce& force_companion  ) const;
		Sforce get_force_int_robot_person() { return force_int_robot_; }
		void set_forces_person( Sforce  force_to_goal, Sforce force_int_person , Sforce force_int_robot, Sforce force_obstacle  );
		void set_forces_person_companion( Sforce force_to_goal, Sforce force_int_person, Sforce force_int_robot, Sforce force_obstacle , Sforce force_companion );
		void set_int_force( Sforce force_int ) { force_int_person_ = force_int;}
		void set_int_robot_force( Sforce force_int ) { force_int_robot_ = force_int;}
		void set_obstacle_force( Sforce force_int ) { force_obstacle_ = force_int;}
		SpointV_cov get_diff_position(){ return diff_pointV_;}
		bool is_observation_updated(){ return observation_update_;}
		void set_observation_update(bool in){
			observation_update_ = in;
		}

		//behavior estimation methods
		virtual Sbehavior * find_behavior_estimation( unsigned int id ) { return NULL;}
		virtual Cperson_abstract::behavior_type get_best_behavior_to_person( unsigned int interacting_person ) const
			{ return Cperson_abstract::Balanced;}

		//methods for trajectory prediction
		virtual void prediction_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 ) {}
		virtual void planning_propagation( double dt , Sforce force = Sforce(), unsigned int index = 0 ) {}
		virtual void planning_propagation_copy( unsigned int prediction_index ) {}
		virtual bool is_needed_to_propagate_person_for_planning( unsigned int parent_index, Spoint robot, unsigned int& new_index_to_be_copied) {return true;}
		virtual void reset_propagation_flag() {}
		virtual const std::vector<SpointV_cov>* get_prediction_trajectory() const {
			std::cout << " (Cperson_abstract) get_prediction_trajectory()"<< std::endl;
			return NULL;
		}//carefull not to use this method if not prepared for predictions
		virtual const std::vector<SpointV_cov>* get_prediction_trajectory_with_target_person() const {
			std::cout << " (Cperson_abstract) get_prediction_trajectory_with_target_person()"<< std::endl;
				return NULL;
		}//carefull not to use this method if not prepared for predictions


		virtual const std::vector<SpointV_cov>* get_planning_trajectory() const {
			std::cout << " (Cperson_abstract)  get_planning_trajectory()"<< std::endl;
			return NULL;
		}//carefull not to use this method if not prepared for predictions
		virtual void clear_prediction_trajectory() { }
		virtual void clear_planning_trajectory() { }
		virtual void reserve_planning_trajectory( unsigned int n) { }


		virtual void rotate_and_translate_trajectory(double R, double thetaZ, double linear_vx,
				double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug_odometry = false){}; // For local tracker

		/* functions person companion (ely) */
		Sforce force_sphe_atractive( const Spoint& interacting_person ,  const std::vector<double>* social_forces_param , const SpointV* virtual_current_point );
		void set_companion_force_goal(Sdestination robot_companion_dest){robot_companion_dest_=robot_companion_dest;};
		Sdestination get_companion_force_goal() const;

		void set_debug_velocity_file(bool debug_velocity_file_in){
			debug_velocity_file_=debug_velocity_file_in;
		}

		void set_debug_velocity_cout(bool debug_velocity_cout_in){
			debug_velocity_cout_=debug_velocity_cout_in;
		}

		void set_debug_filename(std::string in_debug_file_){
			debug_file_=in_debug_file_;
		}

		void set_externa_force_k_near_goal(double in_externa_force_k_near_goal_){
			externa_force_k_near_goal_=in_externa_force_k_near_goal_;
		}

		void set_externa_force_k_far_goal(double in_externa_force_k_far_goal_){
			externa_force_k_far_goal_=in_externa_force_k_far_goal_;
		}

		void set_ex_max_dist_to_near_goal_force_in(double in_max_dist_to_near_goal_force_){
			max_dist_to_near_goal_force_=in_max_dist_to_near_goal_force_;
		}

		virtual const std::deque<SpointV_cov>* get_past_trajectory()=0;

		void clear_Zanlungo_vectors();

		 std::vector<double> get_v_desired_robot_x();
		 std::vector<double> get_v_desired_robot_y();

		 std::vector<double> get_v_current_robot_x();
		 std::vector<double> get_v_current_robot_y();

		 std::vector<double> get_actual_random_goal_x();
		 std::vector<double> get_actual_random_goal_y();

		 std::vector<double> get_actual_distance_in_x();
		 std::vector<double> get_actual_distance_in_y();
		 std::vector<double> get_actual_module_distance();
		 std::vector<double> get_actual_in_desired_velocity();

		 virtual void set_new_time_window(double in_new_time_window){ }

		/*virtual const std::vector<double> get_planning_v()=0;
		virtual const std::vector<double> get_planning_w()=0;
		virtual const std::vector<double> get_planning_x()=0;
		virtual const std::vector<double> get_planning_y()=0;
		virtual const std::vector<double> get_planning_theta()=0;
		virtual const std::vector<double> get_planning_before_av()=0;
		virtual const std::vector<double> get_planning_before_aw()=0;
		virtual const std::vector<double> get_planning_after_av()=0;
		virtual const std::vector<double> get_planning_after_aw()=0;
		virtual const std::vector<double> get_planning_damping_aw()=0;*/






	protected:
		unsigned int person_id_;
		SpointV_cov current_pointV_;
		SpointV diff_pointV_;
		Sdestination best_destination_;
		Sdestination before_best_destination_; // (ely) to solve problems of before probabilities of destinations when the person destination change!
		std::vector<Sdestination> destinations_; //local person destinations. prior probabilities
		double desired_velocity_;
		target_type type_;//enum target_type { Person=0 , Robot , Obstacle, Person_companion }; Virtual_Person <= este ultimo de Gonzalo desapareció antes de mi epoca.
		force_type person_force_type_;
		bool observation_update_;
		Sforce force_to_goal_, force_int_person_ , force_obstacle_, force_int_robot_, force_companion_;
		double now_;

		/* robot companion (ely) */
		Sdestination robot_companion_dest_;
		bool debug_velocity_file_;
		bool debug_velocity_cout_;

		 std::string debug_file_;

		 double externa_force_k_near_goal_;
		 double externa_force_k_far_goal_;
		 double max_dist_to_near_goal_force_;

		 bool debug_force_goal_; // debug for person companion

		 std::deque<SpointV_cov> past_trajectory_;

		    std::vector<double> v_desired_robot_x_; // estan dentro de person abstract es mas complicado.
		    std::vector<double> v_desired_robot_y_;
		    std::vector<double> v_current_robot_x_;
		    std::vector<double> v_current_robot_y_;
		    std::vector<double> actual_random_goal_x_;
		   	std::vector<double> actual_random_goal_y_;

		    std::vector<double> actual_distance_in_x_;
		   	std::vector<double> actual_distance_in_y_;
			std::vector<double> actual_module_distance_;
			std::vector<double> actual_in_desired_velocity_;
};

//global functions to calculate forces
Sforce force_sphe( const SpointV& center_person, const SpointV& interacting_person , const std::vector<double>* social_forces_param );
Sforce force_goal( const Sdestination dest, const SpointV& center_person , const std::vector<double>* social_forces_param );


#endif
