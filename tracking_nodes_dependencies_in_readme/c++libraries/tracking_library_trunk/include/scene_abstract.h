/*
 * scene_abstract.h
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
#ifndef scene_abstract_H
#define scene_abstract_H

#include "scene_elements/person_abstract.h"
#include "iri_geometry.h"
#include "scene_elements/robot.h"
#include <list>
#include <vector>


/**
 *
 * \brief Scene_abstract Class
 *
 * The scene class provides an abstract interface and a set
 * of methods to store, manage,
 * get and set information regarding the moving targets present
 * in a given scene. It also manages the social-forces relative
 * to each of the members present in Cscene
 *
 */

class Cscene_abstract
{
	public:
	    enum person_container_type { Person_Virtual=0, Robot, Obstacle, Person_BHMIP};
		Cscene_abstract( Cperson_abstract::filtering_method filter= Cperson_abstract::No_filtering,
				Cperson_abstract::update_person_method update_method = Cperson_abstract::Autoremove,
				Cperson_abstract::force_type type=Cperson_abstract::Spherical );
		virtual ~Cscene_abstract()=0;
		void set_destinations ( std::vector<Sdestination>& dest );
		const std::vector<Sdestination>* get_destinations() { return &destinations_; }
		//std::vector<Sdestination> get_destinations2() { return destinations_; }
		void set_dt( double dt ) {
			std::cout << " dt_="<<dt_<< std::endl;
			dt_ = dt;
		}
		double get_dt() { return dt_; }
		double get_time(){ return now_;}
		Cperson_abstract* add_person( unsigned int id );//private
		//, Cperson_abstract::target_type type=Cperson_abstract::Person,
		//		Cperson_abstract::force_type f_type=Cperson_abstract::Spherical );
		virtual Cperson_abstract* add_person_container( unsigned int id,
				std::list<Cperson_abstract*>::iterator iit )=0;
		void update_other_person_companion(unsigned int id, SpointV_cov in_other_person_companion_point); // for case group of two people and one robot.
		void remove_person( unsigned int id );
		virtual void update_robot( Spose  observation , bool case_zanlungo=false);
		virtual void update_person_companion( Spose  observation , SpointV_cov obs_spoint);
		const Crobot* get_robot() const { return robot_;}
		Crobot* get_robot2(){ return robot_;}
		Crobot* get_person_companion_akp() {
			return person_companion_;
		}
		void set_person_companion_akp(Crobot* ini_person) { person_companion_=ini_person;}
		void set_robot_destinations( std::vector<Sdestination>& dest ){ robot_->set_destinations( dest );}
		/**
		 * \brief get_scene
		 *
		 * This abstract function is the main method to update a scene  and
		 * its implementation depends on the class of scene.
		 */
		virtual void update_scene(const std::vector<SdetectionObservation>& observation,bool& we_have_person_companion, bool person_or_robot=false) = 0;
		const std::list<Cperson_abstract*>* get_scene() { return &person_list_; }
		bool find_person(unsigned int id);
		bool find_person(unsigned int id , Cperson_abstract** person);
		bool find_person(unsigned int id , Cperson_abstract** person, std::list<Cperson_abstract*>::iterator& it);
		void print();
		void set_scene_force_type( Cperson_abstract::force_type type ){ scene_force_type_ = type;}
		Cperson_abstract::force_type get_scene_force_type( ) { return scene_force_type_;}

		// Sforce methods
		Sforce force_persons_int( Cperson_abstract* person , bool if_robot_is_person_companion=false);
		Sforce force_persons_int_companion( Cperson_abstract* center , unsigned int id_person_companion, bool person_reactive, Cperson_abstract::companion_reactive type_rec_akp); // made by (ely), for forces in robot, companion mode.
		Sforce force_persons_int2( Cperson_abstract* person , std::vector<int> ids_of_persons_in_group ); // made by ely, groups of persons.
		Sforce force_persons_int3( Cperson_abstract* center ); // made by ely (companion, no take into accound the person companion of the group force.)

		Sforce force_objects_laser_int( Cperson_abstract* person );//this function calculates the repulsion forces due to static obstacles
		//void calculate_social_forces_all_persons();//to be deprecated
		virtual const std::vector<double>* get_sfm_int_params( const Cperson_abstract * center_person,
				const Cperson_abstract * interacting_person = NULL);
		const std::vector<double>* get_sfm_params( const Cperson_abstract * center_person );
		std::vector<double> new_get_sfm_params( const Cperson_abstract * center_person);
		virtual void clear_scene()=0;

		//read laser
		void read_laser_scan( const std::vector<Spoint>&  laser_scan , bool person_simulation=false);//this is the only public function for processing laser scans
		void read_laser_scan_person_companion_akp_planner( const std::vector<Spoint>&  laser_scan , bool person_simulation=false);
		void read_laser_scan_companion( const std::vector<Spoint>&  laser_scan , bool person_simulation=false);
		const std::vector<Spoint>* get_laser_obstacles() { return &laser_obstacle_list_; }

		//read map
		bool read_force_map( const char * path );
		Sforce get_force_map( double x, double y );
		Sforce get_force_map_robot_position( ){
			return get_force_map( robot_->get_current_pointV().x, robot_->get_current_pointV().y );}
		bool is_cell_clear_map( double x, double y );
		void get_map_params( float &min_x, float &max_x, float &min_y, float &max_y, float &resolution ,
				unsigned int &map_number_x, unsigned int &map_number_y);
		bool get_map_obstacle(unsigned int i) { return obstacle_map_.at(i); }
		//temporal
		unsigned int xy_to_m(double x, double y);
		Spoint m_to_xy( unsigned int m);
		bool read_destination_map( const char * path );
		bool read_destination_map2( const char * path );

		//force parameters methods
		void set_sfm_to_person( const std::vector<double>& params ){ social_forces_param_to_person_ = params;}
		void set_sfm_to_robot( const std::vector<double>& params ){ social_forces_param_to_robot_ = params;}
		void set_sfm_to_obstacle( const std::vector<double>& params ){ social_forces_param_to_obs_ = params;}
		void set_sfm_to_person_companion( const std::vector<double>& params ){
			social_forces_param_to_person_companion_ = params;
			//std::cout << " social_forces_param_to_person_companion_[0]="<<social_forces_param_to_person_companion_[0]<<"; [1]="<< social_forces_param_to_person_companion_[1]<<"; [2]="<<social_forces_param_to_person_companion_[2]<<"; [3]="<<social_forces_param_to_person_companion_[3]<<"; [4]="<<social_forces_param_to_person_companion_[4]<< std::endl;

			//std::cout << "IN set_sfm_to_person_companion!!!!!!!!!!!!!!!!!!!! SIZE="<<social_forces_param_to_person_companion_.size() << std::endl;
		}

		void set_sfm_to_person_companion_central( const std::vector<double>& params ){
			social_forces_param_to_person_companion_ = params;
			//std::cout << " social_forces_param_to_person_companion_[0]="<<social_forces_param_to_person_companion_[0]<<"; [1]="<< social_forces_param_to_person_companion_[1]<<"; [2]="<<social_forces_param_to_person_companion_[2]<<"; [3]="<<social_forces_param_to_person_companion_[3]<<"; [4]="<<social_forces_param_to_person_companion_[4]<< std::endl;

			//std::cout << "IN set_sfm_to_person_companion!!!!!!!!!!!!!!!!!!!! SIZE="<<social_forces_param_to_person_companion_.size() << std::endl;
		}

		std::vector<double> get_set_sfm_to_person_companion(){
			return social_forces_param_to_person_companion_;
		}

		// local tracker methods
		virtual void rotate_and_traslate_scene(unsigned int id, double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, bool debug  = false){}
		void set_bool_sim(bool in_sim_bool_){sim_bool_=in_sim_bool_;};
		void Cscene_abstract_set_max_d_to_detect_laser_obs(double in_max_d_to_detect_laser_obs){
			max_d_to_detect_laser_obs_=in_max_d_to_detect_laser_obs;
			std::cout << "set Cscene_abstract_set_max_d_to_detect_laser_obs-> max_d_to_detect_laser_obs_="<<max_d_to_detect_laser_obs_<< std::endl;
		}

		void set_max_asos_point_to_person(unsigned int in_max_asos_point_to_person){
			max_asos_point_to_person_=in_max_asos_point_to_person;
		}

		void set_sim(bool in_sim){
			sim_=in_sim;
			std::cout << " Change sim value:"<< std::endl;
			if(sim_){
				std::cout << " Change sim_=true"<< std::endl;
			}else{
				std::cout << " Change sim_=false"<< std::endl;
			}

		}


	protected:
		std::vector<Sdestination> destinations_;//prior scene destinations (all of them)
		Crobot* robot_;//robot is in a different list than normal persons, and additionally we have its pointer for specific robot methods
		Crobot* person_companion_; // for use the akp planner on the person companion.
		std::list<Cperson_abstract *> person_list_;//list of persons in the scene, abstract containers
		double dt_;
		double now_;
		std::vector<Spoint> laser_obstacle_list_;//list of obstacles seen by laser
		bool read_laser_obstacle_success_;
		//std::vector<Spoint> map_obstacle_list_;//TODO: static list of obstacles according to map
		std::vector<Sforce> force_map_;
		std::vector<bool> obstacle_map_;
		//temporal occupancy matrix
		std::vector<Spoint> xy_map_;
		bool read_force_map_success_;
		unsigned int map_number_x_,map_number_y_;
		float min_x_,max_x_,min_y_,max_y_,map_resolution_;
		bool read_destination_map_success_;
		Cperson_abstract::force_type scene_force_type_;
		std::vector<double> social_forces_param_to_person_, social_forces_param_to_robot_, social_forces_param_to_obs_,social_forces_param_to_person_companion_;
		Cperson_abstract::filtering_method filtering_method_;
		Cperson_abstract::update_person_method update_method_;
		bool robot_in_the_scene_;
		bool person_companion_in_the_scene_;

		// read laser private methods
		bool point_corresponds_person( Spoint laser_point);
		bool point_corresponds_person_companion( Spoint laser_point );
		bool point_belongs_to_laser_obstacle( Spoint p , bool person_simulation=false);

		bool sim_bool_;
		double max_d_to_detect_laser_obs_;
		std::list<Cperson_abstract*> real_person_list_;
		unsigned int max_asos_point_to_person_;

		bool debug_update_person_companion_; // debug for person companion
		bool debug_find_person_; // debug for person companion

		bool sim_; // if sim=true case simulations, else if sim=false case real robot.


};
#endif
