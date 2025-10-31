/*
 * prediction_bhmip.h
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
#ifndef prediction_bhmip_H
#define prediction_bhmip_H

#include "scene_abstract.h"
#include "scene_elements/person_abstract.h"
#include "iri_geometry.h"
#include <vector>
#include <list>


/**
 *
 * \brief Prediction_bhmip Class
 *
 * The prediction class provides a set of algorithms
 * solving intentionality prediction and long term prediction.
 * A set of destinations is assumed to be provided
 *
 *
*/
class Cprediction_bhmip : public Cscene_abstract
{
	public:
		Cprediction_bhmip ( Cperson_abstract::filtering_method filter= Cperson_abstract::No_filtering,
				Cperson_abstract::update_person_method update_method = Cperson_abstract::Autoremove );
		virtual ~Cprediction_bhmip();
		virtual void update_scene(const std::vector<SdetectionObservation>& observation ,bool& we_have_person_companion,  bool person_or_robot=false);
		virtual void rotate_and_traslate_scene(unsigned int id, double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug  = false);
		virtual Cperson_abstract* add_person_container( unsigned int id,
				std::list<Cperson_abstract*>::iterator iit );
		/**
		 * scene_intentionality_prediction_bhmip( ) calculates the intentionality prediction
		 * for observed trajectories
		 */
		void scene_intentionality_prediction_bhmip( );
		void see_tracks_on_scene();
		virtual void clear_scene();

		void set_min_v_to_predict( double min_v_to_predict ) { min_v_to_predict_ =  min_v_to_predict;}
		double get_min_v_to_predict() { return min_v_to_predict_;}

		void set_filtering_time_window( double time_window) {
			//std::cout << "IN set_filtering_time_window-> filtering_time_window_="<<filtering_time_window_<< std::endl;
			filtering_time_window_ = time_window;
		}
		double get_filtering_time_window( void ) {return filtering_time_window_;}

		void set_id_person_companion_Cprediction_bhmip(unsigned int in_id_person_companion) {
			id_person_companion_ = in_id_person_companion;
			//std::cout << "IN set_id_person_companion_Cprediction_bhmip-> id_person_companion_="<<id_person_companion_<< std::endl;
		}

		void set_id_second_person_companion_Cprediction_bhmip(unsigned int in_id_second_person_companion){
			id_second_person_companion_=in_id_second_person_companion;
		}

		unsigned int my_id_person_companion_; // id, actual person companion. the id of it selft
		void set_my_id_second_person_companion_sim_Cprediction_bhmip(unsigned int in_my_id_person_companion){
			my_id_person_companion_=in_my_id_person_companion;
		}

		void Cprediction_bhmip_set_max_d_to_detect_laser_obs(double in_max_d_to_detect_laser_obs){
			Cscene_abstract_set_max_d_to_detect_laser_obs(in_max_d_to_detect_laser_obs);
			//std::cout << "set Cprediction_bhmip_set_max_d_to_detect_laser_obs-> in_max_d_to_detect_laser_obs="<<in_max_d_to_detect_laser_obs<< std::endl;
		}

		//void set_debug_filter_person_velocity(bool debug_filter_person_velocity_in){
		//	debug_filter_person_velocity_=debug_filter_person_velocity_in;
		//}

		const std::list<Cperson_abstract*>* get_scene2() { return &person_list_; }
		void print2();
		// INI NEW FACKE CHANGE AND DESAPEAR ID:
		void remove_person_from_outside(unsigned int id_people_to_remove){
			remove_person(id_people_to_remove);
		}
		// FIN NEW FACKE CHANGE AND DESAPEAR ID:
	protected:
		double min_v_to_predict_;
		double filtering_time_window_;
		unsigned int id_person_companion_,id_second_person_companion_;

		//bool debug_filter_person_velocity_;
		bool debug_update_scene_; // debug for person companion



};
#endif
