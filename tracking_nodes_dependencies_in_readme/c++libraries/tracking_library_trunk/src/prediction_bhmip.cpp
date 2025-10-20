#include "prediction_bhmip.h"
#include "scene_elements/person_bhmip.h"
#include <math.h>
#include <algorithm>
#include <stdio.h>
#include <Eigen/Dense>
#include <iostream>



Cprediction_bhmip::Cprediction_bhmip( Cperson_abstract::filtering_method filter,
		Cperson_abstract::update_person_method update_method ) :
		Cscene_abstract(filter, update_method, Cperson_abstract::Spherical),
		id_person_companion_(1),
		id_second_person_companion_(2),
		debug_update_scene_(false)
{
	//read destinations from txt
	//this->set_destinations(dest);
}

Cprediction_bhmip::~Cprediction_bhmip()
{
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		delete (*iit);
	}
	delete robot_;
}

void Cprediction_bhmip::see_tracks_on_scene()
{

	std::cout << " see_tracks_on_scene() "<< std::endl;

	Cperson_abstract* found_person;
	SdetectionObservation  point;

	for(std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
	{
		SpointV_cov point_person=(*iit)->get_current_pointV();
		std::cout << "point_person.x="<< point_person.x<<"; point_person.y="<<point_person.y<< std::endl;
	}
}


void Cprediction_bhmip::update_scene(const std::vector<SdetectionObservation>& observation,bool& we_have_person_companion, bool person_or_robot)
{
	// the filter bool sets if we want to filter velocities or use the provided in the update.
	//associate current targets with tracking observations
	Cperson_abstract* found_person;
	SdetectionObservation  point;


	/*for( Cperson_abstract* iit: person_list_ )
				{
					if ( iit->get_person_type() == Cperson_abstract::Person)
					{

							if(iit->get_id()==2){
								std::cout << "INI (update_scene) FIND CHANGE TARGET PERSON: id="<< iit->get_id()<< std::endl;
								iit->get_current_pointV().print();
							}

					}
				}*/


	if(debug_update_scene_){
		std::cout << "!!!!!!!!!!!!! ENTRO EN Cprediction_bhmip::update_scene id_person_companion_="<<id_person_companion_<<"; observation.empty()"<<observation.empty() <<"; observation.size()="<<observation.size()<< std::endl;
		//std::cout << "observation.size()=" << observation.size() << "; person_or_robot="<<person_or_robot<< std::endl;
	}
	we_have_person_companion=false;


	unsigned int Uint_id_person_act=(unsigned int) id_person_companion_;

	//if(debug_filter_person_velocity_){
	//	std::cout << " PPPPPPPPP (UPDATE SCENE) filtering_method_ = " << filtering_method_ << std::endl;
	// std::cout << " Cprediction_bhmip (1) filtering_time_window_= "<<filtering_time_window_<< std::endl;
	//}
	if(debug_update_scene_){
		std::cout << "filtering_method_"<<filtering_method_ << std::endl;
	}

	//std::cout << " Cprediction_bhmip (1) = ; filtering_time_window_="<<filtering_time_window_<< std::endl;

	for(unsigned int i=0 ; i< observation.size() ; ++i)
	{

		//SpointV actual_obs( observation[i].x , observation[i].y, observation[i].time_stamp, observation[i].vx, observation[i].vy );
		//if(robot_->get_current_pointV().distance(actual_obs)<10)
		//{
			////////////////////


			if(debug_update_scene_){
				std::cout << "iteration" << i << std::endl;
				std::cout << "track.id= observation[i].id" << observation[i].id << std::endl;
				std::cout << " observation[i].time_stamp=" << observation[i].time_stamp << std::endl;
			}

			if( find_person( observation[i].id , &found_person ) )
			{
				if(debug_update_scene_){
					std::cout << "1 (if find person) updating target after observation " << found_person->is_observation_updated() << std::endl;
				}

				point = observation[i];

				if(debug_update_scene_){
					std::cout << "point_observation" << std::endl;
					observation[i].print();
				}

				/*if(!person_or_robot){

					if(debug_update_scene_){
						std::cout << "if(!person_or_robot); person_or_robot=" << person_or_robot << std::endl;
					}*/
					//std::cout << "(if) before found_person->add_pointV person_ID="<<observation[i].id <<"; filtering_time_window_="<<filtering_time_window_<< std::endl;
					found_person->add_pointV( point, filtering_method_,person_or_robot,filtering_time_window_);
					//std::cout << "(if) after found_person->add_pointV" << std::endl;
				/*}else{

					if(debug_update_scene_){
						std::cout << "else_if(!person_or_robot); person_or_robot=" << person_or_robot << std::endl;
					}

					found_person->Cperson_bhmip_add_pointV( point, filtering_method_,  person_or_robot);

					if(debug_update_scene_){
						std::cout << "(after found_person->Cperson_bhmip_add_pointV)found_person->get_prediction_trajectory()->size()=" << found_person->get_prediction_trajectory()->size() << std::endl;
					}
				}*/
				now_ = observation[i].time_stamp;

				if(debug_update_scene_){
					std::cout << "2 (if find person) updating target after observation " << found_person->is_observation_updated() << std::endl;
				}

				if(observation[i].id==id_person_companion_){
					we_have_person_companion=true;
					//std::cout << "encuentro persona companion! " << std::endl;
				}
			}
			//new targets
			else
			{
				if(debug_update_scene_){
					std::cout << "Cprediction_bhmip 1 (else add new person) updating target after observation " << std::endl;
				}

				found_person = add_person( observation[i].id );

				if(debug_update_scene_){
					std::cout << "new person :" << found_person->get_id() << std::endl;
				}

				point = observation[i];
				/*if(!person_or_robot){

					if(debug_update_scene_){
						std::cout << "if(!person_or_robot); person_or_robot=" << person_or_robot << std::endl;
					}*/
				//std::cout << "Cprediction_bhmip 2 (else add new person) updating target after observation " << std::endl;

				if(debug_update_scene_){
					point.print();
					std::cout << "filtering_method_="<<filtering_method_ << std::endl;
					std::cout << "person_or_robot="<<person_or_robot << std::endl;
					std::cout << "point.id="<<point.id<<"; point.x="<<point.x<<"; point.y="<<point.y << std::endl;
				}
				//std::cout << "(else) before found_person->add_pointV person_ID="<<observation[i].id << std::endl;
				found_person->add_pointV( point , filtering_method_,person_or_robot, filtering_time_window_);
				//std::cout << "(else) after found_person->add_pointV " << std::endl;
				/*}else{

					if(debug_update_scene_){
						std::cout << "else_if(!person_or_robot); person_or_robot=" << person_or_robot << std::endl;
					}

					found_person->Cperson_bhmip_add_pointV( point, filtering_method_,person_or_robot);
					if(debug_update_scene_){
						std::cout << "(after found_person->Cperson_bhmip_add_pointV)found_person->get_prediction_trajectory()->size()=" << found_person->get_prediction_trajectory()->size() << std::endl;
					}
				}*/
				now_ = observation[i].time_stamp;
				//std::cout << "Cprediction_bhmip 3 (else add new person) updating target after observation " << std::endl;
				if(debug_update_scene_){
					std::cout << "2 (if add new person) updating target after observation " << found_person->is_observation_updated() << std::endl;
				}

				if(observation[i].id==id_person_companion_){
					we_have_person_companion=true;
					//std::cout << "creo persona! " << std::endl;
				}
			}



			///////////////////////////////
		//}

	}

	//std::cout << "Cprediction_bhmip 1 antes remove persons " << std::endl;

	//Remove persons
	if(debug_update_scene_){
		std::cout << "Remove persons " << std::endl;
	}
	std::vector<unsigned int> removed_targets;
	for(std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
	{
		if(debug_update_scene_){
			std::cout << "analyzing target " << (*iit)->is_observation_updated() << std::endl;
		}

		if ( (*iit)->is_observation_updated() )
		{
			(*iit)->set_observation_update( false );//resets the observation flag until next update

			if(debug_update_scene_){
				std::cout << " updating target " << (*iit)->is_observation_updated() << std::endl;
			}
			// found person companion.
			if((*iit)->get_id()==Uint_id_person_act){
				we_have_person_companion=true;

				if(debug_update_scene_){
					std::cout << "(companion) entro en person to be updated! " << std::endl;
				}
			}

		}
		else
		{
			removed_targets.push_back((*iit)->get_id());

			if(debug_update_scene_){
				std::cout << "trying to remove target " << (*iit)->get_id() << std::endl;
			}
			// person companion TO BE removed.
			if((*iit)->get_id()==Uint_id_person_act){
				we_have_person_companion=false;

				if(debug_update_scene_){
					std::cout << "(companion) entro en person to be removed! " << std::endl;
				}
			}
		}



		if(debug_update_scene_){
			// comprobar que este la persona, es solo para ver cosas, no del metodo. (by ely)
			find_person( (*iit)->get_id(), &found_person );
			std::cout << "(in Remove persons)found_person->get_prediction_trajectory()->size()=" << found_person->get_prediction_trajectory()->size() << std::endl;
		}

	}

	//std::cout << "Cprediction_bhmip 1 en MEDIO remove persons " << std::endl;


	for ( unsigned int i = 0; i < removed_targets.size() ; ++i)
	{
		if( update_method_ == Cperson_abstract::Autoremove )
		{
			if(debug_update_scene_){
				std::cout << "entro en autoremove! " << std::endl;
			}
			remove_person(removed_targets[i]);
			if(removed_targets[i]==Uint_id_person_act){
				we_have_person_companion=false;

				if(debug_update_scene_){
					std::cout << "remove_person_companion! " << std::endl;
				}
			}
		}
		else
		{
			if(debug_update_scene_){
				std::cout << "NO entro en autoremove! " << std::endl;
			}
			if( find_person(removed_targets[i], &found_person ) ){
				found_person->refresh_person(now_);
				we_have_person_companion=true;

				if(debug_update_scene_){
					std::cout << "NO remove_person_companion! " << std::endl;
				}
			}
		}
	}

	// ONLY FOR TEST
	if(debug_update_scene_){
		std::cout << " (ENTRO EN Cprediction_bhmip::update_scene id_person_companion_) Print_list_or_persons " << std::endl;
	}
//	std::cout << "Cprediction_bhmip 1 antes only for test " << std::endl;

	for(std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit!=person_list_.end() ; iit++)
	{
		if(debug_update_scene_){
			(*iit)->print();
		}
		find_person( (*iit)->get_id(), &found_person );

		if(debug_update_scene_){
			std::cout << "(final list)found_person->get_prediction_trajectory()->size()=" << found_person->get_prediction_trajectory()->size() << std::endl;
		}
	}

	if(debug_update_scene_){
		std::cout << "(SALGO DE Cprediction_bhmip::update_scene id_person_companion_)=> person_list_.empty()" << person_list_.empty()<<"; id_person_companion_="<<id_person_companion_ << std::endl;
	}
	//else remove MUST be set manually outside the update function, each time the user wants to remove a person


	/*for( Cperson_abstract* iit: person_list_ )
				{
					if ( iit->get_person_type() == Cperson_abstract::Person)
					{

							if(iit->get_id()==2){
								std::cout << "FIN (update_scene) FIND CHANGE TARGET PERSON: id="<< iit->get_id()<< std::endl;
								iit->get_current_pointV().print();
							}

					}
				}*/




}

void Cprediction_bhmip::rotate_and_traslate_scene(unsigned int id, double R, double thetaZ, double linear_vx, double linear_vy, double v_rot_x, double v_rot_y, std::vector<double> vect_odom_eigen_tf, bool debug)
{ 	// Function for local tracking. This function change the frame of the Current_pose and the window of poses for a concrete person (id), tacking into account the actual position of the robot.
	//  id ->person id ; R -> Robot Translation ; thetaZ -> Robot rotation

	Cperson_abstract* found_person;
	if( find_person( id , &found_person ) )
	{
		//std::cout<<"(1) (people prediction internal person:" << std::endl;
		//found_person->print();
		found_person->rotate_and_translate_trajectory(R,thetaZ,linear_vx,linear_vy,v_rot_x,v_rot_y, vect_odom_eigen_tf,debug);
		//std::cout<<"(2) (people prediction internal person:" << std::endl;
		//found_person->print();
	}
}

Cperson_abstract* Cprediction_bhmip::add_person_container( unsigned int id,
		std::list<Cperson_abstract*>::iterator iit )
{
	Cperson_bhmip* person;
	if((id_person_companion_==id)||(id_second_person_companion_==id)){
		person = new Cperson_bhmip(id,Cperson_abstract::Person_companion, scene_force_type_, filtering_time_window_);
	}else{
		person = new Cperson_bhmip(id,Cperson_abstract::Person, scene_force_type_, filtering_time_window_);
	}


	person_list_.insert( iit , person );
	//std::cout << " Cprediction_bhmip::add_person_container antes set_destinations=> destinations_.empty()="<<destinations_.empty()<< std::endl;
	person->set_destinations(destinations_);

	return (Cperson_abstract*) person;
}


void Cprediction_bhmip::scene_intentionality_prediction_bhmip()
{
	//std::cout << " INI Cprediction_bhmip::scene_intentionality_prediction_bhmip(); min_v_to_predict_="<<min_v_to_predict_<< std::endl;
	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ;  iit++ )
	{
		//if((*iit)->get_id()){
		//std::cout << " (calculate DEST) Person ID="<<(*iit)->get_id()<< std::endl;
		//std::cout << " (calculate DEST) min_v_to_predict_="<<min_v_to_predict_<<"; v_person="<<(*iit)->get_current_pointV().v()<< std::endl;
		//}

		(*iit)->prediction( min_v_to_predict_);
	}

}


void Cprediction_bhmip::clear_scene()
{

}


void Cprediction_bhmip::print2()
{
	std::cout << "Cprediction_bhmip::print() person_list_.empty="<<person_list_.empty()<< std::endl;

	for( std::list<Cperson_abstract*>::iterator iit = person_list_.begin() ; iit != person_list_.end() ; iit++ )
	{
		(*iit)->print_dest();
		(*iit)->print();
	}
}
