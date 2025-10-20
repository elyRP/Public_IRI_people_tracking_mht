/*
 * Clusters_people_tracking_mht_test.cpp
 *
 *  Created on: 11-Febrero-2015
 *      Author: erepiso
 */

//librerias nodo people_prediction.
#include "iri_geometry.h"
#include "scene_sim.h"
#include "prediction_bhmip.h" // antigua: #include "scene/scene.h"


//librerias nodo people_tracking_mht.
#include "mhtStructs.h"
#include "track.h"
#include "mht.h"

#include <stdio.h> //scanf, printf
#include <math.h> //for exponential functions and squares of the function make probabilities.(have to go into that sub function)
#include <vector> //for vectors.
#include <iostream> //for screen messages, for debug!
#include <Eigen/Dense> //for inverse y matrix. (no se si como estaba ya en mi libreria hacia falta akí o no... )


int main(int argc,char *argv[])
{

	Cmht mht1;

	//Cscene_sim scene;
	int type_=0;
	double prob_=0.9;
	std::vector<double> cov_(4,0.0);
	cov_[0]= 0.5; //msg->detection[i].covariances[0];
	cov_[1]= 0.0;
	cov_[2]= 0.0;
	cov_[3]= 0.5; //msg->detection[i].covariances[7];

	//scene.set_dt(0.1);
	//scene.set_number_virtual_people( 30 );

	/*if ( !scene.read_destination_map(
		"/home/erepiso/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/3_destinations.txt"  ) )
	{
		std::cout << "Could not read map destinations file !!!" << std::endl;
	}
	else{
		std::cout << "read destinations map file : SUCCESS!!!" << std::endl;
	}
	scene.set_remove_targets(false);
*/
	// planner
	//Cplan_local_nav planner(8.0,100);
	//Cprediction_behavior planner;
	/*if ( !planner.read_destination_map(
		"/home/gferrer/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/3_destinations.txt"  ) )
	{
		std::cout << "Could not read map destinations file !!!" << std::endl;
	}
	else{
		std::cout << "read destinations map file : SUCCESS!!!" << std::endl;
	}*/
	//planner.set_dt(0.1);



	//double now = 0.0;
	int now=0;
	//std::vector<SdetectionObservation> obs_scene;
	std::vector<Sdetection> obs_test;
	unsigned int num_person;
	num_person=1;

	// Initial points of persons
	obs_test.push_back( Sdetection(0.0 , 7.0, now, cov_, type_ ,prob_ ) );

	Codometry odome;

	while(now < 100.0)
	{
		std::cout << "entering loop sequence at time " << now << std::endl;
		/*obs_scene.clear();
		obs_scene.push_back( SdetectionObservation(0, now ));//void observation, just for the timestamp
		obs_test.clear();
		obs_test.push_back( Sdetection(0.0,0.0, now,cov_,type_,prob_ ));//void observation, just for the timestamp
		if (now > 0.2)
			scene.set_number_virtual_people( 30 );
		if (now > 5.8)
			scene.set_number_virtual_people( 30 );

		scene.update_scene( obs_scene );

		//update planner
		obs_test.clear();
		obs_scene.clear();

		const std::list<Cperson_abstract *>* person_list = scene.get_scene(  );

		for( Cperson_abstract* iit : *person_list)
		{
			obs_scene.push_back( SdetectionObservation( iit->get_id(), now,
								iit->get_current_pointV().x , iit->get_current_pointV().y) );
			 obs_test.push_back( Sdetection(
					iit->get_current_pointV().x , iit->get_current_pointV().y, now, cov_, type_ ,prob_ ) );

		}


		for(unsigned int i=0; i<obs_scene.size(); i++){
			obs_test[i].print();
		}*/
		obs_test.clear();
		for( unsigned int i=0; i<num_person;i++)
		{
			std::cout << "i= " << i << std::endl;
			std::cout << "num_person= " << num_person << std::endl;
			obs_test.push_back( Sdetection(0.0 , 7.0, now, cov_, type_ ,prob_ ) );

		}

		std::cout << "obs_test.size()= " << obs_test.size()<< std::endl;
		for(unsigned int i=0; i<obs_test.size(); i++){
			obs_test[i].print();
		}
		//planner.update_scene( obs_scene , true );

		double vx=0.0;
		double vy=0.0;
		double wz=0.5;
		double dt=1;

		odome.update_odometry(vx,vy,wz,dt);

		mht1.mht_odom_update(odome);
		mht1.mht_update(obs_test);

		//planner.update_robot( Spose(0.0 , 0.0 , now  ) );

		//calculation of
		//planner.scene_prediction();
		//planner.set_robot_goal( Spoint(5,5) );
		//planner.robot_plan();



		now +=1;//0.1;
	}
	std::cout<< "play end"<< std::endl;
	return 0;
}

