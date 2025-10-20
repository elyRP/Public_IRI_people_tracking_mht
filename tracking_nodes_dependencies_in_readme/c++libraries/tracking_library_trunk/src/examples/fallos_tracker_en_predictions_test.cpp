/*
 * local_nav_example.cpp
 *
 *  Created on: 24-marzo-2014
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
	Codometry odome;
	Cscene_sim scene;
	int type_=0;
	double prob_=0.9;
	std::vector<double> cov_(4,0.0);
	cov_[0]= 0.5; //msg->detection[i].covariances[0];
	cov_[1]= 0.0;
	cov_[2]= 0.0;
	cov_[3]= 0.5; //msg->detection[i].covariances[7];

	scene.set_dt(0.1);
	scene.set_number_virtual_people( 20 );

	if ( !scene.read_destination_map(
		"/home/erepiso/iri-lab/ros/iri-ros-pkg/iri_navigation/iri_people_simulation/map/3_destinations.txt"  ) )
	{
		std::cout << "Could not read map destinations file !!!" << std::endl;
	}
	else{
		std::cout << "read destinations map file : SUCCESS!!!" << std::endl;
	}
	scene.set_remove_targets(false);*/

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



	double now = 0.0;

	std::vector<SdetectionObservation> obs_scene;
	std::vector<Sdetection> obs_test;
	double vx, vy, wz;
	//double time_odom_act;
	vx=0;
	vy=0;
	wz=0;
	double detx, dety;
	detx=1;
	dety=1;
	while(now < 100.0)
	{


		std::cout << "entering loop sequence at time " << now << std::endl;
	/*	obs_scene.clear();
		obs_scene.push_back( SdetectionObservation(0, now ));//void observation, just for the timestamp
		obs_test.clear();
		obs_test.push_back( Sdetection(0.0,0.0, now,cov_,type_,prob_ ));//void observation, just for the timestamp
		if (now > 0.2)
			scene.set_number_virtual_people( 20 );
		if (now > 5.8)
			scene.set_number_virtual_people( 20 );

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
*/
		/*for(unsigned int i=0; i<obs_scene.size(); i++){
			obs_test[i].print();
		}*/

		//planner.update_scene( obs_scene , true );
		obs_test.clear();
		obs_test.push_back( Sdetection(
							detx , dety, now, cov_, type_ ,prob_ ) );
		if(now==0.0){
			odome.init_odometry(vx,vy,wz,now);
			std::cout << "detection_pose x="<< obs_test[0].x << std::endl;
			std::cout << "detection_pose y="<< obs_test[0].y << std::endl;
			std::cout << "robot_pose x="<< vx << std::endl;
			std::cout << "robot_pose y="<< vy << std::endl;
		}else{
			odome.update_odometry(vx,vy,wz,now);
			std::cout << "detection_pose x="<< obs_test[0].x << std::endl;
			std::cout << "detection_pose y="<< obs_test[0].y << std::endl;
			std::cout << "robot_pose x="<< vx << std::endl;
			std::cout << "robot_pose y="<< vy << std::endl;
		}
		odome.print_odometry();

		mht1.mht_odom_update(odome);
		mht1.mht_update(obs_test);
		odome.reset_odometry();
		odome.print_odometry();
		//planner.update_robot( Spose(0.0 , 0.0 , now  ) );

		//calculation of
		//planner.scene_prediction();
		//planner.set_robot_goal( Spoint(5,5) );
		//planner.robot_plan();
		now +=0.24;

		vx=0.1;
		vy=0;
		wz=0;
		detx=detx-0.1;
		dety=1;

		std::cout << "exit loop sequence at time " << now << std::endl;
	}
	std::cout<< "play end"<< std::endl;
	return 0;
}

