/*
 * Clusters_people_tracking_mht_test.cpp
 *
 *  Created on: 11-Febrero-2015
 *      Author: Ely Repiso
 */
// Copyright (C) from 2013-until now Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author Ely Repiso
// All rights reserved.
/*
*
*      Created on: 2013 by Ely Repiso and published first as her TFC on 13/12/2013. Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2- humble migration) 
*      Author: Ely Repiso (from 2013 and currently).
*      Furthermore, this code is a modification and extension extracted from the theory of the open source papers of Donald Reid IEEE Transaction on Automatic Control 1979 and Kai Oliver Arras ICRA2008. We never had their code, therefore, we implemented this code from scratch, only taking inspiration from their articles.
*     
*      License (for other authors that will not be the original one): CC BY-NC-ND 4.0 
*      (Attribution-NonCommercial-#NoDerivatives 4.0 International)
*       https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
*
*      This license does not allow other authors to modify or to take profit from these works. 
*      Then, for #modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
*  collaborations (for #journals with other researchers, formal collaborations between UPC and companies, and so on).
*
* Please, to use it cite: Vaquero, Victor, et al. "Low cost, robust and real time system for detecting and tracking moving objects to automate cargo handling in port terminals." Robot 2015: Second Iberian Robotics Conference: Advances in Robotics, Volume 2. Cham: Springer International Publishing, 2015.
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
***********************************************************
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
	obs_test.push_back( Sdetection(1 , 1, now, cov_, type_ ,prob_ ) );
	obs_test.push_back( Sdetection(1.5 , 1.5, now, cov_, type_ ,prob_ ) );

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
			obs_test.push_back( Sdetection(1.5 , 1.5, now, cov_, type_ ,prob_ ) );

			if(now%2==0){  // Caso Par
				obs_test.push_back( Sdetection(1 , 1, now, cov_, type_ ,prob_ ) );
			}else{ //caso impar
				obs_test.push_back( Sdetection(2 , 2, now, cov_, type_ ,prob_ ) );
			}
		}

		std::cout << "obs_test.size()= " << obs_test.size()<< std::endl;
		for(unsigned int i=0; i<obs_test.size(); i++){
			obs_test[i].print();
		}
		//planner.update_scene( obs_scene , true );

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

