 /*
 * track_test.cpp
 *
 *  Created aprox on: 07/07/2013 (aprox)
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
#include "track.h"
#include "prediction.h"
//#include "iri_geometry.h"
#include "peoplePredictionStructs.h"
#include <stdio.h> //scanf, printf

#include <math.h> //para exponenciales y raices de la función crear probabilidades.(tendrá que ir dentro de esa sub funcion)
#include <vector> //para vectores.
#include <iostream> //para mensages por pantalla, debugar!
#include <Eigen/Dense> //para inversas y matrices. (no se si como estaba ya en mi libreria hacia falta akí o no... )
#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

  //Ctrack track;


  //test methods


 //   Ctrack::detection_type type = Ctrack::Laser; //de momento no hace falta, porque el tipo por defecto es laser y en la función al no ponerlo me coge la muestra como laser.

    Ctrack track1(1);


    unsigned int laser_size=0; //laser_size==num iteration=num detection one person, time.

    printf ("Enter laser size: \n");
    scanf("%u",&laser_size);

    std::vector<double> laser_detection_x; //mirar!!!
    std::vector<double> laser_detection_y; //mirar!!!
    laser_detection_x.reserve(laser_size);
    laser_detection_y.reserve(laser_size);
    std::vector<double> laser_prediction_x; //mirar!!!
    std::vector<double> laser_prediction_y; //mirar!!!
    laser_prediction_x.reserve(laser_size);
    laser_prediction_y.reserve(laser_size);
   // Eigen::MatrixXd laser_prediction(2,laser_size);

    //coger laser detection points.
    printf ("Enter laser_detection points: \n");



        for (unsigned int i=0; i< laser_size; ++i )
        {
        	double x, y;

        	if ((i==0)||(i==1)){
        		scanf("%lf\n\n\n\n",&x);
        		scanf("%lf\n\n\n\n",&y);

        	}else if(i<=21){
        		scanf("%lf\n\n\n",&x);
        		scanf("%lf\n\n\n",&y);

        	}else if(i==(laser_size-1)){
        		scanf("%lf\n",&x);
        		scanf("%lf",&y);

        	}else if((i>=22)||(i<(laser_size-1))){
        		scanf("%lf\n\n",&x);
        		scanf("%lf\n\n",&y);

        	}

  //      	std::cout<< "\n x:" << x<< std::endl;
  //      	std::cout<< "y:" << y<< std::endl;
 //       	std::cout<< "size = " << laser_detection_x.size()<< std::endl;

        	laser_detection_x.push_back(x);
        	laser_detection_y.push_back(y);
//        	std::cout<< "\n laser_detection_x_vector:\n" << laser_detection_x[i]<< std::endl;
 //       	std::cout<< "\n laser_detection_y_vector:\n" << laser_detection_y[i]<< std::endl;



        }

        //coger laser prediction points.
        printf ("Enter laser_prediction points: ");

        for (unsigned int i=0; i< laser_size; ++i )
                {
                	double x, y;

                	if ((i==0)||(i==1)){
                		scanf("%lf\n\n\n\n",&x);
                		scanf("%lf\n\n\n\n",&y);

                	}else if(i<=21){
                		scanf("%lf\n\n\n",&x);
                		scanf("%lf\n\n\n",&y);

                	}else if(i==(laser_size-1)){
                		scanf("%lf\n",&x);
                		scanf("%lf",&y);

                	}else if((i>=22)||(i<(laser_size-1))){
                		scanf("%lf\n\n",&x);
                		scanf("%lf\n\n",&y);

                	}

 //               	std::cout<< "\n x:" << x<< std::endl;
 //               	std::cout<< "y:" << y<< std::endl;

                	laser_prediction_x.push_back(x);
                	laser_prediction_y.push_back(y);
 //               	std::cout<< "\n laser_prediction_x_vector:\n" << laser_prediction_x[i]<< std::endl;
 //               	std::cout<< "\n laser_prediction_y_vector:\n" << laser_prediction_y[i]<< std::endl;

                	//laser.push_back( Spoint(x,y) );

                }




        ///////////////


for (unsigned int i=0; i< laser_size; ++i )
{
	Spoint detection_n(laser_detection_x[i], laser_detection_y[i]  );
	Spoint track_n( laser_prediction_x[i],laser_prediction_y[i] );
	Spoint kalmanstate_modifi;
	printf("\n \n ITERATION:%u \n",i);


	track1.set_tracks_and_detections( detection_n, track_n);//type laser by default
	//printf("\n \n detection_n.x:%f",detection_n.x);
	//printf("\n \n detection_n.y:%f",detection_n.y);

	//printf("\n \n track_n.x:%f",track_n.x);
	//printf("\n \n track_n.y:%f",track_n.y);




    if ( track1.is_initialized() )  //si el laser se ha inicializado!!!
	{


    	//cálculo distancias.
        track1.calculation_of_distances_detectors();


        if( (track1.have_detection_laser()==false) || (track1.get_laser_distance()>track1.get_threshold_distance()) )
	 	{

	    //caso no tengo detección de ninguno de los tres!!! (ahora mismo 2, pq solo será vision y laser!!!
        	track1.update_track(track_n);
        	printf("\n \n NEW hipotesis_actual:%d",track1.get_hipotesis_iter() );
        	printf("\n \n NEW probability_of_detection_(max_probabilidad):%f",track1.get_probability() );
        	kalmanstate_modifi=track1.get_kalman_update();
        	printf("\n \n kalmanstate_modifi.x:%f",kalmanstate_modifi.x);
        	printf("\n \n kalmanstate_modifi.y:%f",kalmanstate_modifi.y);
        	printf( " \n \n \n" );

	 	}
	 	else if( (track1.have_detection_laser()==true) && (track1.get_laser_distance()<=track1.get_threshold_distance()) )
	 	{

	 		track1.update_track(detection_n, track_n);  //TODO: cambiar el detection_n y track_n, pq serán solo los del cluster, a d<=!!!
	 		printf("\n \n NEW hipotesis_actual:%d",track1.get_hipotesis_iter() );
	 		printf("\n \n NEW probability_of_detection_(max_probabilidad):%f",track1.get_probability() );
	 		kalmanstate_modifi=track1.get_kalman_update();
	 		printf("\n \n kalmanstate_modifi.x:%f",kalmanstate_modifi.x);
	 		printf("\n \n kalmanstate_modifi.y:%f",kalmanstate_modifi.y);
	 		printf( " \n \n \n" );

	 	}
	 }



    if( track1.have_detection_laser() &&  !track1.is_initialized() ) //hay detección laser!
    {

    	track1.ini_track();
    	track1.set_initialized();

    	printf("\n \n NEW hipotesis_actual:%d",track1.get_hipotesis_iter());
    	printf("\n \n NEW probability_of_detection_(max_probabilidad):%f",track1.get_probability() );

    }





 }
	//fin MAIN






}
