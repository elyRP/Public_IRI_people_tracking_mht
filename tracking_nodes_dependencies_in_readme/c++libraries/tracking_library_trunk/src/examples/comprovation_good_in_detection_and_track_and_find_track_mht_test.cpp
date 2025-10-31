/*
 * comprovation_good_in_detection_and_track_and_find_track_mht_test.cpp
 *
 *  Created on: 2013 
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
*      License (for other authors that will not be the original one): CC BY-NC-ND 4.0 
*      (Attribution-NonCommercial-#NoDerivatives 4.0 International)
*       https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
*      This license does not allow other authors to modify or to take profit from these works. 
*      Then, for #modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
*  collaborations (for #journals with other researchers, formal collaborations between UPC and companies, and so on).
*
* Please, to use it cite: Vaquero, Victor, et al. "Low cost, robust and real time system for detecting and tracking moving objects to automate cargo handling in port terminals." Robot 2015: Second Iberian Robotics Conference: Advances in Robotics, Volume 2. Cham: Springer International Publishing, 2015.
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


//librerias nodo Gonzalo.
#include "prediction.h"
//#include "iri_geometry.h"
#include "peoplePredictionStructs.h"
#include "scene/scene.h"

//librerias mi nodo.
#include "mhtStructs.h"
#include "track.h"
#include "mht.h"

#include <stdio.h> //scanf, printf

#include <math.h> //para exponenciales y raices de la función crear probabilidades.(tendrá que ir dentro de esa sub funcion)
#include <vector> //para vectores.
#include <iostream> //para mensages por pantalla, debugar!
#include <Eigen/Dense> //para inversas y matrices. (no se si como estaba ya en mi libreria hacia falta akí o no... )
#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

 //   Ctrack::detection_type type = Ctrack::Laser; //de momento no hace falta, porque el tipo por defecto es laser y en la función al no ponerlo me coge la muestra como laser.

    //Ctrack track1(1);
	//Cmht mht1;

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

    //coger laser detection points.
    printf ("Enter laser_detection points: \n");

        for (unsigned int i=0; i< laser_size; ++i )
        {
        	double x, y;

        	if ((i==0)||(i==1)){

        		scanf("%lf\n\n\n\n",&x);
        		scanf("%lf\n\n\n\n",&y);

        	/*}else if(i<=21){
        		scanf("%lf\n\n\n",&x);
        		scanf("%lf\n\n\n",&y);*/

        	}else if(i==(laser_size-1)){
        		scanf("%lf\n",&x);
        		scanf("%lf",&y);

        	}else if((i>=22)||(i<(laser_size-1))){
        		scanf("%lf\n\n",&x);
        		scanf("%lf\n\n",&y);
        	}

//        	std::cout<< "\n x:" << x<< std::endl;
//        	std::cout<< "y:" << y<< std::endl;
//        	std::cout<< "size = " << laser_detection_x.size()<< std::endl;

        	laser_detection_x.push_back(x);
        	laser_detection_y.push_back(y);
//        	std::cout<< "\n laser_detection_x_vector:\n" << laser_detection_x[i]<< std::endl;
//        	std::cout<< "\n laser_detection_y_vector:\n" << laser_detection_y[i]<< std::endl;

        }

        //coger laser prediction points.
        printf ("Enter laser_prediction points: ");

        for (unsigned int i=0; i< laser_size; ++i )
                {
                	double x, y;

                	if ((i==0)||(i==1)){
                		scanf("%lf\n\n\n\n",&x);
                		scanf("%lf\n\n\n\n",&y);

                	/*}else if(i<=21){
                		scanf("%lf\n\n\n",&x);
                		scanf("%lf\n\n\n",&y);*/

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
//                	std::cout<< "\n laser_prediction_x_vector:\n" << laser_prediction_x[i]<< std::endl;
 //              	std::cout<< "\n laser_prediction_y_vector:\n" << laser_prediction_y[i]<< std::endl;

                	//laser.push_back( Spoint(x,y) );

                }

//comprovar que verdaderamente va bien pq se hace dentro del tracker o en el mht lo de comprovar que haya deteccion laser
        //y su distancia sea menor que el humbral de 2metros y que el track este inicializado. (main para probar la clase track)


        for (unsigned int i=0; i< laser_size; ++i )
        {

        	//hacer los structs con las detecciones=spoint i los tracks=spose

        	//deteccion1!

        	int type=0; //type laser.
        	double probability=0.0;
        	Sdetection detection1(laser_detection_x[i],laser_detection_y[i],type,probability);

        	//track_in_1

        	double time_stamp=double(i)*0.2;
        	SdetectionObservation track_in_1( 1 , time_stamp , laser_prediction_x[i], laser_prediction_y[i],0.0, 0.0);
        	Cscene escena;

        	//track1 mio salida.

        	Strack track1; //salida!

        	//printf detecciones entrada.
        	printf("\n \n detection1.x:%f",detection1.x);
        	printf("\n \n detection1.y:%f",detection1.y);
        	printf("\n \n type:%d",detection1.type);
        	printf("\n \n probability:%f",detection1.probability);
        	//printf tracks entrada.
        	track_in_1.print();


//vectores detecciones y tracks entrada.
//vector dets
        	 std::vector<Sdetection> obs_test;
        	 obs_test.push_back(detection1);
//vector tracks
        	 std::vector<SdetectionObservation> track_test;
        	 track_test.push_back(track_in_1);
        	 escena.update_scene(track_test);

 //printf valores vectores entrada!!!
        	 //detections
        	 Sdetection obs1=obs_test[0];
        	 obs1.print_spoint_DET();
        	 std::cout<< " type:" <<  obs1.type << std::endl;
        	 std::cout<< " probability detection:" <<  obs1.probability << std::endl;
        	 //tracks

        	 escena.print();

        	 //track_vec.print();


  // comprovacion buen uso find person!!!      find_person(int id , list<Cperson>::iterator& person)
        	 printf("track 1 find!!!");
        	 //SdetectionObservation track_find;

        	 list<Cperson>::iterator person_obj;
        	 escena.find_person(1,person_obj);
        	 Spose person = person_obj->get_current_pose();
        	 person.print();
        	 person_obj->print();

        	printf("\n \n ITERATION:%u \n",i);


        	//MI CLASE!!!!

        	//Cmht mh1;
        	//mh1.mht_update(obs_test,track_test);
        	//mh1.return_results();

        }

}
