/*
 * people_tracking_mht_test.cpp
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
#include "iri_geometry.h"
//#include "prediction.h"

#include "prediction_bhmip.h" // antigua: #include "scene/scene.h"
//librerias mi nodo.
#include "mhtStructs.h"
#include "track.h"
#include "mht.h"

#include <stdio.h> //scanf, printf

#include <math.h> //for exponential functions and squares of the function make probabilities.(have to go into that sub function)
#include <vector> //for vectors.
#include <iostream> //for screen messages, for debug!
//#include <Eigen/Dense> //for inverse y matrix. (no se si como estaba ya en mi libreria hacia falta akí o no... )
//#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

 //   Ctrack::detection_type type = Ctrack::Laser; //at the moment is not necessary, because the default type is laser and the function gets first the sample as laser type.

	//test for the good behavior of the code:


 /*   unsigned int laser_size; //laser_size==num iteration=num detection one person, time.
    laser_size=20;


    std::vector<double> laser_detection_x; //mirar!!!
    std::vector<double> laser_detection_y; //mirar!!!
    laser_detection_x.reserve(laser_size);
    laser_detection_y.reserve(laser_size);

    std::vector<double> laser_detection_x2; //mirar!!!
    std::vector<double> laser_detection_y2; //mirar!!!
    laser_detection_x2.reserve(laser_size);
    laser_detection_y2.reserve(laser_size);

    //get laser detection points.
//    printf ("Enter laser_detection points: \n");

        for (unsigned int i=0; i< laser_size; ++i )
        {
        	double x, y,x2,y2;

        	//fictitious laser detections made with matlab.

        	if(i==0){
        		x=-0.1022;
        	    y=-0.2414;
        	    x2=-0.1022;
        	    y2=5.2414;
        	}else if(i==1){
        	    x=0.6351;
        	    y=1.4699;
        	    x2=0.6351;
        	    y2=6.4699;
        	}else if(i==2){
        	    x=1.2859;
        	    y=1.5865;
        	    x2=1.2859;
        	    y2=6.5865;
        	}else if(i==3){
        	    x=2;
        	    y=2.3093;
        	    x2=2;
        	    y2=7.3093;
        	}else if(i==4){
        	    x=2.7303;
        	    y=3.1714;
        	    x2=2.7303;
        	    y2=8.1714;
        	}else if(i==5){
        		x=3.2303;
                y=3.9714;
                x2=3.2303;
                y2=8.9714;
        	}else if(i==6){
        	    x=3.9403;
        	    y=4.5714;
        	    x2=3.9403;
        	    y2=9.5714;
        	}else if(i==7){

        	}else if(i==8){

        	}else if(i==9){

        	}

//        	std::cout<< "\n x:" << x<< std::endl;
//        	std::cout<< "y:" << y<< std::endl;
//        	std::cout<< "size = " << laser_detection_x.size()<< std::endl;

        	laser_detection_x.push_back(x);
        	laser_detection_y.push_back(y);
        	laser_detection_x2.push_back(x2);
        	laser_detection_y2.push_back(y2);
//        	std::cout<< "\n laser_detection_x_vector:\n" << laser_detection_x[i]<< std::endl;
//        	std::cout<< "\n laser_detection_y_vector:\n" << laser_detection_y[i]<< std::endl;

        }

        //get laser prediction points.

//comprovar que verdaderamente va bien pq se hace dentro del tracker o en el mht lo de comprovar que haya deteccion laser
        //y su distancia sea menor que el humbral de 2metros y que el track este inicializado. (main para probar la clase track)

    	std::cout<< " laser_size:" <<  laser_size << std::endl;

        int type;
        double probability;
        std::vector<Sdetection> obs_test;
//        std::vector<SdetectionObservation> track_test;
        Sdetection obs1;
        Sdetection detection1;
        Sdetection detection2;
//        Cprediction_bhmip scene_tracks;
        Cmht mh1;

       std::vector<double> cov=std::vector<double>();

        for (unsigned int iteration=0; iteration<laser_size; ++iteration )
        {

//        	std::cout<< " Iteration:" <<  iteration << std::endl;
        	//fit the struts with the detections=spoint and the tracks=spose

        	//deteccion1!
        	double time_stamp=double(iteration)*0.2; //for go more slowly for we can see if the code  made the associations well.

        	type=0; //type laser.
        	probability=0.0;
        	Sdetection detection3(laser_detection_x[iteration],laser_detection_y[iteration],time_stamp,cov,type,probability);
        	detection1=detection3;
        	Sdetection detection4(laser_detection_x2[iteration],laser_detection_y2[iteration],time_stamp,cov,type,probability);
        	detection2=detection4;
        	//track_in_1

//        	SdetectionObservation track_in_1( 1 , time_stamp , laser_prediction_x[iteration], laser_prediction_y[iteration],0.0, 0.0);


        	//my output track1.

        	//printf, input detections.
        	printf("\n \n detection1.x:%f",detection1.x);
        	printf("\n \n detection1.y:%f",detection1.y);
        	printf("\n \n type:%d",detection1.type);
        	printf("\n \n probability:%f",detection1.probability);
  */

        	//printf, input tracks.
//        	track_in_1.print();


//input detection vectors and tracks.
//vector dets
 /*       	 obs_test.clear();
        	 obs_test.push_back(detection1);
        	 obs_test.push_back(detection2);
        	 */
//vector tracks
 //       	 track_test.clear();

//        	 track_test.push_back(track_in_1);
//        	 Cprediction_bhmip scene_tracks;
//        	 scene_tracks.update_scene(track_test);

 //printf, values of input vectors!!!

/*
        	 //detections
        	 obs1=obs_test[0];
        	 obs1.print_spoint_DET();
        	 std::cout<< " type:" <<  obs1.type << std::endl;
        	 std::cout<< " probability detection:" <<  obs1.probability << std::endl;
*/

        	 //tracks

//        	 scene_tracks.print();

        	 //track_vec.print();

    /*    	printf("\n \n ITERATION:%u \n",iteration);


        	//MI CLASE!!!!
        	if(iteration<7){
        		mh1.mht_update(obs_test);
        	}else{
        		obs_test.clear();
        		mh1.mht_update(obs_test);
        		if(iteration==73){
    //    			int f;
        		}

        	}
//        	mh1.return_results_Checkout();
        }
*/

}



