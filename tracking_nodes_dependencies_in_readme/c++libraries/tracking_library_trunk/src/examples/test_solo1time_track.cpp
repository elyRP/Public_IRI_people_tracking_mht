/*
 * test_solo1time_track.cpp
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
#include "track.h"
#include "prediction.h"
//#include "iri_geometry.h"

#include <math.h> //para exponenciales y raices de la función crear probabilidades.(tendrá que ir dentro de esa sub funcion)
#include <vector> //para vectores.
#include <iostream> //para mensages por pantalla, debugar!
#include <Eigen/Dense> //para inversas y matrices. (no se si como estaba ya en mi libreria hacia falta akí o no... )
#define PI 3.14159265358979323846

int main(int argc, char *argv[])
{

  //Ctrack track;


  //test methods

	Eigen::MatrixXd	H_(2,2);
    H_(0,0)=1;
    H_(0,1)=0;
    H_(1,0)=0;
    H_(1,1)=1;
  //  Eigen::MatrixXi matrix_hipotesis_anterior_(1,1);

/*double a;
a=1.0;
double b;
b=0.0;*/
	// inicio MAIN
    //inicialization_laser_=0; // COMO HAGO ESTOOOOOOOOOOOOOOOOOOO!!!, que la primera vez sea zero y luego al cambiarla no me la vuelva a cambiar cuando haga lo del tracker...

 for(double r=1.0 ;r<10.0 ;r++)
 {
/*	double y;
	y=r*a+b;
	double y_point;
	y_point= y + (rand() % 1)/2;
	double x_point;
	x_point= r + (rand() % 1)/2;

	double y_pose;
	y_pose= y + (rand() % 1)/2;
	double x_pose;
	x_pose= r + (rand() % 1)/2;

	 std::cout<< "\n y_point=" << y_point<< std::endl;
	 std::cout<< "\n x_point=" << x_point<< std::endl;
	 std::cout<< "\n y_pose=" << y_pose<< std::endl;
	 std::cout<< "\n x_pose=" << x_pose<< std::endl;
*/

    Ctrack::detection_type type = Ctrack::Laser;
    Spoint detection_n(20.0,30.0);//Spoint::Spoint( double x_ , double y_ )
    Spose track_n(22.0 , 31.0);
    Ctrack track1(1);

    track1.set_tracks_and_detections(detection_n, track_n, type);

    if((track1.have_detection_laser_==1) && (track1.inicialization_laser_==0)) //hay detección laser!
    {
    	track1.ini_track();
    	track1.inicialization_laser_=1;
    }

    if(track1.inicialization_laser_==1)  //si el laser se ha inicializado!!!
    {

    	track1.update_track();

    }

 }
	//fin MAIN






}
