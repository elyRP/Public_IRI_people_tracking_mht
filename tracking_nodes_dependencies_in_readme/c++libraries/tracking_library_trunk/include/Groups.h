/*
 * Groups.h
 *
 *  Created on: 3/03/2015
 *      Author: Ely Repiso
 */
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

#include "prediction_bhmip.h"

#include <eigen3/Eigen/Dense> //Library for matrix. Being here, not needed in cpp. (Al estar aquí, no hacen falta en el cpp).
#include <iostream> //Library for screen messages.
#include <math.h> //library for sen cos sqrt etc.
#include <vector>

class Cgroups
{
public:

	Cgroups(unsigned int id, std::vector<unsigned int> tracks_group, std::vector<SpointV_cov> SpointV_cov_tracks_group);
	~Cgroups();
	unsigned int get_id_group(){return id_group_;};
	std::vector<unsigned int> get_tracks_group(){return tracks_group_;};
	bool get_update_group(){return update_group_;};
	double get_group_width(){return group_width_;};
	double get_group_length(){return group_length_;};
	SpointV_cov get_central_group(){return central_group_;};
	std::vector<SpointV_cov> get_SpointVcov_tracks_group(){return SpointV_cov_tracks_group_;};
	void set_update_group(bool update_g){update_group_=update_g;};
	void set_tracks_group(std::vector<unsigned int> tracks_group_act){tracks_group_=tracks_group_act;};
	void set_SpointVcov_tracks_group(std::vector<SpointV_cov> SpointVcovtracks_group_act){SpointV_cov_tracks_group_=SpointVcovtracks_group_act;};
	void set_tracks_with_lessID_group(SpointV_cov track_with_less_id_in_group){track_with_less_id_in_group_=track_with_less_id_in_group;};
	void generation_group_track_pose();
	void print_groups();

private:
	bool cargo_ants;
	unsigned int id_group_;
	double group_width_; // Ancho del grupo
	double group_length_; // largo del grupo
	SpointV_cov central_group_;  //x e y central. tambien velocidad x e y del grupo/objeto.
	SpointV_cov track_with_less_id_in_group_;
	bool update_group_;
	std::vector<unsigned int> tracks_group_;
	std::vector<SpointV_cov> SpointV_cov_tracks_group_;

};


class exitGroup
{

	public:
	exitGroup(unsigned int id, double width,double length, SpointV_cov central_group);
	~exitGroup();
	void print_group();

	unsigned int ex_id_group_;
		double ex_group_width_; // Ancho del grupo
		double ex_group_length_; // largo del grupo
		SpointV_cov ex_central_group_;  //x e y central. tambien velocidad x e y del grupo/objeto.

	private:


};

