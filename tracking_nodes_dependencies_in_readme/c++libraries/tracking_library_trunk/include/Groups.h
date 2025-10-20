/*
 * Groups.h
 *
 *  Created on: 3/03/2015
 *      Author: erepiso
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

