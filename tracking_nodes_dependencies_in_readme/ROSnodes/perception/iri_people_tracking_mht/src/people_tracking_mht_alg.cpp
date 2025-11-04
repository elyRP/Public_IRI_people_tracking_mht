/*
 * people_tracking_mht_alg.cpp
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

#include "people_tracking_mht_alg.h"

PeopleTrackingMhtAlgorithm::PeopleTrackingMhtAlgorithm(void) //:
 // node_(rclcpp::Node::make_shared("people_tracking_mht_algorithm_node"))
{
  myPeopleTrackingMht = new Cmht();
 // node_ = rclcpp::Node::make_shared("people_tracking_mht_algorithm_node");
  //tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock()); // Initialize the tf2 buffer
  //tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_); // Initialize the listener with the buffer
}

/*PeopleTrackingMhtAlgorithm::PeopleTrackingMhtAlgorithm(void)
{
  myPeopleTrackingMht = new Cmht();
}*/

PeopleTrackingMhtAlgorithm::~PeopleTrackingMhtAlgorithm(void)
{
  delete myPeopleTrackingMht;
}

/*void PeopleTrackingMhtAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;
  myPeopleTrackingMht->set_threshold_distance(new_cfg.association_threshold);
  myPeopleTrackingMht->set_threshold_confirmation_track(new_cfg.confirmation_threshold);
  myPeopleTrackingMht->set_threshold_no_deteccion(new_cfg.deletion_threshold);
  myPeopleTrackingMht->set_laser_Pd(new_cfg.laser_probability);
  myPeopleTrackingMht->set_laser_beta_ft(new_cfg.laser_false_alarm);
  myPeopleTrackingMht->set_laser_beta_nt(new_cfg.laser_new_track);
  myPeopleTrackingMht->set_laser_beta_no_detection(new_cfg.laser_no_detection);
  myPeopleTrackingMht->set_laser_beta_track_no_confirmed(new_cfg.laser_no_confirmed);
  myPeopleTrackingMht->set_increment_iteration_track_no_confirmed(new_cfg.laser_no_confirmed_iteration);
  myPeopleTrackingMht->set_covariance_no_track(new_cfg.covariance_no_track_conf);

 myPeopleTrackingMht->set_generate_database_companion(new_cfg.generate_database_companion);
 myPeopleTrackingMht->set_fuse_tibi_and_teo_tracks(new_cfg.fuse_tibi_and_teo_tracks);
 myPeopleTrackingMht->set_debug_fuse_tibi_and_teo_tracks(new_cfg.debug_fuse_tibi_and_teo_tracks);

 myPeopleTrackingMht->set_mht_velocity_margin(new_cfg.mht_velocity_margin);

	myPeopleTrackingMht->set_augment_covariance_track(new_cfg.augment_covariance_track);
  this->unlock();
}*/

// PeopleTrackingMhtAlgorithm Public API

bool PeopleTrackingMhtAlgorithm::iteration(const std::vector<Sdetection> &detections, std::vector<Strack> &tracks, std::vector<Strack> &predictions, std::vector<Exitcluster> &clusters, Codometry &odom, Cvelocity &velocity,std::vector<exitGroup> &groups, bool &local_tracker)
{

  // TODO: cambiar esto a dynamico en ros2 humble con lo del params.yaml con todo lo del dynamic reconfigure
 /* myPeopleTrackingMht->set_threshold_distance(2.0);
  myPeopleTrackingMht->set_threshold_confirmation_track(0.9);
  myPeopleTrackingMht->set_threshold_no_deteccion(0.4);
  myPeopleTrackingMht->set_laser_Pd(0.9);
  myPeopleTrackingMht->set_laser_beta_ft(0.1);
  myPeopleTrackingMht->set_laser_beta_nt(0.11);
  myPeopleTrackingMht->set_laser_beta_no_detection(0.99);
  myPeopleTrackingMht->set_laser_beta_track_no_confirmed(0.02);
  myPeopleTrackingMht->set_increment_iteration_track_no_confirmed(0.01);
  myPeopleTrackingMht->set_covariance_no_track(5.0);

 myPeopleTrackingMht->set_generate_database_companion(false);
 myPeopleTrackingMht->set_fuse_tibi_and_teo_tracks(false);
 myPeopleTrackingMht->set_debug_fuse_tibi_and_teo_tracks(false);

 myPeopleTrackingMht->set_mht_velocity_margin(1.0);

 myPeopleTrackingMht->set_augment_covariance_track(0.15);
  //ROS_INFO(" inicio (iteration) TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
  //myPeopleTrackingMht->mht_Cscene_se_in_tracks();*/

	// ROS_INFO("PeopleTrackingMhtAlgorithm::iteration");
	myPeopleTrackingMht->set_augment_covariance_track(0.5);
	// ROS_INFO("PeopleTrackingMhtAlgorithm::iteration %d", myDetections.size());
  std::vector<Sdetection> myDetections= detections;
  

    //ROS_INFO(" (2) inicio (iteration) TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
    //myPeopleTrackingMht->mht_Cscene_se_in_tracks();
      
    if(local_tracker){  
    	//ROS_INFO("PeopleTrackingMhtAlgorithm::ODOMETRIA");
    	RCLCPP_ERROR(rclcpp::get_logger("NombrePersonalizado"), "PeopleTrackingMhtAlgorithm::ODOMETRIA");
      //RCLCPP_ERROR(this->get_logger(), "PeopleTrackingMhtAlgorithm::ODOMETRIA");
    	Codometry myOdom=odom;
    	myPeopleTrackingMht->mht_odom_update(myOdom);
     
    	Cvelocity myVelocity=velocity;
    	myPeopleTrackingMht->get_total_velocity(myVelocity);
    }



	  //ros::Time now    = ros::Time::now();
    //ROS_INFO( "time of processing mht_update= %f", now.toSec());
   /* if(!detections.empty()){
      ROS_INFO( "myDetections[0].time_stamp= %f",detections[0].time_stamp);
    }*/

   //ROS_INFO( "save_time_stamp_odometry_to_tracker_without_det_= %f",save_time_stamp_odometry_to_tracker_without_det_);
    //ROS_INFO(" ANTES (myPeopleTrackingMht->mht_update) TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
   // myPeopleTrackingMht->mht_Cscene_se_in_tracks();

   myPeopleTrackingMht->mht_update(myDetections, save_time_stamp_odometry_to_tracker_without_det_); //

   // ROS_INFO("TTTTTTTTTT myPeopleTrackingMht->mht_Cscene_se_in_tracks();");
   // myPeopleTrackingMht->mht_Cscene_se_in_tracks();
	  //ROS_INFO("PeopleTrackingMhtAlgorithm::iteration: mht_update done! Detections sent!");
    
    tracks = myPeopleTrackingMht->get_persons_mht();
     
	  //ROS_INFO("PeopleTrackingMhtAlgorithm::iteration: get_persons_mht done! %d Tracks received!", tracks.size());
    
    predictions = myPeopleTrackingMht->get_propagations_mht();
     
	  //ROS_INFO("PeopleTrackingMhtAlgorithm::iteration: get_propagations_mht done! %d propagations received!", tracks.size());
    
    //Function for clusters (cruces personas, crossing people)
    
    clusters = myPeopleTrackingMht->get_clusters_mht();
    
    groups = myPeopleTrackingMht->get_groups_mht();
    
    if(local_tracker){
    	odom.reset_odometry();
    }
  
  return true;
}
