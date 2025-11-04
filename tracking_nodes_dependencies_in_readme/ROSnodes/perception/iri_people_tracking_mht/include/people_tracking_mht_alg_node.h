/*
 * people_tracking_mht_alg_node.h
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

#ifndef _people_tracking_mht_alg_node_h_
#define _people_tracking_mht_alg_node_h_

#include "people_tracking_mht_alg.h"

#include <iri_base_algorithm/iri_base_algorithm.h>
#include <iostream>
#include <string>


// new libs ini:
#include <pluginlib/class_list_macros.hpp> //Importante para librerias con .so
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include "rcl_interfaces/msg/set_parameters_result.hpp" 
#include "mutex.h"
#include "eventserver.h"
using namespace std::chrono_literals;  // Para usar 100ms
// new libs fin

//#include <tf2_ros/buffer.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // Required for tf2::toMsg()
//#include <tf2/impl/utils.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// [publisher subscriber headers]
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iri_perception_msgs/msg/detection_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iri_base_algorithm/iri_base_algorithm.h>
// [service client headers]
#include <queue>



// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */ //, public std::enable_shared_from_this<PeopleTrackingMhtAlgNode> //public algorithm_base::IriBaseAlgorithm<PeopleTrackingMhtAlgorithm>
class PeopleTrackingMhtAlgNode : public rclcpp::Node 
{
  private:
    // new variables ros2:
    rclcpp::TimerBase::SharedPtr timer_;
     // [class atributes]
    CEventServer * event_server_;
    std::string new_map_event_id_;
    //new variables ros2 fin
  
    // [publisher attributes]
    //ros::Publisher tracksMarkers_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracksMarkers_publisher_;
    visualization_msgs::msg::MarkerArray MarkerArray_msg_;
    visualization_msgs::msg::Marker id_track_marker_;
    visualization_msgs::msg::Marker id_cluster_marker_;
    visualization_msgs::msg::Marker cov_track_marker_;
    visualization_msgs::msg::Marker cov_track_marker_head_;
    visualization_msgs::msg::Marker cov_track_marker_arm1_;
    visualization_msgs::msg::Marker cov_track_marker_arm2_;
    visualization_msgs::msg::Marker velocity_track_marker_;
    visualization_msgs::msg::Marker cov_detection_marker_;
    visualization_msgs::msg::Marker cov_prediction_marker_;
    visualization_msgs::msg::Marker cluster_marker_;
    visualization_msgs::msg::Marker group_marker_;

    
    //ros::Publisher tracks_publisher_;
    rclcpp::Publisher<iri_perception_msgs::msg::DetectionArray>::SharedPtr tracks_publisher_;
    iri_perception_msgs::msg::DetectionArray trackingArray_msg_;

    // [subscriber attributes]
    //ros::Subscriber odom_for_medium_velocity_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_for_medium_velocity_subscriber_;

    void odom_for_medium_velocity_callback(const nav_msgs::msg::Odometry::ConstPtr& msg);
    CMutex odom_for_medium_velocity_mutex_;
    //ros::Subscriber odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstPtr& msg);
    CMutex odom_mutex_;
    //ros::Subscriber detections_subscriber_;
    rclcpp::Subscription<iri_perception_msgs::msg::DetectionArray>::SharedPtr detections_subscriber_;

    void detections_callback(const iri_perception_msgs::msg::DetectionArray::ConstPtr& msg);
    CMutex detections_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]
    std::string fixed_frame;
    rclcpp::Node::SharedPtr node_;
	  //tf::TransformListener tf_;
    //tf2_ros::Buffer::SharedPtr tf_buffer_;
    //tf2_ros::TransformListener tf_;
    //tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_; // Use shared pointer for buffer
    //tf2_ros::TransformListener tf_listener_; // Listener initialized using the buffer

    std::vector<Sdetection> detections;
    std::vector<Strack>     tracks;
    std::vector<Strack> predictions;   
    std::vector<Exitcluster> clusters;
    std::vector<exitGroup> groups;
    Codometry odom;
    Cvelocity  velocity;
    bool cargo_ANTS;
    bool first_odometry;
    bool first_odometry_total;
    bool detectionsReceived;
    std::string frame_id;
    std::string robot_frame;
    double total_time;
    double total_time_ant;
    std::vector<std::vector<Strack>> tracks_path;
    double first_time;


   	bool tf_to_eigen_odom(const tf2::Transform& tf_odom_pose_in, Eigen::Matrix4d& act_odom_matrix_tf, double dt); //for odometry 
    void ini_markers();
    void fillTracksMsg();
    void publishMarkers();
    void fill_my_covariance_marker_track( unsigned int i );
    void fill_my_covariance_marker_track_head( unsigned int i );
    void fill_my_covariance_marker_track_arm1( unsigned int i );
    void fill_my_covariance_marker_track_arm2( unsigned int i );
    void fill_my_covariance_marker_detection( unsigned int i );
    void fill_my_covariance_marker_prediction( unsigned int i );
    void fill_my_marker_cluster( unsigned int i );
    void fill_my_marker_group( unsigned int i );
    
    

    bool id_markers, id_cluster_markers, cov_markers, vel_markers, covdet_markers, covpred_markers, clust_markers,group_markers, cov_markers_head, cov_markers_arm1,cov_markers_arm2,track_path;
    bool local_tracker;

    double max_dist_to_detect_people_;
    double robot_actual_pose_x_,robot_actual_pose_y_;


  public:

    //std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PeopleTrackingMhtAlgNode(void);
     static std::shared_ptr<PeopleTrackingMhtAlgNode> create();  // new ros2 
   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PeopleTrackingMhtAlgNode(void);

   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);
     std::shared_ptr<PeopleTrackingMhtAlgorithm> alg_;  // new ros2 
  protected:


   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    //void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#endif
