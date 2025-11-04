/*
 * people_tracking_mht_alg_node.cpp
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


#include "people_tracking_mht_alg_node.h"
//algorithm_base::IriBaseAlgorithm<PeopleTrackingMhtAlgorithm>(),
 //node_(std::make_shared<rclcpp::Node>("people_tracking_mht_node")),
 
PeopleTrackingMhtAlgNode::PeopleTrackingMhtAlgNode(void) :
  Node("people_tracking_mht_node"),
  tf_buffer_(this->get_clock())  // Initialize tf_buffer_ with the node's clock
  //tf_listener_(tf_buffer_)        // Initialize tf_listener_ with the tf_buffer_
{
RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 1 ");
  //tf_listener_(tf_buffer_, shared_from_this()),
  //max_dist_to_detect_people_(10.0),
  //robot_actual_pose_x_(0.0),
  //robot_actual_pose_y_(0.0)
//tf_buffer.setUsingDedicatedThread(true); 

  // TODO: cambiar esto a dynamico en ros2 humble con lo del params.yaml con todo lo del dynamic reconfigure
  this->frame_id = "map";
  this->local_tracker   = false;
  this->id_markers      = true;
  this->vel_markers     = true;
  this->cov_markers     = true;
  this->cov_markers_head= false;
  this->cov_markers_arm1= false;
  this->cov_markers_arm2= false;
  this->covdet_markers  = false;
  this->covpred_markers = false;
 // this->id_cluster_markers      = config.id_cluster_markers;
  this->clust_markers = false;  
  //this->group_markers = config.group_markers;
  this->ini_markers(); 
  this->track_path = false;
  	
  max_dist_to_detect_people_ = 10.0;
  robot_actual_pose_x_=0.0;
  robot_actual_pose_y_=0.0;
  //node_ = std::make_shared<rclcpp::Node>("people_tracking_mht_node");
  
	//ROS_INFO("PeopleTrackingMhtAlgNode:: create");
  RCLCPP_ERROR(get_logger(), "PeopleTrackingMhtAlgNode:: create");
  
  // new ros2 ini
  
  alg_ = std::make_shared<PeopleTrackingMhtAlgorithm>();
 // auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  //auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  using std::placeholders::_1;
   auto callback = std::bind(&PeopleTrackingMhtAlgNode::mainNodeThread, this);
   timer_ = this->create_wall_timer(20ms, callback);
   // new ros2 fin
 RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 3 "); 
  //init class attributes if necessary
 // this->setRate(24);//in [Hz]
 // this->frame_id = "ibeo1";
  //this->frame_id = "/map";
 this->frame_id = "map";
 
  this->detectionsReceived=false;
RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 4... ");
  // [init publishers]
  //this->tracksMarkers_publisher_ = this->public_node_handle_.advertise<visualization_msgs::msg::MarkerArray>("tracksMarkers", 1);
  this->tracksMarkers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracksMarkers", 1);
  
  RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 5 ");
  //this->tracks_publisher_ = this->public_node_handle_.advertise<iri_perception_msgs::msg::DetectionArray>("tracks", 1);
  this->tracks_publisher_ = this->create_publisher<iri_perception_msgs::msg::DetectionArray>("/tracks", 1);
  
  RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 6 ");
  
  // [init subscribers]
  //this->odom_for_medium_velocity_subscriber_ = this->public_node_handle_.subscribe("odom_for_medium_velocity", 1, &PeopleTrackingMhtAlgNode::odom_for_medium_velocity_callback, this);
  this->odom_for_medium_velocity_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom_for_medium_velocity", 
    1, 
    std::bind(&PeopleTrackingMhtAlgNode::odom_for_medium_velocity_callback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 7 ");
  //this->odom_subscriber_ = this->public_node_handle_.subscribe("odom", 1, &PeopleTrackingMhtAlgNode::odom_callback, this);
  this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 
    1, 
    std::bind(&PeopleTrackingMhtAlgNode::odom_callback, this, std::placeholders::_1)
  );
  
  RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo 8 ");
  //this->detections_subscriber_ = this->public_node_handle_.subscribe("detections", 1, &PeopleTrackingMhtAlgNode::detections_callback, this);
  this->detections_subscriber_ = this->create_subscription<iri_perception_msgs::msg::DetectionArray>(
    "/detections", 
    1, 
    std::bind(&PeopleTrackingMhtAlgNode::detections_callback, this, std::placeholders::_1)
  );

  
   RCLCPP_INFO(rclcpp::Node::get_logger(), "laser-people-detection: Inicio creador nodo last ");
  // [init services]
  


  // [init clients]
  
  // [init action servers]
  
  // [init action clients]
}

PeopleTrackingMhtAlgNode::~PeopleTrackingMhtAlgNode(void)
{
  // [free dynamic memory]
}

void PeopleTrackingMhtAlgNode::mainNodeThread(void)
{

//RCLCPP_INFO(get_logger(), "mainNodeThread 1");

	tracks_path.resize(100000);
	
  static bool first = true;
  cargo_ANTS=false;
 // ros::Time now    = ros::Time::now();
  this->detections_mutex_.enter();
  this->odom_mutex_.enter(); 
  if(this->detectionsReceived) // to iterate only when detection are received.
  {
    if(first)
    {
      //initialization of all the markers
      this->ini_markers();
      first_odometry=true;
      first_odometry_total=true;
      first=false;      
    }
    alg_->iteration(this->detections, this->tracks, this->predictions, this->clusters, this->odom, this->velocity, this->groups,this->local_tracker);
	//ROS_INFO( " afther alg_->iteration ");
    this->fillTracksMsg();
//ROS_INFO( " before fill tracks alg_->iteration ");
    this->tracks_publisher_->publish(this->trackingArray_msg_);
    this->publishMarkers();
    this->detections.clear(); 
    this->detectionsReceived=false;
  }
  this->odom_mutex_.exit(); 
  this->detections_mutex_.exit();
  // [fill msg structures]

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]

  //ROS_INFO( "time of processing mht_update= %f", (ros::Time::now()-now).toSec());
//RCLCPP_INFO(get_logger(), "mainNodeThread last");
}

/*  [subscriber callbacks] */
void PeopleTrackingMhtAlgNode::odom_for_medium_velocity_callback(const nav_msgs::msg::Odometry::ConstPtr& msg) 
{ 

RCLCPP_INFO(get_logger(), "odom_for_medium_velocity_callback 1");
  //ROS_INFO("PeopleTrackingMhtAlgNode::odom_for_medium_velocity_callback: New Message Received"); 

  //use appropiate mutex to shared variables if necessary 
  alg_->lock(); 
  this->odom_for_medium_velocity_mutex_.enter(); 
  //std::cout << msg->data << std::endl;
  
  double vx_tot; 
  double vy_tot; 
  double wz_tot; 
  double px_tot; 
  double py_tot; 
  double oz_tot; 
  
  if(first_odometry_total==true){
  	total_time=0;
  	//total_time_ant=msg->header.stamp.toSec();
    total_time_ant = rclcpp::Time(msg->header.stamp).seconds();
  	first_odometry_total=false;
  	
  	double vx_twist,vy_twist,wz_twist;
  	vx_twist = msg->twist.twist.linear.x; 
  	vy_twist = msg->twist.twist.linear.y; // COMO TENDRÍA QUE SER.
  	wz_twist=msg->twist.twist.angular.z;
  	//ROS_INFO("vx_twist= %f",vx_twist); 
  	//ROS_INFO("vy_twist= %f",vy_twist); 
  	//ROS_INFO("wz_twist= %f",wz_twist); 
  	
  	//total_time=(msg->header.stamp.toSec()-total_time_ant);
    total_time = (rclcpp::Time(msg->header.stamp).seconds() - total_time_ant);
  	px_tot = msg->pose.pose.position.x;  //POR CONSTRUCCION DEL BAG CHUNGO!
  	py_tot = msg->pose.pose.position.y;
  	oz_tot = msg->pose.pose.orientation.z;  	
  }else{
    
  px_tot = msg->pose.pose.position.x;  //POR CONSTRUCCION DEL BAG CHUNGO!
  py_tot = msg->pose.pose.position.y;
  oz_tot = msg->pose.pose.orientation.z;
  
  //total_time=total_time+(msg->header.stamp.toSec()-total_time_ant);
  //total_time=(msg->header.stamp.toSec()-total_time_ant);
  total_time = (rclcpp::Time(msg->header.stamp).seconds() - total_time_ant);
  
  velocity.update_velocity(px_tot,py_tot,oz_tot,total_time);
  
  double vx_twist,vy_twist,wz_twist;
  vx_twist = msg->twist.twist.linear.x; 
  vy_twist = msg->twist.twist.linear.y; // COMO TENDRÍA QUE SER.
  wz_twist=msg->twist.twist.angular.z;
  //ROS_INFO("vx_twist= %f",vx_twist); 
  //ROS_INFO("vy_twist= %f",vy_twist); 
  //ROS_INFO("wz_twist= %f",wz_twist);  
  
  }
  
  //unlock previously blocked shared variables 
  alg_->unlock(); 
  this->odom_for_medium_velocity_mutex_.exit(); 
}

bool PeopleTrackingMhtAlgNode::tf_to_eigen_odom(const tf2::Transform& tf_odom_pose_in, Eigen::Matrix4d& act_odom_matrix_tf,double dt) 
{ 

	double roll, pitch, yaw;
	tf_odom_pose_in.getBasis().getRPY(roll, pitch, yaw);
	/*std::cout << "(rad/s) roll/dt=" << roll/dt << std::endl;
	std::cout << "(rad/s) pitch/dt=" << pitch/dt << std::endl;
	std::cout << "(rad/s) yaw/dt=" << yaw/dt << std::endl;
	std::cout << "(grad/s) roll/dt=" << (roll*180/3.14)/dt << std::endl;
	std::cout << "(grad/s) pitch/dt=" << (pitch*180/3.14)/dt << std::endl;
	std::cout << "(grad/s) yaw/dt=" << (yaw*180/3.14)/dt << std::endl;*/
	Eigen::Matrix3d rot;
	rot=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
	//std::cout << "rot=" << rot << std::endl;

	bool T_ok=false;

	act_odom_matrix_tf=Eigen::Matrix4d::Zero();
	if(!std::isnan(rot(0,0)))
	{
		act_odom_matrix_tf.block(0,0,3,3)=rot;
		act_odom_matrix_tf(0,3)=tf_odom_pose_in.getOrigin().x();
		act_odom_matrix_tf(1,3)=tf_odom_pose_in.getOrigin().y();
		act_odom_matrix_tf(2,3)=tf_odom_pose_in.getOrigin().z();
		T_ok=true;	
	}else
	{
		act_odom_matrix_tf.block(0,0,3,3) = Eigen::Matrix3d::Identity();
	}
	act_odom_matrix_tf(3,3)=1.0;
	//std::cout << "act_odom_matrix_tf=" << act_odom_matrix_tf << std::endl;
	
	return T_ok;
}

void PeopleTrackingMhtAlgNode::odom_callback(const nav_msgs::msg::Odometry::ConstPtr& msg) 
{ 

RCLCPP_INFO(get_logger(), "odom_callback 1");
  //ROS_INFO("PeopleTrackingMhtAlgNode::odom_callback: New Message Received time_stamp=%d",msg->header.stamp.toSec()); 

  //use appropiate mutex to shared variables if necessary 
  alg_->lock(); 
  this->odom_mutex_.enter(); 
  
	geometry_msgs::msg::Pose odompose;
	tf2::Transform tf_odom_pose;
	Eigen::Matrix4d odom_eigen_tf;
	Eigen::Matrix3d odom_eigen_tf_3by3;

  // Convert the stamp to seconds using rclcpp::Time
  rclcpp::Time time_stamp = msg->header.stamp;
		
  //std::cout << msg->data << std::endl; 
  if(first_odometry==true){
  
    double dx, dy, dtheta;
  	double before_x, before_y, before_theta;
  	double time_odom_act;
  	dx=0; 
  	dy=0; 
  	dtheta=0;
  	time_odom_act = time_stamp.seconds();
    alg_->save_time_stamp_odometry_to_tracker_without_det_=time_odom_act;

//ROS_INFO( "(in odometry) time_odom_act=%f; save_time_stamp_odometry_to_tracker_without_det_= %f",time_odom_act,alg_->save_time_stamp_odometry_to_tracker_without_det_);

  	first_time = time_stamp.seconds();
  	//double roll=msg->twist.twist.angular.x;
    //double pitch=msg->twist.twist.angular.y;
    //double yaw=msg->pose.pose.orientation.z;
    //geometry_msgs::Quaternion my_orient_Q=tf::createQuaternionMsgFromYaw(yaw);
    if(!cargo_ANTS){    
    	//geometry_msgs::msg::Quaternion my_orient_Q=tf::createQuaternionMsgFromRollPitchYaw(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    	
      // Extract orientation from message (assuming msg->pose.pose.orientation is a geometry_msgs::msg::Quaternion)
      geometry_msgs::msg::Quaternion my_orient_Q = msg->pose.pose.orientation;
      // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
      tf2::Quaternion tf2_quat;
      tf2::fromMsg(my_orient_Q, tf2_quat);
      // Now use tf2::getYaw() with the tf2_quat
      before_theta = tf2::impl::getYaw(tf2_quat);

  		//before_theta=msg->twist.twist.angular.z;
      //tf::getYaw(my_orient_Q);	
  	}else{
  		before_theta=msg->pose.pose.orientation.z;
  	}

  	before_x = msg->pose.pose.position.x; 
  	before_y= msg->pose.pose.position.y;
  	

	// To filter detections to have less tracks in campus nord.
	robot_actual_pose_x_=msg->pose.pose.position.x;
	robot_actual_pose_y_=msg->pose.pose.position.y;
  	//ROS_INFO("dx= %f",dx); 
		//ROS_INFO("dy= %f",dy); 
		//ROS_INFO("dtheta= %f",dtheta); 
		//ROS_INFO("before_x= %f",before_x); 
		//ROS_INFO("before_y= %f",before_y); 
		//ROS_INFO("before_theta= %f",before_theta); 
		//ROS_INFO("time_odom_act= %f",time_odom_act); 
  
    odompose.position      = msg->pose.pose.position;
    odompose.orientation =msg->pose.pose.orientation;
   	/* std::cout <<" odompose="<<  odompose << std::endl;
    std::cout <<" odompose.position.x="<<  odompose.position.x << std::endl;
    std::cout <<" odompose.position.y="<<  odompose.position.y << std::endl;
    std::cout <<" odompose.position.z="<<  odompose.position.z << std::endl;
    std::cout <<" odompose.orientation.x="<<  odompose.orientation.x << std::endl;
    std::cout <<" odompose.orientation.y="<<  odompose.orientation.y << std::endl;
    std::cout <<" odompose.orientation.z="<<  odompose.orientation.z << std::endl;*/
    tf2::fromMsg(odompose,tf_odom_pose);
    //std::cout <<" tf_odom_pose="<<  tf_odom_pose << std::endl;
    bool T_ok=tf_to_eigen_odom(tf_odom_pose,odom_eigen_tf,1.0);
    if(!T_ok)
    {
     //ROS_INFO("Wrong tf_odom_pose to odom_eigen_transform");
     RCLCPP_ERROR(get_logger(), "Wrong tf_odom_pose to odom_eigen_transform");
    }

    /* std::cout <<" odom_eigen_tf="<<  odom_eigen_tf << std::endl;
    std::cout <<" ["<<odom_eigen_tf(0,0)<<","<<odom_eigen_tf(0,1)<<","<<odom_eigen_tf(0,2)<<","<<odom_eigen_tf(0,3)<<"]="<< std::endl;
    std::cout <<" ["<<odom_eigen_tf(1,0)<<","<<odom_eigen_tf(1,1)<<","<<odom_eigen_tf(1,2)<<","<<odom_eigen_tf(1,3)<<"]="<< std::endl;
    std::cout <<" ["<<odom_eigen_tf(2,0)<<","<<odom_eigen_tf(2,1)<<","<<odom_eigen_tf(2,2)<<","<<odom_eigen_tf(2,3)<<"]="<< std::endl;
    std::cout <<" ["<<odom_eigen_tf(3,0)<<","<<odom_eigen_tf(3,1)<<","<<odom_eigen_tf(3,2)<<","<<odom_eigen_tf(3,3)<<"]="<< std::endl;*/

		odom_eigen_tf.block(2,0,1,4) = odom_eigen_tf.block(3,0,1,4);
		odom_eigen_tf.block(0,2,4,1) = odom_eigen_tf.block(0,3,4,1);
		odom_eigen_tf_3by3=odom_eigen_tf.block(0,0,3,3);

		/*std::cout <<" odom_eigen_tf_3by3="<< std::endl;
  	std::cout <<" ["<<odom_eigen_tf_3by3(0,0)<<","<<odom_eigen_tf_3by3(0,1)<<","<<odom_eigen_tf_3by3(0,2)<<"]="<< std::endl;
  	std::cout <<" ["<<odom_eigen_tf_3by3(1,0)<<","<<odom_eigen_tf_3by3(1,1)<<","<<odom_eigen_tf_3by3(1,2)<<"]="<< std::endl;
  	std::cout <<" ["<<odom_eigen_tf_3by3(2,0)<<","<<odom_eigen_tf_3by3(2,1)<<","<<odom_eigen_tf_3by3(2,2)<<"]="<< std::endl;*/
  	//odom.set_act_odomTFmatrix(odom_eigen_tf_3by3);
  	 
  	//ROS_INFO("dx= %f",dx); 
		//ROS_INFO("dy= %f",dy); 
		//ROS_INFO("dtheta= %f",dtheta); 
		//ROS_INFO("before_x= %f",before_x); 
		//ROS_INFO("before_y= %f",before_y); 
		//ROS_INFO("before_theta= %f",before_theta); 
		//ROS_INFO("time_odom_act= %f",time_odom_act); 
  	first_odometry=false;
  	odom.set_ant_odomTFmatrix(odom_eigen_tf_3by3);
  	odom.init_odometryGood(before_x,before_y,before_theta,time_odom_act);
  	
  }
  else{
  
    double dx, dy, dtheta;
  	double before_x, before_y, before_theta;
  	double act_x, act_y, act_theta, act_theta1;
  	double wz;
  	double time_odom_ant, time_odom_act, dt;
  	double linear_vx, linear_vy;
  	time_odom_ant = odom.get_timeStamp();
  	//time_odom_act=msg->header.stamp.toSec();
    time_odom_act = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    alg_->save_time_stamp_odometry_to_tracker_without_det_=time_odom_act;

//ROS_INFO( "(in odometry) time_odom_act=%f; save_time_stamp_odometry_to_tracker_without_det_= %f",time_odom_act,this->alg_.save_time_stamp_odometry_to_tracker_without_det_);

  	odom.set_timeStamp(time_odom_act);
  	dt = time_odom_act - time_odom_ant;
    
  	act_x= msg->pose.pose.position.x;  //POR CONSTRUCCION DEL BAG CHUNGO!
  	act_y= msg->pose.pose.position.y;
  	//double yaw=msg->pose.pose.orientation.z;
    //geometry_msgs::Quaternion my_orient_Q=tf::createQuaternionMsgFromYaw(yaw);


////////////////// ESTO FALLA. uso robot_actual_pose_x_ y robot_actual_pose_y_ luego en el main del nodo, para filtrar las detecciones que me entran por distancia, usando esta pose del robot.
 /*  this->fixed_frame = "/map";
  this->robot_frame  = "/tibi/odom"; // base_footprint (NO) o odom (ok, radio a 30. pasa aprox como sin usar tf) o base_link (delay)

	robot_actual_pose_x_=msg->pose.pose.position.x;
	robot_actual_pose_y_=msg->pose.pose.position.y;
 //Get Robot position: transform empty/zero pose from base_link to map
  std::string target_frame = this->fixed_frame;
  std::string source_frame = this->robot_frame; 
  ros::Time target_time    = ros::Time::now();

  double time_start=ros::Time::now().toSec();
  try
  {
    bool tf_exists = this->tf_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(1), ros::Duration(0.01));
    if(tf_exists)
    {

      geometry_msgs::PoseStamped poseIn;
      geometry_msgs::PoseStamped poseOut;
      poseIn.header.stamp    = target_time;
      poseIn.header.frame_id = source_frame;
      poseIn.pose.orientation.z = 1.0; //valid quaternion
      this->tf_.transformPose(target_frame, poseIn, poseOut);
      robot_actual_pose_x_ = poseOut.pose.position.x;
      robot_actual_pose_y_ = poseOut.pose.position.y;
      //actual_odom_robot_pose.time_stamp = poseOut.header.stamp.toSec();

      //vector of the orientation
      poseIn.pose.position.x = 1.0;
      this->tf_.transformPose(target_frame, poseIn, poseOut);
      //actual_odom_robot_pose.theta = atan2(poseOut.pose.position.y - this->robot_pose_.y , poseOut.pose.position.x - this->robot_pose_.x);
      //this->planner_.update_robot(this->robot_pose_);//updated in main
      //ROS_DEBUG("IriStateMachineCompanionAlgNode::odom_callback: robot pose x,y,th,v,w: %f, %f, %f, %f, %f", this->robot_pose_.x, this->robot_pose_.y, this->robot_pose_.theta, this->robot_pose_.v, this->robot_pose_.w);

    }
    else
    {
      ROS_WARN("IriStateMachineCompanionAlgNode::odom_callback: No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("IriStateMachineCompanionAlgNode::odom_callback: %s",ex.what());
  }

*/
/////////////////


    if(!cargo_ANTS){
    
    	/*tf2::Quaternion my_orient;
    	my_orient = tf2::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    	act_theta=tf2::impl::getYaw(my_orient);
    	//ROS_INFO("act_theta1= %f",act_theta1); 
    	//geometry_msgs::msg::Quaternion my_orient_Q=tf2::createQuaternionMsgFromRollPitchYaw(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    	tf2::Quaternion quat;
      quat.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      geometry_msgs::msg::Quaternion my_orient_Q = tf2::toMsg(quat);
  		//act_theta=msg->twist.twist.angular.z;//tf::getYaw(my_orient_Q);
  		act_theta1=tf2::impl::getYaw(my_orient_Q); 
  		//ROS_INFO("act_theta= %f",act_theta);*/
      tf2::Quaternion my_orient;
      my_orient.setValue(msg->pose.pose.orientation.x, 
                        msg->pose.pose.orientation.y, 
                        msg->pose.pose.orientation.z, 
                        msg->pose.pose.orientation.w);

      // Get the yaw from the tf2::Quaternion
      act_theta = tf2::impl::getYaw(my_orient);

      // Convert to geometry_msgs::msg::Quaternion for other purposes
      geometry_msgs::msg::Quaternion my_orient_Q = tf2::toMsg(my_orient);

      // If you need to get the yaw from the original quaternion, do not convert it
      //act_theta1 = tf2::impl::getYaw(my_orient_Q);
      act_theta1 = act_theta;
  				
  	}else{
  		act_theta=msg->pose.pose.orientation.z;
  	}
  	wz=msg->twist.twist.angular.z;//msg->pose.pose.orientation.w;
  	
  	before_x=odom.get_before_x();
  	before_y=odom.get_before_y(); 
  	before_theta=odom.get_before_theta();
  	
  	dx=act_x-before_x; 
  	dy=act_y-before_y; 
  	dtheta=act_theta-before_theta;
  	//dtheta=act_theta;
  	
		if(cargo_ANTS){
			linear_vx=msg->pose.pose.position.x;
  		linear_vy=msg->pose.pose.position.y;
		}
		else{
  		linear_vx=msg->twist.twist.linear.x;
  		linear_vy=msg->twist.twist.linear.y;
  	}	
  	//ROS_INFO("dx= %f",dx); 
		//ROS_INFO("dy= %f",dy); 
		//ROS_INFO("dtheta= %f",dtheta); 
		//ROS_INFO("before_x= %f",before_x); 
		//ROS_INFO("before_y= %f",before_y); 
		//ROS_INFO("before_theta= %f",before_theta); 
		//ROS_INFO("act_x= %f",act_x); 
		//ROS_INFO("act_y= %f",act_y); 
		//ROS_INFO("act_theta= %f",act_theta); 
	
    odompose.position      = msg->pose.pose.position;
    odompose.orientation =msg->pose.pose.orientation;

    /*std::cout <<" odompose="<<  odompose << std::endl;
    std::cout <<" odompose.position.x="<<  odompose.position.x << std::endl;
    std::cout <<" odompose.position.y="<<  odompose.position.y << std::endl;
    std::cout <<" odompose.position.z="<<  odompose.position.z << std::endl;
    std::cout <<" odompose.orientation.x="<<  odompose.orientation.x << std::endl;
    std::cout <<" odompose.orientation.y="<<  odompose.orientation.y << std::endl;
    std::cout <<" odompose.orientation.z="<<  odompose.orientation.z << std::endl;*/

    tf2::fromMsg(odompose,tf_odom_pose);
    //std::cout <<" tf_odom_pose="<<  tf_odom_pose << std::endl;
    bool T_ok=tf_to_eigen_odom(tf_odom_pose,odom_eigen_tf,dt);

    /* std::cout <<" odom_eigen_tf="<<  odom_eigen_tf << std::endl;
    std::cout <<" ["<<odom_eigen_tf(0,0)<<","<<odom_eigen_tf(0,1)<<","<<odom_eigen_tf(0,2)<<","<<odom_eigen_tf(0,3)<<"]="<< std::endl;
    std::cout <<" ["<<odom_eigen_tf(1,0)<<","<<odom_eigen_tf(1,1)<<","<<odom_eigen_tf(1,2)<<","<<odom_eigen_tf(1,3)<<"]="<< std::endl;
    std::cout <<" ["<<odom_eigen_tf(2,0)<<","<<odom_eigen_tf(2,1)<<","<<odom_eigen_tf(2,2)<<","<<odom_eigen_tf(2,3)<<"]="<< std::endl;
    std::cout <<" ["<<odom_eigen_tf(3,0)<<","<<odom_eigen_tf(3,1)<<","<<odom_eigen_tf(3,2)<<","<<odom_eigen_tf(3,3)<<"]="<< std::endl;*/

    if(!T_ok)
    {
     //ROS_INFO("Wrong tf_odom_pose to odom_eigen_transform");
     RCLCPP_ERROR(get_logger(), "Wrong tf_odom_pose to odom_eigen_transform");
    }
    
		odom_eigen_tf.block(2,0,1,4) = odom_eigen_tf.block(3,0,1,4);
		odom_eigen_tf.block(0,2,4,1) = odom_eigen_tf.block(0,3,4,1);
		odom_eigen_tf_3by3=odom_eigen_tf.block(0,0,3,3);
  	odom.set_act_odomTFmatrix(odom_eigen_tf_3by3);

  	/*std::cout <<" odom_eigen_tf_3by3="<< std::endl;
  	std::cout <<" ["<<odom_eigen_tf_3by3(0,0)<<","<<odom_eigen_tf_3by3(0,1)<<","<<odom_eigen_tf_3by3(0,2)<<"]="<< std::endl;
  	std::cout <<" ["<<odom_eigen_tf_3by3(1,0)<<","<<odom_eigen_tf_3by3(1,1)<<","<<odom_eigen_tf_3by3(1,2)<<"]="<< std::endl;
  	std::cout <<" ["<<odom_eigen_tf_3by3(2,0)<<","<<odom_eigen_tf_3by3(2,1)<<","<<odom_eigen_tf_3by3(2,2)<<"]="<< std::endl;
  	*/
  	//ROS_INFO("dx= %f",dx); 
		//ROS_INFO("dy= %f",dy); 
		//ROS_INFO("dtheta= %f",dtheta); 
		//ROS_INFO("before_x= %f",before_x); 
		//ROS_INFO("before_y= %f",before_y); 
		//ROS_INFO("before_theta= %f",before_theta); 
		//ROS_INFO("act_x= %f",act_x); 
		//ROS_INFO("act_y= %f",act_y); 
		//ROS_INFO("act_theta= %f",act_theta); 
  	
  	odom.update_odometryGood(dx,dy,dtheta,dt,wz,linear_vx,linear_vy); 
  	
  	odom.set_before_x(act_x);
  	odom.set_before_y(act_y); 
  	odom.set_before_theta(act_theta);  	
  	odom.set_ant_odomTFmatrix(odom_eigen_tf_3by3);
  	//ROS_INFO("PeopleTrackingMhtAlgorithm::odom.vx= %f", odom.get_vx());
  	//ROS_INFO("PeopleTrackingMhtAlgorithm::odom.vy= %f", odom.get_vy());
  	//ROS_INFO("PeopleTrackingMhtAlgorithm::odom.wz %f", odom.get_wz());
  	//ROS_INFO("PeopleTrackingMhtAlgorithm::odom.R %f", odom.get_translation());
   	//ROS_INFO("PeopleTrackingMhtAlgorithm::odom.thetaZ %f", odom.get_rotation());
   	//ROS_INFO("PeopleTrackingMhtAlgorithm::odom.dt %f", odom.get_dt());
   
  }
    
  //unlock previously blocked shared variables 
  alg_->unlock(); 
  this->odom_mutex_.exit(); 
}

void PeopleTrackingMhtAlgNode::detections_callback(const iri_perception_msgs::msg::DetectionArray::ConstPtr& msg)
{

RCLCPP_INFO(get_logger(), "detections_callback 1");
std::cout << "msg->detection.size()"<<msg->detection.size() << std::endl;
std::cout << "this->detectionsReceived"<<this->detectionsReceived << std::endl;

std::cout << "cargo_ANTS"<<cargo_ANTS << std::endl;

  //ROS_INFO("DETECTION DETECTION!!! PeopleTrackingMhtAlgNode::detections_callback: New Message Received msg->detection.size()=%d",msg->detection.size());
  //double cov=2.0;  // double cov=0.5;==Covarianza para tibi y dabo!!!
  //use appropiate mutex to shared variables if necessary 
  alg_->lock();
  this->detections_mutex_.enter();
  if(!this->detectionsReceived)
  {
  
   std::cout << "entro en if"<< std::endl;
		//ROS_INFO("PeopleTrackingMhtAlgNode::detections_callback: if(!this->detectionsReceived)");
    this->trackingArray_msg_.header = msg->header;
    //double detection_time = msg->header.stamp.toSec();
    double detection_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

   std::cout << "msg->header.frame_id"<<msg->header.frame_id << std::endl;
std::cout << "this->frame_id"<<this->frame_id << std::endl;



    this->detections.clear();  
    //int id1=msg->header.frame_id;
    //int id2=this->frame_id;
    //ROS_INFO("PeopleTrackingMhtAlgNode::detections_callback: msg->header.frame_id= %d",id1);
     //ROS_INFO("PeopleTrackingMhtAlgNode::detections_callback: this->frame_id= %s",this->frame_id.c_str());
    if(cargo_ANTS){
    	this->frame_id = "ibeo1";
    }
    if(msg->header.frame_id!=this->frame_id)
    {
			//ROS_INFO("PeopleTrackingMhtAlgNode::detections_callback:  if(msg->header.frame_id!=this->frame_id)");
  		if(cargo_ANTS){
    		this->frame_id = "/map";
    	}
      this->trackingArray_msg_.header.frame_id = this->frame_id;
      std::string source_frame = msg->header.frame_id;
      std::string target_frame = this->frame_id;
      //ros::Time   target_time  = msg->header.stamp;
      rclcpp::Time target_time = msg->header.stamp;
      //ROS_INFO("PeopleTrackingMhtAlgNode::try to transform between frames: (source frame)%s-->(target_frame)%s", source_frame.c_str(), target_frame.c_str());

      try
      {
        //bool tf_exists = tf_listener_.waitForTransform(target_frame, source_frame, target_time, ros::Duration(0.5), ros::Duration(0.01));
        
        std::cout << "target_frame"<<target_frame << std::endl;
std::cout << "source_frame"<<source_frame << std::endl;

        bool tf_exists = tf_buffer_.canTransform(target_frame, source_frame, target_time, rclcpp::Duration::from_seconds(0.5), nullptr);
std::cout << "tf_exists"<<tf_exists << std::endl;
				//ROS_INFO("PeopleTrackingMhtAlgNode::detections_callback:  ANTES de  if(tf_exists)");
        if(tf_exists)
        {
          for(unsigned int i=0;i<msg->detection.size();i++)
          {
            geometry_msgs::msg::PoseStamped poseIn;
            geometry_msgs::msg::PoseStamped poseOut;
            poseIn.header             = msg->header;
            poseIn.header.frame_id    = source_frame;
            poseIn.pose.position      = msg->detection[i].position;
            poseIn.pose.orientation.z = 1.0;
            //tf_listener_.transformPose(target_frame, poseIn, poseOut);
            //tf_buffer_.transform(poseIn, poseOut, target_frame, rclcpp::Duration::from_seconds(1.0));
            poseOut = tf_buffer_.transform(poseIn, target_frame, tf2::durationFromSec(1.0));
            double x = poseOut.pose.position.x;
            double y = poseOut.pose.position.y;
            double p = 0.8;//msg->detection[i].probability;
            //Sdetection::detection_type type;
	    			//type = (Sdetection::detection_type) msg->type;
 						int type = msg->type;
            std::vector<double> cov(4,0.0);
            if(cargo_ANTS){
              cov[0]= 7.0;//msg->detection[i].covariances[0];
              cov[1]= 0.0;
              cov[2]= 0.0;
              cov[3]= 7.0;//msg->detection[i].covariances[7];
            }else{
              cov[0]= 0.5;//msg->detection[i].covariances[0];
              cov[1]= 0.0;
              cov[2]= 0.0;
              cov[3]= 0.5;//msg->detection[i].covariances[7];
            }
            /* ROS_INFO("Detection");  
         		ROS_INFO("detection.type: %d",type);   
         		ROS_INFO("detection.x: %f",x); 
         		ROS_INFO("detection.y: %f",y);  
         		ROS_INFO("detection.detection_time: %f",detection_time); 
         		ROS_INFO("detection.p: %f",p);  */
            Sdetection detection(x,y,detection_time,cov,type,p);

	    double dist_detection_to_robot_position; 
		/*if((robot_actual_pose_x_!=0.0)&&(robot_actual_pose_y_!=0.0)){
		dist_detection_to_robot_position=sqrt((robot_actual_pose_x_-x)*(robot_actual_pose_x_-x)+(robot_actual_pose_y_-y)*(robot_actual_pose_y_-y));
		}else{
			dist_detection_to_robot_position=0.0;
		}*/
  //ROS_INFO("detection_x_: %f;  detection_y_=%f",x,y);
  //ROS_INFO("robot_actual_pose_x_: %f;  robot_actual_pose_y_=%f",robot_actual_pose_x_,robot_actual_pose_y_);
  //ROS_INFO("dist_detection_to_robot_position: %f; < max_dist_to_detect_people_=%f",dist_detection_to_robot_position,max_dist_to_detect_people_);

	    //if(dist_detection_to_robot_position < max_dist_to_detect_people_){
 		this->detections.push_back(detection);
	    //}

           
          }
          if(this->detections.size()>0)
            this->detectionsReceived=true;
        }
        else
        {
        
         
          //ROS_INFO("PeopleTrackingMhtAlgNode::No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
          RCLCPP_ERROR(get_logger(), "PeopleTrackingMhtAlgNode::No transform: %s-->%s", source_frame.c_str(), target_frame.c_str());
        }
      }
      catch (tf2::TransformException &ex)
      {
        //ROS_INFO("PeopleTrackingMhtAlgNode:: %s",ex.what());
        RCLCPP_ERROR(get_logger(), "PeopleTrackingMhtAlgNode:: %s",ex.what());
      }
    }
    else
    {
       std::cout << "entro en else"<< std::endl;
      for(unsigned int i=0;i<msg->detection.size();i++)
      {
        double x = msg->detection[i].position.x;
        double y = msg->detection[i].position.y;
        double p = 0.7;//TODO msg->detection[i].probability;
        //Sdetection::detection_type type = (Sdetection::detection_type) msg->type;
 				int type = msg->type;
        std::vector<double> cov(4,0.0);

        if(cargo_ANTS){
          cov[0]= 7.0;//msg->detection[i].covariances[0];
          cov[1]= 0.0;
          cov[2]= 0.0;
          cov[3]= 7.0;//msg->detection[i].covariances[7];
        }else{
           cov[0]= 0.5;//msg->detection[i].covariances[0];
           cov[1]= 0.0;
           cov[2]= 0.0;
           cov[3]= 0.5;//msg->detection[i].covariances[7];
        }
            
        /*ROS_INFO("Detection");  
        ROS_INFO("detection.type: %d",type);   
        ROS_INFO("detection.x: %f",x); 
        ROS_INFO("detection.y: %f",y);  
        ROS_INFO("detection.detection_time: %f",detection_time); 
        ROS_INFO("detection.p: %f",p); */ 
        Sdetection detection(x,y,detection_time,cov,type,p);
	

	/*double dist_detection_to_robot_position;

	if((robot_actual_pose_x_!=0.0)&&(robot_actual_pose_y_!=0.0)){
		dist_detection_to_robot_position=sqrt((robot_actual_pose_x_-x)*(robot_actual_pose_x_-x)+(robot_actual_pose_y_-y)*(robot_actual_pose_y_-y));
	}else{
		dist_detection_to_robot_position=0.0;
	}*/

  //ROS_INFO("detection_x_: %f;  detection_y_=%f",x,y);
  //ROS_INFO("robot_actual_pose_x_: %f;  robot_actual_pose_y_=%f",robot_actual_pose_x_,robot_actual_pose_y_);
  //ROS_INFO("dist_detection_to_robot_position: %f; < max_dist_to_detect_people_=%f",dist_detection_to_robot_position,max_dist_to_detect_people_);
 
	//if(dist_detection_to_robot_position < max_dist_to_detect_people_){
 		this->detections.push_back(detection);
	//}
       



      }
      if(this->detections.size()>0)
        this->detectionsReceived=true;
    }
  }

std::cout << " (last) this->detections.size()"<<this->detections.size()<< std::endl;
  //unlock previously blocked shared variables 
  alg_->unlock();

  this->detections_mutex_.exit();

}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

/*void PeopleTrackingMhtAlgNode::node_config_update(Config &config, uint32_t level)
{
  alg_->lock();
  this->frame_id = config.frame_id;
  this->local_tracker   = config.local_tracker;
  this->id_markers      = config.id_markers;
  this->vel_markers     = config.vel_markers;
  this->cov_markers     = config.cov_markers;
  this->cov_markers_head= config.cov_markers_head;
  this->cov_markers_arm1= config.cov_markers_arm1;
  this->cov_markers_arm2= config.cov_markers_arm2;
  this->covdet_markers  = config.covdet_markers;
  this->covpred_markers = config.covpred_markers;
  this->id_cluster_markers      = config.id_cluster_markers;
  this->clust_markers = config.clust_markers;  
  this->group_markers = config.group_markers;
  this->ini_markers(); 
  this->track_path = config.track_path;
  this->max_dist_to_detect_people_=config.max_dist_to_detect_people_conf;	
	
  alg_->unlock();
}*/

void PeopleTrackingMhtAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{


  rclcpp::init(argc, argv);
    auto node = std::make_shared<PeopleTrackingMhtAlgNode>();
    //auto node = LaserPeopleDetectionAlgNode::create(); 
    //node.mainnodethread();
    //std::thread node_thread(mainnodethread(), node);
    //node_thread.join();
   //auto node = std::make_shared<LaserPeopleDetectionAlgNode>();
   //std::shared_ptr<LaserPeopleDetectionAlgNode> node = std::make_shared<LaserPeopleDetectionAlgNode>();
   
    RCLCPP_INFO(node->get_logger(), "main 1");
    //rclcpp::Rate loop_rate(node->getRate());
    //RCLCPP_INFO(node->get_logger(), "main 2");
   
   while (rclcpp::ok())
  {
    RCLCPP_INFO(node->get_logger(), "main 2");
    node->mainNodeThread();
    RCLCPP_INFO(node->get_logger(), "main 3");
    rclcpp::spin(node);  // hace que se activen (iteren) los callback desde el inicio
    //rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "main 4");
 
  }
  RCLCPP_INFO(node->get_logger(), "main 7");
  rclcpp::shutdown();
   
   //rclcpp::spin(node); 
   //rclcpp::shutdown();
   
  //return algorithm_base::main<LaserPeopleDetectionAlgNode>(argc, argv, "laser_people_detection_alg_node");
  return 0;

  //return algorithm_base::main<PeopleTrackingMhtAlgNode>(argc, argv, "people_tracking_mht_alg_node");
  
}

void PeopleTrackingMhtAlgNode::fillTracksMsg()
{
 std::cout << " (tracks out) this->tracks.size() "<<this->tracks.size()<< std::endl;
  this->trackingArray_msg_.type=123;//TODO tracker type 123

  this->trackingArray_msg_.detection.resize(this->tracks.size());

  for(unsigned int i=0;i<this->tracks.size();i++)
  {
		this->trackingArray_msg_.detection[i].position.x     = tracks[i].x;
    this->trackingArray_msg_.detection[i].position.y     = tracks[i].y;
    this->trackingArray_msg_.detection[i].velocity.x     = tracks[i].vx;
    this->trackingArray_msg_.detection[i].velocity.y     = tracks[i].vy;
    this->trackingArray_msg_.detection[i].id             = tracks[i].id;
    this->trackingArray_msg_.detection[i].probability    = tracks[i].probability_of_detection;
    this->trackingArray_msg_.detection[i].covariances[0] = tracks[i].cov[0];
    this->trackingArray_msg_.detection[i].covariances[1] = tracks[i].cov[1];
    this->trackingArray_msg_.detection[i].covariances[6] = tracks[i].cov[4];
    this->trackingArray_msg_.detection[i].covariances[7] = tracks[i].cov[5];
/*    this->trackingArray_msg_.detection[i].position.x     = this->tracks[i].SpointV_cov_track_.x;
    this->trackingArray_msg_.detection[i].position.y     = this->tracks[i].SpointV_cov_track_.y;
    this->trackingArray_msg_.detection[i].velocity.x     = this->tracks[i].SpointV_cov_track_.vx;
    this->trackingArray_msg_.detection[i].velocity.y     = this->tracks[i].SpointV_cov_track_.vy;
    this->trackingArray_msg_.detection[i].id             = this->tracks[i].id;
    //this->trackingArray_msg_.detection[i].type           = this->tracks[i].type_;
    this->trackingArray_msg_.detection[i].probability    = this->tracks[i].probability_of_detection;
    this->trackingArray_msg_.detection[i].covariances[0] = this->tracks[i].SpointV_cov_track_.cov[0];
    this->trackingArray_msg_.detection[i].covariances[1] = this->tracks[i].SpointV_cov_track_.cov[1];
    this->trackingArray_msg_.detection[i].covariances[6] = this->tracks[i].SpointV_cov_track_.cov[4];
    this->trackingArray_msg_.detection[i].covariances[7] = this->tracks[i].SpointV_cov_track_.cov[5];
*/
    //ROS_WARN("PeopleTrackingMhtAlgNode::fillTracksMsg: vx,vy,cov[0]: %0.2f, %0.2f, %0.4f, %0.4f", tracks[i].vx, tracks[i].vy, tracks[i].cov[0], tracks[i].cov[5]);
    
  }
  std::cout << " (tracks out) this->trackingArray_msg_.detection.size(): "<<this->trackingArray_msg_.detection.size()<< std::endl;
}


void PeopleTrackingMhtAlgNode::publishMarkers()
{
  unsigned int n_track = this->trackingArray_msg_.detection.size(); 
 
 std::cout << " n_track: "<<n_track<<"this->id_markers"<<this->id_markers<< std::endl;
 
  if(cargo_ANTS){
 		this->frame_id = "ibeo1";
 	 	this->trackingArray_msg_.header.frame_id = this->frame_id;
 	}
  //this->MarkerArray_msg_.markers.clear();
  for(unsigned int i=0;i<n_track;i++)
  {
    std::cout << " in if 1 "<< std::endl;
    std::cout << " x "<<this->trackingArray_msg_.detection[i].position.x<< std::endl;
      std::cout << " y "<<this->trackingArray_msg_.detection[i].position.y<< std::endl;
        std::cout << " id "<<this->trackingArray_msg_.detection[i].id<< std::endl;
    //id markers
    if(this->id_markers)
    {
        std::cout << " in if 2 "<< std::endl;
      id_track_marker_.header = this->trackingArray_msg_.header;
      id_track_marker_.id = i;
      id_track_marker_.pose.position.x = this->trackingArray_msg_.detection[i].position.x;
      id_track_marker_.pose.position.y = this->trackingArray_msg_.detection[i].position.y;
      id_track_marker_.pose.position.z = 1.0+this->trackingArray_msg_.detection[i].position.z;
      std::stringstream idText;
      idText << this->trackingArray_msg_.detection[i].id;
      id_track_marker_.text = idText.str();
      std::cout<<"before MarkerArray_msg_.markers.push_back: id_track_marker_"<<std::endl;
      std::cout<<"MarkerArray_msg_.markers.size():"<<MarkerArray_msg_.markers.size()<<std::endl;
      this->MarkerArray_msg_.markers.push_back(id_track_marker_);
    }
      
    //velocity markers
    if(this->vel_markers)
    { std::cout << " in if 3 "<< std::endl;
 	  	velocity_track_marker_.header = this->trackingArray_msg_.header;
      velocity_track_marker_.id = i;
      velocity_track_marker_.pose.position.x = this->trackingArray_msg_.detection[i].position.x;
      velocity_track_marker_.pose.position.y = this->trackingArray_msg_.detection[i].position.y;
      velocity_track_marker_.pose.position.z = this->trackingArray_msg_.detection[i].position.z;
      double yaw = std::atan2(this->trackingArray_msg_.detection[i].velocity.y,this->trackingArray_msg_.detection[i].velocity.x);
      //geometry_msgs::msg::Quaternion myQ = tf2::createQuaternionMsgFromYaw(yaw);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);  // Set the rotation with only the yaw value
      geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);     
      velocity_track_marker_.pose.orientation = myQ;
      double speed = this->trackingArray_msg_.detection[i].velocity.x*this->trackingArray_msg_.detection[i].velocity.x+this->trackingArray_msg_.detection[i].velocity.y*this->trackingArray_msg_.detection[i].velocity.y;
      if(std::sqrt(speed) < 0.1)
      {
        velocity_track_marker_.scale.z = 0.0;
        velocity_track_marker_.scale.y = 0.0;
        velocity_track_marker_.scale.x = 0.0;
      }
      else
      {
        velocity_track_marker_.scale.x = std::sqrt(speed);
        velocity_track_marker_.scale.y = 0.3;
        velocity_track_marker_.scale.z = 0.05;
        MarkerArray_msg_.markers.push_back(velocity_track_marker_);
      }      
    }

    //covariance markers TRACKS (body)
    if(this->cov_markers)
    { std::cout << " in if 4 "<< std::endl;
      fill_my_covariance_marker_track( i );
      MarkerArray_msg_.markers.push_back(cov_track_marker_);
    }
    
    //covariance markers TRACKS (head)
    if(this->cov_markers_head)
    {
      fill_my_covariance_marker_track_head( i );
      MarkerArray_msg_.markers.push_back(cov_track_marker_head_);
    }
    
    //covariance markers TRACKS (arm1)
    if(this->cov_markers_arm1)
    {
      fill_my_covariance_marker_track_arm1( i );
      MarkerArray_msg_.markers.push_back(cov_track_marker_arm1_);
    }
    
    //covariance markers TRACKS (arm2)
    if(this->cov_markers_arm2)
    {
      fill_my_covariance_marker_track_arm2( i );
      MarkerArray_msg_.markers.push_back(cov_track_marker_arm2_);
    } 
    
  }

  //covariance markers DETECTIONS
  if(this->covdet_markers)
  {
    unsigned int n_detection = this->detections.size();
    for(unsigned int count_detec=0;count_detec<n_detection;count_detec++)
    {
      //in this case i==count_detections for each interation.
      fill_my_covariance_marker_detection( count_detec );
      MarkerArray_msg_.markers.push_back(cov_detection_marker_);
    }
  }

  if(this->covpred_markers)
  {
    //covariance markers PREDICTIONS
    unsigned int n_prediction = this->predictions.size();
    for(unsigned int count_pred=0;count_pred<n_prediction;count_pred++)
    {
      fill_my_covariance_marker_prediction( count_pred );
      MarkerArray_msg_.markers.push_back(cov_prediction_marker_);
    }
  }

 if(this->clust_markers)
 {
    //markers clusters
    unsigned int n_clusters = this->clusters.size();
    for(unsigned int count_clus=0;count_clus<n_clusters;count_clus++)
    {
      fill_my_marker_cluster( count_clus );
      MarkerArray_msg_.markers.push_back(cluster_marker_);          
    }
  }

 if(this->id_cluster_markers)
  {
    //id clusters markers
    unsigned int n_clusters = this->clusters.size();
    for(unsigned int count_clus=0;count_clus<n_clusters;count_clus++)
    {
    	id_cluster_marker_.header = this->trackingArray_msg_.header;
    	id_cluster_marker_.id = count_clus;
      id_cluster_marker_.pose.position.x = this->clusters[count_clus].centroid_cluster_.x;
      id_cluster_marker_.pose.position.y = this->clusters[count_clus].centroid_cluster_.y;
      id_cluster_marker_.pose.position.z = 0.0;
      std::stringstream id_clustText;      		
      id_clustText << this->clusters[count_clus].id_cluster_;
      id_cluster_marker_.text = id_clustText.str();
      MarkerArray_msg_.markers.push_back(id_cluster_marker_);
    }
  }

	if(this->group_markers)
	{
		//markers clusters
		unsigned int n_groups = this->groups.size();
		//ROS_INFO(" numero n_groups: %d" , n_groups);
		for(unsigned int count_group=0;count_group<n_groups;count_group++)
		{
		  fill_my_marker_group( count_group );
		  MarkerArray_msg_.markers.push_back(group_marker_);          
		}
	}
	if(this->track_path)
  {  
  	n_track = this->trackingArray_msg_.detection.size();
 		for(unsigned int i=0;i<n_track;i++)
		{
			visualization_msgs::msg::Marker track_path_;
			track_path_.header = this->trackingArray_msg_.header;
  		track_path_.ns     = "track_path";
  		track_path_.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  		track_path_.action = visualization_msgs::msg::Marker::ADD;
 			track_path_.scale.x = 0.1;
			track_path_.header = this->trackingArray_msg_.header;
	  	unsigned int actual_id=this->trackingArray_msg_.detection[i].id;	 	
	  	double x=this->trackingArray_msg_.detection[i].position.x;
	 		double y=this->trackingArray_msg_.detection[i].position.y;
	  	double detection_time=0.0;//this->trackingArray_msg_.header.stamp.toSec();
	  	double vx=0.0;//this->trackingArray_msg_.detection[i].velocity.x;
	  	double vy=0.0;//this->trackingArray_msg_.detection[i].velocity.y;;
	  	std::vector<double> cov(4,0.0);
	  	//cov[0]= this->trackingArray_msg_.detection[i].covariances[0];
	  	//cov[1]= this->trackingArray_msg_.detection[i].covariances[1];
	  	//cov[4]= this->trackingArray_msg_.detection[i].covariances[6];
	  	//cov[5]= this->trackingArray_msg_.detection[i].covariances[7];	  		
	  	double prob=0.0;//this->trackingArray_msg_.detection[i].probability;  	
	  	Strack trackT(x,y,detection_time,vx,vy,cov,actual_id,prob);
	  	tracks_path[actual_id].push_back(trackT);
	  	//fill_my_marker_track_path(actual_id);  			  		
			track_path_.id     = actual_id;//this->groups[i].ex_id_group_;
			unsigned int n_path=tracks_path[actual_id].size();
			
			/*for(unsigned int r=0; r<n_path;r++){
				Spoint t=tracks_path[actual_id][r].getSpointTrack();
				geometry_msgs::Point p;
				p.x = t.x;
				p.y = t.y;
				std::cout<< "\n point("<<r<<"), \n x="<< p.x <<"\n y="<< p.y << std::endl;
				p.z = 0.1;	
				track_path_.points.push_back(p);		  		
			}*/
			
			Spoint t=tracks_path[actual_id][n_path-1].getSpointTrack();
			geometry_msgs::msg::Point p;
			p.x = t.x;
			p.y = t.y;	
			p.z = 0.01;	
			track_path_.points.push_back(p);
			t=tracks_path[actual_id][0].getSpointTrack();
			//geometry_msgs::Point p;
			p.x = t.x;
			p.y = t.y;
			p.z = 0.01;	
			track_path_.points.push_back(p);
			double yaw = std::atan2(tracks_path[actual_id][0].vy,tracks_path[actual_id][0].vx);
			//geometry_msgs::msg::Quaternion myQ = tf::createQuaternionMsgFromYaw(yaw);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, yaw to your desired value
      geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);
			track_path_.pose.orientation = myQ;
			track_path_.color.r = 0.0;
			track_path_.color.g = 0.0;
			track_path_.color.b = 1.0;
			track_path_.color.a = 1.0;
			track_path_.scale.x = 0.2;//major vap
			track_path_.scale.y = 0.2;//minor vap
			track_path_.scale.z = 0.2;
			//track_path_.pose.orientation.w = 0.1;  
			track_path_.lifetime = rclcpp::Duration::from_seconds(0.1f);
		
			MarkerArray_msg_.markers.push_back(track_path_);	 		  			  			  			  		
		}
	}	  
  //this->tracksMarkers_publisher_.publish(this->MarkerArray_msg_);
  this->tracksMarkers_publisher_->publish(this->MarkerArray_msg_);
  
 //std::cout<<"this->MarkerArray_msg_.markers.size()"<< MarkerArray_msg_.markers.size()<<std::endl;
  //ROS_INFO(" size markers: %d" , this->MarkerArray_msg_.markers.size() );
  this->MarkerArray_msg_.markers.clear(); 
}

void PeopleTrackingMhtAlgNode::fill_my_covariance_marker_track( unsigned int i )
{
  cov_track_marker_.header = this->trackingArray_msg_.header;
  cov_track_marker_.id     = this->trackingArray_msg_.detection[i].id; 
  //multi=1,2,3, segun el numero de marker que sea con la n. [i+2*n] [i+3*n]
  cov_track_marker_.pose.position.x = this->trackingArray_msg_.detection[i].position.x;
  cov_track_marker_.pose.position.y = this->trackingArray_msg_.detection[i].position.y;
  cov_track_marker_.pose.position.z = 0.35;
  
  Eigen::Matrix2d cov;
  cov(0, 0) = this->trackingArray_msg_.detection[i].covariances[0];//point.cov.at(0);//xx
  cov(0, 1) = this->trackingArray_msg_.detection[i].covariances[1];//point.cov.at(1);
  cov(1, 0) = this->trackingArray_msg_.detection[i].covariances[6];//point.cov.at(4);
  cov(1, 1) = this->trackingArray_msg_.detection[i].covariances[7];//point.cov.at(5);//yy	 
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);//eig.compute(cov);
  
  if(cargo_ANTS){
  	cov_track_marker_.scale.x = 3.5;//this->groups[i].ex_group_width_;//major vap
  	cov_track_marker_.scale.y = 3.5;//this->groups[i].ex_group_length_;//minor vap
  	cov_track_marker_.scale.z = 1.0;
  	cov_track_marker_.color.a = 0.7; 
  }else{
	  //plot covariances ellipses	  	 
	  cov_track_marker_.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
	  cov_track_marker_.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
	  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
    cov_track_marker_.scale.z = 0.7;//std::max(0.7*(1-cov_size/2.0), 0.1); //0.7;
    cov_track_marker_.pose.position.z = cov_track_marker_.scale.z/2.0;
	  cov_track_marker_.color.a = std::max(0.8*(1-cov_size/2.0), 0.1);
  }
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cov_track_marker_.pose.orientation = tf2::createQuaternionMsgFromYaw(angle);
  tf2::Quaternion q;
  q.setEuler(0, 0, angle); // Set yaw; roll and pitch are set to 0
  cov_track_marker_.pose.orientation = tf2::toMsg(q);

}

void PeopleTrackingMhtAlgNode::fill_my_covariance_marker_track_head( unsigned int i )
{
  cov_track_marker_head_.header = this->trackingArray_msg_.header;
  cov_track_marker_head_.id     = this->trackingArray_msg_.detection[i].id;
  //multi=1,2,3, segun el numero de marker que sea con la n. [i+2*n] [i+3*n]
  cov_track_marker_head_.pose.position.x = this->trackingArray_msg_.detection[i].position.x;
  cov_track_marker_head_.pose.position.y = this->trackingArray_msg_.detection[i].position.y;
  cov_track_marker_head_.pose.position.z = 0.35+0.7; // position of body marcker+size_body_marker = Position of the head.

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = this->trackingArray_msg_.detection[i].covariances[0];//point.cov.at(0);//xx
  cov(0, 1) = this->trackingArray_msg_.detection[i].covariances[1];//point.cov.at(1);
  cov(1, 0) = this->trackingArray_msg_.detection[i].covariances[6];//point.cov.at(4);
  cov(1, 1) = this->trackingArray_msg_.detection[i].covariances[7];//point.cov.at(5);//yy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  cov_track_marker_head_.scale.x = 0.5;//sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  cov_track_marker_head_.scale.y = 0.5;//sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  cov_track_marker_head_.scale.z = 0.5;
  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
  cov_track_marker_head_.color.a = std::max(0.6-cov_size*(0.6/5.0), 0.1);
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cov_track_marker_head_.pose.orientation = tf2::createQuaternionMsgFromYaw(angle);
  tf2::Quaternion q;
  q.setEuler(0, 0, angle); // Set yaw; roll and pitch are set to 0
  cov_track_marker_head_.pose.orientation = tf2::toMsg(q);
}

void PeopleTrackingMhtAlgNode::fill_my_covariance_marker_track_arm1( unsigned int i )
{
	//ROS_INFO(" tracks id: %d", this->trackingArray_msg_.detection[i].id );
  double yaw = std::atan2(this->trackingArray_msg_.detection[i].velocity.y,this->trackingArray_msg_.detection[i].velocity.x);
  //ROS_INFO(" arm1 yaw: %f", yaw );
  double yaw_arm = std::atan2(-this->trackingArray_msg_.detection[i].velocity.x, this->trackingArray_msg_.detection[i].velocity.y);
  //ROS_INFO(" arm1 yaw_arm: %f", yaw_arm );
  //ROS_INFO(" arm1 fabs(yaw_arm): %f", fabs(yaw_arm) );
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, yaw to your desired value
  geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);

    
  cov_track_marker_arm1_.header = this->trackingArray_msg_.header;
 	cov_track_marker_arm1_.id     = this->trackingArray_msg_.detection[i].id;
    
	// pose of the arm, perpendicular to the orientation of the velocity, al +0.15 or -0.15
	cov_track_marker_arm1_.pose.position.z = 0.35+0.1; // position of body marcker+size_body_marker = Position of the arm in the top of the body.
  //if( ((fabs(yaw_arm)<1.7) && (fabs(yaw_arm)>1.4)) ){
  //	ROS_INFO( "draw arm1" );
  	cov_track_marker_arm1_.pose.position.x = this->trackingArray_msg_.detection[i].position.x-0.25*cos(yaw_arm); 
  	cov_track_marker_arm1_.pose.position.y = this->trackingArray_msg_.detection[i].position.y+0.25*sin(yaw_arm);  		
	//} else if(((fabs(yaw_arm)<0.78) && (fabs(yaw_arm)>0))  ){
	//	ROS_INFO( "draw arm1" );
	//	cov_track_marker_arm1_.pose.position.x = this->trackingArray_msg_.detection[i].position.x-0.25*sin(yaw_arm); 
  	//	cov_track_marker_arm1_.pose.position.y = this->trackingArray_msg_.detection[i].position.y+0.25*cos(yaw_arm);  		
	//} else{
	//	ROS_INFO( "draw arm1" );
	//	cov_track_marker_arm1_.pose.position.x = this->trackingArray_msg_.detection[i].position.x-0.25*cos(yaw_arm); 
  	//	cov_track_marker_arm1_.pose.position.y = this->trackingArray_msg_.detection[i].position.y+0.25*sin(yaw_arm);  			
	//}
  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = this->trackingArray_msg_.detection[i].covariances[0];//point.cov.at(0);//xx
  cov(0, 1) = this->trackingArray_msg_.detection[i].covariances[1];//point.cov.at(1);
  cov(1, 0) = this->trackingArray_msg_.detection[i].covariances[6];//point.cov.at(4);
  cov(1, 1) = this->trackingArray_msg_.detection[i].covariances[7];//point.cov.at(5);//yy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  cov_track_marker_arm1_.scale.x = 0.5;//sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  cov_track_marker_arm1_.scale.y = 0.5;//sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  cov_track_marker_arm1_.scale.z = 0.5;
  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
  cov_track_marker_arm1_.color.a = std::max(0.6-cov_size*(0.6/5.0), 0.1);
	//double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cov_track_marker_arm1_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  cov_track_marker_arm1_.color.a =1;
  double speed = this->trackingArray_msg_.detection[i].velocity.x*this->trackingArray_msg_.detection[i].velocity.x+this->trackingArray_msg_.detection[i].velocity.y*this->trackingArray_msg_.detection[i].velocity.y;
	if((std::sqrt(speed) < 0.15)&&std::sqrt(speed) > -0.15)
  {
    //cov_track_marker_arm1_.pose.orientation = tf2::createQuaternionMsgFromYaw(0.0);
    tf2::Quaternion q;
    q.setEuler(0, 0, 0);
    cov_track_marker_arm1_.pose.orientation = tf2::toMsg(q);
  }
  else
  {
  	double yaw = std::atan2(this->trackingArray_msg_.detection[i].velocity.y,this->trackingArray_msg_.detection[i].velocity.x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, yaw to your desired value
    geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);
    cov_track_marker_arm1_.pose.orientation = myQ;
	}      
}

void PeopleTrackingMhtAlgNode::fill_my_covariance_marker_track_arm2( unsigned int i )
{
	//ROS_INFO(" tracks id: %d", this->trackingArray_msg_.detection[i].id );
 
  double yaw = std::atan2(this->trackingArray_msg_.detection[i].velocity.y,this->trackingArray_msg_.detection[i].velocity.x);
  //ROS_INFO(" arm2 yaw: %f", yaw );
  double yaw_arm = std::atan2(-this->trackingArray_msg_.detection[i].velocity.x, this->trackingArray_msg_.detection[i].velocity.y);
  //ROS_INFO(" arm2 yaw_arm: %f", yaw_arm );
  //ROS_INFO(" arm2 fabs(yaw_arm): %f", fabs(yaw_arm) );
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, yaw to your desired value
  geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);
  cov_track_marker_arm2_.header = this->trackingArray_msg_.header;
  cov_track_marker_arm2_.id     = this->trackingArray_msg_.detection[i].id;
  // pose of the arm, perpendicular to the orientation of the velocity, al +0.15 or -0.15
	cov_track_marker_arm2_.pose.position.z = 0.35+0.1; // position of body marcker+size_body_marker = Position of the arm in the top of the body.
  //if( ((fabs(yaw_arm)<1.7) && (fabs(yaw_arm)>1.4)) ){
  // 	ROS_INFO( "draw arm2" );
  	cov_track_marker_arm2_.pose.position.x = this->trackingArray_msg_.detection[i].position.x-0.25*cos(yaw_arm); 
  	cov_track_marker_arm2_.pose.position.y = this->trackingArray_msg_.detection[i].position.y+0.25*sin(yaw_arm);  		
	//} else if(((fabs(yaw_arm)<0.78) && (fabs(yaw_arm)>0))  ){
	//	ROS_INFO( "draw arm2" );
	//	cov_track_marker_arm2_.pose.position.x = this->trackingArray_msg_.detection[i].position.x-0.25*sin(yaw_arm); 
  	//	cov_track_marker_arm2_.pose.position.y = this->trackingArray_msg_.detection[i].position.y+0.25*cos(yaw_arm);  		
	//} else{
	//	ROS_INFO( "draw arm2" );
	//	cov_track_marker_arm2_.pose.position.x = this->trackingArray_msg_.detection[i].position.x-0.25*cos(yaw_arm); 
  	//	cov_track_marker_arm2_.pose.position.y = this->trackingArray_msg_.detection[i].position.y+0.25*sin(yaw_arm);  			
	//}

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = this->trackingArray_msg_.detection[i].covariances[0];//point.cov.at(0);//xx
  cov(0, 1) = this->trackingArray_msg_.detection[i].covariances[1];//point.cov.at(1);
  cov(1, 0) = this->trackingArray_msg_.detection[i].covariances[6];//point.cov.at(4);
  cov(1, 1) = this->trackingArray_msg_.detection[i].covariances[7];//point.cov.at(5);//yy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  cov_track_marker_arm2_.scale.x = 0.5;//sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  cov_track_marker_arm2_.scale.y = 0.5;//sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  cov_track_marker_arm2_.scale.z = 0.5;
  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
  cov_track_marker_arm2_.color.a = std::max(0.6-cov_size*(0.6/5.0), 0.1);
//  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cov_track_marker_arm2_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  double speed = this->trackingArray_msg_.detection[i].velocity.x*this->trackingArray_msg_.detection[i].velocity.x+this->trackingArray_msg_.detection[i].velocity.y*this->trackingArray_msg_.detection[i].velocity.y;
  if((std::sqrt(speed) < 0.10)&&std::sqrt(speed) > -0.10)
  {
  	//cov_track_marker_arm2_.pose.orientation = tf2::createQuaternionMsgFromYaw(0.0);
    tf2::Quaternion q;
    q.setEuler(0, 0, 0); // Set yaw; roll and pitch are set to 0
    cov_track_marker_arm2_.pose.orientation = tf2::toMsg(q);
	}
  else
  {
		double yaw = std::atan2(this->trackingArray_msg_.detection[i].velocity.y,this->trackingArray_msg_.detection[i].velocity.x);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, yaw to your desired value
    geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);
    cov_track_marker_arm2_.pose.orientation = myQ;
	}
  
}


void PeopleTrackingMhtAlgNode::fill_my_covariance_marker_detection( unsigned int i )
{
  cov_detection_marker_.header = this->trackingArray_msg_.header;
  cov_detection_marker_.id = i ;
  //ROS_INFO("detection id: %d", i );
  cov_detection_marker_.pose.position.x = this->detections[i].x;
  cov_detection_marker_.pose.position.y = this->detections[i].y;
  cov_detection_marker_.pose.position.z = 0.25;

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = this->detections[i].cov[0];//point.cov.at(0);//xx
  cov(0, 1) = this->detections[i].cov[1];//point.cov.at(1);
  cov(1, 0) = this->detections[i].cov[2];//point.cov.at(4);
  cov(1, 1) = this->detections[i].cov[3];//point.cov.at(5);//yy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  cov_detection_marker_.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  cov_detection_marker_.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  cov_detection_marker_.scale.z = 1.0;
  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
  cov_detection_marker_.color.a = std::max(0.6-cov_size*(0.6/5.0), 0.1);
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cov_detection_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
  tf2::Quaternion q;
  q.setEuler(0, 0, angle); // Set yaw; roll and pitch are set to 0
  cov_detection_marker_.pose.orientation = tf2::toMsg(q);

}


void PeopleTrackingMhtAlgNode::fill_my_covariance_marker_prediction( unsigned int i )
{
  cov_prediction_marker_.header = this->trackingArray_msg_.header;
  cov_prediction_marker_.id     = this->predictions[i].id;
  cov_prediction_marker_.pose.position.x = this->predictions[i].x;
  cov_prediction_marker_.pose.position.y = this->predictions[i].y;
  cov_prediction_marker_.pose.position.z = 0.25;

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = this->predictions[i].cov[0];//point.cov.at(0);//xx
  cov(0, 1) = this->predictions[i].cov[1];//point.cov.at(1);
  cov(1, 0) = this->predictions[i].cov[4];//point.cov.at(4);
  cov(1, 1) = this->predictions[i].cov[5];//point.cov.at(5);//yy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  cov_prediction_marker_.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  cov_prediction_marker_.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  cov_prediction_marker_.scale.z = 0.5;
  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
  cov_prediction_marker_.color.a = std::max(0.6-cov_size*(0.6/5.0), 0.1);
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cov_prediction_marker_.pose.orientation = tf2::createQuaternionMsgFromYaw(angle);
  tf2::Quaternion q;
  q.setEuler(0, 0, angle); // Set yaw; roll and pitch are set to 0
  cov_prediction_marker_.pose.orientation = tf2::toMsg(q); 
}

void PeopleTrackingMhtAlgNode::fill_my_marker_cluster( unsigned int i )
{
  cluster_marker_.header = this->trackingArray_msg_.header;
  cluster_marker_.id     = this->clusters[i].id_cluster_;
  cluster_marker_.pose.position.x = this->clusters[i].centroid_cluster_.x;
  cluster_marker_.pose.position.y = this->clusters[i].centroid_cluster_.y;
  cluster_marker_.pose.position.z = 0.0;

	//cluster color, depending of is crosse or group
  if(((this->clusters[i].equal_velocity_tracks_==true)&&(this->clusters[i].equal_orientation_tracks_==true))||((this->clusters[i].equal_velocity_tracks_==true)&&(this->clusters[i].mediumVcluster_<0.15)&&(this->clusters[i].mediumVcluster_>-0.15))){ 
		//green if the cluster is a group of persons. (Groups in movemen with the same velocity and orientation + grups with 0 velocity)
		cluster_marker_.color.r = 0.0;
  	cluster_marker_.color.g = 1.0;
  	cluster_marker_.color.b = 1.0;
  }
  else{ //yellow if the cluster is a crosse of persons
  	cluster_marker_.color.r = 1.0;
  	cluster_marker_.color.g = 1.0;
  	cluster_marker_.color.b = 0.0;
  }

  //plot covariances ellipses
  Eigen::Matrix2d cov;
  cov(0, 0) = this->clusters[i].centroid_cluster_with_cluster_area_.cov[0];
  cov(0, 1) = this->clusters[i].centroid_cluster_with_cluster_area_.cov[1];
  cov(1, 0) = this->clusters[i].centroid_cluster_with_cluster_area_.cov[2];
  cov(1, 1) = this->clusters[i].centroid_cluster_with_cluster_area_.cov[3];
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  //eig.compute(cov);
  cluster_marker_.scale.x = sqrt( eig.eigenvalues()[0] ) + 0.001;//major vap
  cluster_marker_.scale.y = sqrt( eig.eigenvalues()[1] ) + 0.001;//minor vap
  cluster_marker_.scale.z = 0.2;
  double cov_size = std::max(sqrt( eig.eigenvalues()[0] ) + 0.001, sqrt( eig.eigenvalues()[1] ) + 0.001);
  cluster_marker_.color.a = std::max(0.6-cov_size*(0.6/5.0), 0.1);
  double angle = atan2( eig.eigenvectors()(1,0), eig.eigenvectors()(0,0) );
  //cluster_marker_.pose.orientation = tf2::createQuaternionMsgFromYaw(angle);
  tf2::Quaternion q;
  q.setEuler(0, 0, angle); // Set yaw; roll and pitch are set to 0
  cluster_marker_.pose.orientation = tf2::toMsg(q);
}

void PeopleTrackingMhtAlgNode::fill_my_marker_group( unsigned int i )
{
  group_marker_.header = this->trackingArray_msg_.header;
  group_marker_.id     = this->groups[i].ex_id_group_;
  group_marker_.pose.position.x = this->groups[i].ex_central_group_.x;
  group_marker_.pose.position.y = this->groups[i].ex_central_group_.y;
  group_marker_.pose.position.z = 0.0;
  group_marker_.color.r = 0.0;
  group_marker_.color.g = 0.0;
  group_marker_.color.b = 1.0;
  group_marker_.color.a = 1.0;
  group_marker_.scale.x = this->groups[i].ex_group_width_;//major vap
  group_marker_.scale.y = this->groups[i].ex_group_length_;//minor vap
  group_marker_.scale.z = 0.2;
  double yaw = std::atan2(this->groups[i].ex_central_group_.vy,this->groups[i].ex_central_group_.vx);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);  // Set roll and pitch to 0, yaw to your desired value
  geometry_msgs::msg::Quaternion myQ = tf2::toMsg(q);
  group_marker_.pose.orientation = myQ;  
  group_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);
}

void PeopleTrackingMhtAlgNode::ini_markers()
{
  std::cout<<" ini_markers "<<std::endl;
  // ini id tracks markers
  id_track_marker_.header = this->trackingArray_msg_.header;
  id_track_marker_.ns     = "id";
  id_track_marker_.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  id_track_marker_.action = visualization_msgs::msg::Marker::ADD;
  id_track_marker_.pose.orientation.x=0.0;
  id_track_marker_.pose.orientation.y=0.0;
  id_track_marker_.pose.orientation.z=0.0;
  id_track_marker_.pose.orientation.w=1.0;
  id_track_marker_.scale.x = 0.6;
  id_track_marker_.scale.y = 0.6;
  id_track_marker_.scale.z = 0.6;
  id_track_marker_.color.r = 0.0;
  id_track_marker_.color.g = 0.0;
  id_track_marker_.color.b = 0.0;
  id_track_marker_.color.a = 1.0;
  id_track_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);
  
  // ini id cluster markers
  id_cluster_marker_.header = this->trackingArray_msg_.header;
  id_cluster_marker_.ns     = "id_cluster";
  id_cluster_marker_.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  id_cluster_marker_.action = visualization_msgs::msg::Marker::ADD;
  id_cluster_marker_.pose.orientation.x=0.0;
  id_cluster_marker_.pose.orientation.y=0.0;
  id_cluster_marker_.pose.orientation.z=0.0;
  id_cluster_marker_.pose.orientation.w=1.0;
  id_cluster_marker_.scale.x = 0.6;
  id_cluster_marker_.scale.y = 0.6;
  id_cluster_marker_.scale.z = 0.6;
  id_cluster_marker_.color.r = 0.0;
  id_cluster_marker_.color.g = 0.0;
  id_cluster_marker_.color.b = 1.0;
  id_cluster_marker_.color.a = 1.0;
  id_cluster_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);

  // ini velocity markers
  velocity_track_marker_.header = this->trackingArray_msg_.header;
  velocity_track_marker_.ns     = "vel";
  velocity_track_marker_.type   = visualization_msgs::msg::Marker::ARROW;
  velocity_track_marker_.action = visualization_msgs::msg::Marker::ADD;
  velocity_track_marker_.color.r = 1.0;
  velocity_track_marker_.color.g = 1.0;
  velocity_track_marker_.color.b = 0.0;
  velocity_track_marker_.color.a = 0.7;
  velocity_track_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);

  // ini covariance markers TRACKS (body cov track marckers)
  cov_track_marker_.header = this->trackingArray_msg_.header;
  cov_track_marker_.ns     = "cov_track";  
  if(cargo_ANTS){
  	cov_track_marker_.type   = visualization_msgs::msg::Marker::CUBE;
  }else{
  	cov_track_marker_.type   = visualization_msgs::msg::Marker::CYLINDER;
  }  
  cov_track_marker_.action = visualization_msgs::msg::Marker::ADD;
  cov_track_marker_.color.r = 0.0;
  cov_track_marker_.color.g = 1.0;
  cov_track_marker_.color.b = 0.0;
  cov_track_marker_.color.a = 0.7;
  cov_track_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);
  
	// ini covariance markers TRACKS (head cov track marckers)
  cov_track_marker_head_.header = this->trackingArray_msg_.header;
  cov_track_marker_head_.ns     = "cov_track_head";
  cov_track_marker_head_.type   = visualization_msgs::msg::Marker::SPHERE;
  cov_track_marker_head_.action = visualization_msgs::msg::Marker::ADD;
  cov_track_marker_head_.color.r = 0.0;
  cov_track_marker_head_.color.g = 1.0;
  cov_track_marker_head_.color.b = 0.0;
  cov_track_marker_head_.color.a = 0.7;
  cov_track_marker_head_.lifetime = rclcpp::Duration::from_seconds(0.1f);
  
  // ini covariance markers TRACKS (arm1 cov track marckers)
  cov_track_marker_arm1_.header = this->trackingArray_msg_.header;
  cov_track_marker_arm1_.ns     = "cov_track_arm1";
  cov_track_marker_arm1_.type   = visualization_msgs::msg::Marker::ARROW;
  cov_track_marker_arm1_.action = visualization_msgs::msg::Marker::ADD;
  cov_track_marker_arm1_.color.r = 0.0;
  cov_track_marker_arm1_.color.g = 1.0;
  cov_track_marker_arm1_.color.b = 0.0;
  cov_track_marker_arm1_.color.a = 0.4;
  cov_track_marker_arm1_.lifetime = rclcpp::Duration::from_seconds(0.1f);
  
  // ini covariance markers TRACKS (arm2 cov track marckers)
  cov_track_marker_arm2_.header = this->trackingArray_msg_.header;
  cov_track_marker_arm2_.ns     = "cov_track_arm2";
  cov_track_marker_arm2_.type   = visualization_msgs::msg::Marker::ARROW;
  cov_track_marker_arm2_.action = visualization_msgs::msg::Marker::ADD;
  cov_track_marker_arm2_.color.r = 0.0;
  cov_track_marker_arm2_.color.g = 1.0;
  cov_track_marker_arm2_.color.b = 0.0;
  cov_track_marker_arm2_.color.a = 0.4;
  cov_track_marker_arm2_.lifetime = rclcpp::Duration::from_seconds(0.1f);
   
  // ini covariance markers DETECTIONS
  cov_detection_marker_.header = this->trackingArray_msg_.header;
  cov_detection_marker_.ns     = "cov_detection";
  cov_detection_marker_.type   = visualization_msgs::msg::Marker::CYLINDER;
  cov_detection_marker_.action = visualization_msgs::msg::Marker::ADD;
  cov_detection_marker_.color.r = 1.0;
  cov_detection_marker_.color.g = 0.0;
  cov_detection_marker_.color.b = 0.0;
  cov_detection_marker_.color.a = 1.0;
  cov_detection_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);

  // ini covariance markers PREDICTIONS
  cov_prediction_marker_.header = this->trackingArray_msg_.header;
  cov_prediction_marker_.ns     = "cov_prediction";
  cov_prediction_marker_.type   = visualization_msgs::msg::Marker::CYLINDER;
  cov_prediction_marker_.action = visualization_msgs::msg::Marker::ADD;
  cov_prediction_marker_.scale.z = 0.1;
  cov_prediction_marker_.color.r = 0.0;
  cov_prediction_marker_.color.g = 0.0;
  cov_prediction_marker_.color.b = 1.0;
  cov_prediction_marker_.color.a = 0.4;
  cov_prediction_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);
   
  // ini markers CLUSTERS
  cluster_marker_.header = this->trackingArray_msg_.header;
  cluster_marker_.ns     = "cluster_size";
  cluster_marker_.type   = visualization_msgs::msg::Marker::CYLINDER;
  cluster_marker_.action = visualization_msgs::msg::Marker::ADD;
  cluster_marker_.scale.z = 0.1;
  cluster_marker_.color.a = 0.2; 
  cluster_marker_.lifetime = rclcpp::Duration::from_seconds(0.1f);
  
  // ini markers GRUPOS
  group_marker_.header = this->trackingArray_msg_.header;
  group_marker_.ns     = "group_size";
  group_marker_.type   = visualization_msgs::msg::Marker::CUBE;
  group_marker_.action = visualization_msgs::msg::Marker::ADD;
  group_marker_.scale.z = 0.1;     
}
