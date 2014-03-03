/*
 *  dumbo_velocity_integrator.h
 *
 *
 *  Created on: Mar 3, 2014
 *  Authors:   Yuquan Wang
 *            yuquan <at> kth.se
 */

/* Copyright (c) 2014, Yuquan Wang, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DUMBO_VELOCITY_INTEGRATOR_H
#define DUMBO_VELOCITY_INTEGRATOR_H

#include <stdio.h>
#include <stdlib.h>
#include <exception>
#include <ros/ros.h>


#include <brics_actuator/JointVelocities.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

enum{
  Joints = 7
};

using namespace std;

class dumbo_velocity_integrator{

 private:
  // declaration of topics to publish (For Multi_obj_control )
  bool control_is_initialized_;
  
  ros::Publisher topicpub_joint_l_;
  ros::Publisher topicpub_joint_r_;
  
  control_msgs::JointTrajectoryControllerState controller_state_l_msg_;
  control_msgs::JointTrajectoryControllerState controller_state_r_msg_;

  // declaration of topics to publish (For Visualization )
  ros::Publisher topicpub_joint_states_;
  sensor_msgs::JointState joint_states_msg_;  

  // declaration of topics to subscribe, callback is called for new messages arriving
  ros::Subscriber topicsub_joint_v_l_;
  ros::Subscriber topicsub_joint_v_r_;
  

  ros::Time m_Joint_v_stamp_l_;
  ros::Time m_Joint_v_stamp_r_;
  ros::Time m_Joint_states_stamp_;
  
  std::vector<std::string> m_JointNames;
  std::vector<double> Joint_v_l_;
  std::vector<double> Joint_v_r_;
  std::vector<double> Joint_v_;

  std::vector<double> Joint_v_buffer_l_;
  std::vector<double> Joint_v_buffer_r_;

  std::vector<double> Joint_pos_l_;
  std::vector<double> Joint_pos_r_;
  std::vector<double> Joint_pos_;

  std::vector<double> Joint_pos_l_last_;
  std::vector<double> Joint_pos_r_last_;
  std::vector<double> Joint_pos_last_;

  std::vector<double> Dual_dof_initial_;
  
  bool m_received_js_v_l_;
  bool m_received_js_v_r_;
  
  void getROSParameters();

  void topicCallback_joint_v_l_(const brics_actuator::JointVelocities::ConstPtr& msg);
  void topicCallback_joint_v_r_(const brics_actuator::JointVelocities::ConstPtr& msg);

  struct publish_pr2_thread_data{
    control_msgs::JointTrajectoryControllerState  * joint_vel_msg_ptr;
    ros::Publisher * publisher_ptr;
  };
  struct publish_sensor_thread_data{
    sensor_msgs::JointState  * joint_vel_msg_ptr;
    ros::Publisher * publisher_ptr;
  };

  static void * publish_arm_velocity(void * data);
  static void * publish_robot_velocity(void * data);
  
 public:
  ros::NodeHandle n_s;
  bool params_OK;
  unsigned int m_DOF; 
  double Sampling_peroid_;
  ros::Time last_publish_time_;

  void publish_joints();    
  bool one_step_forward();
  
  dumbo_velocity_integrator(){
    //    n_s = ros::NodeHandle("dumbo_velocity_integrater_node");
    n_s = ros::NodeHandle("~");
    // This is related to some Namespace stuff. Go and check it later 
    
    params_OK = true;
    m_received_js_v_l_ = false;
    m_received_js_v_r_ = false;
    control_is_initialized_ = false;
    getROSParameters();
    
    // Publish the joint velocities to left and right arms 
    topicpub_joint_l_ = n_s.advertise<control_msgs::JointTrajectoryControllerState>("/left_arm_controller/state", 1);
    topicpub_joint_r_ = n_s.advertise<control_msgs::JointTrajectoryControllerState>("/right_arm_controller/state", 1);

    // publish the joint_states to the "robot state pulisher"
    topicpub_joint_states_ = n_s.advertise<sensor_msgs::JointState> ("/joint_states", 1);    
    // Subscribe the joint velocities to left and right arms 
    topicsub_joint_v_l_ = n_s.subscribe("/left_arm_controller/command_vel", 10, &dumbo_velocity_integrator::topicCallback_joint_v_l_, this);         
    topicsub_joint_v_r_ = n_s.subscribe("/right_arm_controller/command_vel", 10, &dumbo_velocity_integrator::topicCallback_joint_v_r_, this);         
  }
  ~dumbo_velocity_integrator(){  

  }

};

#endif
