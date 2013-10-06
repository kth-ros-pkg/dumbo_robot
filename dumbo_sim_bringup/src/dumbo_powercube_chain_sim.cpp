/*
 * dumbo_powercube_chain_sim.cpp
 *
 *  Created on: Feb 9, 2013
 *  Authors:   Francisco Viña   
 *            fevb <at> kth.se  
 */

/* Copyright (c) 2013, Francisco Vina, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
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


// ROS includes
#include <ros/ros.h>
// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>


class PowerCubeChainSimNode
{
public:
	ros::NodeHandle n_;

	// for the left arm
	ros::Publisher topicPub_OperationMode_l;
	ros::ServiceServer srvServer_SetOperationMode_l;

	// for the right arm
	ros::Publisher topicPub_OperationMode_r;
	ros::ServiceServer srvServer_SetOperationMode_r;

	ros::Time last_publish_time_;

	PowerCubeChainSimNode()
	{
		n_ = ros::NodeHandle("");
		topicPub_OperationMode_l = n_.advertise<std_msgs::String> ("/left_arm_controller/current_operationmode", 1);
		topicPub_OperationMode_r = n_.advertise<std_msgs::String> ("/right_arm_controller/current_operationmode", 1);

		srvServer_SetOperationMode_l = n_.advertiseService("/left_arm_controller/set_operation_mode", &PowerCubeChainSimNode::srvCallback_SetOperationMode, this);
		srvServer_SetOperationMode_r = n_.advertiseService("/right_arm_controller/set_operation_mode", &PowerCubeChainSimNode::srvCallback_SetOperationMode, this);

		last_publish_time_ = ros::Time::now();
	}

	~PowerCubeChainSimNode()
	{

	}

	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res)
	{

		ROS_DEBUG("Set Operation Mode Service");

		if(req.operation_mode.data != "position" && req.operation_mode.data != "velocity")
		{
			res.success.data = false;
		}
		else
		{
			res.success.data = true;
		}


		return true;
	}

	void PublishOpMode()
	{
		std_msgs::String opmode_msg;
		opmode_msg.data = "velocity";

		topicPub_OperationMode_l.publish(opmode_msg);
		topicPub_OperationMode_r.publish(opmode_msg);

	}


};


int main(int argc, char **argv) {

	ros::init(argc, argv, "dumbo_powercube_chain_sim");

	PowerCubeChainSimNode pc_sim_node;


	ros::Duration min_publish_duration;
	min_publish_duration.fromSec(0.1);

	ros::Rate loop_rate(100.0);


	while(pc_sim_node.n_.ok())
	{

		if((ros::Time::now() - pc_sim_node.last_publish_time_) >= min_publish_duration)
			pc_sim_node.PublishOpMode();

		ros::spinOnce();
		loop_rate.sleep();
	}







	return 0;
}

