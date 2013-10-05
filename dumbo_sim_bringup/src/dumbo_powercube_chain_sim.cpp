/*
 * dumbo_powercube_chain_sim.cpp
 *
 *  Created on: Feb 9, 2013
 *      Author: fevb
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

