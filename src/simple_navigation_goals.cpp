#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navigation_goals");

//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
}

	move_base_msgs::MoveBaseGoal goal;

//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = 1.4;
	goal.target_pose.pose.position.y = 0.3;
//	goal.target_pose.pose.orientation.w = 1.0;

//We want to convert the Euler angle to quaternion
//	double radians = 87.0 * (M_PI/180);
	double radians = 1.40;
	tf::Quaternion quaternion;
	quaternion = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(quaternion, qMsg);

	goal.target_pose.pose.orientation = qMsg;

	ROS_INFO("Sending goal #1!");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, goal #1 completed!");
	else
		ROS_INFO("Uh oh, something went wrong :/");

//	goal.target_pose.pose.orientation.w = 0.5;

//	ROS_INFO("Orienting!");
//	ac.sendGoal(goal);
//	ac.waitForResult();

//	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//		ROS_INFO("Hooray, 1st orientation correct!");
//	else
//		ROS_INFO("Uh oh, something went wrong :/");

	goal.target_pose.pose.position.x = 1.4;
	goal.target_pose.pose.position.y = 1.4;
//	goal.target_pose.pose.orientation.w = 0.5;

	radians = 3.05;
	tf::Quaternion quaternion2;
	quaternion2 = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg2;
	tf::quaternionTFToMsg(quaternion2, qMsg2);

	goal.target_pose.pose.orientation = qMsg2;

	ROS_INFO("Sending goal #2");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, goal #2 completed!");
	else
		ROS_INFO("Uh oh, something went wrong :/");

//	goal.target_pose.pose.orientation.w = -1.0;

//	ROS_INFO("Orienting!");
//	ac.sendGoal(goal);
//	ac.waitForResult();

//	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//		ROS_INFO("Hooray, 2nd orientation correct!");
//	else
//		ROS_INFO("Uh oh, something went wrong :/");


	goal.target_pose.pose.position.x = 0.3;
	goal.target_pose.pose.position.y = 1.4;
//	goal.target_pose.pose.orientation.w = -1.0;

	radians = -1.40;
	tf::Quaternion quaternion3;
	quaternion3 = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg3;
	tf::quaternionTFToMsg(quaternion3, qMsg3);

	goal.target_pose.pose.orientation = qMsg3;

	ROS_INFO("Sending goal #3");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, goal #3 completed!");
	else
		ROS_INFO("Uh oh, something went wrong :/");

//	goal.target_pose.pose.orientation.w = -0.5;

//	ROS_INFO("Orienting!");
//	ac.sendGoal(goal);
//	ac.waitForResult();

//	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//		ROS_INFO("Hooray, 3rd orientation correct!");
//	else
//		ROS_INFO("Uh oh, something went wrong :/");

	goal.target_pose.pose.position.x = 0.3;
	goal.target_pose.pose.position.y = 0.3;
//	goal.target_pose.pose.orientation.w = 1.0;

	radians = 0.0;
	tf::Quaternion quaternion4;
	quaternion4 = tf::createQuaternionFromYaw(radians);

	geometry_msgs::Quaternion qMsg4;
	tf::quaternionTFToMsg(quaternion4, qMsg4);

	goal.target_pose.pose.orientation = qMsg4;

	ROS_INFO("Sending final goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, all turn goals complete!");
	else
		ROS_INFO("Uh oh, final goal failed for some reason");

//	goal.target_pose.pose.orientation.w = 1.0;

//	ROS_INFO("Orienting!");
//	ac.sendGoal(goal);
//	ac.waitForResult();

//	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//		ROS_INFO("Hooray, final orientation correct!");
//	else
//		ROS_INFO("Uh oh, something went wrong :/");

	return 0;
}
