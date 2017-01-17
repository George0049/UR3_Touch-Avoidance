#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


void tcpMove(const geometry_msgs::Pose& msg_pose)
{
	moveit::planning_interface::MoveGroup group("manipulator");
	std::cout<<msg_pose.position.x<<msg_pose.position.y<<msg_pose.position.z<<msg_pose.orientation.w<<msg_pose.orientation.x<<msg_pose.orientation.y<<msg_pose.orientation.z<<std::endl;
	geometry_msgs::Pose msg_pose0 = msg_pose;
	geometry_msgs::Pose msg_pose2 = msg_pose;
	msg_pose0.position.z += 0.1;
	msg_pose2.position.x -= 0.1;

	std::cout<<"move"<<std::endl;
	group.setPoseTarget(msg_pose0);
	bool success = group.move();

	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");

	group.setPoseTarget(msg_pose);
	success = group.move();

	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");

	group.setPoseTarget(msg_pose0);
	success = group.move();

	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");

	group.setPoseTarget(msg_pose2);
	success = group.move();

	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_custom_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	ros::Subscriber tcp_sub = node_handle.subscribe("ur/tcp",100,&tcpMove);
	moveit_msgs::DisplayTrajectory display_trajectory;

//	std::vector<double> group_variable_values;
	std::cout<<"DefualtPlannerID: "<<group.getDefaultPlannerId("manipulator")<<std::endl;

//	group_variable_values[0] = -0.0;
//	group_variable_values[1] = -1.5708;
//	group_variable_values[2] = 1.5708;
//	group_variable_values[3] = -1.5708;
//	group_variable_values[4] = -1.5708;
//	group_variable_values[5] = 0.0;
	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = 0.2218;
	target_pose1.position.y = 0.1942;
	target_pose1.position.z = 0.1965;
	target_pose1.orientation.x = -0.211307;
	target_pose1.orientation.y = 0.467779;
	target_pose1.orientation.z = 0.640848;
	target_pose1.orientation.w = 0.570828;
	group.setPoseTarget(target_pose1);

//	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.move();

	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");

	ros::spin();
	return 0;
}
