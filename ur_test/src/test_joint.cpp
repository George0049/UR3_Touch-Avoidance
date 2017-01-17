#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
	std::string target_name = "ready";
	if(argc > 1)
		target_name = argv[1];

	ros::init(argc, argv, "test_joint_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	std::vector<double> group_variable_values;
	group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	for(std::size_t i = 0; i < group_variable_values.size(); i++)
		ROS_INFO("joint %d: %f",int(i) , group_variable_values[i]);

	group.rememberJointValues("current", group_variable_values);

	group_variable_values[0] = -0.0;
	group_variable_values[1] = -1.5708;
	group_variable_values[2] = 1.5708;
	group_variable_values[3] = -1.5708;
	group_variable_values[4] = -1.5708;
	group_variable_values[5] = 0.0;

	group.rememberJointValues("ready", group_variable_values);

	for(int i = 0; i < 6; ++i)
		group_variable_values[i] = 0;

	group.rememberJointValues("zero", group_variable_values);
//	group.setJointValueTarget(group_variable_values);
	group.setNamedTarget(target_name);	
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);
	bool success2 = group.execute(my_plan);
	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");
	ROS_INFO("Visuallizing execute 1 (pose goal) %s", success2?"":"FAILED");

	sleep(1.0);
	ros::shutdown();
	return 0;
}
