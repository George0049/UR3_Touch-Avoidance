#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_collision_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	moveit_msgs::CollisionObject flat;
	flat.id = "ur_add_flat";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.05;
	primitive.dimensions[primitive.BOX_Y] = 0.2;
	primitive.dimensions[primitive.BOX_Z] = 0.01;

	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x =  0.3;
	pose.position.y =  0.2;
	pose.position.z =  0.26;

	flat.primitives.push_back(primitive);
	flat.primitive_poses.push_back(pose);
	flat.operation = flat.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(flat);

	planning_scene_interface.addCollisionObjects(collision_objects);
	sleep(2);

	group.setPlanningTime(10);

	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = 0.4218;
	target_pose1.position.y = 0.1942;
	target_pose1.position.z = 0.0965;
	target_pose1.orientation.x = -0.211307;
	target_pose1.orientation.y = 0.467779;
	target_pose1.orientation.z = 0.640848;
	target_pose1.orientation.w = 0.570828;
	group.setStartState(*group.getCurrentState());
	group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.move();
	//	bool success = group.plan(my_plan);
	//	bool success2 = group.execute(my_plan);
	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");
	//	ROS_INFO("Visuallizing execute 1 (pose goal) %s", success2?"":"FAILED");

	sleep(1.0);
	ros::shutdown();
	return 0;
}
