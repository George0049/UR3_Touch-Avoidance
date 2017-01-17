#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>

#include<ros/ros.h>
#include<iomanip>
//
#include <geometry_msgs/Twist.h>
//#include<test_force.cpp>

//subscribes 

geometry_msgs::Pose target_pose1;

int ix,iy,iz;

void poseMessageReceived ( const geometry_msgs::Pose& msg ) 
{
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	std::cout<<"positon_x="<<msg.position.x<<" position_y="<<msg.position.y<<" position_z="<<msg.position.z<<std::endl;

//------------------------------------------------
	ix=1-rand()%3;
	iy=1-rand()%3;
	iz=1-rand()%3;

	target_pose1.position.x = 0.314288;
	target_pose1.position.y = 0.153859;
	target_pose1.position.z = 0.226919;
	target_pose1.orientation.x = 0.03678;
	target_pose1.orientation.y = 0.677712;
	target_pose1.orientation.z = 0.039687;
	target_pose1.orientation.w = 0.733334;

	group.setPoseTarget(target_pose1);
	bool success0 = group.move();
	sleep(0.5);
//------------------------------------------------


	target_pose1.position.x = 0.314288 +ix*msg.position.x/10;
	target_pose1.position.y = 0.153859 +iy*msg.position.y/10;
	target_pose1.position.z = 0.226919 +iz*msg.position.z/10;
	group.setPoseTarget(target_pose1);
	bool success = group.move();
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
	moveit_msgs::DisplayTrajectory display_trajectory;

	std::cout<<"DefualtPlannerID: "<<group.getDefaultPlannerId("manipulator")<<std::endl;


//-----------------------------------------------------
	ros::Subscriber sub= node_handle.subscribe("ur3_force",1000,&poseMessageReceived);

	// ros::spin();

//-------------------------------------------------------------------------------
	//target_pose1.position.x += fixed;

/*
	target_pose1.position.x = 0.314288;
	target_pose1.position.y = 0.153859;
	target_pose1.position.z = 0.226919;
	target_pose1.orientation.x = 0.03678;
	target_pose1.orientation.y = 0.677712;
	target_pose1.orientation.z = 0.039687;
	target_pose1.orientation.w = 0.733334;

	group.setPoseTarget(target_pose1);
	bool success = group.move();
	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");
*/	


	sleep(1.0);
	
	ros::spin();
	// ros::shutdown();
	return 0;
}
