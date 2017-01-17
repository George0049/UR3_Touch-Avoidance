#include <ros/ros.h>
#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_kinematic_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup kinematic_group("manipulator");
	robot_state::RobotStatePtr kinematic_state = kinematic_group.getCurrentState();

	robot_model::RobotModelConstPtr kinematic_model = kinematic_state->getRobotModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
	
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model_ik = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state_ik(new robot_state::RobotState(kinematic_model_ik));
	kinematic_state_ik->setToDefaultValues();
	const robot_state::JointModelGroup *joint_model_group_ik = kinematic_model_ik->getJointModelGroup("manipulator");

	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
//	kinematic_state_ik->copyJointGroupPositions(joint_model_group_ik, joint_values);
	for(std::size_t i = 0; i < joint_names.size(); i++)
	{
		ROS_INFO("Joint %s:%f", joint_names[i].c_str(), joint_values[i]);
	}
//Eigen, 开源矩阵算法库
	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("ee_link");
	ROS_INFO_STREAM("Translation: "<<end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: "<<end_effector_state.rotation());

	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

	std::vector<double> joint_values_ik;
	if(found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values_ik);
		for(std::size_t i = 0; i < joint_names.size(); i++)
		{
			ROS_INFO("Joint %s:%f", joint_names[i].c_str(), joint_values_ik[i]);
		}
	}
	else 
	{
		ROS_INFO("Did not find IK solution");
	}


	ros::shutdown();
	return 0;
}
