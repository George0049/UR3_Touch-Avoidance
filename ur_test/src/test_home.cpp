#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/topic.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <sensor_msgs/JointState.h>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_home_node");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//获取机器人当前姿态信息
	boost::shared_ptr<const sensor_msgs::JointState> mmm = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", node_handle, ros::Duration(0.5));
	
	//实例化各种类
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	//新建机器人状态传给planning_scene
	robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
	std::vector<double> joint_val(mmm->position);
	double temp = joint_val[0];
	joint_val[0] = joint_val[2];
	joint_val[2] = temp;
	
	robot_state.setJointGroupPositions("manipulator", joint_val);
	planning_scene->setCurrentState(robot_state);
	for(std::size_t i = 0; i < joint_val.size(); ++i)
	{
		ROS_INFO("Joint [%d]: %f", (int)i, joint_val[i]);
	}
	
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	if(!node_handle.getParam("/test_home_node/planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Count not find planner name");
	ROS_INFO("%s",planner_plugin_name.c_str() );
	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
 	}
	try
	{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if(!planner_instance->initialize(robot_model, node_handle.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planning interface ''" << planner_instance->getDescription() << "''");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for(std::size_t i=0; i<classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "'" << ex.what() << std::endl << "Available plugins: " << ss.str());
	}

	ros::WallDuration sleep_time(5.0);
	sleep_time.sleep();

	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_link";

	pose.pose.position.x = 0.4218;
	pose.pose.position.y = 0.1942;
	pose.pose.position.z = 0.1965;
	pose.pose.orientation.x = -0.211307;
	pose.pose.orientation.y = 0.467779;
	pose.pose.orientation.z = 0.640848;
	pose.pose.orientation.w = 0.570828;

	geometry_msgs::PoseStamped pose2 = pose;
	pose2.pose.position.z = 0.1;

	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	//添加障碍物
	moveit_msgs::CollisionObject flat;
	flat.id = "ur_add_flat";

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[primitive.BOX_X] = 0.05;
	primitive.dimensions[primitive.BOX_Y] = 0.2;
	primitive.dimensions[primitive.BOX_Z] = 0.01;

	geometry_msgs::Pose pose_collison;
	pose_collison.orientation.w = 1.0;
	pose_collison.position.x =  0.3;
	pose_collison.position.y =  0.2;
	pose_collison.position.z =  0.26;

	flat.primitives.push_back(primitive);
	flat.primitive_poses.push_back(pose_collison);
	flat.operation = flat.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(flat);

	planning_scene->processCollisionObjectMsg(flat);
	//执行规划
	req.group_name = "manipulator";
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_link", pose, tolerance_pose, tolerance_angle);
	moveit_msgs::Constraints pose_goal2 = kinematic_constraints::constructGoalConstraints("ee_link", pose2, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal2);
	req.goal_constraints.push_back(pose_goal);
 
	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

	context->solve(res);
	if(res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("visual the trajectory");
	moveit_msgs::MotionPlanResponse respone;
	res.getMessage(respone);

	display_trajectory.trajectory_start = respone.trajectory_start;
	display_trajectory.trajectory.push_back(respone.trajectory);
	display_publisher.publish(display_trajectory);


	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::MoveGroup::Plan my_plan;

	my_plan.start_state_ = respone.trajectory_start;
	my_plan.trajectory_ = respone.trajectory;

//	moveit::planning_interface::MoveGroup group("manipulator");
//	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


//	std::vector<geometry_msgs::Pose> waypoints;
//	geometry_msgs::Pose taret_pose3 = taret_pose2;
//	taret_pose3.position.x += 0.2;
//	taret_pose3.position.z += 0.2;
//	waypoints.push_back(taret_pose3);
//	
//	taret_pose3.position.y -= 0.2;
//	waypoints.push_back(taret_pose3);
//
//	taret_pose3.position.z -= 0.2;
//	taret_pose3.position.y += 0.2;
//	taret_pose3.position.x -= 0.2;
//	waypoints.push_back(taret_pose3);
//
//	moveit_msgs::RobotTrajectory trajectory;
//	double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
//	ROS_INFO("Visulizing plan 4 (cartesian path) (%.2f%% achived)", fraction * 100.0);
//	group.setPoseTarget(taret_pose2);
//
//	moveit::planning_interface::MoveGroup::Plan my_plan;
//	bool success = group.move();
	bool success2 = group.execute(my_plan);
//	ROS_INFO("Visuallizing plan 1 (pose goal) %s", success?"":"FAILED");
	ROS_INFO("Visuallizing execute 1 (pose goal) %s", success2?"":"FAILED");

	sleep(1.0);
	ros::shutdown();
	return 0;
}
