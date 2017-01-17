#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_move_group");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("manipulator");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
	planning_interface::PlannerManagerPtr planner_instance;
	std::string planner_plugin_name;

	/*
	 * 
	 *
	if(!node_handle.getParam("planning_plugin", planner_plugin_name))
		ROS_FATAL_STREAM("Could not find planner plugin name ");
	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_FATAL_STREAM("Exception while creating planning plugin loader" << ex.what());
	}
	try
	{
		planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
		if(!planner_instance->initialize(robot_model, node_handle.getNamespace()))
			ROS_FATAL_STREAM("Could not initialize planner instance");
		ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
	}
	catch(pluginlib::PluginlibException& ex)
	{
		const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
		std::stringstream ss;
		for (std::size_t i = 0; i < classes.size(); ++i)
			ss << classes[i] << " ";
		ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
	}
	 * *********
	*/

	ros::WallDuration sleep_time(5.0);
	sleep_time.sleep();

	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "ee_link";
	pose.pose.position.x = 0.4218;
	pose.pose.position.y = 0.1942;
	pose.pose.position.z = 0.0965;
	pose.pose.orientation.x = -0.211307;
	pose.pose.orientation.y = 0.467779;
	pose.pose.orientation.z = 0.640848;
	pose.pose.orientation.w = 0.570828;

	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	req.group_name = "manipulator";
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_link", pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
	context->solve(res);
	if(res.error_code_.val != res.error_code_.SUCCESS )
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);

	moveit::planning_interface::MoveGroup::Plan myplan;
	myplan.start_state_ = response.trajectory_start;
	moveit_msgs::RobotTrajectory robottra = response.trajectory;
	myplan.trajectory_ = robottra;
	group.plan(myplan);
	group.execute(myplan);

	sleep_time.sleep();


	ros::shutdown();
	return 0;
}
