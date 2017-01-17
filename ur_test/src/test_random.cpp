#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_random_node", ros::init_options::AnonymousName);
	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();
	// this connecs to a running instancd of the move_group node 
	move_group_interface::MoveGroup group("manipulator");
	// specify that our targert will be a randow one
	group.setRandomTarget();
	// plan the motion and then move the group to the sampled target
	group.move();
	ros::waitForShutdown();
}
