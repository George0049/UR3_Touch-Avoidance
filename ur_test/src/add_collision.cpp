#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "add_collision_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // We obtain the current planning scene and wait until everything is up
		moveit::planning_interface::PlanningSceneInterface current_scene;
		sleep(3.0);

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

		current_scene.addCollisionObjects(collision_objects);

		ros::shutdown();
		return 0;
}
