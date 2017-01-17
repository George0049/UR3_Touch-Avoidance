#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <stdlib.h>

#include <iostream>


//--------------------------
#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>
//------------------------------

//------publisher------

//#include "ros/ros.h"
//#include <geometry_msgs/Twist.h> // For geometry_msgs:: Twist
//#include <stdlib.h> // For rand() and RAND_MAX

int main ( int argc , char ** argv ) 
{
    // Initialize the ROS system and become a node .
    ros::init ( argc , argv , "publish_force_node" ) ;
    ros::NodeHandle nh ;

    // Create a publisher obj ect .
    ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("ur3_force" , 10000 ) ;

    // Seed the random number generator .
    srand (time(0)) ;

    // Loop at 2Hz until the node is shut down.
    ros::Rate rate (1) ;
    while (ros::ok ()) 
    {
        // Create and fill in the message . The other four
        // fields , which are ignored by turtl esim , default to 0.
        geometry_msgs::Pose msg ;

        msg.position.x = 0.314288;
	    msg.position.y = 0.153859;
    	msg.position.z = 0.226919;
        msg.orientation.x = 0.03678;
      	msg.orientation.y = 0.677712;
    	msg.orientation.z = 0.039687;
    	msg.orientation.w = 0.733334;

        msg.position.x = double(rand())/double(RAND_MAX) ;
        msg.position.y = double(rand())/double(RAND_MAX) ;
        msg.position.z = double(rand())/double(RAND_MAX) ;

        // Publish the message .
        pub.publish(msg) ;

        // Send a message to rosout with the details .
        ROS_INFO_STREAM("Sending random force command : "<<" position_x="<<msg.position.x<<" position_y="<<msg.position.y<<" position_z="<<msg.position.z);

        // Wait untilit's time for another iteration .
        rate.sleep ( ) ;
    }

    return 0;
}

/*
int main(int argc,char**argv)
{
    //initialize the ROS system and become a node
    ros::init(argc,argv,"publish_force");
    ros::NodeHandle nh;

    //creat a pulisher object
    //ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("ur3/force",1000);
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    //seed the random number generator.
    srand (time(0));

    //loop at 2Hz until the node is shut down 
    ros::Rate rate(2);
    int count =0;
msg
    while (ros::ok)
    {
        //geometry_msgs::Pose msg;
        std_msgs::String msg;
        std::stringstream ss;
        ss<<"0.1"<<count;
        //=double(rand())/double(RAND_MAX);
        msg.data = ss.str();
        //msg.angular.z=2*double(rand())/double(RAND_MAX)-1;

        ROS_INFO("%s",msg.data.c_str());

        //publish the msg
        chatter_pub.publish(msg);
        ros::spinOnce();
        //send a message to rosout with the details 
        //ROS_INFO_STREAM("Sending random force command:"<<"linear="<<msg.linear.x<<"argular="<<msg.angular.z);

        //what untillt's time for another iteration
        rate.sleep();
        ++count;    
    }


    //std::cout<<"123"<<std::endl;
    return 0;
}
*/