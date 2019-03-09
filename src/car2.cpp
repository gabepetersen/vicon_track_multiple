// Gabe Petersen
// car1.cpp - 2 Mar 2019
// Purpose: to get pose from vicon object and keep update on collisions of other objects

#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Pose2D.h"																		
#include "geometry_msgs/Twist.h"		
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"	

class ViconTrack {
	public:
		// Default constructor to connect topics
		ViconTrack() {
			// used to get the pose of the duckiebot to transfer to turtlesim
			vicon_stream = nh.subscribe("vicon/car2/car2", 5, &ViconTrack::vs_cb, this);
			// register sub to get current position/pose
			car1_sub = nh.subscribe("/car1/pose", 5, &ViconTrack::c1_cb, this);
			car3_sub = nh.subscribe("/car3/pose", 5, &ViconTrack::c3_cb, this);
			// register pub to send twist velocity (cmd_vel)
			pose_pub = nh.advertise<geometry_msgs::Pose2D>("/car2/pose", 100);
		}
		// Function declarations
		void vs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void c1_cb(const geometry_msgs::Pose2D::ConstPtr& msg);
		void c3_cb(const geometry_msgs::Pose2D::ConstPtr& msg);

	private:
		// ros variables
		ros::NodeHandle nh;
		ros::Subscriber vicon_stream;
		ros::Subscriber car1_sub;
		ros::Subscriber car3_sub;
		ros::Publisher pose_pub;
				
		// helper variable to track movement									
		geometry_msgs::Pose2D CurPose;	
};

int main(int argc, char** argv) {
	// initialize the node
	ros::init(argc, argv, "car2");
	// instantiate the class
	ViconTrack vt;
	
	ros::spin();	
	return 0;
}

// call back function to update pose of vicon object
void ViconTrack::vs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	// get message in TransformStamped
	geometry_msgs::TransformStamped tfs_data = *msg;
	// get quaternion from TransformStamped and normalize
	tf2::Quaternion myRotation;
	tf2::convert(tfs_data.transform.rotation, myRotation);
	myRotation.normalize();
	// get z-angle and x and y positions of transform
	tf2::Matrix3x3 m(myRotation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	CurPose.theta = yaw;
	/*******************************************************************************/
	/********* Adjust additions for current vicon configuration ********************/
	/*******************************************************************************/
	CurPose.x = tfs_data.transform.translation.x;
	CurPose.y = tfs_data.transform.translation.y;
	// Publish the Current Pose of car1
	pose_pub.publish(CurPose);
}
// call back function to track if there are collisions with car2
void ViconTrack::c1_cb(const geometry_msgs::Pose2D::ConstPtr& msg)			
{
	geometry_msgs::Pose2D car2Pose = *msg;		
	float x_dist = CurPose.x - car2Pose.x;
	float y_dist = CurPose.y - car2Pose.y;	
	/// declare threshold values for collision boundaries		
	if((x_dist <= 0.15 && x_dist >= -0.15)  && (y_dist <= 0.1 && y_dist >= -0.1)) {
		ROS_INFO("Car 1 is Colliding with Car 2!");
	}		
	return;
}
// call back function to track if there are collisions with car3
void ViconTrack::c3_cb(const geometry_msgs::Pose2D::ConstPtr& msg)			
{
	geometry_msgs::Pose2D car3Pose = *msg;		
	float x_dist = CurPose.x - car3Pose.x;
	float y_dist = CurPose.y - car3Pose.y;	
	/// declare threshold values for collision boundaries		
	if((x_dist <= 0.15 && x_dist >= -0.15)  && (y_dist <= 0.1 && y_dist >= -0.1)) {
		ROS_INFO("Car 1 is Colliding with Car 3!");
	}		
	return;
}
