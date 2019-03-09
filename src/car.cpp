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
			/// Create a private NodeHandler
			ros::NodeHandle nhp("~");
			/// Check if parameters exist
			if(!nhp.hasParam("obj_name") || !nhp.hasParam("number") || !nhp.hasParam("x_thresh") || !nhp.hasParam("y_thresh") ) {
				ROS_INFO("Parameters cannot be found");
			}
			// Get parameters thru helper variables
			std::string viconName;
			nhp.getParam("number", carNum);
			nhp.getParam("obj_name", viconName);
			nhp.getParam("x_thresh", x_threshold);
			nhp.getParam("y_thresh", y_threshold);
		
			// Create vicon topic string thru string addition
			std::string buf = "/vicon/";
			std::string v_topic = buf + viconName + '/' + viconName;
			ROS_INFO("Node Created and Subscribed to: %s", v_topic.c_str());
			
			// used to get the pose of the duckiebot to transfer to turtlesim
			vicon_stream = nh.subscribe(v_topic, 5, &ViconTrack::vs_cb, this);
	
			// register subscribers to get current position/pose of other cars
			// based on the number of car that is being initialized
			if(carNum == 1) {
				car2_sub = nh.subscribe("/car2/pose", 5, &ViconTrack::c2_cb, this);
				car3_sub = nh.subscribe("/car3/pose", 5, &ViconTrack::c3_cb, this);
				// register pub to send twist velocity (cmd_vel)
				pose_pub = nh.advertise<geometry_msgs::Pose2D>("/car1/pose", 100);
			} else if(carNum == 2) {
				car2_sub = nh.subscribe("/car1/pose", 5, &ViconTrack::c2_cb, this);
				car3_sub = nh.subscribe("/car3/pose", 5, &ViconTrack::c3_cb, this);
				// register pub to send twist velocity (cmd_vel)
				pose_pub = nh.advertise<geometry_msgs::Pose2D>("/car2/pose", 100);
			} else if(carNum == 3) {
				car2_sub = nh.subscribe("/car1/pose", 5, &ViconTrack::c2_cb, this);
				car3_sub = nh.subscribe("/car2/pose", 5, &ViconTrack::c3_cb, this);
				// register pub to send twist velocity (cmd_vel)
				pose_pub = nh.advertise<geometry_msgs::Pose2D>("/car3/pose", 100);
			} else {
				ROS_INFO("invalid car number");
			}	
		}
		// Function declarations
		void vs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void c2_cb(const geometry_msgs::Pose2D::ConstPtr& msg);
		void c3_cb(const geometry_msgs::Pose2D::ConstPtr& msg);

	private:	
		// ros variables
		ros::NodeHandle nh;
		ros::Subscriber vicon_stream;
		ros::Subscriber car2_sub;
		ros::Subscriber car3_sub;
		ros::Publisher pose_pub;
				
		// helper variable to track movement									
		geometry_msgs::Pose2D CurPose;	
		// variable to tell the car number
		int carNum;
		float x_threshold;
		float y_threshold;
};

int main(int argc, char** argv) {
	// initialize the node
	ros::init(argc, argv, "default");
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
	
	// Get current position
	CurPose.x = tfs_data.transform.translation.x;
	CurPose.y = tfs_data.transform.translation.y;
	// Publish the Current Pose of car1
	pose_pub.publish(CurPose);
}
// call back function to track if there are collisions with car2
void ViconTrack::c2_cb(const geometry_msgs::Pose2D::ConstPtr& msg)			
{
	geometry_msgs::Pose2D car2Pose = *msg;		
	float x_dist = CurPose.x - car2Pose.x;
	float y_dist = CurPose.y - car2Pose.y;	
	/// declare threshold values for collision boundaries		
	if((x_dist <= x_threshold && x_dist >= -x_threshold) && (y_dist <= y_threshold && y_dist >= -y_threshold)) {
		/// get car number that potentially it is colliding with
		int cn;
		if(carNum == 1) {
			cn = 2;
		} else {
			cn = 1;
		}
		/// Collision is announced
		ROS_INFO("Car %d is Colliding with Car %d!", carNum, cn);
	}		
}
// call back function to track if there are collisions with car3
void ViconTrack::c3_cb(const geometry_msgs::Pose2D::ConstPtr& msg)			
{
	geometry_msgs::Pose2D car3Pose = *msg;		
	float x_dist = CurPose.x - car3Pose.x;
	float y_dist = CurPose.y - car3Pose.y;	
	/// declare threshold values for collision boundaries		
	if((x_dist <= x_threshold && x_dist >= -x_threshold) && (y_dist <= y_threshold && y_dist >= -y_threshold)) {
		/// get car number that potentially it is colliding with
		int cn;
		if(carNum == 3) {
			cn = 2;
		} else {
			cn = 3;
		}
		/// Collision is announced
		ROS_INFO("Car %d is Colliding with Car %d!", carNum, cn);
	}		
}
