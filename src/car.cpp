// Gabe Petersen
// car.cpp - 6 Apr 2019
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
			/// Create a private NodeHandler for private parameters
			ros::NodeHandle nhp("~");
			/// Check if parameters exist
			if(!nhp.hasParam("obj_name") || !nhp.hasParam("number") || !nhp.hasParam("x_thresh") || !nhp.hasParam("y_thresh") ) {
				ROS_INFO("Error: Parameters Cannot be found");
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
		bool detectCollision(geometry_msgs::Pose2D otherPose);
		bool evaluateProjections(float Norm[2], float dots1[8], float dots2[8]);
		void getDots(float dots[], geometry_msgs::Pose2D pose);

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
		float x_threshold; // width
		float y_threshold; // height
};

int main(int argc, char** argv) {
	// initialize the node
	ros::init(argc, argv, "default");
	
	// instantiate the class
	ViconTrack vt;

	// declare sample rate
	ros::Rate r(100);
	while(ros::ok()) {
		r.sleep();
		ros::spinOnce();
	}	
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
	// ROS_INFO("Position: %f, %f   Theta: %f", CurPose.x, CurPose.y, CurPose.theta);
	// Publish the Current Pose of car1
	pose_pub.publish(CurPose);
}
// call back function to track if there are collisions with car2
void ViconTrack::c2_cb(const geometry_msgs::Pose2D::ConstPtr& msg) {
	geometry_msgs::Pose2D opose = *msg;
	
	/// if other car falls within threshold of collision boundary	
	/// error check a little in case vicon fails
	if(detectCollision(opose)) {
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
void ViconTrack::c3_cb(const geometry_msgs::Pose2D::ConstPtr& msg) {
	geometry_msgs::Pose2D opose = *msg;
	
	/// if other car falls within threshold of collision boundary		
	if(detectCollision(*msg) && ((CurPose.x - opose.x) < 0.5) && ((CurPose.y - opose.y) < 0.5)) {
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
/* detectCollision
 * Calculates if two objects of two center points with given height and width are colliding
 * Utilizes Seperability of Axis Theorem 
 * Returns true if collision between the two objects
 */
bool ViconTrack::detectCollision(geometry_msgs::Pose2D otherPose) {
	bool case1 = false, case2 = false, case3 = false, case4 = false;
	/// Get the dots of both the objects analyzed
	float dots1[8], dots2[8];
	getDots(dots1, CurPose);
	getDots(dots2, otherPose);
	
	/// Check axes surrounding current box
	/// Calculate Normal direction Vectors from Edges of Box Made from Dots1
	float dirL1[2] = {(dots1[0] - dots1[6]), (dots1[1] - dots1[7])};
	float dirW1[2] = {(dots1[0] - dots1[2]), (dots1[1] - dots1[3])};
	float dirL2[2] = {(dots2[0] - dots2[6]), (dots2[1] - dots2[7])};
	float dirW2[2] = {(dots2[0] - dots2[2]), (dots2[1] - dots2[3])};
	
	/// get magnitude of the normal direction vectors
	float magL1 = pow((pow(dirL1[0], 2) + pow(dirL1[1], 2)), 0.5);
	float magW1 = pow((pow(dirW1[0], 2) + pow(dirW1[1], 2)), 0.5);
	float magL2 = pow((pow(dirL2[0], 2) + pow(dirL2[1], 2)), 0.5);
	float magW2 = pow((pow(dirW2[0], 2) + pow(dirW2[1], 2)), 0.5);

	/// normalize the direction vectors to get the unit axes
	float NormL1[2] = {(dirL1[0] / magL1), (dirL1[1] / magL1)};
	float NormW1[2] = {(dirW1[0] / magW1), (dirW1[1] / magW1)};
	float NormL2[2] = {(dirL2[0] / magL2), (dirL2[1] / magL2)};
	float NormW2[2] = {(dirW2[0] / magW2), (dirW2[1] / magW2)};
	
	/// evalute across x-axis of current object
	case1 = evaluateProjections(NormW1, dots1, dots2);
	/// automatically return false if one of the cases return false so no xtra computing
	if(!case1)
		return false;
	/// evalute across y-axis of current object
	case2 = evaluateProjections(NormL1, dots1, dots2);
	if(!case2)
		return false;
	/// evalute across x-axis of other object
	case3 = evaluateProjections(NormL2, dots1, dots2);
	if(!case3)
		return false;
	/// evalute across y-axis of other object
	case4 = evaluateProjections(NormW2, dots1, dots2);	
	if(!case4)
		return false;

	/// if all cases return true, there is a collision!
	return true;
}
/* evaluateProjections
 * Will calculate the projections of the object dots onto specified axis: Norm[2]
 * Will evaluate if the lines made by the min and max are overlapping - if so possible collision
 * Returns a bool - true = possible collision, false = no collision
 */	
bool ViconTrack::evaluateProjections(float Norm[2], float dots1[8], float dots2[8]) {
	/// Project along x-axis of current box
	float min1 = 1000, min2 = 1000, max1 = -1000, max2 = -1000;
	float dot_proj1, dot_proj2;
	/// For each dot projection, find the min and max
	for(int i = 0; i < 8; i = i+2) {
		// dot product of object axis and dots of object to find scalar projections
		dot_proj1 = (Norm[0] * dots1[i]) + (Norm[1] * dots1[i+1]);
		// find min/max of the scalar projections
		if(dot_proj1 < min1) {
			min1 = dot_proj1;
		} else if(dot_proj1 > max1) {
			max1 = dot_proj1;
		}
		// dot product of object axis and dots of object to find scalar projections
		dot_proj2 = (Norm[0] * dots2[i]) + (Norm[1] * dots2[i+1]);
		// find min/max of the scalar projections
		if(dot_proj2 < min2) {
			min2 = dot_proj2;
		} else if(dot_proj2 > max2) {
			max2 = dot_proj2;
		}
	}
	
	/// check for a gap between the two projected lines
	/// if there is a gap, case 1 is false and no possible collision
	if( (min1 < min2) && (min2 < max1) ) {
		return true;
	} else if( (min1 < max2) && (max2 < max1) ) {
		return true;
	} else if( (min2 < min1) && (min1 < max2) ) {
		return true;
	} else if( (min2 < max1) && (max1 < max2) ) {
		return true;
	} else {
		/// if theres not a gap, case 1 is true and there is a possible collision
		return false;
	}
}

/* getDots
 * Will calculate the Points/Dots of the corners of the ecapsulating boundary box
 * Returns thru reference an array of dots = [dot1x, dot1y, dot2x, ... , dot4y]
 */
void ViconTrack::getDots(float dots[], geometry_msgs::Pose2D pose) {
	/// get radius and angle between width and height
	float r = pow( (pow( (x_threshold/2) , 2) + pow( (y_threshold/2) , 2)), 0.5);
	float phi = atan( (y_threshold/2) / (x_threshold/2) );
	/// calculate array of points from the required boundary shape
	/// First Dot
	dots[0] = pose.x + r*cos(phi + pose.theta);
	dots[1] = pose.y + r*sin(phi + pose.theta);
	/// Second Dot
	dots[2] = pose.x + r*cos((M_PI - phi) + pose.theta);
	dots[3] = pose.y + r*sin((M_PI - phi) + pose.theta);
	/// Third Dot
	dots[4] = pose.x - r*cos(phi + pose.theta);
	dots[5] = pose.y - r*sin(phi + pose.theta);
	/// Fourth Dot
	dots[6] = pose.x - r*cos((M_PI - phi) + pose.theta);
	dots[7] = pose.y - r*sin((M_PI - phi) + pose.theta);
}
