#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "robotics_pkg/JointSolution.h"
#include "robotics_pkg/PubSub.h"

#include <math.h>
#include <bits/stdc++.h>
#include <iostream>

template<>
void PubSub<robotics_pkg::JointSolution, geometry_msgs::Point>::SubCallback(const geometry_msgs::Point::ConstPtr& received_msgs)
{
	const double pi = M_PI;  // PI constants
	const double px = received_msgs->x, py = received_msgs->y, pz = received_msgs->z; // position of the point

	// Specification parameters
	const double d1 = -4, a2 = 10, a3 = 2;   // unit:cm (to be modified)
	const double R = sqrt(pow(px,2)+pow(py,2)+pow(d1-pz,2));
	robotics_pkg::JointSolution joint;	// Topic to be published

	std::cout << "Ready to calculate inverse kinematics..." << std::endl;
	// Illegal operation region protection
	if (pz>d1 || R>a2+a3 || R<sqrt(pow(a2,2)+pow(a3,2))) {
		if (pz>d1)
			std::cerr << "[Error] The point should not be higher than " << d1 << " (cm)!!" << std::endl;
		if (R>a2+a3)
			std::cerr << "[Error] The point is too far to reach!" << std::endl;
		if (R<sqrt(pow(a2,2)+pow(a3,2)))
			std::cerr << "[Error] The point is too close to reach!" << std::endl;
		return;
	}

	// Joint 1
	joint.joint1[0] = atan2(py, px)*180/pi;	// joint1[0] & joint1[1] is for the 1st config. of joint1
	joint.joint1[1] = atan2(py, px)*180/pi;	//

	// Joint 2
	const double theta_large = atan2(d1-pz, sqrt(pow(px,2)+pow(py,2)));
	const double theta_small = acos((pow(a2,2)+pow(R,2)-pow(a3,2))/(2*a2*R));
	joint.joint2[0] = (theta_large - theta_small)*180/pi;		// joint2[0] is for upper config.
	joint.joint2[1] = (theta_large + theta_small)*180/pi;		// joint2[1] is for lower config.

	// Joint 3
	joint.joint3[0] = acos((pow(px,2)+pow(py,2)+pow(d1-pz,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3))*180/pi;   // joint3[0] is for upper config.
	joint.joint3[1] = -acos((pow(px,2)+pow(py,2)+pow(d1-pz,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3))*180/pi;  // joint3[1] is for lower config.

	// Illegal joint angle protection
	const double joint1_bound[2] = {-90, 90}, joint2_bound[2] = {0, 90}, joint3_bound[2] = {-90, 90};   // to be modified
	for (int i=0; i<2; i++) {
		joint.legal[i] = true;
		std::cout << "#" << i+1 << " solution: joint1=" << joint.joint1[i] << ", joint2=" << joint.joint2[i] << ", joint3=" << joint.joint3[i] << std::endl;
		if (joint.joint1[i] < joint1_bound[0] || joint.joint1[i] > joint1_bound[1]) {
			std::cerr << "[Error] Joint 1 is out of bound!" << std::endl;
			joint.legal[i] = false;
		}
		if (joint.joint2[i] < joint2_bound[0] || joint.joint2[i] > joint2_bound[1]) {
			std::cerr << "[Error] Joint 2 is out of bound!" << std::endl;
			joint.legal[i] = false;
		}
		if (joint.joint3[i] < joint3_bound[0] || joint.joint3[i] > joint3_bound[1]) {
			std::cerr << "[Error] Joint 3 is out of bound!" << std::endl;
			joint.legal[i] = false;
		}
	}

	PubObject.publish(joint);
	std::cout << "Successfully publish the message!" << std::endl << std::endl;
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "invK_calc");
  PubSub<robotics_pkg::JointSolution, geometry_msgs::Point> invK_calc("joint_solution", "obj_point", 1);
  ros::spin();

  return 0;
}
