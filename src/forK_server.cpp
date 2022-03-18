#include "ros/ros.h"
#include "robotics_pkg/forK.h"

#include <vector>
#include <math.h>
#include <iostream>

std::vector<std::vector<double>> DH_model(const double theta, const double d, const double a, const double alpha) {
	std::vector<std::vector<double>> A {
		{cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) , a*cos(theta)},
		{sin(theta), cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a*sin(theta)},
		{0, sin(alpha), cos(alpha), d},
		{0, 0, 0, 1}
	};
	return A;
}

template<typename T>
std::vector<std::vector<T>> multiply(const std::vector<std::vector<T>> A, const std::vector<std::vector<T>> B) {
	size_t Ar = A.size(), Br = B.size();
	size_t Ac = A[0].size(), Bc = B[0].size();

	if (Ac != Br) {
		std::cerr << "[Error] Matrix dimension mismatched" << std::endl;
    	exit(1);
	}

	std::vector<std::vector<T>> ret(Ar, std::vector<T>(Bc));
	for (int i=0; i<Ar; i++) {
		for (int j=0; j<Bc; j++) {
			for (int k=0; k<Ac; k++) {
				ret[i][j] += A[i][k]*B[k][j];
			}
		}
	}

	return ret;
}

bool forwardKinematics(robotics_pkg::forK::Request &req, robotics_pkg::forK::Response &res) {
  	// Specification parameters
  	const double pi = M_PI;  // PI constants
  	const double d1 = -4, a2 = 10, a3 = 2;   // unit:cm (to be modified)

  	// Illegal protection
  	const double joint1_bound[2] = {-90, 90}, joint2_bound[2] = {0, 90}, joint3_bound[2] = {-90, 90};   // to be modified
  	if (req.joint1 < joint1_bound[0] || req.joint1 > joint1_bound[1]) {
		std::cerr << "[Error] Joint 1 is out of bound!" << std::endl;
  		return false;
  	}
  	if (req.joint2 < joint2_bound[0] || req.joint2 > joint2_bound[1]) {
		std::cerr << "[Error] Joint 2 is out of bound!" << std::endl;
  		return false;
  	}
  	if (req.joint3 < joint3_bound[0] || req.joint3 > joint3_bound[1]) {
		std::cerr << "Error! Joint 3 is out of bound!" << std::endl;
  		return false;
	}

  	// Calculate A1 ~ A4 and T4
  	const std::vector<std::vector<double>> A1 = DH_model(req.joint1*pi/180, d1, 0, -pi/2), A2 = DH_model(req.joint2*pi/180, 0, a2, 0);
  	const std::vector<std::vector<double>> A3 = DH_model(req.joint3*pi/180, 0, a3, pi/2), A4 = DH_model(0, 0, 0, 0);
	const std::vector<std::vector<double>> T4 = multiply(multiply(A1, A2), multiply(A3, A4));

  	res.x = T4[0][3];
  	res.y = T4[1][3];
  	res.z = T4[2][3];

	return true;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "forK_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("forK", forwardKinematics);
	std::cout << "Ready to calculate forward kinematics." << std::endl;
	ros::spin();

	return 0;
}
