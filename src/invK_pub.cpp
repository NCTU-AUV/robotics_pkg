#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include <iostream>
int main(int argc, char **argv)
{
	// ROS object
	ros::init(argc, argv, "invK_pub");
	ros::NodeHandle n;
	ros::Publisher pub_point = n.advertise<geometry_msgs::Point>("obj_point", 1);
	ros::Rate loop_rate(1);

	// the message to be published
	geometry_msgs::Point obj_point;

	// loop control
	while (ros::ok()) {
		// Prompt the user to enter Cartesian point obj_point(x, y, z)
		std::cout << "Enter a point(x, y, z): " << std::endl;
		std::cin >> obj_point.x >> obj_point.y >> obj_point.z;
		pub_point.publish(obj_point);

		std::cout << "Successfully publish the message!" << std::endl << std::endl;

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
