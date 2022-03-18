#include "ros/ros.h"
#include "robotics_pkg/forK.h"
#include <cstdlib>

int main(int argc, char **argv) {
	ros::init(argc, argv, "forK_client");
   
	if (argc != 4) {
		std::cerr << "[Error] Usage: forK_client joint1 joint2 joint3" << std::endl;
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<robotics_pkg::forK>("forK");
	robotics_pkg::forK srv;
	srv.request.joint1 = atof(argv[1]);
	srv.request.joint2 = atof(argv[2]);
	srv.request.joint3 = atof(argv[3]);

	if (client.call(srv)) {
		std::cout << "Calculation complete! Point: [" << srv.response.x << ", " << srv.response.y << ", " << srv.response.z << "]" << std::endl;
	}
	else {
		std::cerr << "[Error] Failed to call service forK" << std::endl;
		return 1;
	}

	return 0;
}
