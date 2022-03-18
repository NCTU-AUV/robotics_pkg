#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "robotics_pkg/JointSolution.h"
#include "robotics_pkg/pca9685_output.h"

#include <math.h>

PCA9685_output motor(243.75);

void chatterCallback(const robotics_pkg::JointSolution::ConstPtr& joint)
{
	double legal_solution[3]{0};	// Find the legal solution from topic "joint_solution"
	for (int i=0; i<2; i++) {
		if (joint->legal[i]) {
			legal_solution[0] = joint->joint1[i];
			legal_solution[1] = joint->joint2[i];
			legal_solution[2] = joint->joint3[i];
			break;
		}
	}
	std::cout << "Successfully subscribe message from 'invK_calc'!!" << std::endl;
	std::cout << "Joint 1 (in degree): " << legal_solution[0] << std::endl;
	std::cout << "Joint 2 (in degree): " << legal_solution[1] << std::endl;
	std::cout << "Joint 3 (in degree): " << legal_solution[2] << std::endl;
	
	std::cout << "Begin jogging..." << std::endl;
	/* Set angle value of each servo respectively */
	// Joint_1 moves to the desired position and Joint_3 resolutes 90 degree
	motor.set_PWM_OFF(JOINT_PIN[0], Deg2PWM(legal_solution[0]));  
	motor.set_PWM_OFF(JOINT_PIN[2], Deg2PWM(90));
	sleep(3);
	motor.set_PWM_OFF(JOINT_PIN[3], Deg2PWM(150));	// Open the claw (to be modified)
	sleep(3);

	// Step 2: Joint_2 and Joint_3 moves to the desired position respectively
	motor.set_PWM_OFF(JOINT_PIN[1], Deg2PWM(legal_solution[1]));
	motor.set_PWM_OFF(JOINT_PIN[2], Deg2PWM(legal_solution[2]));
	sleep(3);
	motor.set_PWM_OFF(JOINT_PIN[3], Deg2PWM(90));	// Close the claw (to be modified)
	sleep(3);

	// Step 3: return to the original configuration 
	motor.set_PWM_OFF(JOINT_PIN[0], Deg2PWM(0));
	motor.set_PWM_OFF(JOINT_PIN[1], Deg2PWM(0));
	motor.set_PWM_OFF(JOINT_PIN[2], Deg2PWM(90));
	sleep(3);
	motor.set_PWM_OFF(JOINT_PIN[3], Deg2PWM(150));	// Open the claw (to release the ball) 
	sleep(3);
	motor.set_PWM_OFF(JOINT_PIN[3], Deg2PWM(90));		// Close the claw (to return to the original configuration)
		
	/* Print angle value */
	std::cout << "Finish jogging!!" << std::endl;
}

int main(int argc, char **argv)
{
	// ROS objects
	ros::init(argc, argv, "invK_sub");
	ros::NodeHandle n;
	
	// Subscribe to topic "joint_solution"(joint1[4], joint2[4], joint3[4])
	ros::Subscriber sub = n.subscribe("joint_solution", 1, chatterCallback);

	ros::spin();

	return 0;
}
