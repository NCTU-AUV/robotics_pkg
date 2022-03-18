#include "ros/ros.h"
#include "robotics_pkg/JointSolution.h"
#include "robotics_pkg/pca9685_output.h"

#include <unistd.h>
#include <iostream>

PCA9685_output motor(243.75);

int main(int argc, char **argv)
{
	while (1) {
		int mode = 0;
		std::cout << "(1) Move one joint according to resolving angle." << std::endl;
		std::cout << "(2) Move ALL joint according to resolving angle." << std::endl;
		std::cout << "(3) Move one joint according to PWM signal." << std::endl;
		std::cout << "(4) Move ALL joint according to PWM signal." << std::endl;
		std::cout << "Please select mode for testing, or enter \'-1\' to exit... " ;
		std::cin >> mode;
		if (mode == -1) break;

		int joint, degree, PWM_i, PWM_o, lower_bound, upper_bound;
		if (mode == 1 || mode == 2) {
			while (1) {
				// Prompt the user to enter resolving joint (in mode 1)
				if (mode == 1) {
					std::cout << "Please enter desired joint, or enter \'-1\' to exit... ";
					std::cin >> joint;	
				}
				if (joint == -1) break;
				else if (joint == 2 || joint == 3) {
					lower_bound = -135;
					upper_bound = 135;
				} else if (joint == 1 || joint == 4) {
					lower_bound = -90;
					upper_bound = 90;
				} else {
					std::cerr << "[Error] Undefined joint number!!" << std::endl;
					break;
				}
			
				// Prompt the user to enter resolving angle in degree
				std::cout << "Please enter resolving angle in degree... ";
				std::cin >> degree;
				if (degree > upper_bound || degree < lower_bound) {
					std::cerr << "[Error] Degrees of resolving angle out of bound!!" << std::endl;
					break;
				}

				// Prepare for moving
				PWM_i = Deg2PWM(degree, lower_bound, upper_bound);		// calibrated input PWM
				PWM_o = PWM_i * 0.92 + 56;		// desired output PWM
				if (mode == 1) {
					motor.set_PWM_OFF(JOINT_PIN[joint-1], PWM_i);
					std::cout << "Joint " << joint << " moves to degree " << degree << " with desired PWM signal " << PWM_o << std::endl;
				} else if (mode == 2) {
					motor.set_PWM_OFF(JOINT_PIN[0], PWM_i);
					motor.set_PWM_OFF(JOINT_PIN[1], PWM_i);
					motor.set_PWM_OFF(JOINT_PIN[2], PWM_i);
					motor.set_PWM_OFF(JOINT_PIN[3], PWM_i);
					std::cout << "ALL joint move to degree " << degree << " with desired PWM signal " << PWM_o << std::endl;
				}
			}	
		} else if (mode == 3 || mode == 4) {
			while (1) {
				// Prompt the user to enter resolving joint (in mode 1)
				if (mode == 3) {
					std::cout << "Please enter resolving joint, or enter \'-1\' to exit... ";
					std::cin >> joint;	
				}
				if (joint == -1) break;
				else if (joint <= 0 || joint >= 5) {
					std::cerr << "[Error] Undefined joint number!!" << std::endl;
					break;
				}
				
				// Prompt the user to enter desired PWM signal
				std::cout << "Please enter desired PWM signal...  ";
				std::cin >> PWM_o;
				if (PWM_o > 2500 || PWM_o < 500) {
					std::cerr << "[Error] PWM signal out of bound!!" << std::endl;
					break;
				}

				// Prepare for moving
				PWM_i = (PWM_o - 56) / 0.92;		// calibrated input PWM
				if (mode == 3) {
					motor.set_PWM_OFF(JOINT_PIN[joint-1], PWM_i);
					std::cout << "Joint " << joint << " moves to desired PWM " << PWM_o << " when entering calibrated PWM " << PWM_i << std::endl;
				} else if (mode == 4) {
					motor.set_PWM_OFF(JOINT_PIN[0], PWM_i);
					motor.set_PWM_OFF(JOINT_PIN[1], PWM_i);
					motor.set_PWM_OFF(JOINT_PIN[2], PWM_i);
					motor.set_PWM_OFF(JOINT_PIN[3], PWM_i);
					std::cout << "ALL joint move to desired PWM " << PWM_o << " when entering calibrated PWM " << PWM_i << std::endl;
				}
			}	
		}
	}

	return 0;
}
