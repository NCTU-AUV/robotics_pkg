#ifndef PCA9685_OUTPUT_H
#define PCA9685_OUTPUT_H

#include <wiringPiI2C.h>

constexpr int ADDRESS = 0x40;
constexpr int MIN_PULSE_WIDTH = 500;
constexpr int MAX_PULSE_WIDTH = 2500;
constexpr int DEF_PULSE_WIDTH = 1500;
constexpr int JOINT_PIN[4]{8, 10, 12, 14};

class PCA9685_output
{
private:
	int fd;

public:
	PCA9685_output(const double f);
	~PCA9685_output();
	int set_sleep();
	int unset_sleep();
	void set_PWM_ON(const int ch, const unsigned short int value);
	void set_PWM_OFF(const int ch, unsigned short int value);

};

//const int Deg2PWM(const int degree, const int lower, const int upper);
const int Deg2PWM(const int, const int=0, const int=180);

#endif
