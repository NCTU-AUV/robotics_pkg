#ifndef PUB_SUB_H
#define PUB_SUB_H

#include <ros/ros.h>
#include <string>

template <typename PubType, typename SubType>
class PubSub
{
public:
	PubSub() {}
	PubSub(std::string PubTopicName, std::string SubTopicName, int queueSize) {
		PubObject = n.advertise<PubType> (PubTopicName, queueSize);
		SubObject = n.subscribe<SubType> (SubTopicName, queueSize, &PubSub::SubCallback, this);
	}
	void SubCallback(const typename SubType::ConstPtr& received_msgs);
protected:
	ros::Publisher PubObject;
	ros::Subscriber SubObject;
	ros::NodeHandle n;
};

#endif