/*
Authors: Seok-Ki Lee
*/
#include <ros/ros.h>
#include "can_msgs/FD_Frame.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class can_fd_bridge
{
private:
	// ROS NodeHandle
	ros::NodeHandle node_handle_;
	ros::NodeHandle priv_node_handle_;

	ros::Timer read_timer_;

	ros::Publisher can_fd_pub_;
	ros::Subscriber can_fd_sub_;

	can_msgs::FD_Frame ros_read_frame_;

	std::string can_device;

	/*can variable*/
	int s;
	int enable_canfd;
	const char *ifname;
	struct sockaddr_can addr;
	struct canfd_frame read_frame_, write_frame_;
	struct ifreq ifr;

public:
	can_fd_bridge();
	~can_fd_bridge();

	bool initCan(const char *can_device_);

	void readClock(const ros::TimerEvent &);
	void writeClock(const ros::TimerEvent &);

	void can_fd_Callback(const can_msgs::FD_Frame::ConstPtr &CANFD);

	void error_handling(const char *message);
};