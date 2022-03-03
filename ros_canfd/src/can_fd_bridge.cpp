/*
Authors: Seok-Ki Lee
*/

#include "ros_canfd/can_fd_bridge.h"

using namespace std;

can_fd_bridge::can_fd_bridge() // constructor
    : node_handle_(""),
      priv_node_handle_("~")
{
    can_device = priv_node_handle_.param<std::string>("can_device", "port");
    read_timer_ = priv_node_handle_.createTimer(ros::Duration(0.001), &can_fd_bridge::readClock, this); // 10ms
    can_fd_pub_ = priv_node_handle_.advertise<can_msgs::FD_Frame>("receive", 100);
    can_fd_sub_ = priv_node_handle_.subscribe("write", 100, &can_fd_bridge::can_fd_Callback, this);

    cout << "can_device name  :  " << can_device << endl;
    initCan(const_cast<char *>(can_device.c_str()));
}
can_fd_bridge::~can_fd_bridge()
{
    if (s)
        close(s);
}

bool can_fd_bridge::initCan(const char *can_device_)
{
    bool result = false;
    s = 0;
    enable_canfd = 1;
    ifname = can_device_;
    ROS_WARN("Please Check : %s", can_device_);

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        error_handling("Error while opening can");
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        error_handling("Error in can bind");
    }
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

    return result;
}
void can_fd_bridge::readClock(const ros::TimerEvent &) // socketcan read to ros pub
{
    read(s, &read_frame_, sizeof(struct canfd_frame));
    ros_read_frame_.id = read_frame_.can_id;
    ros_read_frame_.len = read_frame_.len;
    ros_read_frame_.data = {0,};

    for (int i = 0; i < read_frame_.len; i++)
    {
        ros_read_frame_.data[i] = read_frame_.data[i];
    }
    ros_read_frame_.header.stamp = ros::Time::now();
    can_fd_pub_.publish(ros_read_frame_);
}
void can_fd_bridge::can_fd_Callback(const can_msgs::FD_Frame::ConstPtr &CANFD)
{
    ROS_INFO("Call back");
    write_frame_.can_id = CANFD->id;
    write_frame_.len = CANFD->len;

    for (int i = 0; i < CANFD->len; i++)
        write_frame_.data[i] = CANFD->data[i];

    if (write(s, &write_frame_, sizeof(struct canfd_frame)) != sizeof(struct canfd_frame))
    {
        perror("Write");
    }
}
void can_fd_bridge::error_handling(const char *message)
{
    fputs(message, stderr);
    fputc('\n', stderr);
    exit(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_fd_bridge");
    ros::NodeHandle node_handle("");

    can_fd_bridge bridge;

    ros::spin();

    return 0;
}