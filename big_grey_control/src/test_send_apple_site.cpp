#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher applesite_pub = n.advertise<std_msgs::Float32MultiArray>("apple_site", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        double x = 0.4, y = 0.7, z = 1.6;
        std_msgs::Float32MultiArray apple_site_msg;

        apple_site_msg.data.resize(3);
        apple_site_msg.data[0]=x;
        apple_site_msg.data[1]=y;
        apple_site_msg.data[2]=z;

        applesite_pub.publish(apple_site_msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}