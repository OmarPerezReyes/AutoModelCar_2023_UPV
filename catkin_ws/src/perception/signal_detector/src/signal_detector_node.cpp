#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h" 

#include <opencv2/opencv.hpp>

    
ros::Publisher pub_stop;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)    
{
    bool decision_stop = false;

    std_msgs::Bool bool_msg;

    bool_msg.data = decision_stop;
    pub_stop.publish(bool_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "signal_detector_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1000, imageCallback);

    pub_stop = n.advertise<std_msgs::Bool>("/bool_stop", 1000);
 
    ros::spin(); 

    return 0;
}
