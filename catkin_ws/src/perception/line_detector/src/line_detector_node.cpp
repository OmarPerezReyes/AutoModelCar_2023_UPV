#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32MultiArray.h" 
#include "std_msgs/Bool.h" 

#include <opencv2/opencv.hpp>

ros::Publisher pub_line;
ros::Publisher pub_loss; 

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)    
{
    float obs_rho_pub = 0;
    float obs_theta_pub = 0;
    float best_rho_pub = 0;
    float best_theta_pub = 0;  
    bool line_loss = false;

    std_msgs::Bool bool_msg;
    std_msgs::Float32MultiArray lane_msg;

    bool_msg.data = line_loss;    
    lane_msg.data.push_back(obs_rho_pub);
    lane_msg.data.push_back(obs_theta_pub);
    lane_msg.data.push_back(best_rho_pub);
    lane_msg.data.push_back(best_theta_pub);


    pub_loss.publish(bool_msg);
    pub_line.publish(lane_msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "line_detector_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1000, imageCallback);

    pub_line = n.advertise<std_msgs::Float32MultiArray>("/line_right", 1);
    pub_loss = n.advertise<std_msgs::Bool>("/bool_loss", 1000);
    ros::spin(); 

    return 0;
}
