#include <librealsense2/rs.hpp>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>


void imageCallback(const sensor_msgs::Image::ConstPtr& msg)    
{
    cv::Mat color(cv::Size(640, 480), CV_8UC3, (unsigned char*)msg->data.data(), cv::Mat::AUTO_STEP);
    int key = cv::waitKey(1);
    cv::imshow("img", color);

    if(key==32){
        cv::imwrite("/home/ros/source_code/Assets/frame.jpg", color);   
        std::cout<<"Frame saved"<<std::endl;
        exit(1);
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "frame_capture_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1000, imageCallback);
    std::cout<<"Clic space in video to save"<<std::endl;

    ros::spin(); 

    return 0;

}
