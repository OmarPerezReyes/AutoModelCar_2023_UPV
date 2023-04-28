#include <librealsense2/rs.hpp>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>

cv::VideoWriter video("/home/ros/source_code/Assets/out.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30,cv::Size(640,480));

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)    
{
    cv::Mat color(cv::Size(640, 480), CV_8UC3, (unsigned char*)msg->data.data(), cv::Mat::AUTO_STEP);
    int key = cv::waitKey(1);
    if(key==32){
        std::cout<<"Finished recording"<<std::endl; 
        video.release();
        exit(1);;            
    }
    video.write(color);

    //cv::imshow("img", color);
    //cv::waitKey(1);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_detector_node");

    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1000, imageCallback);
    std::cout<<"Clic space in video to save"<<std::endl;

    ros::spin(); 

    return 0;

}
