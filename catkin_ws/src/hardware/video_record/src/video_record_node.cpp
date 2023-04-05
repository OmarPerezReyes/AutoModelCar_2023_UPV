#include <librealsense2/rs.hpp>
#include <signal.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <opencv2/opencv.hpp>

cv::VideoWriter video("/home/ros/out.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30,cv::Size(640,480));

void my_handler(sig_atomic_t s){
    std::cout<<"Grabación terminada"<<std::endl; 
    video.release();
    exit(1);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)    
{
    cv::Mat color(cv::Size(640, 480), CV_8UC3, (unsigned char*)msg->data.data(), cv::Mat::AUTO_STEP);
/*    int key = cv::waitKey(1);
    if(key==27){
        std::cout << "Clic" <<std::endl;
        video.release();            
    }
  */  video.write(color);

    cv::imshow("img", color);
    cv::waitKey(1);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_detector_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1000, imageCallback);


    ros::spin(); 
    signal (SIGINT, my_handler); 
    return 0;

}
