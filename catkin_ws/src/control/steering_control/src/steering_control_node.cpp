#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <signal.h>
#include <unistd.h>

#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringPWMCommand.h>


ros::Publisher pubSpeed;
ros::Publisher pubSteering;

autominy_msgs::SteeringPWMCommand steeringMsg;
autominy_msgs::SpeedPWMCommand speedMsg;

bool first_time = true;

//

void my_handler(sig_atomic_t s){
    int speed = 0;  
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
    exit(1);
}

int mssleep(long milliseconds){

  struct timespec rem;
  struct timespec req = {
        (int)(milliseconds / 1000), /*secs (Must be Non-Negative)*/
        (milliseconds % 1000) * 1000000 /*nano (Must be in range of 0 to 999999999)*/
  };

  return nanosleep(&req, &rem); 
}

void steering_calculate(float obs_rho, float obs_theta, float best_rho, float best_theta){
    int speed;
    int steering;
    //En espera del line detector
}



void stopCallback(const std_msgs::Bool msg){
  int speed;

  if (msg.data){
        mssleep(3500);
        speed = 0;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(4500);
  } else {
        speed = 40;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(200);

   }
}


void laneCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    steering_calculate(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}


//
int main(int argc, char* argv[]){
    ros::init(argc, argv, "steering_control_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/line_right", 1, laneCallback);
    ros::Subscriber stop = n.subscribe("/bool_stop", 1, stopCallback);

    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);

    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);
    signal(SIGINT,my_handler);
    ros::spin();

    return 0;
}
