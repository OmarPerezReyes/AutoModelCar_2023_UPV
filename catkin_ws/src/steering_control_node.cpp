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

bool first_time = true;

//
int mssleep(long milliseconds){

  struct timespec rem;
  struct timespec req = {
        (int)(milliseconds / 1000), /*secs (Must be Non-Negative)*/
        (milliseconds % 1000) * 1000000 /*nano (Must be in range of 0 to 999999999)*/
  };

  return nanosleep(&req, &rem); 

}



void stopCallback(const std_msgs::Bool msg){

  int speed;

   autominy_msgs::SteeringPWMCommand steeringMsg;
      autominy_msgs::SpeedPWMCommand speedMsg;

   if (msg.data){
        speed = 0;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(3000);
   } else {
        speed = 60;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(100);

   }

}


void steering_calculate(float obs_rho, float obs_theta, float best_rho, float best_theta)
{
    int speed;
    int steering;
   
    autominy_msgs::SteeringPWMCommand steeringMsg;
    autominy_msgs::SpeedPWMCommand speedMsg;

    if (ros::ok()) {
        speed = 50;  
    }

    if (first_time) {
        steering = 1470;
        steeringMsg.value = steering;
        pubSteering.publish(steeringMsg);

        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);

        first_time = false;
        
    }

}

void laneCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    steering_calculate(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "steering_control_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/line_right", 1, laneCallback);
    ros::Subscriber stop = n.subscribe("/bool_stop", 1, stopCallback);

    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);

    // This MUST be here
    mssleep(100);

    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);

    // This MUST be here
    mssleep(100);

    autominy_msgs::SteeringPWMCommand steeringMsg;
    autominy_msgs::SpeedPWMCommand speedMsg;

    if (ros::ok()) {

        fprintf(stdout, "Aquí entró\n");
        fflush(stdout);

        autominy_msgs::SteeringPWMCommand steeringMsg;
        int steering = 1470;
        steeringMsg.value = static_cast<int16_t>(steering);
        pubSteering.publish(steeringMsg);
            
        mssleep(100);

        int speed = 60;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);

        mssleep(100);

        fprintf(stdout, "Aquí salió\n");
        fflush(stdout);


    }
    
    ros::spin();

    return 0;
}
