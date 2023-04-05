#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <signal.h>

#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringPWMCommand.h>

ros::Publisher pubSpeed;
ros::Publisher pubSteering;

bool first_time = true;

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

    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);
    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);

    
    ros::spin();

    return 0;
}
