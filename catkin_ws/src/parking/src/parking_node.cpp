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

const int MAX_SPEED = 50;
const int sleep_msec = 200;
const int STEERING_CENTER = 1440;

ros::Publisher pubSpeed;
ros::Publisher pubSteering;

autominy_msgs::SteeringPWMCommand steeringMsg;
autominy_msgs::SpeedPWMCommand speedMsg;


int mssleep(long milliseconds){

  struct timespec rem;
  struct timespec req = {
        (int)(milliseconds / 1000), /*secs (Must be Non-Negative)*/
        (milliseconds % 1000) * 1000000 /*nano (Must be in range of 0 to 999999999)*/
  };

  return nanosleep(&req, &rem); 
}


void my_handler(sig_atomic_t s){
    int speed = 0;  
    
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
    mssleep(sleep_msec);
    int steering = STEERING_CENTER;
    steeringMsg.value = static_cast<int16_t>(steering);
    pubSteering.publish(steeringMsg);
        
    exit(1);
}




//
int main(int argc, char* argv[]){
    int steering = STEERING_CENTER;
 int speed = MAX_SPEED;

    ros::init(argc, argv, "steering_control_node");
    ros::NodeHandle n;

    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm",
              10);
    mssleep(sleep_msec);

    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);
    mssleep(sleep_msec);

    signal(SIGINT,my_handler);

    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
    mssleep(sleep_msec);

    steeringMsg.value = static_cast<int16_t>(steering);
    pubSteering.publish(steeringMsg);
    mssleep(sleep_msec);

       fprintf(stdout, "Parking Inicia\n");
       fflush(stdout);
       mssleep(450);

//recto

       fprintf(stdout, "Recto\n");
       fflush(stdout);
  steering = STEERING_CENTER;
steeringMsg.value = static_cast<int16_t>(steering);
        pubSteering.publish(steeringMsg);
       mssleep(11000);


    speed = 0;
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
    mssleep(500);


//izq

    fprintf(stdout, "Izq\n");
    fflush(stdout);
    steering = 900;
    steeringMsg.value = static_cast<int16_t>(steering);
    pubSteering.publish(steeringMsg);
    mssleep(500);
    speed = MAX_SPEED;
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
    mssleep(2500);

//derecha
    fprintf(stdout, "Izq\n");
    fflush(stdout);
    steering = 2000;
    steeringMsg.value = static_cast<int16_t>(steering);
    pubSteering.publish(steeringMsg);
    mssleep(500);
    speed = -MAX_SPEED;
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
    mssleep(3500);

//reversa Recta

    fprintf(stdout, "Recta reversa\n");
    fflush(stdout);
    steering = 1440;
    steeringMsg.value = static_cast<int16_t>(steering);
    pubSteering.publish(steeringMsg);
    mssleep(1900);

    speed = 0;
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
 

fprintf(stdout, "Parking Finaliza\n");
       fflush(stdout);
       
    ros::spinOnce();

     mssleep(sleep_msec);

    return 0;
}
