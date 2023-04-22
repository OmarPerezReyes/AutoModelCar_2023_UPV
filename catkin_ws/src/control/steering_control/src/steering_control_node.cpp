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

const int MAX_SPEED = 40;
const int sleep_msec = 200;
const int STEERING_CENTER = 1440;

ros::Publisher pubSpeed;
ros::Publisher pubSteering;

autominy_msgs::SteeringPWMCommand steeringMsg;
autominy_msgs::SpeedPWMCommand speedMsg;

double goal_rho_r = 0.0; // Golden reference right
double goal_theta_r = 0.0; // Golden reference right
double right_mean_rho = 0.0;
double right_mean_theta = 0.0;

double goal_rho_l = 0.0; // Golden reference left
double goal_theta_l = 0.0; // Golden reference left
double left_mean_rho = 0.0;
double left_mean_theta = 0.0;

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


void stopCallback(const std_msgs::Bool msg){
  int speed;

  if (msg.data){
        mssleep(3500);
        speed = 0;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(4500);
  } else {
        speed = MAX_SPEED;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(sleep_msec);

   }
}

void lossCallback(const std_msgs::Bool msg){
  int steering;

  /*if (msg.data){
        mssleep(sleep_msec);
        steering = STEERING_CENTER;
        steeringMsg.value = static_cast<int16_t>(steering);
        pubSteering.publish(steeringMsg);
        mssleep(sleep_msec);
  }*/
}

void right_line_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  right_mean_rho = msg->data[0];
  right_mean_theta = msg->data[1]; 
  goal_rho_r = msg->data[2]; // Golden reference right
  goal_theta_r = msg->data[3]; // Golden reference right

  fprintf(stdout, "Right: %lf %lf %lf %lf\n",
                   msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  fflush(stdout);            

}

void left_line_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  left_mean_rho = msg->data[0];
  left_mean_theta = msg->data[1]; 
  goal_rho_l = msg->data[2]; // Golden reference right
  goal_theta_l = msg->data[3]; // Golden reference right

  fprintf(stdout, "Left: %lf %lf %lf %lf\n",
                  msg->data[0], msg->data[1], msg->data[2], msg->data[3]);  
  fflush(stdout);               

}


//
int main(int argc, char* argv[]){

 int speed = MAX_SPEED;

 double error_rho_r = 0.0;
 double error_theta_r = 0.0;
 double error_rho_l = 0.0;
 double error_theta_l = 0.0;
 double error_rho = 0.0;
 double error_theta = 0.0;       
 double steer = STEERING_CENTER;
 int steering = STEERING_CENTER;
 double k_rho = 0.001; // Proportional constant (This value  MUST BE TUNED)
 double k_theta = 0.01; // Proportional constant (This value  MUST BE TUNED)

    ros::init(argc, argv, "steering_control_node");
    ros::NodeHandle n;

    ros::Subscriber sub_right_line = n.subscribe("/line_right", 1,
                                         right_line_callback);
    ros::Subscriber sub_left_line = n.subscribe("/line_left", 1, 
                                         left_line_callback);
    ros::Subscriber stop = n.subscribe("/bool_stop", 1, stopCallback);
    ros::Subscriber loss = n.subscribe("/bool_loss", 1, lossCallback);

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
    
    while (ros::ok()){

       error_rho_r = right_mean_rho - goal_rho_r;
       error_theta_r = right_mean_theta - goal_theta_r;
       error_rho_l = goal_rho_l - left_mean_rho;
       error_theta_l = goal_theta_l - left_mean_theta;
       error_rho = 0.0;
       error_theta = 0.0;       
  
       /*if (right_mean_rho != 0 && left_mean_rho != 0){
          error_rho   = (error_rho_l + error_rho_r)/2;
          error_theta = (error_theta_l + error_theta_r)/2; 
       } else 
       if (right_mean_rho != 0){
          error_rho   = error_rho_r;
          error_theta = error_theta_r;       
       } else 
       if (left_mean_rho != 0){
          error_rho   = error_rho_l;
          error_theta = error_theta_l;       
       }        
*/

       error_rho   = error_rho_r;
       error_theta = error_theta_r;       
  
       if (left_mean_rho != 0){
          error_rho   = error_rho_l;
          error_theta = error_theta_l;       
       }     

       // error_rho: -65.729876 error_theta: 0.191362
       steer = (-k_rho*error_rho 
                   - k_theta*error_theta) * 100;   
       steer *= -47;
       steer += STEERING_CENTER;

       if (steer > 2060)
          steer = 2060;
       if (steer < 880)
          steer = 880;

       steering = round(steer);
       
       fprintf(stdout, "steering: %d steer: %lf\n", steering, steer);
       fflush(stdout);

/*
       fprintf(stdout, "error_rho_r: %lf error_theta_r: %lf\n",
                  error_rho_r, error_theta_r);    
       fprintf(stdout, "error_rho_l: %lf error_theta_l: %lf\n",
                  error_rho_l, error_theta_l);                      
       fprintf(stdout, "error_rho: %lf error_theta: %lf\n",
                  error_rho, error_theta); 
*/       

       autominy_msgs::SteeringPWMCommand steeringMsg;
       steeringMsg.value = static_cast<int16_t>(steering);
       pubSteering.publish(steeringMsg);

       ros::spinOnce();

       mssleep(sleep_msec);

    }


    return 0;
}
