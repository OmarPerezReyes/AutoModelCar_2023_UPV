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
double rho_r = 0.0;
double theta_r = 0.0;

double goal_rho_l = 0.0; // Golden reference left
double goal_theta_l = 0.0; // Golden reference left
double rho_l = 0.0;
double theta_l = 0.0;

bool road_junction = false;

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

void junctionCallback(const std_msgs::Bool msg){
  int speed;

   road_junction = msg.data;

}



void stopCallback(const std_msgs::Bool msg){
  int speed;

  if (msg.data){
        mssleep(3000);
        speed = 0;  
        speedMsg.value = speed;
        pubSpeed.publish(speedMsg);
        mssleep(5000);
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

  rho_r = msg->data[0];
  theta_r = msg->data[1]; 
  goal_rho_r = msg->data[2]; // Golden reference right
  goal_theta_r = msg->data[3]; // Golden reference right

  fprintf(stdout, "Right: %lf %lf %lf %lf\n",
                   msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
  fflush(stdout);            

}

void left_line_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){

  rho_l = msg->data[0];
  theta_l = msg->data[1]; 
  goal_rho_l = msg->data[2]; // Golden reference right
  goal_theta_l = msg->data[3]; // Golden reference right

  //fprintf(stdout, "Left: %lf %lf %lf %lf\n", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);  
  //fflush(stdout);               

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
    ros::Subscriber junction = n.subscribe("/bool_junction", 1, junctionCallback);

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

     //if (theta_r < 0) theta_r += 2 * M_PI;
/*
     rho_r = 67.276276;
     theta_r = -1.150287;
     goal_rho_r = 152.939651;
     goal_theta_r = 0.285398;
*/
     if (theta_r != 0 || theta_l != 0){
      
       error_rho_r = rho_r - goal_rho_r;
       error_theta_r = theta_r - goal_theta_r;
       error_rho_l = goal_rho_l - rho_l;
       error_theta_l = goal_theta_l - theta_l;
/*
       fprintf(stdout, "rho_r: %lf\n", rho_r);
       fprintf(stdout, "theta_r: %lf\n", theta_r);
       fprintf(stdout, "goal_rho_r: %lf\n", goal_rho_r);
       fprintf(stdout, "goal_theta_r: %lf\n", goal_theta_r);
       fprintf(stdout, "error_rho_r: %lf\n", error_rho_r);
       fprintf(stdout, "error_theta_r: %lf\n", error_theta_r);
       fflush(stdout);
*/
       //error_rho = 0.0;
       //error_theta = 0.0;       
  
       error_rho  = error_rho_r;
       error_theta = error_theta_r;  
/*
       fprintf(stdout, "error_rho: %lf\n", error_rho);
       fprintf(stdout, "error_theta: %lf\n", error_theta);
       fflush(stdout);
*/
       int sign = 1;
       if (theta_r < 0) sign = -1;
/*
       fprintf(stdout, "Sign: %d\n", sign);
       fflush(stdout);
*/
       if (rho_l != 0){
          error_rho   = error_rho_l;
          error_theta = error_theta_l;       
          sign = 1;
       }  
	
       // error_rho: -65.729876 error_theta: 0.191362
       steer = k_rho * error_rho + k_theta * error_theta;   
       steer *= 100;
       steer *= 47;
/*
       fprintf(stdout, "Steer original: %lf\n", steer);
       fflush(stdout);
*/
       steer = STEERING_CENTER + sign * steer;

       steering = round(steer);

       if (steering > 2060) steering = 2060;
       if (steering < 880) steering = 880;
     
       fprintf(stdout, "steering: %d steer: %lf\n", steering, steer);
       fflush(stdout);
   
     } // if (theta_r != 0 || theta_l != 0)  

     if (road_junction){

       fprintf(stdout, "Junction is true\n");
       fflush(stdout);   
       steering = STEERING_CENTER;
       mssleep(500);
     }

     autominy_msgs::SteeringPWMCommand steeringMsg;
     steeringMsg.value = static_cast<int16_t>(steering);
     pubSteering.publish(steeringMsg);

     road_junction = false;

     ros::spinOnce();

     mssleep(sleep_msec);

     //break;
 
   } // End while


    return 0;
}
