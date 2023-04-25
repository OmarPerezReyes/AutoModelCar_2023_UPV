#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <signal.h>
#include <termios.h>

#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringPWMCommand.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

ros::Publisher pubSpeed;
ros::Publisher pubSteering;
autominy_msgs::SpeedPWMCommand speedMsg;
autominy_msgs::SteeringPWMCommand steeringMsg;

int speed = 0;
int steering = 1500;
bool die = false;

void publishSpeedMsg() {
    speedMsg.value = speed;
    pubSpeed.publish(speedMsg);
}

void publishSteeringMsg() {
    steeringMsg.value = steering;
    pubSteering.publish(steeringMsg);
}

void initializeKeyboard() {
    int kfd = 0;
    char c;
    struct termios cooked, raw;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    ROS_INFO("Reading from keyboard");
    ROS_INFO("---------------------------");
    ROS_INFO("Moving around:");
    ROS_INFO("       w    ");
    ROS_INFO("   a   s   d");
    ROS_INFO("---------------------------");
}

void keyControl(ros::Rate r) {
    initializeKeyboard();
    char c;
    int kfd = 0;
    while (ros::ok() && !die) {
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }
        switch (c) {
            case KEYCODE_W:
                if (speed < 100) {
                    speed += 10;
                }
                publishSpeedMsg();
                break;
            case KEYCODE_S:
                speed = 0;
                publishSpeedMsg();
                break;
            case KEYCODE_A:
                if (steering > 1000) {
                    steering -= 100;
                }
                publishSteeringMsg();
                break;
            case KEYCODE_D:
                if (steering < 2000) {
                    steering += 100;
                }
                publishSteeringMsg();
                break;
            default:
                die = true;
                break;
        }
        r.sleep();
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle n;
    ros::Rate r(50); // 50 Hz
    pubSpeed = n.advertise<autominy_msgs::SpeedPWMCommand>("/actuators/speed_pwm", 10);
    pubSteering = n.advertise<autominy_msgs::SteeringPWMCommand>("/actuators/steering_pwm", 10);
    keyControl(r);
    speedMsg.value = 0;
    publishSpeedMsg();
    steeringMsg.value = 1500;
    publishSteeringMsg();
    ros::shutdown();
    return 0;
}
