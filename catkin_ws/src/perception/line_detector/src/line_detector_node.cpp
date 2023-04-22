#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h" 
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float32MultiArray.h"

#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

ros::Publisher pub_right_line;
ros::Publisher pub_left_line;
ros::Publisher pub_loss; 

cv::Mat image_roi;
cv::Mat HLS;
cv::Mat lightness;
cv::Mat dx;
cv::Mat dy;
cv::Mat abs_dx;
cv::Mat abs_dy;
cv::Mat grad;
cv::Mat thresh_saturation;
cv::Mat thresh_red;
cv::Mat rs_binary;
cv::Mat temp_output;
cv::Mat temp_color; 
cv::Mat temp_canny; 

 std::vector<cv::Mat> channels_HLS;
 std::vector<cv::Mat> channels_BGR; 


void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
  float obs_rho_pub = 0;
  float obs_theta_pub = 0;
  float best_rho_pub = 0;
  float best_theta_pub = 0;  
  bool line_loss = true;

  cv_bridge::CvImagePtr orig;

  try{
       orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  }
  catch (cv_bridge::Exception &e){
       ROS_ERROR("cv_bridge exception %s", e.what());
       return;
  }

  std_msgs::Bool bool_msg;
  std_msgs::Float32MultiArray right_line_msg;
  std_msgs::Float32MultiArray left_line_msg;

  // Start image processing
       cv::Rect myROI(0, 200, 639, 279);      
       image_roi = orig->image(myROI);
       
       // From BGR to HLS
       cv::cvtColor(image_roi, HLS, cv::COLOR_BGR2HLS);
       // Get lightness  
       cv::split(HLS, channels_HLS);
       
       // Compute Sobel
       int scale =  1;
       int ksize = 3;
       int delta =  0;
       const cv::Size kernelSize = cv::Size(3, 3);
       cv::GaussianBlur(channels_HLS[1], channels_HLS[1], 
                               kernelSize, 0);
       cv::Sobel(channels_HLS[1], dx, CV_32FC1, 1, 0, ksize, scale, 
                   delta, cv::BORDER_DEFAULT);
       cv::GaussianBlur(channels_HLS[1], channels_HLS[1], 
                               kernelSize, 0);
       cv::Sobel(channels_HLS[1], dy, CV_32FC1, 0, 1, ksize, scale, 
                   delta, cv::BORDER_DEFAULT);   
              
       // converting back to CV_8U
       cv::convertScaleAbs(dx, abs_dx);
       cv::convertScaleAbs(dy, abs_dy);
       
       cv::addWeighted(abs_dx, 0.5, abs_dy, 0.5, 0, grad);
       cv::dilate(grad, grad, 3);
       
       // Saturation thresholding
       int thresholdVal = 50; // Originalmente en 50
       cv::threshold(channels_HLS[2], thresh_saturation, 
               thresholdVal, 255, cv::THRESH_BINARY);
 
       // Get rs_binary color from R channel of the original image
       cv::split(image_roi, channels_BGR);
       thresholdVal = 120;       
       cv::threshold(channels_BGR[2], thresh_red, thresholdVal, 255,
                cv::THRESH_BINARY);

       //bitwise red and saturation
       cv::bitwise_and(thresh_saturation, thresh_red, rs_binary); 
       cv::bitwise_and(rs_binary, grad, temp_output); // Probar más adelante

       cv::Canny(temp_output, temp_canny, 50, 200, 3);
       cv::cvtColor(temp_canny, temp_color, cv::COLOR_GRAY2BGR);
       
       // Probabilistic Line Transform
       std::vector<cv::Vec4i> linesP; // will hold the results of the detection
       cv::HoughLinesP(temp_output, linesP, 1, CV_PI/180, 80, 50, 1 ); // runs the actual detection
       
       // Compute best line and draw lines  
       std::vector<double> right_rho_vector;      
       std::vector<double> right_theta_vector;
       std::vector<double> left_rho_vector;      
       std::vector<double> left_theta_vector;         
       std::vector<cv::Vec4i> right_lines;      
       std::vector<cv::Vec4i> left_lines;
       std::vector<double> right_wts;
       std::vector<double> left_wts;       

       double right_total_wt = 0.0;              
       double left_total_wt = 0.0;
       
       int x_center = (int)round(temp_output.cols/(double)2.0);
       int y_center = (int)round(temp_output.rows/(double)2.0);       
                     
       int max_lines = 4;
       // Para procesar sólo un máximo de                      
       // lineas indicado por max_lines 
       if (linesP.size() < max_lines) 
         max_lines = linesP.size();
         
       //max_lines = linesP.size();  
                     
       for( size_t i = 0; i < max_lines; i++ ){
           cv::Vec4i l = linesP[i];
           int x1 = l[0];
           int y1 = l[1]; 
           int x2 = l[2];
           int y2 = l[3];          
                                           
           // Translate lines to bottom center
           int nx1 = x1 - x_center; // x1
           int nx2 = x2 - x_center; // x2
           int ny1 = y_center - y1; // y1
           int ny2 = y_center - y2; // y2             
          
           // Filter lines
           double rho;
           double theta;
           // Get normal form
           double A = ny2 - ny1; // y2 - y1
           double B = nx1 - nx2; // x1 - x2            
           double C = A*nx1 + B*ny1; // A*x1 + B*y1
           theta = atan2(B, A);
           rho   = C / (double)sqrt(pow(A, 2) + pow(B, 2));
           if (rho < 0){
              theta += M_PI;
              rho = -rho;
           }   
           
           cv::Vec4i nl;           
           nl[0] = nx1;
           nl[1] = ny1;
           nl[2] = nx2;
           nl[3] = ny2;
          
           double wt = 0.0;
           if ((theta > -(M_PI/2 - 0.3) && theta < -0.1) 
             || (theta > 0.1 && theta < (M_PI/2 - 0.3))){
              right_lines.push_back(nl);
              wt = sqrt(pow(nx2 - nx1, 2) + pow(ny2 - ny1, 2));
              right_wts.push_back(wt);
              right_total_wt += wt;
              // Save the parameters of every line   
              right_rho_vector.push_back(rho);
              right_theta_vector.push_back(theta);
              
              cv::line( temp_color, cv::Point(x1, y1), 
                    cv::Point(x2, y2), cv::Scalar(0,0,255), 3,
                                 cv::LINE_AA);                      
                            
           } else 
           if ((theta > (M_PI/2 + 0.3) && theta < M_PI*0.9) 
             || (theta > -0.9 * M_PI && theta < -(M_PI/2 + 0.3))){
              left_lines.push_back(nl);
              wt = sqrt(pow(nx2 - nx1, 2) + pow(ny2 - ny1, 2));
              left_wts.push_back(wt);
              left_total_wt += wt;
              // Save the parameters of every line   
              left_rho_vector.push_back(rho);
              left_theta_vector.push_back(theta); 
              
              cv::line( temp_color, cv::Point(x1, y1), 
                    cv::Point(x2, y2), cv::Scalar(255,255,0), 3,
                                 cv::LINE_AA);                  
                                                    
           } else {
              cv::line( temp_color, cv::Point(x1, y1), 
                    cv::Point(x2, y2), cv::Scalar(50,255,50), 
                              3, cv::LINE_AA);                        
           } 
           
       } // end for size_t i = 0      

       // Normalize wts  of the right lines to the interval [0,1]
       // and get rho and theta averages for right lanes
       double right_mean_rho = 0.0;
       double right_mean_theta = 0.0;    
       for( size_t j = 0; j < right_wts.size(); j++ ){
           right_wts[j] = 
               right_wts[j] / (double) right_total_wt;
           right_mean_rho += right_rho_vector[j] * right_wts[j];
           right_mean_theta += 
               right_theta_vector[j] * right_wts[j];
       }
   
       // Normalize wts  of the left lines to the interval [0,1]
       // and get rho and theta averages for left lanes
       double left_mean_rho = 0.0;
       double left_mean_theta = 0.0; 
       for( size_t j = 0; j < left_wts.size(); j++ ){
          left_wts[j] = 
               left_wts[j] / (double) left_total_wt;
               left_mean_rho += left_rho_vector[j] * left_wts[j];
               left_mean_theta += 
                       left_theta_vector[j] * left_wts[j];
       }
                      
       // Draw right line
       int shape0 = temp_output.rows;       
       int shape1 = temp_output.cols;
       int length = shape0;
       
       if( (right_mean_rho != 0) && (right_mean_theta != 0)){
          double a  = cos(right_mean_theta);
          double b  = sin(right_mean_theta);
          int x1 = (int)round(a * right_mean_rho - b 
                    * length 
                    + (shape1)/(double)1.25);
          int y1 = (int)round(shape0 - (b 
                    * right_mean_rho 
                    + a * length));
          int x2 = (int)round(a * right_mean_rho + b 
                    * length 
                    + (shape1/(double)1.25));
          int y2 = (int)round(shape0 - (b 
                    * right_mean_rho 
                    - a * length));
          cv::line( temp_color, cv::Point(x1, y1), 
                cv::Point(x2, y2), cv::Scalar(255,0,0), 4,
                          cv::LINE_AA); 

           line_loss = false;
                                 
       } // if( (right_mean_rho != 0)
           
       // Draw left line
       if( (left_mean_rho != 0) && (left_mean_theta != 0)){
          double a  = cos(left_mean_theta);
          double b  = sin(left_mean_theta);
          int x1 = (int)round(a * left_mean_rho - b 
                    * length  
                    + shape1/(double)3.25);
          int y1 = (int)round(shape0 - (b 
                    * left_mean_rho 
                    + a * length));
          int x2 = (int)round(a * left_mean_rho + b 
                    * length  
                    + shape1/(double)3.25);
          int y2 = (int)round(shape0 - (b 
                    * left_mean_rho 
                    - a * length));
          cv::line( temp_color, cv::Point(x1, y1), 
                cv::Point(x2, y2), cv::Scalar(255,0,255), 4,
                          cv::LINE_AA); 
                                 
          line_loss = false;

       } // if( (left_mean_rho != 0) 



       fprintf(stdout, "right avr rho: %lf right avr theta: %lf\n",
                  right_mean_rho, right_mean_theta);
       fprintf(stdout, "left avr rho: %lf left avr theta: %lf\n",
                  left_mean_rho, left_mean_theta);
       fflush(stdout);         
      
       // right avr rho: 202.939646 right avr theta: 0.785398
       // left avr rho: 214.442498 left avr theta: 2.356194
       double goal_rho_r = 202.939646; // Golden reference right
       double goal_theta_r = 0.785398; // Golden reference right
       double goal_rho_l = 214.442498; // Golden reference left
       double goal_theta_l = 2.356194; // Golden reference left

      //cv::imshow("Original video", orig);
       cv::imshow("Image roi", image_roi);  
       //cv::imshow("HLS video", HLS);    
       //cv::imshow("Sobel x", dx);
       //cv::imshow("Sobel y", dy);
       // cv::imshow("Abs x", abs_dx);
       // cv::imshow("Abs y", abs_dy);       
       // cv::imshow("Grad", grad);       
       // cv::imshow("Thresh Sat", thresh_saturation);         
       // cv::imshow("Thresh Red", thresh_red);  
       // cv::imshow("red_sat_binary", rs_binary);    

       //cv::imshow("Temp_output", temp_output);          
       cv::imshow("Temp color", temp_color); 
       //cv::imshow("Temp canny", temp_canny);                                    
       cv::waitKey(200); 


  // End image processing 

  bool_msg.data = line_loss;    
  pub_loss.publish(bool_msg);

  right_line_msg.data.push_back(right_mean_rho);
  right_line_msg.data.push_back(right_mean_theta);
  right_line_msg.data.push_back(goal_rho_r);
  right_line_msg.data.push_back(goal_theta_r);

  left_line_msg.data.push_back(left_mean_rho);
  left_line_msg.data.push_back(left_mean_theta);
  left_line_msg.data.push_back(goal_rho_l);
  left_line_msg.data.push_back(goal_theta_l);

  pub_right_line.publish(right_line_msg);
  pub_left_line.publish(left_line_msg);

  line_loss = true;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "line_detector_node");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1, imageCallback);

    pub_right_line = n.advertise<std_msgs::Float32MultiArray>("/line_right", 1);
    pub_left_line = n.advertise<std_msgs::Float32MultiArray>("/line_left", 1);
    pub_loss = n.advertise<std_msgs::Bool>("/bool_loss", 1);
    ros::spin(); 

    return 0;
}
