#include <ctype.h>
#include <stdio.h>
#include <pthread.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/times.h>

#include <opencv2/opencv.hpp>
  
// Get the color of a pixel (x,y) of the original image
void get_pixel_color(const cv::Mat &frame, int x, 
                                 int y, unsigned char *color); 

// set the color of a pixel (x,y) of the original image
void set_pixel_color(const cv::Mat &frame, int x, 
                   int y, unsigned char *color);
        
// Get the color of a pixel (x,y) of an image
unsigned char get_pixel_grey(const cv::Mat &frame, int x, int y);      

// Get the color of a pixel (x,y) of an image
unsigned char set_pixel_grey(const cv::Mat &frame, int x, 
                              int y, unsigned char intensity);
               
// Filter pixel colors defined by color and returns the number of pixels found               
int filter_red_color(const cv::Mat &frame);


//
unsigned char get_pixel_gray_IplImage(IplImage *frame, int x, int y);

//
unsigned char set_pixel_gray_IplImage(IplImage *frame, int x, int y, unsigned char intensity);


