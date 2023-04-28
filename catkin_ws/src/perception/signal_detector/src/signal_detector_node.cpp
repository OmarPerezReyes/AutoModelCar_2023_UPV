#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h" 
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include <ctype.h>
#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/times.h>

#include <signal.h>
#include <unistd.h>

#include <iostream>

#include "sign_detection.h"

#include "sift.h"
#include "kdtree.h"
#include "imgfeatures.h"
#include "utils.h"
    
ros::Publisher pub_stop;

// Time structure
struct timezone tz = {0, 0};
struct timeval old_time; 

// Constants
const int MAX_STRING_LENGTH = 256;
const int MAX_NUM_SIGNS = 5;
char signs[MAX_NUM_SIGNS][MAX_STRING_LENGTH];

// Flow control variables
int NUM_SIGNS  = 1;
bool die = false;
int videodevice = 0;

struct feature *sign_feat[MAX_NUM_SIGNS], *feat;
int feature_type = FEATURE_LOWE;
int num_sign_features[MAX_NUM_SIGNS];  

IplImage iplimage;    


void imageCallback(const sensor_msgs::Image::ConstPtr& msg){

  cv_bridge::CvImagePtr orig;

  try{
       orig = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  }
  catch (cv_bridge::Exception &e){
       ROS_ERROR("cv_bridge exception %s", e.what());
       return;
  }

  bool decision_stop = false;
  std_msgs::Bool bool_msg;

  int intvls = SIFT_INTVLS;
  double sigma = SIFT_SIGMA;
  double contr_thr = SIFT_CONTR_THR;
  int curv_thr = SIFT_CURV_THR;
  int img_dbl = SIFT_IMG_DBL;
  int descr_width = SIFT_DESCR_WIDTH;
  int descr_hist_bins = SIFT_DESCR_HIST_BINS;
  struct kd_node *kd_root;
  struct feature **nbrs;
  // Sift 
  struct feature *current_feat = NULL;  

  int matches = 0;

  cv::Mat gray;  
  // Define the images that will be used first

  //fprintf(stdout, "New image\n");
  //fflush(stdout);
  
  // RGB to gray scale
  cv::namedWindow( "Original image", cv::WINDOW_AUTOSIZE );

    // Filtering red color
//    int num_pixels = filter_red_color(orig);
//    fprintf( stdout, "Found %d red pixels.\n",num_pixels);
//    fflush(stdout);          
    //cv::Rect myROI(200,100,439,279);
    //orig->image=orig->image(myROI);
    cv::cvtColor(orig->image, gray, cv::COLOR_BGR2GRAY);

    int x,y;
    char intensity;
    for (x = 0; x < iplimage.width; x++){
       for (y = 0; y < iplimage.height; y++) {
            intensity = get_pixel_grey(gray, x, y);        
            set_pixel_gray_IplImage(&iplimage, x, y, intensity);
       }
    } 
    
    gettimeofday (&old_time, &tz);

    // Extract features from the current view
    int num_current_feat = _sift_features( &iplimage, &current_feat, 
                           intvls, sigma, contr_thr, curv_thr, img_dbl,
                           descr_width, descr_hist_bins );

    //fprintf( stdout, "Found %d features in the current view.\n",
               //num_current_feat);
    //fflush(stdout);

    if (num_current_feat > 0){
    
       // Proceed with detection 
       int i;
       for (i = 0; i < NUM_SIGNS; i++){
           matches = 0;    

           kd_root = kdtree_build( current_feat, num_current_feat );
           int j = 0;
           for( j = 0; j < num_sign_features[i]; j++ ){
               feat = sign_feat[i] + j;
               int k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs,
                        KDTREE_BBF_MAX_NN_CHKS );
               if ( k == 2 ){
	          double d0 = descr_dist_sq( feat, nbrs[0] );
	          double d1 = descr_dist_sq( feat, nbrs[1] );
	          if( d0 < d1 * NN_SQ_DIST_RATIO_THR ){
                      matches++;
	              sign_feat[i][j].fwd_match = nbrs[0];
	          }
	       }
               free( nbrs );
           } // ( j = 0; j < num_sign_feat[i])


           fprintf (stdout, "Num matches %s: %d\n", signs[i], matches); 
           fflush(stdout);

           kdtree_release(kd_root);  

       } // (i = 0; i < NUM_SIGNS; i++)

    } // (num_current_feat > 0)
   
   
    if ( matches >= 4){
       bool_msg.data = true; // stop
               
    }

    pub_stop.publish(bool_msg);  

    bool_msg.data = false; // go

    // Display elapsed time  
    //fprintf (stdout, "Detection loop took %.2f s\n", elapsed_time()); 
    //fflush(stdout);

    draw_features( orig->image, current_feat, num_current_feat );

    //cv::imshow("Original image", orig->image);         
    //cv::waitKey(1); 
 
}

int main(int argc, char **argv){

  ros::init(argc, argv, "signal_detector_node");

  ros::NodeHandle n;

  // signs names
  strcpy(signs[0], "stop_original_hess");
  strcpy(signs[1], "stop_original_scaled");

  iplimage.alphaChannel = 0;
  iplimage.align = 8;
  iplimage.dataOrder = 0;
  iplimage.ID = 0;  
  iplimage.nSize = 144;    
  iplimage.origin = 0;  
  iplimage.depth = IPL_DEPTH_8U; // Const value 8
  iplimage.nChannels = 1; //¿gray.channels()?
  iplimage.width = 640;
  iplimage.height = 480;
  iplimage.widthStep = 640;
  iplimage.imageSize = iplimage.height * iplimage.widthStep;

  iplimage.imageData = new char[iplimage.imageSize];  
  iplimage.imageDataOrigin = iplimage.imageData;

  //fprintf(stdout, "Reading keypoint DB... ");
  //fflush(stdout); 

  int i;
  char path[MAX_STRING_LENGTH];
  // Import features
  for (i = 0; i < NUM_SIGNS; i++){
     strcpy(path, "./src/perception/signal_detector/src/BDStopSign/");	     	
     strcat(path, signs[i]);  
     strcat(path, ".key");
     if ((num_sign_features[i] = import_features( path, feature_type,
            &sign_feat[i])) == -1){
        //fprintf(stdout, 
               //"Error: cannot find file %s\n continuing...\n", path);
        //fflush(stdout);          
        continue; 
     } 
  } // for    

  //fprintf(stdout, "Done.");
  //fflush(stdout); 

  ros::Subscriber sub = n.subscribe("/realsense/color_raw", 1, imageCallback);
  pub_stop = n.advertise<std_msgs::Bool>("/bool_stop", 1);

  ros::spin(); 

  // Release mem
  for (i = 0; i < NUM_SIGNS; i++){   
     free(sign_feat[i]);
  }  
  delete iplimage.imageData;

 return 0;
}


//
double elapsed_time (void){
 struct timeval curr_time;
 double time;

     gettimeofday (&curr_time, &tz);
     time = (double) (curr_time.tv_sec - old_time.tv_sec);
     time += ((double) (curr_time.tv_usec - old_time.tv_usec) * 1e-6);
 
  return time;

}

