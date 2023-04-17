/*   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *   
 */

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

#include "utils.h"

// Get the color of a pixel (x,y) of the original image
void get_pixel_color(const cv::Mat &frame, int x, int y, unsigned char *color){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }

   // Get data buffer 
   uchar* ptr = (uchar *)frame.data;

   // Clean memory
   //memset(color, 0, 3);
   offset += y * frame.step;
   // Offset
   offset +=  frame.channels() * x;

   // Assume BGR pixel order
   color[0] = (unsigned char)ptr[offset++];
   color[1] = (unsigned char)ptr[offset++];
   color[2] = (unsigned char)ptr[offset];

}


// Set the color of a pixel (x,y) of the original image
void set_pixel_color(const cv::Mat &frame, int x, int y, unsigned char *color){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }
   
      // Get data buffer 
   uchar* ptr = (uchar *)frame.data;

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   // Assume BGR pixel order
   ptr[offset++] = (unsigned char)color[0];
   ptr[offset++] = (unsigned char)color[1];
   ptr[offset] = (unsigned char)color[2];

}


// Get the color of a pixel (x,y) of an image
unsigned char get_pixel_grey(const cv::Mat &frame, int x, int y){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   return (frame.data[offset]);

}

//
unsigned char get_pixel_gray_IplImage(IplImage *frame, int x, int y){

   int offset = 0; 
   unsigned char gray = 0;

   if (x < 0) { x = 0; } 
   if (x >= frame->width) { x = frame->width - 1; }
   if (y < 0) { y = 0; } 
   if (y >= frame->height) { y = frame->height - 1; }

   offset += y * frame->widthStep;   
   // Offset 
   offset += x;

   gray = frame->imageData[offset];

  return gray;

}


// Get the color of a pixel (x,y) of an image
unsigned char set_pixel_grey(const cv::Mat &frame, int x, int y, unsigned char intensity){

   int offset = 0;

   if (x < 0) { return 0; }
   if (x >= frame.cols) { return 0;  }
   if (y < 0) { return 0;  }
   if (y >= frame.rows) { return 0;  }

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   // Assume BGR pixel order
   frame.data[offset] = intensity;

 return 1;

}

//
unsigned char set_pixel_gray_IplImage(IplImage *frame, int x, int y, unsigned char intensity){

   int offset = 0; 
   unsigned char gray = 0;

   if (x < 0) { return 0; }
   if (x >= frame->width) { return 0; }
   if (y < 0) { return 0; } 
   if (y >= frame->height) { return 0; }

   offset += y * frame->widthStep;   
   // Offset 
   offset += x;

   frame->imageData[offset] = intensity;

  return 1;

}


// Filter pixel colors defined by color and returns the number of pixels found
int filter_red_color(const cv::Mat &frame){

    int num_pixels = 0;

    // Filtering red color
    int x, y;
    unsigned char color[3];
    for (x = 0; x < 640; x++){
        for (y = 0; y < 480; y++){
            get_pixel_color(frame, x, y, color);
 
            // Assume BGR order
            if (!(color[0] < 42 && color[1] < 48 && color[2] > 115)){
               color[0] = 255;
               color[1] = 255;
               color[2] = 255;
               set_pixel_color(frame, x, y, color);    
            } else // if 
               num_pixels += 1;
         } // for y
     } // for x    

  return num_pixels;

}




