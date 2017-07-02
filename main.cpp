#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>

#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/video/tracking.hpp"

#include "raspicam/raspicam_cv.h"

#define PI 3.14159

using namespace std;
using namespace cv;
using namespace raspicam;

int lowThreshold = 90;
unsigned int highSpeed = 200;
unsigned int lowSpeed = 160;
int imageWidth = 320;
int minWidth = 30;

static volatile bool running = true;

int zb;



int main(int argc, char** argv) 
{
	
	

	RaspiCam_Cv Camera;	
 //   Camera.set (CV_CAP_PROP_FORMAT, CV_8UC1 );
 //   Camera.set (CV_CAP_PROP_GAIN, 50 );    
    Camera.set (CV_CAP_PROP_FRAME_WIDTH, 640 );
    Camera.set (CV_CAP_PROP_FRAME_HEIGHT, 480 );
    //Camera.set (CV_CAP_ROTATION, 180 );
 //   Camera.set (CV_CAP_PROP_BRIGHTNESS, 60 );
//	Camera.set (CV_CAP_PROP_CONTRAST, 70 );	 
//	Camera.set (CV_CAP_PROP_GAIN, 50 );
    
    if ( !Camera.open() ) {
        fprintf(stderr, "Failed to init open camera\n");   
	//	exit(-1);
    }
    



	    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

  int iLowH = 170;
 int iHighH = 179;

  int iLowS = 150; 
 int iHighS = 255;

  int iLowV = 60;
 int iHighV = 255;

createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
 createTrackbar("HighH", "Control", &iHighH, 179);

  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
 createTrackbar("HighS", "Control", &iHighS, 255);

  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
 createTrackbar("HighV", "Control", &iHighV, 255);


  int iLastX = -1; 
 int iLastY = -1;

  //Capture a temporary image from the camera
 Mat imgTmp;
	Camera.grab();
        Camera.retrieve(imgTmp);

  //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 

    while (true)
    {
        Mat imgOriginal;
	Camera.grab();
        Camera.retrieve(imgOriginal);


      int pusher = 240;
    Mat imgHSV;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
   Mat imgThresholded;

   inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

   //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  vector<vector<Point> > contours;
  vector<Vec4i > hierarchy;

  findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

  double maxArea = 0;
  int biggestContour = -1;

  for(int i=0; i<contours.size(); i++){


    double area = contourArea(contours[i]);
    if(area > maxArea){
		maxArea = area;
		biggestContour = i;
	}
  }
if(biggestContour > -1 && maxArea > 100){
    drawContours(imgOriginal, contours, biggestContour, Scalar(255,0,0), 2, 8, hierarchy,0, Point());
   //Calculate the moments of the thresholded image
  Moments oMoments = moments(contours[biggestContour]);

   double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

   // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 100)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;        
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {

    double vectx = posX - iLastX;
    double vecty = posY - iLastY;
    double mag = sqrt((vectx * vectx) + (vecty * vecty));
    
    if(mag > 0.5){
      //only if moving
      double uvectx = vectx / mag;
      double uvecty = vecty / mag;
      double dir = atan2(vecty, vectx) * 180 / PI;
      double dirR = atan2(vecty, vectx);
      printf("dir is %f vectx is %f vecty is %f\n", dir, vectx, vecty);
      Scalar lineCol = Scalar(0,255,0);
     

      if(dir < 90 && dir > -90){
        double a = (640-posX) * tan(dirR);
        
        if(((posY + a) > 0) && ((posY + a) < 480)){
	  lineCol = Scalar(0,0,255);
          printf("%f %d direct \n",a, posY);
          pusher = posY + a;
        }
      }
      //Draw a red line from the previous point to the current point
      line(imgOriginal, Point(posX, posY), Point(posX + (uvectx * 500), posY + (uvecty * 500)), lineCol, 2);
    }   
}

	  circle(imgOriginal, Point(640, pusher), 50, Scalar(255,255,0),-1);
    iLastX = posX;
   iLastY = posY;
  }



 //  imgOriginal = imgOriginal + imgLines;
}
  imshow("Original", imgOriginal); //show the original image

        if (waitKey(5) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            break; 
       }
    }

   return 0;


}
