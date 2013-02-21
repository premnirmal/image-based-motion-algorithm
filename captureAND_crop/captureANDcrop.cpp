#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;


// A Simple Camera Capture Framework
int main() 
{
  char name[100];
  char buff[100];
  int i = 1;
  char ch;

  printf("Enter name for file:\n");
  scanf ("%s", name);  

  while (1)
    {  
      cout << "Position the robot and camera, then press 't' to take next image, 'q' to exit\n";
      cin >> ch;
      while( ch != 't' && ch != 'q')
	{
	  cout << "Invalid key, please try again\n";
	  cin >> ch;
	}
      if (ch == 'q')
	return 0;
      else if (ch == 't')
	{
	  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
	  if( !capture ) {
	    fprintf( stderr, "ERROR: capture is NULL \n" );
	    getchar();
	    return -1;
	  }
  
	  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );
	  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );

	  sprintf(buff, "../images/%s%d.jpg", name, i);
  
	  IplImage* frame = cvQueryFrame( capture );
	  if( !frame ) 
	    {
	      fprintf( stderr, "ERROR: frame is null...\n" );
	      return 0;
	    }
	
	      /* Set image ROI */
	      cvSetImageROI(frame, cvRect(50,22,580,454) );

	      IplImage *cropped = cvCreateImage(cvGetSize(frame),
						frame->depth,
						frame->nChannels);
	      /* copy subimage */
	      cvCopy(frame, cropped, NULL);
	      /* always reset the Region of Interest */

	      cvResetImageROI(frame);
	      //save cropped image                                                                                                           
	      cvSaveImage(buff, cropped);
	      cout << "Image saved as " << buff << "\n";
	      
	      cvReleaseImage(&frame);
	      cvReleaseImage(&cropped);
     
	  cvReleaseCapture( &capture );
	  i++;
	}
      }// end while(true)

  
  return 0;
}

