#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <stdio.h>

using namespace std;

int main()
{
  char name[100];
  char buff[100];// buffer to store source image name and location
  char buff2[100]; //buffer to store destination image name and location
  int i,n;
  printf("Enter image name (without index):\n");
  cin >> name;
  printf("Enter number of images:\n");
  cin >> n;
	
  for (i = 1; i <= n; i++)
    {
      sprintf(buff, "%s%d.jpg", name, i);
      sprintf(buff2, "cropped%s%d.jpg", name, i);
	
      /* Load image */
      IplImage* img = cvLoadImage(buff, 1);
 
      /* Set image ROI */
      cvSetImageROI(img, cvRect(50,22,580,454) );
  
      IplImage *cropped = cvCreateImage(cvGetSize(img),
					img->depth,
					img->nChannels);
 
      /* copy subimage */
      cvCopy(img, cropped, NULL);
 
      /* always reset the Region of Interest */
      cvResetImageROI(img);

      //save cropped image
      cvSaveImage(buff2, cropped);
      cvReleaseImage(&img);
      cvReleaseImage(&cropped);	
    }

  printf("All images successfully cropped and saved.\n");
  return 0;
}
