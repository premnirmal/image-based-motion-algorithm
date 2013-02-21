#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry> 
#include <string>
#include <sstream>
#include <cctype>
#include <fstream>
#include <iomanip>

using std::ofstream;
using Eigen::MatrixXd;
using namespace Eigen;
using namespace std;

/********* FUNCTION DECLARATIONS ********************************************/
Matrix3d read3DMatrix(char* buff);
MatrixXd compute_calibrated_homography(const MatrixXd& K, const MatrixXd& H21);
Matrix3d calculateRprimePOS(double d1, double d2, double d3, double x1, double x3);
Vector3d calculatetprimePOS(double d1, double d2, double d3, double x1, double x3);
double calcx1(double d1, double d2, double d3, double e1);
double calcx3(double d1, double d2, double d3, double e3);
double calcsintheta(double d1, double d2, double d3, double x1, double x3);
double calccostheta(double d1, double d2, double d3, double x1, double x3);
double calcsintheta2(double d1, double d2, double d3, double x1, double x3);
double calccostheta2(double d1, double d2, double d3, double x1, double x3);
/****************************************************************************/

/************ FUNCTIONS *****************************************************/

Matrix3d read3DMatrix(char* buff)
{
  // READ 3D MATRIX FROM FILE (COMMA DELIMITED)
  string str[3][3];
  ifstream myfile(buff);
  int a,b;
  Matrix3d MAT;
  if(!myfile) //ALWAYS TEST WHETHER FILE EXISTS OR NOT
    {
      cerr << "Error opening file or file doesn't exist!\n";
      abort();
    }
  for (a = 0; a < 3; a++)
    {
      for (b = 0; b < 3; b++)
	{ // COMMA SEPERATED DATA
	  getline(myfile,str[a][b],',');
	  // CONVERT std::string TO double AND INSERT INTO MATRIX:
	  MAT(a,b) = atof(str[a][b].c_str());
	}
    }
  return MAT;
}

MatrixXd compute_calibrated_homography(const MatrixXd& K, const MatrixXd& H21)
{
  return K.inverse() * H21 * K;
}

Matrix3d calculateRprimePOS(double d1, double d2, double d3, double x1, double x3)
{
  double x2 = 0;
  Matrix3d R;
  if (x1 == x2 && x2 == 0)
    {
      cout << "NO ROTATION!\n";
      abort();
    }
  else
    {
      double sintheta, costheta;
      sintheta = calcsintheta(d1,d2,d3,x1,x3);
      costheta = calccostheta(d1,d2,d3,x1,x3);
      R << costheta, 0, -sintheta,
	0, 1, 0,
	sintheta, 0, costheta;
      return R;	    
    }//end else
}

Vector3d calculatetprimePOS(double d1, double d2, double d3, double x1, double x3)
{
  Vector3d X;
  X << x1,0,-x3;
  return (d1 - d3) * X; // TRANSLATION VECTOR t'  
}

double calcx1(double d1, double d2, double d3, double e1)
{
  return e1 * sqrt( ((d1*d1)-(d2*d2))/((d1*d1)-(d3*d3)) );
}

double calcx3(double d1, double d2, double d3, double e3)
{
  return e3 * sqrt( ((d2*d2)-(d3*d3))/((d1*d1)-(d3*d3)) );
}

double calcsintheta(double d1, double d2, double d3, double x1, double x3)
{
  return (d1 - d3) * x1 * x3 / d2;
}

double calccostheta(double d1, double d2, double d3, double x1, double x3)
{
  return ( (d1 * x3 * x3) + (d3 * x1 * x1) ) / d2;
}

Matrix3d calculateRprimeNEG(double d1, double d2, double d3, double x1, double x3)
{
  double x2 = 0;
  Matrix3d R;
  if (x1 == x2 && x2 == 0)
    {
      cout << "NO ROTATION!\n";
      abort();
    }
  else
    {
      double sintheta, costheta;
      sintheta = calcsintheta2(d1,d2,d3,x1,x3);
      costheta = calccostheta2(d1,d2,d3,x1,x3);
      R << costheta, 0, sintheta,
	0, -1, 0,
	sintheta, 0, -costheta;
      return R;	    
    }//end else
}

Vector3d calculatetprimeNEG(double d1, double d2, double d3, double x1, double x3)
{
  Vector3d X;
  X << x1,0,x3;
  return (d1 + d3) * X; // TRANSLATION VECTOR t'  
}

double calcsintheta2(double d1, double d2, double d3, double x1, double x3)
{
  return (d1 + d3) * x1 * x3 / d2;
}

double calccostheta2(double d1, double d2, double d3, double x1, double x3)
{
  return ( (d3 * x1 * x1) - (d1 * x3 * x3) ) / d2;
}

/*******************************************************************************/
