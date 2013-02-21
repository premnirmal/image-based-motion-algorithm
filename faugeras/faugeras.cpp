/* PREM NIRMAL
   FRCV, COMP. SCI. DEPT.
   FORDHAM UNIV. BRONX NY 10458
*/

/* COMPLETE MOTION ALGORITHM FROM FAUGERAS ET. AL
   IMPLEMENTED IN C++ USING EIGEN3
   INPUT IS HOMOGRAPHY MATRIX AND CAMERA INTRINSIC MATRIX
   RETURNS THE ROTATION MATRIX AND TRANSLATION VECTOR
*/

/* This version is a modified version, generating output in
   tabular format, showing all cases -PN 04/11/12
*/

#include "faugeras.h"

#define EPS 1E-5

int main()
{
  int depth, pixelspermm;
  
  pixelspermm = 1; //126; // CALCULATED PIXELS PER MM USING CCD CHIP SIZE
  depth = 2400; // DEPTH OF SCENE IN MM

  // PROMPT HOMOGRAPHY FILE AND OUTPUT FILENAMES AS INPUT
  string homographyFile;
  cout << "ENTER HOMOGRAPHY FILE NAME (WITHOUT .txt)\n";
  cin >> homographyFile;

  /*** EXTRACT INDEX FROM INPUT AND USE 
       THE INDEX FOR OUTPUT FILE NAME ************/
  string temp;
  int index = 0;
  for (int j = 0; j < homographyFile.size(); j++)
    {
      if (isdigit(homographyFile[j]))
	{
	  for (int a = j; a < homographyFile.size(); a++)
	    temp += homographyFile[a];
	  break;
	}
    }
  istringstream stream(temp);
  stream >> index;
  /**********************************************/

  // CONVERT STRINGS TO CHARACTER ARRAYS
  char homography[50];
  char output[50]; 
  char intrinsic[50];
  sprintf( homography, "../homographies/%s.txt", homographyFile.c_str() );
  sprintf( output, "OUTPUT%d.txt", index );
  sprintf( intrinsic,"intrinsic.txt" );

  // OUTPUT FILE MUST BE CREATED AND OPENED
  ofstream fout;
  fout.open(output);  
  fout << "*******" << output << "*******" << "\n"; // HEADER
  
  /* READ HOMOGRAPHY AND INTRINSIC MATRIX FROM FILE */
  Matrix3d K, H21; // CAMERA INTRINSIC MATRIX, HOMOGRAPHY MATRIX
  H21 = read3DMatrix(homography);
  K = read3DMatrix(intrinsic);

  /**************************************************************************/

  /* COMPUTE CALIBRATED HOMOGRAPHY */

  Matrix3d H; // CALIBRATED HOMOGRAPHY H
  H = compute_calibrated_homography(K, H21); // COMPUTE CALIBRATED HOMOGRAPHY
  // PRINT CALIBRATED HOMOGRAPHY H
  fout << "H:\n" << H << "\n";

  /**************************************************************************/

  /* SINGULAR VALUE DECOMPOSITION OF H */
  
  Vector3d Sing; //diagonal entries in singular value matrix S fron SVD of H
  Matrix3d U,Vt,S; //U,V^T,S in SVD of H = U S V^T
  // SINGULAR VALUE DECOMPOSITION FUNCTION BUILT IN EIGEN
  JacobiSVD<MatrixXd> svd(H, ComputeFullU | ComputeFullV);
  Sing = svd.singularValues();
  Vt = svd.matrixV().transpose();
  U = svd.matrixU();
  // CONSTRUCT SINGULAR VALUE (DIAGONAL) MATRIX
  S << Sing(0,0), 0, 0,
    0, Sing(1,0), 0,
    0, 0, Sing(2,0);
  double s1,s2,s3; // SINGULAR VALUES
  s1 = Sing(0,0);
  s2 = Sing(1,0);
  s3 = Sing(2,0);

  cout << "Singular Values:\n" << S << endl;
  fout << "Singular Values:\n" << S << endl;
  fout << "U:\n" << U << endl;
  fout << "V^t :\n" << Vt << endl;
  fout<<setfill('-')<<setw(50)<<"-"<<endl;
  /**************************************************************************/

  /* RETURN ERROR MESSAGE IF ANY OF THE SINGULAR VALUES ARE EQUAL 
     OR NEGATIVE */

  if ( fabs(s1 - s2) < EPS || fabs(s1 - s3) < EPS || fabs(s2 - s3) < EPS || s1 < 0 || s2 < 0 || s3 < 0 )
    {
      cerr << "ERROR, some of the singular values are equal, or negative!\n";
      return -1;
    }

  /**************************************************************************/

  /***** COMPUTE R AND t ****************************************************/

  Matrix3d Rprime, R;
  Vector3d tprime, t;
  double dprime, d, costheta, sintheta;
  double x1,x2,x3;
  int e1, e3;
  double theta, theta2; // ROTATION IN DEGREES
  int counter = 1; // INDEX NUMBER OF CASES/SUB-CASES
  double s; // s = det(U) * det(V)
  s = U.determinant() * svd.matrixV().determinant(); // s FROM PAGE 6 OF FAUGERAS' PAPER
  dprime = s2;
  fout << setfill(' ');
  fout << "# |" << setw(6) << "x1" << "|" << setw(6) << "x3" << "|" << setw(8) << "atan" << "|" << setw(8) << "atan2" << "|" << setw(15) << "translation" << setw(6) << "|\n";
  /* 2 CASES: dprime = +- s2 */
  for (double i = dprime; i >= -fabs(dprime); i -= 2*dprime)
    {
      /* 4 SUB-CASES: e1, e3 = +- 1 */
      for (e1 = -1; e1 <= 1; e1+=2)
	{
	  for (e3 = -1; e3 <= 1; e3+=2)
	    {
	      x1 = calcx1(s1, s2, s3, e1);
	      x2 = 0;
	      x3 = calcx3(s1, s2, s3, e3);
	      fout << counter << " |";
	      fout << setw(6) << setiosflags(ios::fixed) << setprecision(3) << x1 << "|";
	      fout << setw(6) << setiosflags(ios::fixed) << setprecision(3) << x3 << "|";
	      // case to rule out rotation:
	      if ( (fabs(x1 - x2) <= EPS) && (fabs(x3) == 1) )
		{
		  printf("No rotation\n x1 = x2 && x3 == 1 !!\n");
		  printf("No rotation\n x1 = x2 && x3 == 1 !!\n");	      
		}
	      if (dprime > 0)
		{
		  Rprime = calculateRprimePOS(s1,s2,s3,x1,x3);
		  tprime = calculatetprimePOS(s1,s2,s3,x1,x3);
		  costheta = calccostheta(s1,s2,s3,x1,x3);
		  sintheta = calcsintheta(s1,s2,s3,x1,x3);
		}
	      else if (dprime < 0)
		{
		  Rprime = calculateRprimeNEG(s1,s2,s3,x1,x3);
		  tprime = calculatetprimeNEG(s1,s2,s3,x1,x3);
		  costheta = calccostheta2(s1,s2,s3,x1,x3);
		  sintheta = calcsintheta2(s1,s2,s3,x1,x3);
		}
	      R = s * U * Rprime * Vt;
	      t = U * tprime * (depth/pixelspermm);
	      d = s * dprime;
	      theta = atan( sintheta/costheta ) * (180/M_PI);
	      theta2 = atan2( sintheta, costheta ) * (180/M_PI);
	      /*** PRINT OUTPUT****************/
	      fout << setfill(' ');
	      fout << setw(8) << setiosflags(ios::fixed) << setprecision(3) << theta << "|";
	      fout << setw(8) << setiosflags(ios::fixed) << setprecision(3) << theta2 << "|";
	      fout << setw(8) << setiosflags(ios::fixed) << setprecision(3) << t(0) << "x|";
	      fout << setw(8) << setiosflags(ios::fixed) << setprecision(3) << t(2) << "z|";
	      fout << "\n";

	      counter++; //NEXT SUB-CASE
	    }
	}
    }
  fout<<setfill('*')<<setw(50)<<"*\n";
  /**************************************************************************/

  cout << "DONE! SEE OUTPUT FILE!\n";
  return 0;
} // END MAIN
