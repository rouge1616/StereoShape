
#include "utils.h"


// invert 3*3 matrix
bool invertMatrix3x3(const double a[3][3], double invOut[3][3])
{
	float det = a[0][0]*(a[1][1]*a[2][2]-a[2][1]*a[1][2])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);//adjoin
	//std::cout <<"\n Determinant : " << det << std::endl;;

	invOut[0][0]=(a[1][1]*a[2][2]-a[2][1]*a[1][2])/det;
	invOut[0][1]=-(a[1][0]*a[2][2]-a[1][2]*a[2][0])/det;
	invOut[0][2]=(a[1][0]*a[2][1]-a[2][0]*a[1][1])/det;
	invOut[1][0]=-(a[0][1]*a[2][2]-a[0][2]*a[2][1])/det;
	invOut[1][1]=(a[0][0]*a[2][2]-a[0][2]*a[2][0])/det;
	invOut[1][2]=-(a[0][0]*a[2][1]-a[2][0]*a[0][1])/det;
	invOut[2][0]=(a[0][1]*a[1][2]-a[0][2]*a[1][1])/det;
	invOut[2][1]=-(a[0][0]*a[1][2]-a[1][0]*a[0][2])/det;
	invOut[2][2]=(a[0][0]*a[1][1]-a[1][0]*a[0][1])/det;

    return true;
}
// invert 4*4 matrix
bool invertMatrix4x4(const double m[16], double invOut[16])
{
    double inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
}


// mutiply 4*1 vector with 4*4 matrix
void vecMultMat4d(double* m,  double u[4], double v[4])
{
    v[0] = m[0]*u[0] + m[4]*u[1] + m[ 8]*u[2] + m[12]*u[3] ;
    v[1] = m[1]*u[0] + m[5]*u[1] + m[ 9]*u[2] + m[13]*u[3] ;
    v[2] = m[2]*u[0] + m[6]*u[1] + m[10]*u[2] + m[14]*u[3] ;
    v[3] = m[3]*u[0] + m[7]*u[1] + m[11]*u[2] + m[15]*u[3] ;
}

// mutiply 3*1 vector with 3*3 matrix
void vecMultMat3d(double m[3][3], double u[3], double v[3])
{
    int i, j;

    for (i = 0; i < 3; i++) {
        v[i] = 0;
        for (j = 0; j < 3; j++) {
            v[i] += m[i][j]*u[j];
        }
    }
}

// left-top -> center
cv::Point2f lt2c(cv::Point2f p, int width, int height)
{
	//return p;
	return cvPoint2D32f(p.x - width*0.5, height*0.5 - p.y);
}


// center -> left-top
cv::Point2f c2lt(cv::Point2f p, int width, int height)
{
	//return cvPoint(p.x,p.y);
	return cvPoint(p.x + width*0.5, height*0.5 - p.y);
}

// check if the coordinates lies on th object
bool isInBoundingBox(cv::Point3f pt, int size, float tol)
{
	float boundary = size*0.5 + tol;
	if (pt.x > -boundary && pt.x < boundary && pt.y > -boundary && pt.y < boundary && pt.z > -boundary && pt.z < boundary) return true;
	else return false;
}


void splitLine(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
	}
}

std::vector<cv::Point3f> getCoordFromFile(std::string coordFileName)
{	
  std::vector<cv::Point3f> vec;
  std::vector<std::string> triplet;
  double x,y,z;
  const char* v;
  std::ifstream coordf(coordFileName.c_str());
  if (coordf.is_open())
  {
    while ( coordf.good() )
    {
      std::string line;
      getline (coordf, line);
      if (line != "") 
      {     
          splitLine(line, ' ', triplet);
	  v = triplet[0].c_str();
          x = strtod(triplet[1].c_str(),NULL) ;
          y = strtod(triplet[2].c_str(),NULL) ;
          z = strtod(triplet[3].c_str(),NULL) ;

          vec.push_back( cv::Point3f(x,y,z) );
      }
      triplet.clear();
    }
    coordf.close();
    return vec;
  }
  else std::cout << "WARNING : Unable to open file" << std::endl; 
}

