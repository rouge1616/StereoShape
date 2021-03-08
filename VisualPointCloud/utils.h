#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"


bool invertMatrix3x3(const double a[3][3], double invOut[3][3]);
bool invertMatrix4x4(const double m[16], double invOut[16]);
cv::Point2f lt2c(cv::Point2f p, int w, int h);
cv::Point2f c2lt(cv::Point2f p, int w, int h);
void vecMultMat4d(double* m,  double u[4], double v[4]);
void vecMultMat3d(double m[3][3], double u[3], double v[3]);
bool isInBoundingBox(cv::Point3f, int size, float tol);
std::vector<cv::Point3f> getCoordFromFile(std::string coordFileName);
