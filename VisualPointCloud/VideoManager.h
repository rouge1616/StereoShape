#ifndef DEF_VideoManager
#define DEF_VideoManager

// std
#include <stdio.h>
#include <iostream>
#include <fstream>

// opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace cv;

#define CONFIDENCE 0.60
#define RANSAC_DIST 0.0

class VideoManager
{
	public:
		VideoManager();
		~VideoManager();

		void initialize(int, int, float);
		void sparseMatching(cv::Mat, cv::Mat);	
		void showImages(cv::Mat, cv::Mat);
		void buildPointCloud();	
		Mat_<double> linearLSTriangulation(Point3d u, Matx34d P,  Point3d u1,  Matx34d P1);
		Mat_<double> iterativeLinearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1);
		cv::Point3f triangulatePoint(cv::Point2f ptl, cv::Point2f ptr, Mat_<double> KL, Mat_<double> KR, float scaleZ);
		cv::Point3f convertPoint(cv::Point2f ptl, Mat_<double> KL, float scaleZ);
		void computeQualities();
		std::vector<cv::Point2f> getPointsL();
		std::vector<cv::Point2f> getPointsR();
		std::vector<cv::Point3f> getPointCloud();
		std::vector<double> getQualities();
		std::vector<int> getOFStatus();
		void setPointsL(std::vector<cv::Point2f>);
		void setPointsR(std::vector<cv::Point2f>);
		void setOFStatus(int ind, int val);


	private:
		std::vector<cv::Point2f> m_pointsL;
		std::vector<cv::Point2f> m_pointsR;
		std::vector<double> m_hessianResponse;
		std::vector<double> m_haarDistance;
		std::vector<double> m_qualities;
		std::vector<int> m_opticalFlowStatus;
		std::vector<cv::Point3f> m_pointCloud;

		int m_detectorParam;
		int m_optic;
		float m_mdFilter;

};

#endif
