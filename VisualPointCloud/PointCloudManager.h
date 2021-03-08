#ifndef DEF_PointCloudManager
#define DEF_PointCloudManager

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

// pcl
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

#include "ControlPoint.h"

using namespace cv;

class PointCloudManager
{
	public:
		PointCloudManager();
		~PointCloudManager();

		void initialize(int isClusters, float radiusMls, float radiusUps, float leadfSizeDws);
		void mlsSmoothing();
		void poissonReconstruction();
		int buildClusters(int knnRadius, int kNeighbors, std::vector<double> qualities);
		void updateClusters();
		std::vector<cv::Point3f> getPointCloud();
		void setPointCloud(std::vector<cv::Point3f>);
		std::vector<float> getX();
		std::vector<float> getY();
		std::vector<float> getZ();
		void saveOutput(std::string, std::string, std::string, std::string);

	private:
		std::vector<cv::Point3f> m_pointCloud;
		std::vector<cv::Point3f> m_smoothed;
		std::vector<cv::Point3f> m_sampled;
		std::vector< ControlPoint > m_controlPoints;
		int m_isClusters;
		float m_radiusMls;
		float m_radiusUps;
		float m_leadfSizeDws;
		pcl::PolygonMesh m_mesh;
};

#endif
