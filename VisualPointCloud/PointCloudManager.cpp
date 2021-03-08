#include "PointCloudManager.h"

PointCloudManager::PointCloudManager()
{

}

PointCloudManager::~PointCloudManager()
{

}

void PointCloudManager::initialize(int isClusters, float radiusMls, float radiusUps, float leadfSizeDws)
{
	m_radiusMls = radiusMls;
	m_radiusUps = radiusUps;
	m_leadfSizeDws = leadfSizeDws;
	m_isClusters = isClusters;
}

void PointCloudManager::mlsSmoothing()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	for (int i=0; i< m_pointCloud.size(); i++ ) {	
		cloud->push_back(pcl::PointXYZ(m_pointCloud[i].x, m_pointCloud[i].y,m_pointCloud[i].z));
	}
	
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	// Create the MLS processor
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls; 
	// Set parameters
	mls.setInputCloud (cloud);
	mls.setComputeNormals (true);
	mls.setSearchRadius (m_radiusMls);
	mls.setPolynomialFit (true);
	mls.setPolynomialOrder (3);
	mls.setSearchMethod (tree);
	// Upsampling
	if (m_radiusUps > 0.05) {
		mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
		mls.setUpsamplingRadius (m_radiusUps);
		mls.setUpsamplingStepSize (0.3);
	}

	// Moving Least Squares processed
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
	mls.process (*cloud_smoothed);
	// Get the set of indices with each point in output having the corresponding point in input. 
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	inliers = mls.getCorrespondingIndices();

	//std::cout << inliers->indices.size() << std::endl;

	// voxelGrid DownSampling 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud (cloud_smoothed);
	vg.setLeafSize (m_leadfSizeDws, m_leadfSizeDws, m_leadfSizeDws);
	vg.filter (*cloud_sampled);

	for(int i = 0; i < cloud_smoothed->size(); i++) {
		m_smoothed.push_back(cv::Point3f(cloud_smoothed->at(i).x,cloud_smoothed->at(i).y,cloud_smoothed->at(i).z));
	}

	for(int i = 0; i < cloud_sampled->size(); i++) {
		m_sampled.push_back(cv::Point3f(cloud_sampled->at(i).x,cloud_sampled->at(i).y,cloud_sampled->at(i).z));
	}

	std::cout << "[MLS] Moving Least Square Smoothing performed." << std::endl;
}

void PointCloudManager::poissonReconstruction()
{
	// Build the normals from the original point cloud

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	for (int i=0; i< m_pointCloud.size(); i++ ) {	
		cloud->push_back(pcl::PointXYZ(m_pointCloud[i].x, m_pointCloud[i].y,m_pointCloud[i].z));
	}

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud);
	ne.setRadiusSearch (5);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
	ne.compute (*cloud_normals);
	for (size_t i = 0; i < cloud_normals->size (); ++i)
	{
	  	cloud_normals->points[i].normal_x *= -1;
	  	cloud_normals->points[i].normal_y *= -1;
	  	cloud_normals->points[i].normal_z *= -1;
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
	concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

	// Reconstruct mesh with Poisson
	pcl::Poisson<pcl::PointNormal> poisson;
	poisson.setDepth (9);
	poisson.setInputCloud(cloud_with_normals);
	poisson.reconstruct (m_mesh);

	std::cout << "[POISSON] Poisson mesh reconstruction performed." << std::endl;
}

int PointCloudManager::buildClusters(int knnRadius, int kNeighbors, std::vector<double> qualities)
{
	for (int i = 0 ; i < m_sampled.size() ; i++) {
		m_controlPoints.push_back( ControlPoint( m_sampled.at(i), kNeighbors, knnRadius ) );
	}  	

	int kCount; // the effectif number of neighbors	
	int nbEffectifCp = 0; //  the number of tracked control points
	std::vector<vector<int> > neighborsStatus; // for each control point, give the status of the neighbors (lost)

	// Create the Index on the Matched point only
	cv::flann::Index flannIndex(cv::Mat(m_pointCloud).reshape(1), cv::flann::KDTreeIndexParams(32));

	// for each Control Point we  compute the KNN 
	for( int i = 0; i < m_controlPoints.size(); i++ )
	{	 
		std::vector<int> tempNeiStat;
		int k = m_controlPoints[i].getMaxNbNeighbors(); // the maximum number of neighbors
		float radius = m_controlPoints[i].getRadius();  // the radius of search specific on each cp
		kCount = 0;

		vector<float> controlPointMat = Mat( m_controlPoints[i].getInitCp() ); // convert a 2d point to float matrix
		vector<int> knnIndices; // the indices of nearest neighbors
		vector<float> knnDists; // the distances of nearest neighbors, 0 for no neighbor found

		//flannIndex.knnSearch(controlPointMat, knnIndices, knnDists, k, cv::flann::SearchParams());
		int found = flannIndex.radiusSearch(controlPointMat, knnIndices, knnDists, radius*radius, k, cv::flann::SearchParams(32));
		//std::cout << found << std::endl;
		// this step is to resize the number of neighbors found, from k to kCount (if found is not null)		
		if (found)
		{ 
			for (int j = 0; j < k; j++)
			   if (knnDists[j]) {
				m_controlPoints[i].addNIndice( knnIndices[j] ); // the indices in the point cloud of each neighbor
				m_controlPoints[i].addNDist( knnDists[j] ); // the distance of each neighbor
				m_controlPoints[i].addNQuality( qualities[knnIndices[j]] ); // initialise quality with Hessian response		

				//std::cout << ((keypoints_l[i].response + keypoints_r[i].response)*0.5)/maxResp << std::endl;
				tempNeiStat.push_back(1); // status vector for the neighbors
				kCount++;
				}
			m_controlPoints[i].setNbNeighbors( kCount );
			m_controlPoints[i].setCp(m_controlPoints[i].getInitCp()); 

			nbEffectifCp ++;
		}
		else 
		{
			m_controlPoints[i].setNbNeighbors(0); 	
			tempNeiStat.push_back(-1); // status vector for the neighbors 
			//std::cout << "no neighbors found for the control point "<< i << std::endl;
		}
		neighborsStatus.push_back(tempNeiStat);
	}

	std::cout << "[CLUSTERING] Number of Initial Control Points:  \t" <<  m_controlPoints.size() << std::endl; 
	std::cout << "[CLUSTERING] Number of Tracked Control Points:  \t" <<  nbEffectifCp << std::endl; 

	return nbEffectifCp;
}

void PointCloudManager::updateClusters()
{

}

std::vector<cv::Point3f> PointCloudManager::getPointCloud()
{
	return m_pointCloud;
}

void PointCloudManager::setPointCloud(std::vector<cv::Point3f> p)
{
	m_pointCloud = p;
}

std::vector<float> PointCloudManager::getX()
{
	std::vector<float> vecX;

	if (m_isClusters) { // send the clusters
		for(int i = 0; i < m_controlPoints.size(); i++) 
			vecX.push_back(m_controlPoints[i].getCp().x);
	}
	else { // send all the features
		for(int i = 0; i < m_pointCloud.size(); i++) 
			vecX.push_back(m_pointCloud[i].x);
	}

	return vecX;
}

std::vector<float> PointCloudManager::getY()
{
	std::vector<float> vecY;

	if (m_isClusters) { // send the clusters
		for(int i = 0; i < m_controlPoints.size(); i++) 
			vecY.push_back(m_controlPoints[i].getCp().y);
	}
	else { // send all the features
		for(int i = 0; i < m_pointCloud.size(); i++) 
			vecY.push_back(m_pointCloud[i].y);
	}

	return vecY;

}

std::vector<float> PointCloudManager::getZ()
{
	std::vector<float> vecZ;

	if (m_isClusters) { // send the clusters
		for(int i = 0; i < m_controlPoints.size(); i++) 
			vecZ.push_back(m_controlPoints[i].getCp().z);
	}
	else { // send all the features
		for(int i = 0; i < m_pointCloud.size(); i++) 
			vecZ.push_back(m_pointCloud[i].z);
	}

	return vecZ;

}

void PointCloudManager::saveOutput(std::string filename1, std::string filename2, std::string filename3, std::string filename4)
{
	std::ofstream file1;
	file1.open(filename1.c_str());
   	for( int i = 0; i < m_pointCloud.size(); i++ )
	{ 
		file1 << "v "<< m_pointCloud[i].x << " " << m_pointCloud[i].y << " " << m_pointCloud[i].z <<  std::endl; 	
	}
	file1.close();

	std::ofstream file2;
	file2.open(filename2.c_str());
   	for( int i = 0; i < m_smoothed.size(); i++ )
	{ 
		file2 << "v "<< m_smoothed[i].x << " " << m_smoothed[i].y << " " << m_smoothed[i].z <<  std::endl; 	
	}
	file2.close();

	std::ofstream file3;
	file3.open(filename3.c_str());
   	for( int i = 0; i < m_sampled.size(); i++ )
	{ 
		file3 << "v "<< m_sampled[i].x << " " << m_sampled[i].y << " " << m_sampled[i].z <<  std::endl; 	
	}
	file3.close();

	pcl::io::saveVTKFile(filename4, m_mesh);

	std::cout << "[OUTPUT] output files saved ! " << std::endl;
}


	


