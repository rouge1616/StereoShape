#include "VideoManager.h"

VideoManager::VideoManager()
{

}

VideoManager::~VideoManager()
{

}

void VideoManager::initialize(int optic, int detectorParam, float mdFilter)
{
	m_detectorParam = detectorParam;
	m_mdFilter = mdFilter;
	m_optic = optic;
}	

void VideoManager::sparseMatching(cv::Mat img_l, cv::Mat img_r)
{

	// detect keypoints
	std::vector<KeyPoint> keypoints_l, keypoints_r;
	SurfFeatureDetector surfDetector( m_detectorParam );
	surfDetector.detect( img_l, keypoints_l);
	surfDetector.detect( img_r, keypoints_r);

	if (keypoints_l.empty() || keypoints_r.empty() )
	{	
		std::cout << "\n[SPARSE MATCHING] No features extracted. Check the input images  ..." << std::endl; 
		exit(-1);	
	}

	// compute descriptor
	SiftDescriptorExtractor siftDescriptor;	
	Mat descriptors_l, descriptors_r;
	siftDescriptor.compute( img_l, keypoints_l, descriptors_l );
	siftDescriptor.compute( img_r, keypoints_r, descriptors_r );

	// initialize the matcher
	FlannBasedMatcher flannmatcher;
	std::vector< DMatch > matches_init;
	flannmatcher.match( descriptors_l, descriptors_r, matches_init );

	if (matches_init.empty())
	{	
		std::cout << "[SPARSE MATCHING] No matched features. Try to check the extraction parameters." << std::endl; 
		exit(-1);	
	}
	else std::cout << "[SPARSE MATCHING] Number of matched features without filtering: " << matches_init.size() << std::endl; 

	//Quick calculation of max and min distances between keypoints
	double max_dist = 0;
	double min_dist = 1e10; 
	for( int i = 0; i < descriptors_l.rows; i++ )
	{ 
		double dist = matches_init[i].distance;
	 	if( dist < min_dist ) min_dist = dist;
	   	if( dist > max_dist ) max_dist = dist;
	}
	//std::cout << "dist min : " << min_dist << " / dist max : " << max_dist << " / point rejected if higher than " << m_mdFilter*max_dist << std::endl;

	//-- Minimal Distance filtering
	std::vector< DMatch > matches_md;
	for( int i = 0; i < descriptors_l.rows; i++ )
	  if( matches_init[i].distance <= m_mdFilter*max_dist )
		matches_md.push_back( matches_init[i]);

	if (matches_md.empty())
	{	
		std::cout << "[SPARSE MATCHING] No matched features after Minimal distance filtering." << std::endl; 
		exit(-1);	
	}
	else std::cout << "[SPARSE MATCHING] Number of matched features after Minimal distance filtering: " << matches_md.size() << std::endl; 

	// -- Epipolar Contraints filtering
	std::vector< DMatch > matches_epc;
	
	std::vector<Point2f> vecPointsTmp_l, vecPointsTmp_r;
	for (int i = 0; i < matches_md.size(); i++) {
		vecPointsTmp_l.push_back( keypoints_l[matches_md[i].queryIdx].pt );
		vecPointsTmp_r.push_back( keypoints_r[matches_md[i].trainIdx].pt );
	}

	cv::Mat inliers;
	//compute fundamental matrix from initial matches with RANSAC

	cv::Mat fundamentalMatrix = findFundamentalMat(vecPointsTmp_l, vecPointsTmp_r, cv::FM_RANSAC, RANSAC_DIST, CONFIDENCE, inliers);
	//std::cout << "Fundamental Matrix : \n" << fundamentalMatrix << "\n" << std::endl;

	if (inliers.empty())
	{	
		std::cout << "[SPARSE MATCHING] ERROR : No matched features after Epipolar Contraint Filtering." << std::endl; 
		exit(-1);	
	}
	vecPointsTmp_l.clear();
	vecPointsTmp_r.clear();

	std::vector<cv::KeyPoint> vecKeypoints_l, vecKeypoints_r;

	for(int i = 0 ; i < inliers.rows ; i++) {
		if (inliers.at<int>(0,i)) {
			// if the point is valid (lie to the epipolar line)
			matches_epc.push_back(matches_md[i]);
			vecKeypoints_l.push_back( keypoints_l[matches_md[i].queryIdx] );
			vecKeypoints_r.push_back( keypoints_r[matches_md[i].trainIdx] );
			vecPointsTmp_l.push_back( keypoints_l[matches_md[i].queryIdx].pt );
			vecPointsTmp_r.push_back( keypoints_r[matches_md[i].trainIdx].pt );
		}
	}


	std::cout << "[SPARSE MATCHING] Number of matched features after Epipolar Contraints filtering:  \t" << matches_epc.size() << std::endl; 

	//recompute fundamental matrix with filtred matches	
	//fundamentalMatrix = findFundamentalMat(vecPointsTmp_l, vecPointsTmp_r, cv::FM_RANSAC); 
	//std::cout << "Fundamental Matrix : \n" << fundamentalMatrix << "\n" << std::endl;	

	// load the class members with the final list of point
   	for( int i = 0; i < vecKeypoints_l.size(); i++ )
	{ 
		// 2D point vector for optical flow
		m_pointsL.push_back( vecKeypoints_l[i].pt );
		m_pointsR.push_back( vecKeypoints_r[i].pt );

		m_hessianResponse.push_back( (double)(keypoints_l[i].response + keypoints_r[i].response)/2);
		m_haarDistance.push_back(matches_epc[i].distance);

		m_opticalFlowStatus.push_back(1);
	}

	m_pointCloud.resize(m_pointsL.size());
	m_qualities.resize(m_pointsL.size());
	computeQualities();

	vecKeypoints_l.clear();
	vecKeypoints_r.clear();
	keypoints_l.clear();
	keypoints_r.clear();

}

void VideoManager::computeQualities()
{
	double maxResp = *max_element(m_hessianResponse.begin(),m_hessianResponse.end());
	double maxDist = *max_element(m_haarDistance.begin(),m_haarDistance.end());
	
	for( int i = 0; i < m_qualities.size(); i++ ){
//		m_qualities[i] = m_hessianResponse[i]/maxResp;
		m_qualities[i] = m_haarDistance[i]/maxDist;
	}
}	

void VideoManager::showImages(cv::Mat img_l, cv::Mat img_r)
{
	// Show detected matches
	Mat imgResult(img_l.rows,2*img_l.cols,img_l.type());
	Mat roiImgResult_Left = imgResult(Rect(0,0,img_l.cols,img_l.rows)); 
	Mat roiImgResult_Right = imgResult(Rect(img_r.cols,0,img_r.cols,img_r.rows)); 
	img_l.copyTo(roiImgResult_Left);
	img_r.copyTo(roiImgResult_Right);

	Mat overlayResult;
	float opacity = 0.4;
	imgResult.copyTo(overlayResult);

	for (int i = 0 ; i < m_pointsL.size() ; i++)
	{
		Point p1 = m_pointsL[i];
		Point p2 = Point(m_pointsR[i].x + img_l.cols, m_pointsR[i].y); 
		line(overlayResult, p1, p2, Scalar(255,255,0), 0.1, 8, 0);	
		//circle(img_l, p1, 2, Scalar(255,255,0), 0.2, 8, 0);	
		if (m_opticalFlowStatus[i]) {
			circle(imgResult, p1, 1, Scalar(0,255,0), 0.6, 2, 0);
			circle(imgResult, p2, 1, Scalar(0,255,0), 0.6, 2, 0);
		}
		else {
			circle(imgResult, p1, 1, Scalar(0,0,255), 0.6, 2, 0);
			circle(imgResult, p2, 1, Scalar(0,0,255), 0.6, 2, 0);
		}
 	}
	addWeighted(overlayResult, opacity, imgResult, 1 - opacity, 0, imgResult);

	imshow( "Result", imgResult );	
	//imshow( "Left", img_l );
	//imshow( "Right", img_r );

}

Mat_<double> VideoManager::linearLSTriangulation(Point3d u, Matx34d P,  Point3d u1,  Matx34d P1)
{

/**
From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/

	Matx43d A(u.x*P(2,0)-P(0,0),	u.x*P(2,1)-P(0,1),	u.x*P(2,2)-P(0,2),	
		u.y*P(2,0)-P(1,0),	u.y*P(2,1)-P(1,1),	u.y*P(2,2)-P(1,2),	
		u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),	u1.x*P1(2,2)-P1(0,2),	
		u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),	u1.y*P1(2,2)-P1(1,2)
	);
	Matx41d B(-(u.x*P(2,3)	-P(0,3)),
		-(u.y*P(2,3)	-P(1,3)),
		-(u1.x*P1(2,3)	-P1(0,3)),
		-(u1.y*P1(2,3)	-P1(1,3))
	);

	Mat_<double> X;
	solve(A,B,X,DECOMP_SVD);

	return X;
}

Mat_<double> VideoManager::iterativeLinearLSTriangulation(Point3d u, Matx34d P, Point3d u1, Matx34d P1)
{
	double wi = 1, wi1 = 1;
	Mat_<double> X(4,1);

	Mat_<double> X_ = linearLSTriangulation(u,P,u1,P1);
	X(0) = X_(0); 
	X(1) = X_(1); 
	X(2) = X_(2); 
	X(3) = 1.0;

	for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
		
		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);

		//breaking point
		//if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		Matx43d A((u.x*P(2,0)-P(0,0))/wi,	(u.x*P(2,1)-P(0,1))/wi,	(u.x*P(2,2)-P(0,2))/wi,	
			(u.y*P(2,0)-P(1,0))/wi,	(u.y*P(2,1)-P(1,1))/wi,	(u.y*P(2,2)-P(1,2))/wi,	
			(u1.x*P1(2,0)-P1(0,0))/wi1,	(u1.x*P1(2,1)-P1(0,1))/wi1,	(u1.x*P1(2,2)-P1(0,2))/wi1,	
			(u1.y*P1(2,0)-P1(1,0))/wi1,	(u1.y*P1(2,1)-P1(1,1))/wi1,	(u1.y*P1(2,2)-P1(1,2))/wi1
		);

		Mat_<double> B = (Mat_<double>(4,1) <<	-(u.x*P(2,3)	-P(0,3))/wi,
							-(u.y*P(2,3)	-P(1,3))/wi,
							-(u1.x*P1(2,3)	-P1(0,3))/wi1,
							-(u1.y*P1(2,3)	-P1(1,3))/wi1
		);

		solve(A,B,X_,DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}

	return X;
}

cv::Point3f VideoManager::triangulatePoint(cv::Point2f ptl, cv::Point2f ptr, Mat_<double> KL, Mat_<double> KR, float scaleZ)
{
	// [I|0]
	Matx34d P = Matx34d(1, 0, 0, 0 ,
				0, 1, 0, 0,
				0, 0, 1, 0
			);

	// [R|t] 	// FOR DAVINCI
	Matx34d P1dv = Matx34d(0.9999,   -0.0012,   -0.0135, 5.01642,
    				0.0013,    1.0000,    0.0083, -1.77011,
    				0.0134,   -0.0083,   0.9999, -0.44018 
			);

	Point3d u(ptl.x,ptl.y,1.0);
	Point3d u1(ptr.x,ptr.y,1.0);

	Mat_<double> um = KL.inv() * Mat_<double>(u); // mutiply the point by the inverse of the K matrix
	u.x = um(0); u.y = um(1); u.z = um(2);

	Mat_<double> um1 = KR.inv() * Mat_<double>(u1); // mutiply the point by the inverse of the K matrix
	u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);
	Mat_<double> X;

	X = iterativeLinearLSTriangulation(u,P,u1,P1dv);

        return cv::Point3f(X(0), X(1), X(2)*scaleZ);
}

cv::Point3f VideoManager::convertPoint(cv::Point2f ptl, Mat_<double> KL, float scaleZ)
{
//	Mat camera_mat = (Mat_<float>(3,3) <<  515.3647, 0., 313.99056, 0., 515.2777, 233.34047, 0., 0., 1.  );
	Mat_<float> um = KL.inv() * Mat_<float>(ptl); // mutiply the point by the inverse of the K matrix
	return cv::Point3f(um(0), um(1), scaleZ );
		
}

void VideoManager::buildPointCloud()
{
	// Davinci camera matrix
	Mat cameraMatrixL, cameraMatrixR;
        cameraMatrixL = (Mat_<double>(3,3) << 716.73664, 0., 527.62345, 0.,
					       770.26558, 241.86880, 0., 0., 1. );

        cameraMatrixR = (Mat_<double>(3,3) << 710.45284, 0.0, 462.01800, 0.,
       						766.16933, 246.61107, 0., 0., 1. );

   	for( int i = 0; i < m_pointsL.size(); i++ )
	{ 
		m_pointCloud[i] = triangulatePoint(m_pointsL[i], m_pointsR[i], cameraMatrixL, cameraMatrixR, 1);
	}

}


std::vector<cv::Point2f> VideoManager::getPointsL()
{
	return m_pointsL;
}

std::vector<cv::Point2f> VideoManager::getPointsR()
{
	return m_pointsR;
}

std::vector<cv::Point3f> VideoManager::getPointCloud()
{
	return m_pointCloud;
}

std::vector<double> VideoManager::getQualities()
{
	return m_qualities;
}

std::vector<int> VideoManager::getOFStatus()
{
	return m_opticalFlowStatus;
}

void VideoManager::setPointsL(std::vector<cv::Point2f> vpl)
{
	m_pointsL = vpl;
}

void VideoManager::setPointsR(std::vector<cv::Point2f> vpr)
{
	m_pointsR = vpr;
}

void VideoManager::setOFStatus(int ind, int val)
{
	if (m_opticalFlowStatus.size() > ind) 
		m_opticalFlowStatus[ind] = val;
}






