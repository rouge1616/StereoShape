// boost
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/asio.hpp>

#include "utils.h"
#include "PointCloudManager.h"
#include "ControlPoint.h"
#include "Capsule.h"
#include "VideoManager.h"

using namespace cv;
using namespace std;
using boost::asio::ip::tcp;

#define PORT 8888

int main( int argc, char** argv )
{  

	char c;
	VideoCapture capL, capR;
	Mat img_l, img_r;
	Mat grayL, prevGrayL, grayR, prevGrayR;
	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
	Size winSize(51,51);
	int optic = 0;  // 0 for stereo, 1 for mono 
	int nbFrame = 0;
	std::vector<int> subset;

	std::string vidLeft = "left.avi";
	std::string vidRight = "right.avi";
	int detectorParam = 1000;
	float mdFilter = 0.5;
	std::string maskFile = "0";
	int sendViaTcp = 0;
		
	if( argc == 7)
	{	
		vidLeft = argv[1];
		vidRight = argv[2];
		detectorParam = atoi(argv[3]);
		mdFilter = atof(argv[4]);
		maskFile = argv[5];
		sendViaTcp = atoi(argv[6]);
	}
	else exit(-1);

	capL.open(vidLeft);
	if(!capL.isOpened()) { 
		std::cout << "INPUT VIDEO : ERROR in opening left video file : " << vidLeft << std::endl;
		exit(-1);
	}
	else std::cout << "INPUT VIDEO : Loading left video File : " << vidLeft << std::endl;

	capR.open(vidRight);
	if(!capR.isOpened()) { 
		std::cout << "INPUT VIDEO : ERROR in opening right video file : " << vidRight << std::endl;
		exit(-1);
	}
	else std::cout << "INPUT VIDEO : Loading right video File : " << vidRight << std::endl;

	
	float fps = capL.get(CV_CAP_PROP_FPS);
	int width = capL.get(CV_CAP_PROP_FRAME_WIDTH);
	int height = capL.get(CV_CAP_PROP_FRAME_HEIGHT);

	std::cout << "INPUT VIDEO : fps = "<< (float)fps << std::endl;
	std::cout << "INPUT VIDEO : Width = "<< width << std::endl;
	std::cout << "INPUT VIDEO : Height = "<< height << std::endl;

	capL >> img_l;
	capR >> img_r;

	VideoManager videoManager;
	videoManager.initialize(optic, detectorParam, mdFilter);
	videoManager.sparseMatching(img_l, img_r);
	videoManager.buildPointCloud();

	PointCloudManager pointCloudManager;
	pointCloudManager.initialize(0, 2, 0, 2);
	pointCloudManager.setPointCloud(videoManager.getPointCloud());
	pointCloudManager.mlsSmoothing();
	pointCloudManager.poissonReconstruction();
	pointCloudManager.buildClusters(0, 5, videoManager.getQualities());

	pointCloudManager.saveOutput("output/stereo.obj","output/mls.obj","output/clusters.obj","output/poisson.ply");

	std::vector<Point2f> prevPointsL, currentPointsL, prevPointsR, currentPointsR;
	std::vector<int> opticalFlowStatus;
	prevPointsL = videoManager.getPointsL();
	prevPointsR = videoManager.getPointsR();
	currentPointsL.resize(prevPointsL.size());
	currentPointsR.resize(prevPointsR.size());
	opticalFlowStatus.resize(prevPointsL.size());

	// ******************************************************************************
	// --------------------------------- MAIN LOOP ----------------------------------
	// ******************************************************************************

  	for(;;)
  	{

		// Capture images and convert them to gray level
		capL >> img_l;
		capR >> img_r;

		if( img_l.empty() || img_r.empty()) break;
	
		img_l.copyTo(grayL);
		img_r.copyTo(grayR);

		vector<uchar> statusL, statusR;		
		vector<float> errL;
		if(prevGrayL.empty())
			grayL.copyTo(prevGrayL);
		vector<float> errR;
		if(prevGrayR.empty())
			grayR.copyTo(prevGrayR);

		int i, j = 0;

		calcOpticalFlowPyrLK(prevGrayL, grayL, prevPointsL, currentPointsL, statusL, errL, winSize, 3, termcrit, 0, 0.001);
		calcOpticalFlowPyrLK(prevGrayR, grayR, prevPointsR, currentPointsR, statusR, errR, winSize, 3, termcrit, 0, 0.001);

		videoManager.setPointsL(currentPointsL);
		videoManager.setPointsR(currentPointsR);
		videoManager.buildPointCloud();

		for (int i = 0; i < statusL.size(); i++)
			if( !statusL[i] || !statusR[i]) {	
				videoManager.setOFStatus(i,0);
		}

		pointCloudManager.setPointCloud(videoManager.getPointCloud());
		pointCloudManager.updateClusters();

		if (sendViaTcp)
		{
		    try
		    {
			// stream connection and acceptor
			boost::asio::io_service io_s;
			tcp::acceptor data_acceptor(io_s, tcp::endpoint(tcp::v4(), PORT));
			tcp::iostream data_stream;
			data_acceptor.accept(*(data_stream.rdbuf()));

			// sending
			std::stringstream imageString;
			imageString.write((const char*)img_l.data, img_l.total()*img_l.elemSize());
			boost::archive::binary_oarchive oa(data_stream);
			Capsule cap_sent;

			// send the image and the point cloud
			cap_sent = Capsule(
				imageString.str(),
				img_l.depth(),
				img_l.channels(),
				img_l.cols,
				img_l.rows, 
				nbFrame,
				pointCloudManager.getX(),
				pointCloudManager.getY(),
				pointCloudManager.getZ(), 
				videoManager.getOFStatus(),
				videoManager.getQualities(), 
				subset
			);
			oa << cap_sent;
			data_stream.flush();

			// receiving back
			Capsule cap_recv;
			boost::archive::binary_iarchive ia(data_stream);
			ia >> cap_recv; 

			// stream closing
			data_stream.close();
		    }
		    catch (std::exception& e)
		    {
		    	std::cerr << e.what() << std::endl;
		    }
		}	


		videoManager.showImages(img_l, img_r);

		c = (char)waitKey( 100/fps );
		if( c == 27 )
		    break;
		nbFrame++;

		//-- swap buffers
		std::swap(currentPointsL, prevPointsL);
		std::swap(prevGrayL, grayL);
		std::swap(currentPointsR, prevPointsR);
		std::swap(prevGrayR, grayR);

 	}

	return 0;
}
