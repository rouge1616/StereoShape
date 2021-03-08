// Capasule  manipulator

#ifndef DEF_CAPSULE
#define DEF_CAPSULE

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>

class Capsule
{
public:
	Capsule();
	Capsule(
		std::string img,
		int imgDepth,
		int imgNC,
		int imgWidth,
		int imgHeight,
		int dt,
		std::vector<float> currentPositionX,
		std::vector<float> currentPositionY,
		std::vector<float> currentPositionZ,
		std::vector<int> status,
		std::vector<double> currentQualities,
		std::vector<int> subset
        );
	~Capsule();
	
	std::string getImg();
	int getImgDepth();
	int getImgNC();
	int getImgWidth();
	int getImgHeight();
	int getDT();
	std::vector<float> getCurrentPositionX();
	std::vector<float> getCurrentPositionY();
	std::vector<float> getCurrentPositionZ();
	std::vector<int> getStatus();
	std::vector<double> getCurrentQualities();
	std::vector<int> getSubset();

private:
  	friend class boost::serialization::access;

  	template <typename Archive>
  	void serialize(Archive &ar, const unsigned int version)
  	{
  	  ar & m_img;
  	  ar & m_imgDepth;
  	  ar & m_imgNC;
  	  ar & m_imgWidth;
  	  ar & m_imgHeight;
  	  ar & m_dt;
  	  ar & m_currentPositionX;
  	  ar & m_currentPositionY;
  	  ar & m_currentPositionZ;
  	  ar & m_status;
  	  ar & m_currentQualities;;
  	  ar & m_subset;
  	}
	std::string m_img;
	int m_imgDepth;
	int m_imgNC;
	int m_imgWidth;
	int m_imgHeight;
	int m_dt;
	std::vector<float> m_currentPositionX;
	std::vector<float> m_currentPositionY;
	std::vector<float> m_currentPositionZ;
	std::vector<int> m_status;
	std::vector<double> m_currentQualities;
	std::vector<int> m_subset;

};

#endif
