#include "ControlPoint.h"

ControlPoint::ControlPoint()
{

}

ControlPoint::ControlPoint(
		cv::Point3f initCp,
		int maxNbNeighbors,
		float radius
    )
    :
    m_initCp(initCp),
    m_maxNbNeighbors(maxNbNeighbors),
    m_radius(radius)
{

}

ControlPoint::ControlPoint(
		cv::Point3f initCp,
		cv::Point3f cp,
		int maxNbNeighbors,
		int nbNeighbors,
		float radius,
		std::vector<int> nnIndices,
		std::vector<float> nnDists,
		std::vector<float> nnQualities
    )
    :
    m_initCp(initCp),
    m_cp(cp),
    m_maxNbNeighbors(maxNbNeighbors),
    m_nbNeighbors(nbNeighbors),
    m_radius(radius),
    m_nnIndices(nnIndices),
    m_nnDists(nnDists),
    m_nnQualities(nnQualities)
{

}

ControlPoint::~ControlPoint()
{
}

// *** getter

cv::Point3f ControlPoint::getInitCp()
{
  return m_initCp;
}

cv::Point3f ControlPoint::getCp()
{
  return m_cp;
}

int ControlPoint::getMaxNbNeighbors()
{
  return m_maxNbNeighbors;
}

int ControlPoint::getNbNeighbors()
{
  return m_nbNeighbors;
}

float ControlPoint::getRadius()
{
  return m_radius;
}

float ControlPoint::getGlobalQuality()
{
  std::vector<float> vecQ = getNNQualities();
  float q = 1;
  float sumQ = 0;
  int j = 0;
  for (int i = 0; i < vecQ.size(); i ++) {	
	if (vecQ[i] > 0) { 
		sumQ += vecQ[i];
		j++;
	}
  //std::cout << "vecQ = "<< vecQ[i] << std::endl;
  }
  q = (float)sumQ/j;
  //std::cout << "q = "<< q << std::endl;

  return q;
}

std::vector<int> ControlPoint::getNNIndices()
{
  return m_nnIndices;
}

std::vector<float> ControlPoint::getNNDists()
{
  return m_nnDists;
}

std::vector<float> ControlPoint::getNNQualities()
{
  return m_nnQualities;
}

// *** setter

void ControlPoint::setGlobalQuality(float gQua)
{
	m_globalQuality = gQua;
}

void ControlPoint::setCp(cv::Point3f pt)
{
  m_cp = pt;
}

void ControlPoint::setNbNeighbors(int nb)
{
  m_nbNeighbors = nb;
}

void ControlPoint::setNNIndices(std::vector<int> nnIndic)
{
  m_nnIndices.clear();
  for (int i = 0; i < nnIndic.size(); i++) 
	  m_nnIndices.push_back(nnIndic.at(i));
}


void ControlPoint::setNNDists(std::vector<float> nnDis)
{
  m_nnDists.clear();
  for (int i = 0; i < nnDis.size(); i++) 
	  m_nnDists.push_back(nnDis.at(i));
}

void ControlPoint::setNNQualities(std::vector<float> nnQua)
{
  m_nnQualities.clear();
  for (int i = 0; i < nnQua.size(); i++) 
	  m_nnQualities.push_back(nnQua.at(i));
}

void ControlPoint::addNIndice(int indic)
{
  m_nnIndices.push_back(indic);
}

void ControlPoint::addNDist(float dist)
{
  m_nnDists.push_back(dist);
}

void ControlPoint::addNQuality(float qua)
{
  m_nnQualities.push_back(qua);
}

void ControlPoint::setNQuality(float qua, int indic)
{
  if (m_nnQualities.at(indic) > 0) 
  	m_nnQualities.at(indic) = qua;
}

void ControlPoint::setNDist(float dis, int indic)
{
  if (m_nnDists.at(indic) > 0) 
  	m_nnDists.at(indic) = dis;
}


