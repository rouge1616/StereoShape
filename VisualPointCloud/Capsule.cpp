#include "Capsule.h"

Capsule::Capsule()
{

}

Capsule::Capsule(
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
    )
    :
    m_img(img),
    m_imgDepth(imgDepth),
    m_imgNC(imgNC),
    m_imgWidth(imgWidth),
    m_imgHeight(imgHeight),
    m_dt(dt),
    m_currentPositionX(currentPositionX),
    m_currentPositionY(currentPositionY),
    m_currentPositionZ(currentPositionZ),
    m_status(status),
    m_currentQualities(currentQualities),
    m_subset(subset)
{

}

Capsule::~Capsule()
{
}

// *** getter and setter

std::string Capsule::getImg()
{
  return m_img;
}

int Capsule::getImgDepth()
{
  return m_imgDepth;
}

int Capsule::getImgNC()
{
  return m_imgNC;
}

int Capsule::getImgWidth()
{
  return m_imgWidth;
}

int Capsule::getImgHeight()
{
  return m_imgHeight;
}

int Capsule::getDT()
{
  return m_dt;
}


std::vector<float> Capsule::getCurrentPositionX()
{
  return m_currentPositionX;
}

std::vector<float> Capsule::getCurrentPositionY()
{
  return m_currentPositionY;
}

std::vector<float> Capsule::getCurrentPositionZ()
{
  return m_currentPositionZ;
}

std::vector<int> Capsule::getStatus()
{
  return m_status;
}

std::vector<double> Capsule::getCurrentQualities()
{
  return m_currentQualities;
}

std::vector<int> Capsule::getSubset()
{
  return m_subset;
}
