#ifndef SAVEIMAGES_H
#define SAVEIMAGES_H

#include "string"
#include <opencv2/opencv.hpp>

// save PDM images
bool WritePDM(
    const std::string&              FileName,
    const cv::Mat&                  Image
    )
{
  if( Image.type() != CV_32FC1 ) {
    return false;
  }

  std::ofstream pFile( FileName.c_str(), std::ios::out | std::ios::binary );
  pFile << "P7" << std::endl;
  pFile << Image.cols << " " << Image.rows << std::endl;
  const unsigned int Size = Image.elemSize1() * Image.rows * Image.cols;
  pFile << 4294967295 << std::endl;
  pFile.write( (const char*)Image.data, Size );
  pFile.close();
  return true;
}

// save image to HardDisk (RGB images)
void SaveImgToDisk(std::string sPath, char* pImgbuf, int ImgWidth, int ImgHeight)
{
  sPath = sPath + ".pgm";
  cv::Mat RGB(cv::Size( ImgWidth, ImgHeight), CV_8UC3, pImgbuf, cv::Mat::AUTO_STEP);
  cv::imwrite(sPath,RGB);

//  cv::imshow("RGB", RGB);
//  cvWaitKey(1);
}


// save image to HardDisk (Depth images)
void SaveImgToDisk(std::string sPath, float* pImgbuf, int ImgWidth, int ImgHeight)
{
  sPath = sPath + ".pdm";
  cv::Mat FloatDepth(cv::Size( ImgWidth, ImgHeight), CV_32F, pImgbuf, cv::Mat::AUTO_STEP);
  WritePDM(sPath, FloatDepth);

//  cv::Mat CharDepth = cv::Mat::zeros(ImgHeight, ImgWidth, CV_8UC1);
//  FloatDepth.convertTo(CharDepth, CV_8U, 255.0);
//  cv::imshow("depth", FloatDepth);
//  cv::waitKey(10);
}

#endif // SAVEIMAGES_H
