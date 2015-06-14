#include <ros/ros.h>
#include <ros/package.h>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include "opencv_boost_serialization/opencv_rich_feature.h"
#include "opencv_boost_serialization/opencv_boost_serialization.h"


void printkeypoint(const cv::KeyPoint& kpts)
{
  std::cout << std::endl;
  std::cout << kpts.angle << std::endl;
  std::cout << kpts.class_id << std::endl;
  std::cout << kpts.octave << std::endl;
  std::cout << kpts.pt << std::endl;
  std::cout << kpts.response << std::endl;
  std::cout << kpts.size << std::endl;
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  std::string path = ros::package::getPath("opencv_boost_serialization") + "/test/";

  std::cout << path << std::endl;

  cv::Mat img = cv::imread(path + "Lenna.png", CV_LOAD_IMAGE_GRAYSCALE);

  assert(!img.empty());

  cv::OrbFeatureDetector detector( 500 );
  cv::OrbDescriptorExtractor extractor;

  std::vector<cv::KeyPoint> img_keypoints;
  cv::Mat img_descriptors;

  detector.detect(img, img_keypoints);
  extractor.compute(img, img_keypoints, img_descriptors);

  cv::KeyPoint lktps, clkpts;

  std::cout << "kpt :" << std::endl;
  printkeypoint(img_keypoints.at(0));

  serialization::saveBin(img_keypoints.at(0), path+"kpts.feat", false);
  serialization::saveBin(img_keypoints.at(0), path+"kpts.cfeat", true);

  std::cout << "kpt saved." << std::endl;

  serialization::loadBin(lktps, path+"kpts.feat", false);
  serialization::loadBin(clkpts, path+"kpts.cfeat", true);

  std::cout << "kpt loaded : " << std::endl;
  printkeypoint(lktps);
  std::cout << "compressed kpt loaded : " << std::endl;
  printkeypoint(clkpts);

  cv::rfeat::richFeature rFeature(img_keypoints.at(0), img_descriptors.row(0),
                                  cv::Vec3d(4.4,4.4,4.4), 20.20);

  cv::rfeat::richFeature lrFeature, clrFeature;

  std::cout << "richFeature :" << std::endl;
  std::cout << rFeature << std::endl;

  cv::rfeat::saveFeature(rFeature, path+"rfeat.feat", false);
  cv::rfeat::saveFeature(rFeature, path+"rfeat.cfeat", true);

  cv::rfeat::loadFeature(lrFeature, path+"rfeat.feat", false);
  cv::rfeat::loadFeature(clrFeature, path+"rfeat.cfeat", true);

  std::cout << "loaded richFeature :" << std::endl;
  std::cout << lrFeature << std::endl;

  std::cout << "loaded compressed richFeature :" << std::endl;
  std::cout << clrFeature << std::endl;

  std::cout << "loaded == init ? " << (lrFeature == rFeature) << std::endl;
  std::cout << "compressed loaded == init ? " << (clrFeature == rFeature) << std::endl;

  cv::rfeat::richFeature krFeature = cv::rfeat::richFeature(img_keypoints.at(2), img_descriptors.row(2),
                                                            cv::Vec3d(1.11,1.22,1.33), 14.56);

  std::cout << "krFeature :" << std::endl;
  std::cout << krFeature << std::endl;


  cv::rfeat::DescriptorVector desv, desv1;

  desv.push_back(img_descriptors.row(0).clone());
  desv.push_back(img_descriptors.row(2).clone());

  for (size_t i=0; i<desv.size(); ++i)
    std::cout << desv[i] << std::endl;

  serialization::saveBin(desv, path+"matv.feat", false);
  serialization::loadBin(desv1, path+"matv.feat", false);

  for (size_t i=0; i<desv1.size(); ++i)
    std::cout << desv1[i] << std::endl;

  cv::rfeat::richFeatureVector rFeatVec;
  cv::rfeat::richFeatureVector lrFeatVec, clrFeatVec;

  rFeatVec.push_back(lrFeature);
  rFeatVec.push_back(clrFeature);
  rFeatVec.push_back(krFeature);

  std::cout << "feature vec : " << rFeatVec.size() << std::endl;
  for (size_t i = 0; i < rFeatVec.size(); ++i)
    std::cout << "Feat : " << i << " \n" << rFeatVec.at(i) << std::endl;


  cv::rfeat::saveFeatureVec(rFeatVec, path+"rfeatVec.feat", false);
  cv::rfeat::saveFeatureVec(rFeatVec, path+"rfeatVec.cfeat", true);

  std::cout << "feature vec saved." << std::endl;

  cv::rfeat::loadFeatureVec(lrFeatVec, path+"rfeatVec.feat", false);
  cv::rfeat::loadFeatureVec(clrFeatVec, path+"rfeatVec.cfeat", true);

  std::cout << "feature vec loaded." << std::endl;

  std::cout << "loaded feature vec : " << std::endl;
  for (size_t i = 0; i < lrFeatVec.size(); ++i)
    std::cout << "Feat : " << i << " \n" << lrFeatVec.at(i);

  std::cout << "loaded compressed feature vec : " << std::endl;
  for (size_t i = 0; i < clrFeatVec.size(); ++i)
    std::cout << "Feat : " << i << " \n" << clrFeatVec.at(i);

  return 0;
}
