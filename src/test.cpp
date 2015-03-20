#include <ros/ros.h>
#include <ros/package.h>

#include <opencv_boost_serialization/opencv_boost_serialization.h>

int main(int argc, char** argv)
{
  std::string path = ros::package::getPath("opencv_boost_serialization") + "/test/etc/";

  cv::Vec3i pose_3i;
  cv::Vec4d pose_4d;

  cv::Vec3i lpose_3i;
  cv::Vec4d lpose_4d;

  cv::Vec3i lcpose_3i;
  cv::Vec4d lcpose_4d;

  pose_3i[0] = 1;
  pose_3i[1] = 2;
  pose_3i[2] = 3;

  pose_4d[0] = 0.15;
  pose_4d[1] = 1.25;
  pose_4d[2] = 2.35;
  pose_4d[3] = 3.45;

  std::cout << "pose 3i : " << pose_3i << std::endl;
  std::cout << "pose 4d : " << pose_4d << std::endl;

  cv::serialization::saveCVBin(pose_3i, path+"pose3i.feat", false);
  cv::serialization::saveCVBin(pose_4d, path+"pose4d.feat", false);

  std::cout << "vec saved." << std::endl;

  cv::serialization::loadCVBin(lpose_3i, path+"pose3i.feat", false);
  cv::serialization::loadCVBin(lpose_4d, path+"pose4d.feat", false);

  std::cout << "loaded lpose_3i : " << lpose_3i << std::endl;
  std::cout << "loaded lpose_4d : " << lpose_4d << std::endl;

  cv::serialization::saveCVBin(pose_3i, path+"pose3i.cfeat", true);
  cv::serialization::saveCVBin(pose_4d, path+"pose4d.cfeat", true);

  cv::serialization::loadCVBin(lcpose_3i, path+"pose3i.cfeat", true);
  cv::serialization::loadCVBin(lcpose_4d, path+"pose4d.cfeat", true);

  std::cout << "loaded lcpose_3i : " << lcpose_3i << std::endl;
  std::cout << "loaded lcpose_4d : " << lcpose_4d << std::endl;

  cv::Point2i pt_2i, lpt_2i, lcpt_2i;
  cv::Point3d pt_d3, lpt_d3, lcpt_d3;

  pt_2i.x = 55;
  pt_2i.y = 44;

  pt_d3.x = 123.456;
  pt_d3.y = 456.789;
  pt_d3.z = 789.123;

  std::cout << "pt_2i : " << pt_2i << std::endl;
  std::cout << "pt_d3 : " << pt_d3 << std::endl;

  cv::serialization::saveCVBin(pt_2i, path+"pt2i.feat", false);
  cv::serialization::saveCVBin(pt_d3, path+"pt3d1.feat", false);

  std::cout << "points saved." << std::endl;

  cv::serialization::loadCVBin(lpt_2i, path+"pt2i.feat", false);
  cv::serialization::loadCVBin(lpt_d3, path+"pt3d1.feat", false);

  std::cout << "loaded lpose_3i : " << lpt_2i << std::endl;
  std::cout << "loaded lpose_4d : " << lpt_d3 << std::endl;

  cv::serialization::saveCVBin(pt_2i, path+"pt2i.cfeat", true);
  cv::serialization::saveCVBin(pt_d3, path+"pt3d1.cfeat", true);

  cv::serialization::loadCVBin(lcpt_2i, path+"pt2i.cfeat", true);
  cv::serialization::loadCVBin(lcpt_d3, path+"pt3d1.cfeat", true);

  std::cout << "loaded lcpose_3i : " << lcpt_2i << std::endl;
  std::cout << "loaded lcpose_4d : " << lcpt_d3 << std::endl;


  return 0;
}
