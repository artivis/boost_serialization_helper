#ifndef OPENCV_RICH_FEATURE_H
#define OPENCV_RICH_FEATURE_H

#include <opencv_boost_serialization/opencv_boost_serialization.h>

namespace
{
  const double CV_THRESH = 1e-9;
}

namespace cv {
  namespace rfeat {

    typedef cv::Mat Feature; //single feature
    typedef cv::Mat Features; //set of features
    typedef std::vector<Feature> FeatureVector; //vector of features

    typedef cv::Mat Descriptor; //single feature
    typedef cv::Mat Descriptors; //set of features
    typedef std::vector<Descriptor> DescriptorVector; //vector of features

    typedef cv::KeyPoint Keypoint;
    typedef std::vector<cv::KeyPoint> KeypointVector;

    struct richFeatures
    {
      richFeatures() :
        stamp(-1.) {}
      richFeatures(KeypointVector kpts, Descriptors dscp) :
        keypoints(kpts), descriptors(dscp), stamp(-1.) {}

      void operator+=(richFeatures T)
      {
        if (!this->descriptors.empty() && !T.descriptors.empty())
          cv::vconcat(descriptors, T.descriptors, descriptors);
        else if (!T.descriptors.empty())
          T.descriptors.copyTo(this->descriptors);

        keypoints.insert(keypoints.end(),T.keypoints.begin(),T.keypoints.end());
      }

      void setStamp(double t) {stamp = t;}

      void setPose(double pose_in) {pose = pose_in;}

      void clear()
      {
        keypoints.clear();
        descriptors.release();
      }

      KeypointVector keypoints;
      Descriptors descriptors;

      double stamp;
      cv::Vec3d pose;
    };

    /////////////////////////////////////

    class richFeature : public cv::KeyPoint
    {
    public:

      richFeature() :
        stamp(-1.) {}

      richFeature(const cv::KeyPoint& kpts, const cv::Mat& desc) :
        stamp(-1.)
      {
        angle = kpts.angle;
        class_id = kpts.class_id;
        octave = kpts.octave;
        pt = kpts.pt;
        response = kpts.response;
        size = kpts.size;

        descriptor = desc.clone();
      }

      richFeature(const cv::KeyPoint& kpts, const cv::Mat& desc, const cv::Vec3d& pose_in) :
        stamp(-1.)
      {
        angle = kpts.angle;
        class_id = kpts.class_id;
        octave = kpts.octave;
        pt = kpts.pt;
        response = kpts.response;
        size = kpts.size;

        descriptor = desc.clone();

        pose = pose_in;
      }

      richFeature(const cv::KeyPoint& kpts, const cv::Mat& desc, const cv::Vec3d& pose_in, double stamp_in)
      {
        angle = kpts.angle;
        class_id = kpts.class_id;
        octave = kpts.octave;
        pt = kpts.pt;
        response = kpts.response;
        size = kpts.size;

        descriptor = desc.clone();

        pose = pose_in;

        stamp = stamp_in;
      }

      richFeature(const richFeature& f)
      {
        angle = f.angle;
        class_id = f.class_id;
        octave = f.octave;
        pt = f.pt;
        response = f.response;
        size = f.size;
        descriptor = f.descriptor.clone();
        pose = f.pose;
        stamp = f.stamp;
      }

      richFeature& operator=(const richFeature& T)
      {
        angle = T.angle;
        class_id = T.class_id;
        octave = T.octave;
        pt = T.pt;
        response = T.response;
        size = T.size;
        descriptor = T.descriptor.clone();
        pose = T.pose;
        stamp = T.stamp;

        return *this;
      }

      bool operator==(const richFeature& T)
      {
        if ( cv::norm(descriptor, T.descriptor) < CV_THRESH )
          if (pt == T.pt)
            if (pose == T.pose)
              if (stamp == T.stamp)
                return true;

        return false;
      }

      friend std::ostream &operator<<(std::ostream &out, const richFeature &f)
      {
        out << std::endl;
        out << "keypoint : "    << std::endl;
        out << " angle : "      << f.angle       << std::endl;
        out << " class_id : "   << f.class_id    << std::endl;
        out << " octave : "     << f.octave      << std::endl;
        out << " pt : "         << f.pt          << std::endl;
        out << " response : "   << f.response    << std::endl;
        out << " size : "       << f.size        << std::endl;
        out << "descriptor : "  << f.descriptor  << std::endl;
        out << "stamp : "       << f.stamp       << std::endl;
        out << "pose : "        << f.pose        << std::endl;
        out << std::endl;
      }

      void setStamp(double t) {stamp = t;}

      void setPose(double pose_in) {pose = pose_in;}

      friend void saveFeature(richFeature& m, std::string filename, bool compress);
      friend void loadFeature(richFeature& m, std::string filename, bool compress);

      Descriptor descriptor;

      double stamp;
      cv::Vec3d pose;

    protected:

      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & boost::serialization::base_object<cv::KeyPoint>(*this);  //serialize base class
        ar & descriptor;
        ar & pose;
        ar & stamp;
      }
    };

    void saveFeature(richFeature& m, std::string filename, bool compress = false)
    {
      serialization::saveBin(m, filename, compress);
    }

    void loadFeature(richFeature& m, std::string filename, bool compress = false)
    {
      serialization::loadBin(m, filename, compress);
    }

    typedef std::vector<richFeature> richFeatureVector;

    void saveFeatureVec(richFeatureVector& v, std::string filename, bool compress = false)
    {
      serialization::saveBin(v, filename, compress);
    }

    void loadFeatureVec(richFeatureVector& v, std::string filename, bool compress = false)
    {
      serialization::loadBin(v, filename, compress);
    }

//    void saveRichFeatureVec(const std::string& filename, const richFeature& f)
//    {
//      cv::FileStorage fs(filename, cv::FileStorage::WRITE);

//      if (!fs.isOpened())
//        return;

//      fs << "features " << "[";

//      fs << "{" << "angle" << f.angle << "classid" << f.class_id << "octave" << f.octave
//         << "pose" << f.pose << "pt" << f.pt << "response" << f.response << "size" << f.size
//         << "stamp" << f.stamp << "descriptor" << f.descriptor << "}";

//      fs << "]";

//      fs.release();
//    }

//    void loadRichFeatureVec(const std::string& filename, richFeature& f)
//    {
//      cv::FileStorage fs(filename, cv::FileStorage::READ);

//      if (!fs.isOpened())
//        return;

//      cv::FileNode features = fs["features"];
//      cv::FileNodeIterator it = features.begin();

//      (*it)["angle"] >> f.angle;
//      (*it)["classid"] >> f.class_id;
//      (*it)["octave"] >> f.octave;
//      (*it)["pose"] >> f.pose;
//      (*it)["pt"] >> f.pt;
//      (*it)["response"] >> f.response;
//      (*it)["size"] >> f.size;
//      (*it)["stamp"] >> f.stamp;
//      (*it)["descriptor"] >> f.descriptor;

//      fs.release();
//    }

//    void saveRichFeatureVec(const std::string& filename, const richFeatureVector& v)
//    {
//      cv::FileStorage fs(filename, cv::FileStorage::WRITE); cv::rfeat::richFeature t; t.descriptor.ptr();

//      if (!fs.isOpened())
//        return;

//      fs << "features " << "[";

//      for (int i=0; i<v.size(); ++i)
//      {
//        fs << "{" << "angle" << v.at(i).angle << "classid" << v.at(i).class_id << "octave" << v.at(i).octave
//           << "pose" << v.at(i).pose << "pt" << v.at(i).pt << "response" << v.at(i).response << "size" << v.at(i).size
//           << "stamp" << v.at(i).stamp << "descriptor" << v.at(i).descriptor << "}";
//      }

//      fs << "]";

//      fs.release();
//    }

//    void loadRichFeatureVec(const std::string& filename, richFeatureVector& v)
//    {
//      cv::FileStorage fs(filename, cv::FileStorage::READ);

//      if (!fs.isOpened())
//        return;

//      v.clear();

//      cv::FileNode features = fs["features"];
//      cv::FileNodeIterator it = features.begin(), it_end = features.end();

//      for ( ; it!=it_end; ++it)
//      {
//        cv::rfeat::richFeature r;

//        (*it)["angle"] >> r.angle;
//        (*it)["classid"] >> r.class_id;
//        (*it)["octave"] >> r.octave;
//        (*it)["pose"] >> r.pose;
//        (*it)["pt"] >> r.pt;
//        (*it)["response"] >> r.response;
//        (*it)["size"] >> r.size;
//        (*it)["stamp"] >> r.stamp;
//        (*it)["descriptor"] >> r.descriptor;

//        v.push_back(r);
//      }

//      fs.release();
//    }

  } //namespace rfeat
} // namespace cv

BOOST_CLASS_TRACKING(cv::rfeat::DescriptorVector, boost::serialization::track_always)
BOOST_CLASS_TRACKING(cv::rfeat::richFeature, boost::serialization::track_always)
BOOST_CLASS_TRACKING(cv::rfeat::richFeatureVector, boost::serialization::track_always)

namespace boost {
  namespace serialization {

    template<class Archive>
    void serialize(Archive &ar, cv::rfeat::richFeature& feat, const unsigned int version)
    {
      ar & boost::serialization::base_object<cv::KeyPoint>(feat);  //serialize base class
      ar & feat.descriptor;
      ar & feat.pose;
      ar & feat.stamp;
    }

//    template<class Archive>
//    void serialize(Archive &ar, cv::rfeat::DescriptorVector& desv, const unsigned int version)
//    {
//      split_free(ar, desv, version);
//    }

//    template<class Archive>
//    void save(Archive &ar, cv::rfeat::DescriptorVector& featv, const unsigned int version)
//    {
////      ar & featv.size();
////      std::copy(featv.begin(), featv.end(), ar);

//      // this is not called
//      // default stl container save fct called instead

//      std::cout << "dere save" << std::endl;
//      size_t vec_size = featv.size();
//      ar << vec_size;
//      for (size_t i = 0; i <featv.size(); ++i)
//        ar << featv[i];
//    }

//    template<class Archive>
//    void load(Archive &ar, cv::rfeat::DescriptorVector& featv, const unsigned int version)
//    {
//      size_t vec_size;
//      std::cout << "got here" << std::endl;
////      ar >> vec_size;

////      std::cout << "vec_size" << vec_size << std::endl;

//      ar >> featv;

////      for (size_t i = 0; i < vec_size; ++i)
////      {
////        cv::Mat m;
////        ar >> m;
////        std::cout << m << " *0* " << std::endl;
////        featv.push_back(m);
////      }
//    }

//    template<class Archive>
//    void serialize(Archive &ar, cv::rfeat::richFeatureVector& featv, const unsigned int version)
//    {
//      split_free(ar, featv, version);
//    }

//    template<class Archive>
//    void save(Archive &ar, cv::rfeat::richFeatureVector& featv, const unsigned int version)
//    {
////      ar & featv.size();
////      std::copy(featv.begin(), featv.end(), ar);
//      std::cout << "dere save" << std::endl;
//      for (size_t i = 0; i <featv.size(); ++i)
//        ar << featv[i];
//    }

//    template<class Archive>
//    void load(Archive &ar, cv::rfeat::richFeatureVector& featv, const unsigned int version)
//    {

//    }

  } // namespace serialization
} // namespace boost



#endif // OPENCV_RICH_FEATURE_H
