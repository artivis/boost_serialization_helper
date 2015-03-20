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
      cv::serialization::saveCVBin(m, filename, compress);
    }

    void loadFeature(richFeature& m, std::string filename, bool compress = false)
    {
      cv::serialization::loadCVBin(m, filename, compress);
    }

    class richFeatureVector : std::vector<richFeature>
    {
      friend class boost::serialization::access;

      template<class Archive>
      void serialize(Archive &ar, const unsigned int version)
      {
        ar & *this;
      }
    };

    void saveFeatureVec(richFeatureVector& v, std::string filename, bool compress = false)
    {
      cv::serialization::saveCVBin(v, filename, compress);
    }

    void loadFeatureVec(richFeatureVector& v, std::string filename, bool compress = false)
    {
      cv::serialization::loadCVBin(v, filename, compress);
    }

  } //namespace rfeat
} // namespace cv



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
//    void serialize(Archive &ar, cv::rfeat::richFeatureVector& featv, const unsigned int version)
//    {
//      split_free(ar, featv, version);
//    }

    template<class Archive>
    void serialize(Archive &ar, std::vector< cv::Mat >* desv, const unsigned int version)
    {
      ar & desv;
    }

//    template<class Archive>
//    void save(Archive &ar, cv::rfeat::richFeatureVector& featv, const unsigned int version)
//    {
//      ar & featv.size();
//      std::copy(featv.begin(), featv.end(), ar);
//    }

//    template<class Archive>
//    void load(Archive &ar, cv::rfeat::richFeatureVector& featv, const unsigned int version)
//    {
////      boost::archive::iterators::istream_iterator<cv::rfeat::richFeatureVector> iter_begin(ar);
////      boost::archive::iterators::istream_iterator<cv::rfeat::richFeatureVector> iter_end;

////      std::copy(iter_begin, iter_end, std::back_inserter(featv));
////      cv::rfeat::richFeature r;
//      cv::rfeat::richFeatureVector featv_tmp;
//      size_t size;
//      ar & size;

//      featv_tmp.reserve(size);

//      ar & featv_tmp;
////      featv.push_back(r);

////      std::copy(std::istream_iterator<cv::rfeat::richFeature>(ar), std::istream_iterator<cv::rfeat::richFeature>(),
////                std::back_inserter<cv::rfeat::richFeatureVector>(featv));




////      ar & featv;
////      featv.push_back(feat);
//    }

  } // namespace serialization
} // namespace boost



#endif // OPENCV_RICH_FEATURE_H
