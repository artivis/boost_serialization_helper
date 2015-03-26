#ifndef _OPENCV_BOOST_SERIALIZATION_
#define _OPENCV_BOOST_SERIALIZATION_

// Std C++ headers
#include <string>
#include <fstream>

// Boost headers
#include <boost/filesystem.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/archive/iterators/istream_iterator.hpp>

#include <boost/serialization/binary_object.hpp>

// OpenCV header
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace cv
{
  namespace serialization
  {

    namespace io = boost::iostreams;

    //////////////
    /**
     * @brief save cv obj into binari file
     * @param cv obj & file name
     * @param compress - whether binari file should be compressed or not
     * @see loadCVBin()
     */
    template <class T>
    void saveCVBin(T& t, std::string filename, bool compress)
    {
      std::ofstream ofs(filename.c_str());

      { //Scope ensure out dies before ofs
        if (compress)
        {
          io::filtering_streambuf<io::output> out;
          out.push(io::zlib_compressor(io::zlib::best_speed));
          out.push(ofs);

          boost::archive::binary_oarchive oa(out);
          oa << t;
        }
        else
        {
          boost::archive::text_oarchive oa(ofs);
          oa << t;
        }
      }

      ofs.close();
    }

    /**
     * @brief load cv obj from binari file
     * @param cv obj & file name
     * @param compress - whether binari file is compressed or not
     * @see saveCVBin()
     */
    template <class T>
    void loadCVBin(T& m, std::string filename, bool compress)
    {
      std::ifstream ifs(filename.c_str());

      { //Scope ensure in dies before ifs

        if (compress)
        {
          io::filtering_streambuf<io::input> in;
          in.push(io::zlib_decompressor());
          in.push(ifs);

          boost::archive::binary_iarchive ia(in);
          ia >> m;

          while (try_stream_next(ia, ifs, m));
        }
        else
        {
          boost::archive::text_iarchive ia(ifs);
          ia >> m;

          while (try_stream_next(ia, ifs, m));
        }
      }

      ifs.close();
    }

  } //namespace serialization
} //namespace cv

namespace
{
  template<class Archive, class Stream, class Obj>
  bool try_stream_next(Archive &ar, const Stream &s, Obj &o)
  {
    bool success = false;

    try {
      ar >> o;
      success = true;
    } catch (const boost::archive::archive_exception &e) {
      if (e.code != boost::archive::archive_exception::input_stream_error)
        throw;
    }
    return success;
  }
} // namespace

namespace boost {
  namespace serialization {

    template<class Archive>
    void serialize(Archive & ar, cv::Mat& mat, const unsigned int version)
    {
      split_free(ar, mat, version);
    }

    /** Serialization support for cv::Mat */
    template<class Archive>
    void save(Archive &ar, const cv::Mat& m, const unsigned int version)
    {
      size_t elem_size = m.elemSize(); //CV_ELEM_SIZE(cvmat->type)
      size_t elem_type = m.type();

      ar << m.cols;
      ar << m.rows;
      ar << elem_size;
      ar << elem_type;

      const size_t data_size = m.cols * m.rows * elem_size;
      ar << make_array(m.ptr(), data_size);
    }

    /** Serialization support for cv::Mat */
    template<class Archive>
    void load(Archive &ar, cv::Mat& m, const unsigned int version)
    {
      int cols, rows;
      size_t elem_size, elem_type;

      std::cout << "load cv mat" << std::endl;

      ar >> cols;
      ar >> rows;
      ar >> elem_size;
      ar >> elem_type;

      m.create(rows, cols, elem_type);

      size_t data_size = m.cols * m.rows * elem_size;
      ar >> make_array(m.ptr(), data_size);
    }

    template<class Archive, class T, int n, int m>
    void serialize(Archive & ar, cv::Matx<T, n, m>& mat, const unsigned int version)
    {
      split_free(ar, mat, version);
    }

    template<class Archive, class T, int n, int m>
    void save(Archive &ar, const cv::Matx<T, n, m>& mat,
              const unsigned int version)
    {
      int cols = mat.cols;
      int rows = mat.rows;

      ar << cols;
      ar << rows;

      ar << make_array(&mat.val[0], cols * rows);
    }

    template<class Archive, class T, int n, int m>
    void load(Archive &ar, cv::Matx<T, n, m>& mat, const unsigned int version)
    {
      int cols, rows;

      ar >> cols;
      ar >> rows;

      ar >> make_array(&mat.val[0], cols * rows);
    }

    template<class Archive, class T, int S>
    void serialize(Archive &ar, cv::Vec<T, S>& vec, const unsigned int version)
    {
      ar & boost::serialization::base_object<cv::Matx<T, S, 1> >(vec);  //serialize base class
    }

    template<class Archive, class T>
    void serialize(Archive &ar, cv::Point_<T>& pt, const unsigned int version)
    {
      ar & pt.x;
      ar & pt.y;
    }

    template<class Archive, class T>
    void serialize(Archive &ar, cv::Point3_<T>& pt, const unsigned int version)
    {
      ar & pt.x;
      ar & pt.y;
      ar & pt.z;
    }

    template<class Archive>
    void serialize(Archive &ar, cv::KeyPoint& kpt, const unsigned int version)
    {
      ar & kpt.angle;
      ar & kpt.class_id;
      ar & kpt.octave;
      ar & kpt.pt;
      ar & kpt.response;
      ar & kpt.size;
    }

  } //namespace serialization
} //namespace boost

#endif
