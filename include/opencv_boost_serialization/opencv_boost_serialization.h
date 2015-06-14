#ifndef _OPENCV_BOOST_SERIALIZATION_
#define _OPENCV_BOOST_SERIALIZATION_

// Boost serialization headers
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>

// Boost filesystem header
#include <boost/filesystem.hpp>

// Boost ptr_container header
#include <boost/ptr_container/serialize_ptr_vector.hpp>

// Boost algorithm header
#include <boost/algorithm/string/predicate.hpp>

// Boost archive headers
#include <boost/archive/iterators/istream_iterator.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

// Boost iostreams headers
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>

#include <fstream>

// OpenCV header
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace
{
  template<class Archive, class Stream, class Obj>
  bool try_stream_next(Archive &ar, const Stream &s, Obj &o)
  {
    bool success = false;
    try
    {
      ar >> o;
      success = true;
    }
    catch (const boost::archive::archive_exception &e)
    {
      if (e.code != boost::archive::archive_exception::input_stream_error)
        throw;
    }
    return success;
  }

  bool check_path_file(const std::string& file)
  {
    boost::filesystem::path p(file);
    if (!boost::filesystem::is_directory(p.parent_path()) &&
        p.parent_path() != "")
    {
      std::cerr << "ERROR! Directory not found: " << p.parent_path() << std::endl;
      return false;
    }
    else if (!boost::filesystem::is_regular_file(p))
    {
      std::cerr << "ERROR! File not found: " << p.filename() <<
                " in " << p.parent_path() << std::endl;
      return false;
    }
  }
} // namespace


namespace serialization
{
  namespace io = boost::iostreams;

  //////////////
  /**
  * @brief save obj into binari file
  * @param obj & file name
  * @param compress - whether binari file should be compressed or not
  * @see loadBin()
  */
  template <class T>
  void saveBin(T& t, std::string filename, bool compress)
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
        boost::archive::binary_oarchive oa(ofs);
        oa << t;
      }
    }
    ofs.close();
  }

  /**
  * @brief load obj from binari file
  * @param obj & file name
  * @param compress - whether binari file is compressed or not
  * @see saveBin()
  */
  template <class T>
  void loadBin(T& m, std::string filename, bool compress)
  {
    check_path_file(filename);

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
        boost::archive::binary_iarchive ia(ifs);
        ia >> m;
        while (try_stream_next(ia, ifs, m));
      }
    }
    ifs.close();
  }

  //////////////
  /**
  * @brief save obj into binari file
  * @param obj & file name
  * @see loadBin()
  */
  template <class T>
  void saveTxt(T& t, std::string filename)
  {
    std::ofstream ofs(filename.c_str());

    { //Scope ensure out dies before ofs
      boost::archive::text_oarchive oa(ofs);
      oa << t;
    }
    ofs.close();
  }

  /**
  * @brief load obj from binari file
  * @param obj & file name
  * @param compress - whether binari file is compressed or not
  * @see saveBin()
  */
  template <class T>
  void loadTxt(T& m, std::string filename)
  {
    check_path_file(filename);

    std::ifstream ifs(filename.c_str());
    { //Scope ensure in dies before ifs
      boost::archive::text_iarchive ia(ifs);
      ia >> m;
      while (try_stream_next(ia, ifs, m));
    }
    ifs.close();
  }

} //namespace serialization

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
