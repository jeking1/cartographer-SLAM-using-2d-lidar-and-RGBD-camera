
/*
    common/port.h 主要功能
    1. 使用std::Lround对浮点数进行四舍五入取整运算
    2. 使用gzip对字符串压缩与解压缩

*/
#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>

namespace cartographer {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

namespace common {

inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

inline void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {
  boost::iostreams::filtering_ostream out;
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size());
}

inline void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PORT_H_
