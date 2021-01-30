
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
//对x进行四舍五入
inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }


inline void FastGzipString(const std::string& uncompressed,
                           std::string* compressed) {

//对string进行压缩，uncompressed为未压缩string,compressed为压缩后的string
  boost::iostreams::filtering_ostream out;  //创建输出流
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed)); // gzip压缩
  out.push(boost::iostreams::back_inserter(*compressed));   //对compressed使用后插迭代器
  boost::iostreams::write(out,
                          reinterpret_cast<const char*>(uncompressed.data()),
                          uncompressed.size()); //插入compressed
}

inline void FastGunzipString(const std::string& compressed,
                             std::string* decompressed) {
//解压缩string,compressed为压缩过的，decompressed为解压后的
  boost::iostreams::filtering_ostream out;  //创建输出流
  out.push(boost::iostreams::gzip_decompressor());  //gzip解压
  out.push(boost::iostreams::back_inserter(*decompressed)); //对decompressed使用后插迭代器
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
                          compressed.size());   //插入decompressed
}

}  // namespace common
}  // namespace cartographer

/*
c++11 已经支持back_inserter。
std::back_inserter执行push_back操作, 返回值back_insert_iterator, 并实现自增.
具体用法可见 port_le_test.cpp
*/

/*
reinterpret_cast的转换格式：reinterpret_cast <type-id> (expression)
允许将任何指针类型转换为指定的指针类型
*/


#endif  // CARTOGRAPHER_COMMON_PORT_H_
