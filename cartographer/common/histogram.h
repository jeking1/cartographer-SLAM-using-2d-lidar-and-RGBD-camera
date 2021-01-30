/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
Histogram直方图类,
提供2个操作:
1，Add()添加元素
2，ToString(buckets )以字符串的形式输出buckets个直方图信息,bin大小是篮子个数.

Histogram只有一个数据成员，用vector<float>表示

*/

#ifndef CARTOGRAPHER_COMMON_HISTOGRAM_H_
#define CARTOGRAPHER_COMMON_HISTOGRAM_H_

#include <string>
#include <vector>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

class Histogram {
 public:
  void Add(float value); //添加value,可乱序
  std::string ToString(int buckets) const;  //分为几块


 private:
  std::vector<float> values_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_HISTOGRAM_H_
