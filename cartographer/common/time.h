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
预备知识:
在C++11中，<chrono>是标准模板库中与时间有关的头文件。
该头文件中所有函数与类模板均定义在std::chrono命名空间中。

std::chrono::duration：记录时间长度，表示一段时间，如1分钟、2小时、10毫秒等。
表示为类模板duration的对象，用一个count representation与
一个period precision表示。例如，10毫秒的10为count representation，
毫秒为period precision。

第一个模板参数为表示时间计数的数据类型。
成员函数count返回该计数。第二个模板参数表示计数的一个周期，
一般是std::ratio类型，表示一个周期（即一个时间滴答tick）是秒钟的倍数或分数，
在编译时应为一个有理常量。

std::chrono::time_point:记录时间点的，表示一个具体的时间。
例如某人的生日、今天的日出时间等。表示为类模板time_point的对象。
用相对于一个固定时间点epoch(起点)的duration来表示。

std::chrono::clocks:时间点相对于真实物理时间的框架。至少提供了3个clock：
1）system_clock:当前系统范围（即对各进程都一致）的一个实时的日历时钟(Wallclock)。
2）steady_clock:当前系统实现的一个维定时钟，该时钟的每个时间滴答单位是均匀的（即长度相等）。
3）high_resolution_clock:当前系统实现的一个高分辨率时钟。

每一个clock类中都有确定的time_point, duration, Rep, Period类型。
操作有：
now() 当前时间time_point
to_time_t() time_point转换成time_t秒
from_time_t() 从time_t转换成time_point


common/time.h主要功能是提供时间转换函数：


 */

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

//719162是0001年1月1日到1970年1月1日所经历的天数
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;// 0.1us = 10ns
  using duration = std::chrono::duration<rep, period>; 
  
  //time_point的模板参数是UniversalTimeScaleClock
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration; //us
using Time = UniversalTimeScaleClock::time_point;   //时间点

// Convenience functions to create common::Durations.
//将秒数seconds转为c++的duration实例对象
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.

double ToSeconds(Duration duration);
double ToSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
//将UTC时间(微秒)转化为c++的time_point对象
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
//将c++的time_point对象转为UTC时间,单位是us
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
//重载<<操作符,将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time);

// CPU time consumed by the thread so far, in seconds.
double GetThreadCpuTimeSeconds();

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_


/*
Durations
操作：
template <class Rep, class Period = ratio<1> > class duration;

其中：
Rep表示一种数值类型，用来表示Period的数量，比如int float double
Period是ratio类型，用来表示【用秒表示的时间单位】比如second milisecond
常用的duration<Rep,Period>已经定义好了，在std::chrono::duration下：
ratio<3600, 1>                hours
ratio<60, 1>                    minutes
ratio<1, 1>                      seconds
ratio<1, 1000>               microseconds
ratio<1, 1000000>         microseconds
ratio<1, 1000000000>    nanosecons

在其中的ratio这个类模板的原型为
template <intmax_t N, intmax_t D = 1> class ratio;

N代表分子，D代表分母，所以ratio表示一个分数值。
注意，我们自己可以定义Period，比如ratio<1, -2>表示单位时间是-0.5秒。
*/

/*
参考了以下文章：
https://blog.csdn.net/learnmoreonce/article/details/73695081?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.control
https://www.cnblogs.com/jwk000/p/3560086.html
https://www.cnblogs.com/gary-guo/p/13522545.html

c++11 chrono全面解析 https://blog.csdn.net/qq_31175231/article/details/77923212
*/
