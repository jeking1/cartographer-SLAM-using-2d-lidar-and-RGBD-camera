# common

common文件夹主要文件及功能：

1. common/port.h
    主要功能：   1. 使用std::Lround对浮点数进行四舍五入取整运算
                2. 使用gzip对字符串压缩与解压缩
    · int RoundToInt() //四舍五入
    · void FastGzipString(const std::string& uncompressed,std::string* compressed)  //对string进行压缩
    · void FastGunzipString(const std::string& compressed,std::string* decompressed)    //解压缩string

2. common/time.h
    主要功能：提供时间转换函数
    · FromSeconds(double seconds);  //将秒数seconds转为c++的duration实例对象
    · FromMilliseconds(int64 milliseconds); //将秒数seconds转为c++的duration实例对象
    · ToSeconds(Duration duration)  //返回给定的持续时间
    · ToSeconds(std::chrono::steady_clock::duration duration);
    · FromUniversal(int64 ticks);   //将UTC时间(微秒)转化为c++的time_point对象
    · ToUniversal(Time time);   //将c++的time_point对象转为UTC时间,单位是us

3. common/fixed_ratio_sampler.h
    主要功能：FixedRatioSampler是频率固定的采样器类，目的是从数据流中均匀的按照固定频率采样数据。FixedRatioSampler不可拷贝,不可赋值.
    · Pulse()   //产生一个事件pulses,并且:如果计入采样samples，返回true
    · DebugString() //以string形式输出采样率

4. common/internal/rate_timer.h
    主要功能：定义了 RateTimer-脉冲频率计数类,作用是计算在一段时间内的脉冲率
    · RateTimer(const common::Duration window_duration)  //提供时间段Duration
    · double ComputeRate() // 返回事件脉冲率,单位hz
    · double ComputeWallTimeRateRatio() // 返回真实时间与墙上挂钟时间的比率
    · void Pulse(common::Time time) // 脉冲记录事件发生
    · std::string DebugString() // 以字符串形式返回debug描述
    · std::vector<double> ComputeDeltasInSeconds()// 计算连续脉冲之间的以秒为单位的差
    · std::string DeltasDebugString() // 返回增量的平均值和标准偏差

5. common/histogram.h
    主要功能：Histogram直方图类
    · void Add(float value); //添加value,可乱序
    · std::string ToString(int buckets) const;  //分为几块

6. common/math.h
    主要功能：实现数学计算，包括：区间截断.求n次方.求平方.幅度角度转换.归一化.反正切值
    · Clamp(const T value, const T min, const T max) //将val截取到区间min至max中.
    · Power(T base, int exponent)   //计算base的exp次方
    · Pow2(T a) //求平方
    · double DegToRad(double deg) //角度到弧度的转换. 60° -> pi/3
    · double RadToDeg(double rad) //弧度到角度的转换, pi/3 -> 60°
    · NormalizeAngleDifference(T difference) //将角度差转换为[-pi;pi]
    · atan2(const Eigen::Matrix<T, 2, 1>& vector)//atan2 返回原点至点(x,y)的方位角，即与 x 轴的夹角，也可以理解为计算复数 x+yi 的辐角,范围是[-pi,pi]

7. common/make_unique.h