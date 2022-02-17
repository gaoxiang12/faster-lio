//
// Created by xiang on 2021/10/8.
//

#ifndef FAST_LIO_OPTIONS_H
#define FAST_LIO_OPTIONS_H

namespace faster_lio::options {

/// fixed params
constexpr double INIT_TIME = 0.1;
constexpr double LASER_POINT_COV = 0.001;
constexpr int PUBFRAME_PERIOD = 20;
constexpr int NUM_MATCH_POINTS = 5;      // 匹配点数
constexpr int MIN_NUM_MATCH_POINTS = 3;  // 最小匹配点数

/// configurable params
extern int NUM_MAX_ITERATIONS;
extern float ESTI_PLANE_THRESHOLD;
extern bool FLAG_EXIT;

}  // namespace faster_lio::options

#endif  // FAST_LIO_OPTIONS_H
