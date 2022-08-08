/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-12 09:28:31
 * @LastEditors: Tianyun Xuan
 */
#ifndef POINT_TYPE_H_
#define POINT_TYPE_H_

#define PCL_NO_PRECOMPILE
// #include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointA {
    PCL_ADD_POINT4D;
    double timestamp;
    std::uint16_t intensity;
    std::uint8_t flags;
    std::uint8_t elongation;
    std::uint16_t scan_id;
    std::uint16_t scan_idx;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointA,
    (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(
        std::uint16_t, intensity, intensity)(std::uint8_t, flags, flags)(
        std::uint8_t, elongation, elongation)(std::uint16_t, scan_id,
                                               scan_id)(std::uint16_t, scan_idx,
                                                        scan_idx));
#endif  // POINT_TYPE_H_