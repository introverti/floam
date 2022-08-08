// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_
#define _LIDAR_H_

// define lidar parameter

namespace Lidar {

struct Lidar {
    double max_distance = 150;
    double min_distance = 10;
    int num_lines = 150;
    size_t max_points_per_line = 1200;
    size_t min_points_per_line = 64;
};

}  // namespace Lidar

#endif  // _LIDAR_H_
