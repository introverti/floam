#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>

#include "laserMappingClass.h"
#include "laserProcessingClass.h"
#include "lidar.h"
#include "odomEstimationClass.h"
#include "point_type.h"

void AtoXYZI(const pcl::PointCloud<PointA>::Ptr& cloud_in,
             pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_out) {
    cloud_out->clear();
    for (auto& point : cloud_in->points) {
        pcl::PointXYZI temp{point.z, -point.y, point.x};
        temp.intensity = point.intensity;
        cloud_out->emplace_back(temp);
    }
}
int main(int argc, char** argv) {
    std::vector<std::string> pcds{};
    std::string folder;
    if (argc > 1) {
        folder = argv[1];
        for (auto& item : std::filesystem::directory_iterator(folder)) {
            std::filesystem::path temp = item;
            std::filesystem::path filename = temp.stem();
            if (temp.extension() == ".pcd") {
                pcds.emplace_back(filename);
            }
        }
        std::sort(pcds.begin(), pcds.end());
    }

    if (pcds.size()) {
        // Initialization
        LaserProcessingClass feature = LaserProcessingClass();
        OdomEstimationClass odometry = OdomEstimationClass();
        LaserMappingClass mapping = LaserMappingClass();
        Lidar::Lidar param;
        double map_resolution = 0.4;
        bool is_odom_inited = false;
        feature.init(param);
        odometry.init(param, map_resolution);
        mapping.init(map_resolution);
        std::cout << "Initialization Done, total frame size : " << pcds.size()
                  << std::endl;
        std::ofstream f("/home/xavier/repos/floam/pose.txt", std::ios::app);
        // Loop
        for (auto& file : pcds) {
            unsigned long timestamp = std::stoll(file);
            pcl::PointCloud<PointA>::Ptr cloud(new pcl::PointCloud<PointA>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_edge(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_surf(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pc_feature(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::io::loadPCDFile<PointA>(folder + "/" + file + ".pcd", *cloud);
            std::cout << "Load file " << file
                      << " with point size : " << cloud->size() << std::endl;

            AtoXYZI(cloud, pc_in);
            feature.featureExtraction(pc_in, pc_edge, pc_surf);
            std::cout << "Edge feature point size : " << pc_edge->size()
                      << std::endl;
            std::cout << "Surf feature point size : " << pc_surf->size()
                      << std::endl;

            // if (is_odom_inited == false) {
            //     odometry.initMapWithPoints(pc_edge, pc_surf);
            //     is_odom_inited = true;
            //     std::cout << "Odometry inited" << std::endl;
            // } else {
            //     odometry.updatePointsToMap(pc_edge, pc_surf);
            //     std::cout << "Odometry estimation Done" << std::endl;
            // }

            // Eigen::Quaterniond q_current(odometry.odom.rotation());
            // // q_current.normalize();
            // Eigen::Vector3d t_current = odometry.odom.translation();
            // // tx ty tz qx qy qz qw
            // f << timestamp << " " << t_current.x() << " " << t_current.y()
            //   << " " << t_current.z() << " " << q_current.x() << " "
            //   << q_current.y() << " " << q_current.z() << " " << q_current.w()
            //   << std::endl;
            // pc_feature = pc_edge;
            // *pc_feature += *pc_surf;
            // std::cout << "Combined current frame size : " << pc_feature->size()
            //           << std::endl;
            // mapping.updateCurrentPointsToMap(pc_feature, odometry.odom);
            // std::cout << "Map updated" << std::endl;
        }
        // pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = mapping.getMap();
        // pcl::io::savePCDFileBinary(folder + "/map.pcd", *pc_map);
        // f.close();
    }
}