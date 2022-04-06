/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-17 21:48:17
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-04-05 11:26:56
 */

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
// struct PointXYZIRL
// {
//     PCL_ADD_POINT4D
//     PCL_ADD_INTENSITY;
//     int ring;
//     int line;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRL,
//                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(int, ring, ring)(int, line, line))

typedef pcl::PointXYZI PointType;

typedef pcl::PointCloud<PointType> PointCloud;

struct smoothness
{
    int id;
    double value;
};

struct by_value
{
    bool operator()(smoothness &left, smoothness &right)
    {
        return (left.value < right.value);
    }
};

class BaseModule
{
public:
    int N_SCAN = 1800;
    int LINES = 16;
    double angle_gap = 2;
    double angle_x = 0.2;

    BaseModule() = default;
    virtual ~BaseModule() = default;

    PointCloud::Ptr Concert2PCLCloud(const sensor_msgs::PointCloud2Ptr &cloud_msg)
    {
        PointCloud::Ptr cloud(new PointCloud());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        return cloud;
    };

    sensor_msgs::PointCloud2 Convert2SensorMsg(PointCloud::Ptr in_cloud)
    {
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(*in_cloud, out_msg);
        return out_msg;
    };

    void RemoveNaNPoints(PointCloud::Ptr in_cloud)
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*in_cloud, *in_cloud, indices);
    };
};