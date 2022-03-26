/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-17 21:48:17
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-03-25 16:36:05
 */

#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
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
    int N_SCAN;
    int LINES;

    BaseModule() = default;
    virtual ~BaseModule() = default;

    PointCloud::Ptr Concert2PCLCloud(const sensor_msgs::PointCloud2Ptr &cloud_msg)
    {
        PointCloud::Ptr cloud(new PointCloud());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        return cloud;
    }

    sensor_msgs::PointCloud2 Convert2SensorMsg(PointCloud &in_cloud)
    {
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(in_cloud, out_msg);
        return out_msg;
    }
};