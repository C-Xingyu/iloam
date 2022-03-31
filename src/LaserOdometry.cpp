/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-24 20:25:44
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-03-31 17:35:44
 */
#pragma once
#include "../include/utility.h"
#include <queue>
#include <mutex>
#include <thread>
#include <pcl/kdtree/kdtree_flann.h>
#include <chrono>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "LiDARfactor.hpp"

class iLaserOdometry : public BaseModule
{
private:
    ros::NodeHandle inh;

    ros::Subscriber fullCloudSub;
    ros::Subscriber sharpCloudSub;
    ros::Subscriber lessSharpCloudSub;
    ros::Subscriber surfCloudSub;
    ros::Subscriber lessSurfCloudSub;

    ros::Publisher odometryPub;

    std::queue<sensor_msgs::PointCloud2::Ptr> fullCloudBuf;
    std::queue<sensor_msgs::PointCloud2::Ptr> sharpCloudBuf;
    std::queue<sensor_msgs::PointCloud2::Ptr> lessSharpCloudBuf;
    std::queue<sensor_msgs::PointCloud2::Ptr> surfCloudBuf;
    std::queue<sensor_msgs::PointCloud2::Ptr> lessSurfCloudBuf;
    std::mutex bufMutex;

    PointCloud::Ptr fullCloud;
    PointCloud::Ptr sharpCloud;
    PointCloud::Ptr lessSharpCloud;
    PointCloud::Ptr surfCloud;
    PointCloud::Ptr lessSurfCloud;

    double timeSharpCloud;
    double timeLessSharpCloud;
    double timeSurfCloud;
    double timeLessSurfCloud;
    double timeFullCloud;

    bool init;
    pcl::KdTreeFLANN<PointType>::Ptr lastSharpCloudTree;
    pcl::KdTreeFLANN<PointType>::Ptr lastSurfCloudTree;
    PointCloud::Ptr lastSharpPointCloud;
    PointCloud::Ptr lastSurfPointCloud;

    double NearestThreshold;
    int NEAR_SCAN_NUM;
    int ITER_NUM;
    int minCorrespondence;
    double param_q[4];
    double param_t[3];
    Eigen::Map<Eigen::Quaterniond> Q_last_curr = Eigen::Map<Eigen::Quaterniond>(param_q);
    Eigen::Map<Eigen::Vector3d> T_last_curr = Eigen::Map<Eigen::Vector3d>(param_t);
    Eigen::Quaterniond Q_world_curr;
    Eigen::Vector3d T_world_curr;

public:
    iLaserOdometry(ros::NodeHandle &nh) : inh(nh), fullCloud(new PointCloud()),
                                          sharpCloud(new PointCloud()), lessSharpCloud(new PointCloud()),
                                          surfCloud(new PointCloud()), lessSurfCloud(new PointCloud()),
                                          lastSharpPointCloud(new PointCloud()), lastSurfPointCloud(new PointCloud()),
                                          lastSharpCloudTree(new pcl::KdTreeFLANN<PointType>()),
                                          lastSurfCloudTree(new pcl::KdTreeFLANN<PointType>())
    {
        InitialParams();
        fullCloudSub = inh.subscribe("/sorted_cloud", 1, &iLaserOdometry::FullCloudHandler, this);
        sharpCloudSub = inh.subscribe("/sharp_cloud", 1, &iLaserOdometry::SharpCloudHandler, this);
        lessSharpCloudSub = inh.subscribe("/less_sharp_cloud", 1, &iLaserOdometry::LessSharpCloudHandler, this);
        surfCloudSub = inh.subscribe("/surf_cloud", 1, &iLaserOdometry::SurfCloudHandler, this);
        lessSurfCloudSub = inh.subscribe("/less_surf_cloud", 1, &iLaserOdometry::LessSurfCloudHandler, this);
        odometryPub = inh.advertise<nav_msgs::Odometry>("/odom", 10);

        if (IsEmpty() && AlignTimeStamp())
        {
            Process();
        }
        else
        {
            ROS_INFO("Please ensure the buffer of point cloud isn't empty ,or align the timestamp!");
            ROS_BREAK();
        }
        ros::spin();
    }

    void FullCloudHandler(const sensor_msgs::PointCloud2::Ptr &fullCloud)
    {
        std::lock_guard<std::mutex> lock(bufMutex);
        fullCloudBuf.push(fullCloud);
    }
    void SharpCloudHandler(const sensor_msgs::PointCloud2::Ptr &sharpCloud)
    {
        std::lock_guard<std::mutex> lock(bufMutex);
        sharpCloudBuf.push(sharpCloud);
    }

    void LessSharpCloudHandler(const sensor_msgs::PointCloud2::Ptr &lessSharpCloud)
    {
        std::lock_guard<std::mutex> lock(bufMutex);
        lessSharpCloudBuf.push(lessSharpCloud);
    }

    void SurfCloudHandler(const sensor_msgs::PointCloud2::Ptr &surfCloud)
    {
        std::lock_guard<std::mutex> lock(bufMutex);
        surfCloudBuf.push(surfCloud);
    }
    void LessSurfCloudHandler(const sensor_msgs::PointCloud2::Ptr &lessSurfCloud)
    {
        std::lock_guard<std::mutex> lock(bufMutex);
        lessSurfCloudBuf.push(lessSurfCloud);
    }

    bool IsEmpty()
    {

        if (fullCloudBuf.empty() || sharpCloudBuf.empty() || lessSharpCloudBuf.empty() ||
            surfCloudBuf.empty() || lessSurfCloudBuf.empty())
            return false;
        return true;
    }

    bool AlignTimeStamp()
    {
        timeSharpCloud = sharpCloudBuf.front()->header.stamp.toSec();
        timeLessSharpCloud = lessSharpCloudBuf.front()->header.stamp.toSec();
        timeSurfCloud = surfCloudBuf.front()->header.stamp.toSec();
        timeLessSurfCloud = lessSurfCloudBuf.front()->header.stamp.toSec();
        timeFullCloud = fullCloudBuf.front()->header.stamp.toSec();

        if ((timeSharpCloud != timeFullCloud) || (timeLessSharpCloud != timeFullCloud) ||
            (timeSurfCloud != timeFullCloud) || (timeLessSurfCloud != timeFullCloud))
            return false;
        return true;
    }

    void Process()
    {
        sharpCloud->clear();
        lessSharpCloud->clear();
        surfCloud->clear();
        lessSurfCloud->clear();
        fullCloud->clear();

        bufMutex.lock();
        sharpCloud = Concert2PCLCloud(sharpCloudBuf.front());
        sharpCloudBuf.pop();
        lessSharpCloud = Concert2PCLCloud(lessSharpCloudBuf.front());
        lessSharpCloudBuf.pop();
        surfCloud = Concert2PCLCloud(surfCloudBuf.front());
        surfCloudBuf.pop();
        lessSurfCloud = Concert2PCLCloud(lessSurfCloudBuf.front());
        lessSurfCloudBuf.pop();
        fullCloud = Concert2PCLCloud(fullCloudBuf.front());
        fullCloudBuf.pop();
        bufMutex.unlock();

        if (!init)
        {
            ROS_INFO("Initialed!");
            init = true;
        }
        else
        {
            SetOdometry();
            PublishTopics();
            FreeMemory();
        }
        lastSharpPointCloud = sharpCloud;
        lastSurfPointCloud = surfCloud;
        lastSharpCloudTree->setInputCloud(sharpCloud);
        lastSurfCloudTree->setInputCloud(surfCloud);
    }

    void SetOdometry()
    {
        int sharpCloudNum = sharpCloud->size();
        int surfCloudNum = surfCloud->size();

        ceres::LossFunction *loss_func = new ceres::HuberLoss(0.1);
        ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(param_q, 4, q_parameterization);
        problem.AddParameterBlock(param_t, 3);

        auto start = std::chrono::steady_clock::now();

        for (int iter = 0; iter < ITER_NUM; ++iter)
        {
            int sharpCorrespondence = 0;
            int surfCorrespondence = 0;
            std::vector<int> sharpPointsIndices;
            std::vector<float> sharpPointsDist;
            std::vector<int> surfPointsIndices;
            std::vector<float> surfPointsDist;

            for (int i = 0; i < sharpCloudNum; ++i)
            {
                PointType currSharpPoint = sharpCloud->points[i];
                PointType transedSharpPoint;
                Transform2Start(currSharpPoint, transedSharpPoint);
                lastSharpCloudTree->nearestKSearch(transedSharpPoint, 1, sharpPointsIndices, sharpPointsDist);

                if (sharpPointsDist[0] < NearestThreshold)
                {
                    int lastSharpPointScanIdx = int(lastSharpPointCloud->points[sharpPointsIndices[0]].intensity) % 100;
                    int closestPointRowIdx = sharpPointsIndices[0];

                    double minDist2 = std::numeric_limits<double>::max();

                    int minDistPointIdx2;

                    PointType closestPoint = lastSharpPointCloud->points[closestPointRowIdx];
                    for (int j = closestPointRowIdx + 1; j < sharpCloudNum; j++)
                    {
                        int thisLastSharpPointScanIdx = int(sharpCloud->points[i].intensity) % 100;
                        if (thisLastSharpPointScanIdx <= lastSharpPointScanIdx)
                            continue;
                        if (thisLastSharpPointScanIdx > lastSharpPointScanIdx + NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSharpPointCloud->points[j];

                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);
                        if (dist < minDist2)
                        {
                            minDist2 = dist;
                            minDistPointIdx2 = j;
                        }
                    }
                    for (int j = closestPointRowIdx - 1; j >= 0; j--)
                    {
                        int thisLastSharpPointScanIdx = int(sharpCloud->points[i].intensity) % 100;
                        if (thisLastSharpPointScanIdx <= lastSharpPointScanIdx)
                            continue;
                        if (thisLastSharpPointScanIdx > lastSharpPointScanIdx - NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSharpPointCloud->points[j];

                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);
                        if (dist < minDist2)
                        {
                            minDist2 = dist;
                            minDistPointIdx2 = j;
                        }
                    }

                    if (minDistPointIdx2 >= 0)
                    {
                        Eigen::Vector3d currP{sharpCloud->points[i].x, sharpCloud->points[i].y,
                                              sharpCloud->points[i].z};
                        Eigen::Vector3d closestP{lastSharpPointCloud->points[closestPointRowIdx].x,
                                                 lastSharpPointCloud->points[closestPointRowIdx].y,
                                                 lastSharpPointCloud->points[closestPointRowIdx].z};
                        Eigen::Vector3d closestP2{lastSharpPointCloud->points[minDistPointIdx2].x,
                                                  lastSharpPointCloud->points[minDistPointIdx2].y,
                                                  lastSharpPointCloud->points[minDistPointIdx2].z};
                        double ratio = (int(sharpCloud->points[i].intensity) / 100) / N_SCAN;
                        ceres::CostFunction *cost_func = LidarEdegFactor::Create(currP, closestP, closestP2, ratio);
                        problem.AddResidualBlock(cost_func, loss_func, param_q, param_t);
                        sharpCorrespondence++;
                    }
                }
            }

            for (int i = 0; i < surfCloudNum; ++i)
            {
                PointType currSurfPoint = surfCloud->points[i];
                PointType transedSurfPoint;
                Transform2Start(currSurfPoint, transedSurfPoint);
                lastSurfCloudTree->nearestKSearch(transedSurfPoint, 1, surfPointsIndices, surfPointsDist);
                if (surfPointsDist[0] < NearestThreshold)
                {
                    int lastSurfPointScanIdx = int(lastSurfPointCloud->points[surfPointsIndices[0]].intensity) % 100;
                    int closestPointIdx = surfPointsIndices[0];
                    double minDist2 = std::numeric_limits<double>::max();
                    double minDist3 = std::numeric_limits<double>::max();
                    int minDistPointIdx2, minDistPointIdx3;
                    PointType closestPoint = lastSurfPointCloud->points[closestPointIdx];

                    for (int j = closestPointIdx + 1; j < surfCloudNum; ++j)
                    {
                        int thisLastSurfPointScanIdx = int(surfCloud->points[i].intensity) % 100;
                        if (thisLastSurfPointScanIdx <= lastSurfPointScanIdx)
                            continue;
                        if (thisLastSurfPointScanIdx > lastSurfPointScanIdx + NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSurfPointCloud->points[j];
                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);
                        if (dist < minDist2)
                        {
                            minDist2 = dist;
                            minDistPointIdx2 = j;
                        }
                    }
                    for (int j = closestPointIdx - 1; j >= 0; --j)
                    {
                        int thisLastSurfPointScanIdx = int(surfCloud->points[i].intensity) % 100;
                        if (thisLastSurfPointScanIdx >= lastSurfPointScanIdx)
                            continue;
                        if (thisLastSurfPointScanIdx < lastSurfPointScanIdx + NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSurfPointCloud->points[j];
                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);
                        if (dist < minDist3)
                        {
                            minDist3 = dist;
                            minDistPointIdx3 = j;
                        }
                    }

                    if (minDistPointIdx2 >= 0 && minDistPointIdx3 >= 0)
                    {
                        Eigen::Vector3d currP{surfCloud->points[i].x, surfCloud->points[i].y,
                                              surfCloud->points[i].z};
                        Eigen::Vector3d closestP{lastSurfPointCloud->points[closestPointIdx].x,
                                                 lastSurfPointCloud->points[closestPointIdx].y,
                                                 lastSurfPointCloud->points[closestPointIdx].z};
                        Eigen::Vector3d closeP2{lastSurfPointCloud->points[minDistPointIdx2].x,
                                                lastSurfPointCloud->points[minDistPointIdx2].y,
                                                lastSurfPointCloud->points[minDistPointIdx2].z};
                        Eigen::Vector3d closeP3{lastSurfPointCloud->points[minDistPointIdx3].x,
                                                lastSurfPointCloud->points[minDistPointIdx3].y,
                                                lastSurfPointCloud->points[minDistPointIdx3].z};

                        double ratio = (int(surfCloud->points[i].intensity) / 100) / N_SCAN;
                        ceres::CostFunction *cost_func = LidarSurfFactor::Create(currP, closestP, closeP2, closeP3, ratio);
                        problem.AddResidualBlock(cost_func, loss_func, param_q, param_t);
                        surfCorrespondence++;
                    }
                }
            }

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 4;
            options.minimizer_progress_to_stdout = false;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
            if (sharpCorrespondence + surfCorrespondence < minCorrespondence)
            {
                ROS_INFO("Too little correspence!********************");
            }
        }
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> iter_time{end - start};
        ROS_INFO("The time of feature matching and iterative optimization :  %ld s", iter_time.count());

        T_world_curr = T_world_curr + Q_world_curr * T_last_curr;
        Q_world_curr = Q_world_curr * Q_last_curr;
    }

    void PublishTopics()
    {
        nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry());
        odom->child_frame_id = "rslidar";
        odom->header.frame_id = "odom";
        odom->header.stamp = ros::Time().fromSec(timeLessSurfCloud);
        odom->pose.pose.position.x = T_world_curr.x();
        odom->pose.pose.position.y = T_world_curr.y();
        odom->pose.pose.position.z = T_world_curr.z();
        odom->pose.pose.orientation.w = Q_world_curr.w();
        odom->pose.pose.orientation.x = Q_world_curr.x();
        odom->pose.pose.orientation.y = Q_world_curr.y();
        odom->pose.pose.orientation.z = Q_world_curr.z();
        odometryPub.publish(odom);
    }

    void Transform2Start(PointType &inPoint, PointType &outPoint)
    {
        int column_index = int(inPoint.intensity / 100);
        double ratio = column_index / N_SCAN;
        Eigen::Quaterniond Q_last_point = Eigen::Quaterniond::Identity().slerp(ratio, Q_last_curr);

        Eigen::Vector3d tmp_point(inPoint.x, inPoint.y, inPoint.z);

        tmp_point = Q_last_point * tmp_point + T_last_curr;
        outPoint.x = tmp_point.x();
        outPoint.y = tmp_point.y();
        outPoint.z = tmp_point.z();
        outPoint.intensity = inPoint.intensity;
    }

    void InitialParams()
    {
        init = false;
        inh.param<int>("N_SCAN", N_SCAN, 1800);
        inh.param<double>("NearestThreshold", NearestThreshold, 25);
        inh.param<int>("NEAR_SCAN_NUM", NEAR_SCAN_NUM, 3);
        inh.param<int>("ITER_NUM", ITER_NUM, 3);
        inh.param<int>("minCorrespondence", minCorrespondence, 20);

        fullCloudBuf = std::queue<sensor_msgs::PointCloud2::Ptr>();
        sharpCloudBuf = std::queue<sensor_msgs::PointCloud2::Ptr>();
        lessSharpCloudBuf = std::queue<sensor_msgs::PointCloud2::Ptr>();
        surfCloudBuf = std::queue<sensor_msgs::PointCloud2::Ptr>();
        lessSurfCloudBuf = std::queue<sensor_msgs::PointCloud2::Ptr>();

        param_q[0] = 0;
        param_q[1] = 0;
        param_q[2] = 0;
        param_q[3] = 1;
        param_t[0] = 0;
        param_t[1] = 0;
        param_t[2] = 0;

        Q_world_curr = {1, 0, 0, 0};
        T_world_curr = {0, 0, 0};
    }
    void FreeMemory()
    {
        lastSharpCloudTree->~KdTreeFLANN();
        lastSurfCloudTree->~KdTreeFLANN();
        // *lastSharpCloudTree = pcl::KdTreeFLANN<PointType>();
        // *lastSurfCloudTree = pcl::KdTreeFLANN<PointType>();
    }

    ~iLaserOdometry() {}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iLaserOdometry");
    ros::NodeHandle nh;
    iLaserOdometry iLO(nh);
    return 0;
}