/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-24 20:25:44
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-04-08 22:34:35
 */

#pragma once
#pragma GCC optimize(3)
#include "../include/utility.h"
#include <queue>
#include <mutex>
#include <thread>
#include <pcl/kdtree/kdtree_flann.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
    ros::Publisher odomPathPub;

    std::queue<sensor_msgs::PointCloud2::ConstPtr> fullCloudBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> sharpCloudBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> lessSharpCloudBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> surfCloudBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> lessSurfCloudBuf;
    std::mutex bufMutex;

    PointCloud::Ptr fullCloud;
    PointCloud::Ptr sharpCloud;
    PointCloud::Ptr lessSharpCloud;
    PointCloud::Ptr surfCloud;
    PointCloud::Ptr lessSurfCloud;
    // PointCloudRL::Ptr sharpCloudRL;
    // PointCloudRL::Ptr surfCloudRL;

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
    // PointCloud::Ptr lastLessSharpPointCloud;
    // PointCloud::Ptr lastLessSurfPointCloud;

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
    int frame_count;

public:
    iLaserOdometry(ros::NodeHandle &nh) : inh(nh)
    {

        InitialParams();
        fullCloudSub = inh.subscribe("/sorted_cloud", 100, &iLaserOdometry::FullCloudHandler, this);
        sharpCloudSub = inh.subscribe("/sharp_cloud", 100, &iLaserOdometry::SharpCloudHandler, this);
        lessSharpCloudSub = inh.subscribe("/less_sharp_cloud", 100, &iLaserOdometry::LessSharpCloudHandler, this);
        surfCloudSub = inh.subscribe("/surf_cloud", 100, &iLaserOdometry::SurfCloudHandler, this);
        lessSurfCloudSub = inh.subscribe("/less_surf_cloud", 100, &iLaserOdometry::LessSurfCloudHandler, this);
        odometryPub = inh.advertise<nav_msgs::Odometry>("/odom", 100);
        odomPathPub = inh.advertise<nav_msgs::Path>("/odom_path", 100);
        ros::Rate rate(100);

        while (ros::ok())
        {
            ros::spinOnce();
            if (!IsEmpty())
            {
                if (!AlignTimeStamp())
                {
                    printf("Please ensure align the timestamp!\n");
                    ROS_BREAK();
                }
                frame_count++;
                printf("\nThis is the %dst frame!\n", frame_count);
                auto start = std::chrono::system_clock::now();
                Process();
                auto end = std::chrono::system_clock::now();
                auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                printf("Total time of laser odometry: %ld ms\n", total_time.count());
            }

            rate.sleep();
        }
    }

    void FullCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &fullCloud)
    {
        bufMutex.lock();
        fullCloudBuf.push(fullCloud);
        bufMutex.unlock();
    }
    void SharpCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &sharpCloud)
    {

        bufMutex.lock();
        sharpCloudBuf.push(sharpCloud);
        bufMutex.unlock();
    }
    void LessSharpCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &lessSharpCloud)
    {

        bufMutex.lock();
        lessSharpCloudBuf.push(lessSharpCloud);
        bufMutex.unlock();
    }

    void SurfCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &surfCloud)
    {

        bufMutex.lock();
        surfCloudBuf.push(surfCloud);
        bufMutex.unlock();
    }

    void LessSurfCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &lessSurfCloud)
    {

        bufMutex.lock();
        lessSurfCloudBuf.push(lessSurfCloud);
        bufMutex.unlock();
    }

    bool IsEmpty()
    {

        if (fullCloudBuf.empty() || sharpCloudBuf.empty() || surfCloudBuf.empty() || lessSharpCloudBuf.empty() || lessSurfCloudBuf.empty())
        {
            return true;
        }
        return false;
    }

    bool AlignTimeStamp()
    {
        timeSharpCloud = sharpCloudBuf.front()->header.stamp.toSec();

        timeSurfCloud = surfCloudBuf.front()->header.stamp.toSec();

        timeFullCloud = fullCloudBuf.front()->header.stamp.toSec();

        timeLessSharpCloud = lessSharpCloudBuf.front()->header.stamp.toSec();

        timeLessSurfCloud = lessSurfCloudBuf.front()->header.stamp.toSec();

        if ((timeSharpCloud != timeFullCloud) || (timeSurfCloud != timeFullCloud) || (timeLessSharpCloud != timeFullCloud) || (timeLessSurfCloud != timeFullCloud))
        {

            return false;
        }

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
            printf("Initialed!\n");
            init = true;
        }
        else
        {
            SetOdometry();
        }
        PublishTopics();

        // lastLessSharpPointCloud = lessSharpCloud;
        // lastLessSurfPointCloud = lessSurfCloud;
        // printf("lessSharpCloud->size(): %d \n", lessSharpCloud->size());
        // *lastSharpPointCloud = *lessSharpCloud;
        // *lastSurfPointCloud = *lessSurfCloud;

        PointCloud::Ptr tmp_cloud = lessSharpCloud;
        lessSharpCloud = lastSharpPointCloud;
        lastSharpPointCloud = tmp_cloud;

        tmp_cloud = lessSurfCloud;
        lessSurfCloud = lastSurfPointCloud;
        lastSurfPointCloud = tmp_cloud;

        // printf("lastSharpPointCloud->size(): %d \n", lastSharpPointCloud->size());
        lastSharpCloudTree->setInputCloud(lastSharpPointCloud);
        lastSurfCloudTree->setInputCloud(lastSurfPointCloud);
    }

    void SetOdometry()
    {
        // printf("SetOdometry\n");
        //  RemoveNaNPoints(sharpCloud);
        //  RemoveNaNPoints(surfCloud);

        int lastSharpCloudNum = lastSharpPointCloud->size();
        int lastSurfCloudNum = lastSurfPointCloud->size();

        // printf("lastSharpCloudNum: %d \n", lastSharpCloudNum);

        int sharpCloudNum = sharpCloud->size();
        int surfCloudNum = surfCloud->size();

        auto start = std::chrono::system_clock::now();

        for (int iter = 0; iter < ITER_NUM; ++iter)
        {
            ceres::LossFunction *loss_func = new ceres::HuberLoss(0.1);
            ceres::LocalParameterization *q_parameterization =
                new ceres::EigenQuaternionParameterization();
            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            problem.AddParameterBlock(param_q, 4, q_parameterization);
            problem.AddParameterBlock(param_t, 3);

            int sharpCorrespondence = 0;
            int surfCorrespondence = 0;
            std::vector<int> sharpPointsIndices;
            std::vector<float> sharpPointsDist;
            std::vector<int> surfPointsIndices;
            std::vector<float> surfPointsDist;
            auto stop1 = std::chrono::system_clock::now();

            for (int i = 0; i < sharpCloudNum; ++i)
            {

                PointType currSharpPoint = sharpCloud->points[i];
                PointType transedSharpPoint;
                // printf("1\n");
                Transform2Start(currSharpPoint, transedSharpPoint);
                // printf("2\n");
                lastSharpCloudTree->nearestKSearch(transedSharpPoint, 1, sharpPointsIndices, sharpPointsDist);
                // printf("3\n");
                if (sharpPointsDist[0] < NearestThreshold)
                {
                    int closestPointIdx = sharpPointsIndices[0];
                    int lastSharpPointScanIdx = int(lastSharpPointCloud->points[closestPointIdx].intensity) % 100;
                    PointType closestPoint = lastSharpPointCloud->points[closestPointIdx];
                    double minDist2 = std::numeric_limits<double>::max();

                    int minDistPointIdx2;
                    // printf("4\n");
                    for (int j = closestPointIdx + 1; j < lastSharpCloudNum; j++)
                    {
                        int thisLastSharpPointScanIdx = int(lastSharpPointCloud->points[j].intensity) % 100;
                        // printf("5\n");
                        if (thisLastSharpPointScanIdx <= lastSharpPointScanIdx)
                            continue;
                        if (thisLastSharpPointScanIdx > lastSharpPointScanIdx + NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSharpPointCloud->points[j];
                        // printf("6\n");
                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);
                        if (dist < minDist2)
                        {
                            minDist2 = dist;
                            minDistPointIdx2 = j;
                        }
                    }
                    // printf("7\n");
                    for (int j = closestPointIdx - 1; j >= 0; j--)
                    {
                        int thisLastSharpPointScanIdx = int(lastSharpPointCloud->points[j].intensity) % 100;
                        if (thisLastSharpPointScanIdx >= lastSharpPointScanIdx)
                            continue;
                        if (thisLastSharpPointScanIdx < lastSharpPointScanIdx - NEAR_SCAN_NUM)
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
                    // printf("8\n");
                    if (minDistPointIdx2 >= 0 && minDistPointIdx2 < lastSharpCloudNum)
                    {
                        // printf("9\n");
                        Eigen::Vector3d currP{sharpCloud->points[i].x, sharpCloud->points[i].y,
                                              sharpCloud->points[i].z};

                        // printf("10\n");
                        //  printf("closestPointIdx: %d\n", closestPointIdx);
                        //  printf("minDistPointIdx2: %d\n", minDistPointIdx2);
                        //  printf("lastSharpCloudNum: %d\n", lastSharpCloudNum);
                        Eigen::Vector3d closestP{lastSharpPointCloud->points[closestPointIdx].x,
                                                 lastSharpPointCloud->points[closestPointIdx].y,
                                                 lastSharpPointCloud->points[closestPointIdx].z};
                        Eigen::Vector3d closestP2{lastSharpPointCloud->points[minDistPointIdx2].x,
                                                  lastSharpPointCloud->points[minDistPointIdx2].y,
                                                  lastSharpPointCloud->points[minDistPointIdx2].z};

                        // printf("currP: %f %f %f \n", currP.x(), currP.y(), currP.z());
                        // printf("closestP: %f %f %f \n", closestP.x(), closestP.y(), closestP.z());
                        // printf("closestP2: %f %f %f \n", closestP2.x(), closestP2.y(), closestP2.z());

                        // printf("sharpCloud->points[i].intensity: %f \n", sharpCloud->points[i].intensity);
                        double ratio = (int(sharpCloud->points[i].intensity) / 100.0) / N_SCAN;

                        ceres::CostFunction *cost_func = LidarEdegFactor::Create(currP, closestP, closestP2, ratio);
                        problem.AddResidualBlock(cost_func, loss_func, param_q, param_t);
                        sharpCorrespondence++;
                    }
                }
            }
            printf("sharpCorrespondence: %d\n", sharpCorrespondence);
            auto stop2 = std::chrono::system_clock::now();
            auto time12 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - stop1);
            // printf("Time of sharp points opt: %ld ms\n", time12.count());
            for (int i = 0; i < surfCloudNum; ++i)
            {
                // printf("11\n");
                PointType currSurfPoint = surfCloud->points[i];
                PointType transedSurfPoint;
                Transform2Start(currSurfPoint, transedSurfPoint);
                lastSurfCloudTree->nearestKSearch(transedSurfPoint, 1, surfPointsIndices, surfPointsDist);
                // printf("12\n");
                if (surfPointsDist[0] < NearestThreshold)
                {
                    int closestPointIdx = surfPointsIndices[0];
                    int lastSurfPointScanIdx = int(lastSurfPointCloud->points[closestPointIdx].intensity) % 100;
                    PointType closestPoint = lastSurfPointCloud->points[closestPointIdx];
                    double minDist2 = std::numeric_limits<double>::max();
                    double minDist3 = std::numeric_limits<double>::max();
                    int minDistPointIdx2, minDistPointIdx3;
                    // printf("13\n");
                    for (int j = closestPointIdx + 1; j < lastSurfCloudNum; ++j)
                    {
                        int thisLastSurfPointScanIdx = int(lastSurfPointCloud->points[j].intensity) % 100;
                        // if (thisLastSurfPointScanIdx <= lastSurfPointScanIdx)
                        //     continue;
                        if (thisLastSurfPointScanIdx > lastSurfPointScanIdx + NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSurfPointCloud->points[j];
                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);
                        if (dist < minDist2 && thisLastSurfPointScanIdx >= lastSurfPointScanIdx)
                        {
                            minDist2 = dist;
                            minDistPointIdx2 = j;
                        }
                        if (dist < minDist3 && thisLastSurfPointScanIdx < lastSurfPointScanIdx)
                        {
                            minDist3 = dist;
                            minDistPointIdx3 = j;
                        }
                    }
                    // printf("14\n");
                    for (int j = closestPointIdx - 1; j >= 0; --j)
                    {
                        int thisLastSurfPointScanIdx = int(lastSurfPointCloud->points[j].intensity) % 100;
                        // if (thisLastSurfPointScanIdx >= lastSurfPointScanIdx)
                        //     continue;
                        if (thisLastSurfPointScanIdx < lastSurfPointScanIdx - NEAR_SCAN_NUM)
                            break;
                        PointType thisPoint = lastSurfPointCloud->points[j];
                        double dist = (closestPoint.x - thisPoint.x) * (closestPoint.x - thisPoint.x) +
                                      (closestPoint.y - thisPoint.y) * (closestPoint.y - thisPoint.y) +
                                      (closestPoint.z - thisPoint.z) * (closestPoint.z - thisPoint.z);

                        if (dist < minDist2 && thisLastSurfPointScanIdx >= lastSurfPointScanIdx)
                        {
                            minDist2 = dist;
                            minDistPointIdx2 = j;
                        }
                        if (dist < minDist3 && thisLastSurfPointScanIdx < lastSurfPointScanIdx)
                        {
                            minDist3 = dist;
                            minDistPointIdx3 = j;
                        }
                    }

                    if (minDistPointIdx2 >= 0 && minDistPointIdx3 >= 0 && minDistPointIdx2 < lastSurfCloudNum && minDistPointIdx3 < lastSurfCloudNum)
                    {
                        // printf("closestPointIdx: %d \n", closestPointIdx);
                        // printf("minDistPointIdx2: %d \n", minDistPointIdx2);
                        // printf("minDistPointIdx3: %d \n", minDistPointIdx3);
                        // printf("lastSurfCloudNum: %d \n", lastSurfCloudNum);

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

                        double ratio = (int(surfCloud->points[i].intensity) / 100.0) / N_SCAN;
                        // printf("ratio: %f \n", ratio);
                        ceres::CostFunction *cost_func = LidarSurfFactor::Create(currP, closestP, closeP2, closeP3, ratio);
                        problem.AddResidualBlock(cost_func, loss_func, param_q, param_t);
                        surfCorrespondence++;
                    }
                }
            }
            // printf("before solving, param_t: %lf %lf %lf \n", param_t[0], param_t[1], param_t[2]);

            ceres::Solver::Options options;
            options.linear_solver_type = ceres::DENSE_QR;
            options.max_num_iterations = 10;
            options.minimizer_progress_to_stdout = true;
            ceres::Solver::Summary summary;
            auto stop3 = std::chrono::system_clock::now();
            ceres::Solve(options, &problem, &summary);
            auto stop4 = std::chrono::system_clock::now();
            auto time34 = std::chrono::duration_cast<std::chrono::milliseconds>(stop4 - stop3);
            printf("Time of solving : %ld ms\n", time34.count());
            // printf("param_t: %lf %lf %lf \n", param_t[0], param_t[1], param_t[2]);
            if (sharpCorrespondence + surfCorrespondence < minCorrespondence)
            {
                printf("Too little correspence!********************\n");
            }
        }
        auto end = std::chrono::system_clock::now();
        auto iter_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        printf("The time of feature matching and iterative optimization : %ld milliseconds \n", iter_time.count());

        T_world_curr = T_world_curr + Q_world_curr * T_last_curr;
        Q_world_curr = Q_world_curr * Q_last_curr;
    }

    void PublishTopics()
    {

        nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry());
        odom->child_frame_id = "rslidar";
        odom->header.frame_id = "odom";
        odom->header.stamp = ros::Time().fromSec(timeFullCloud);
        odom->pose.pose.position.x = T_world_curr.x();
        odom->pose.pose.position.y = T_world_curr.y();
        odom->pose.pose.position.z = T_world_curr.z();
        odom->pose.pose.orientation.w = Q_world_curr.w();
        odom->pose.pose.orientation.x = Q_world_curr.x();
        odom->pose.pose.orientation.y = Q_world_curr.y();
        odom->pose.pose.orientation.z = Q_world_curr.z();
        odometryPub.publish(odom);
        // printf("T_last_curr: %lf %lf %lf \n", T_last_curr.x(), T_last_curr.y(), T_last_curr.z());
        printf("Orientation: %lf  %lf  %lf  %lf \n", Q_world_curr.w(), Q_world_curr.x(), Q_world_curr.y(), Q_world_curr.z());
        printf("Translation: %lf  %lf  %lf\n", T_world_curr.x(), T_world_curr.y(), T_world_curr.z());

        geometry_msgs::PoseStamped odomPose;
        odomPose.header.frame_id = "odom";
        odomPose.pose = odom->pose.pose;
        odomPose.header.stamp = ros::Time().fromSec(timeFullCloud);
        nav_msgs::Path odomPath;
        odomPath.header = odomPose.header;
        odomPath.header.frame_id = "odom";
        odomPath.poses.push_back(odomPose);
        odomPathPub.publish(odomPath);
    }

    void Transform2Start(const PointType &inPoint, PointType &outPoint)
    {

        double ratio = (int(inPoint.intensity) / 100.0) / N_SCAN;

        Eigen::Quaterniond Q_last_point = Eigen::Quaterniond::Identity().slerp(ratio, Q_last_curr);

        Eigen::Vector3d tmp_point(inPoint.x, inPoint.y, inPoint.z);

        tmp_point = Q_last_point * tmp_point + ratio * T_last_curr;
        outPoint.intensity = inPoint.intensity;
        outPoint.x = tmp_point.x();
        outPoint.y = tmp_point.y();
        outPoint.z = tmp_point.z();
    }

    void InitialParams()
    {

        fullCloud.reset(new PointCloud());
        sharpCloud.reset(new PointCloud());
        surfCloud.reset(new PointCloud());
        lessSharpCloud.reset(new PointCloud());
        lessSurfCloud.reset(new PointCloud());

        lastSharpPointCloud.reset(new PointCloud());
        lastSurfPointCloud.reset(new PointCloud());
        // lastLessSharpPointCloud.reset(new PointCloud());
        // lastLessSurfPointCloud.reset(new PointCloud());

        lastSharpCloudTree.reset(new pcl::KdTreeFLANN<PointType>());
        lastSurfCloudTree.reset(new pcl::KdTreeFLANN<PointType>());

        init = false;
        frame_count = 0;
        inh.param<int>("N_SCAN", N_SCAN, 1800);
        inh.param<double>("NearestThreshold", NearestThreshold, 25);
        inh.param<int>("NEAR_SCAN_NUM", NEAR_SCAN_NUM, 2.5);
        inh.param<int>("ITER_NUM", ITER_NUM, 2);
        inh.param<int>("minCorrespondence", minCorrespondence, 20);

        param_q[0] = 0.0;
        param_q[1] = 0.0;
        param_q[2] = 0.0;
        param_q[3] = 1.0;

        param_t[0] = 0.0;
        param_t[1] = 0.0;
        param_t[2] = 0.0;

        Q_world_curr = {1.0, 0.0, 0.0, 0.0};
        T_world_curr = {0.0, 0.0, 0.0};
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