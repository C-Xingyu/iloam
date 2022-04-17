/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-04-06 20:54:49
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-04-17 20:31:12
 */
#include "../include/utility.h"
#include <nav_msgs/Odometry.h>
#include <queue>
#include <mutex>
#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "../include/utility.h"
#include "LiDARfactor.hpp"
class iLaserMapping : public BaseModule
{
private:
    ros::NodeHandle inh;
    std::mutex bufMutex;

    ros::Subscriber odomSub;
    ros::Subscriber sharpCloudSub;
    ros::Subscriber surfCloudSub;
    ros::Subscriber fullCloudSub;

    std::queue<nav_msgs::Odometry::ConstPtr> odomBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> sharpCloudBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> surfCloudBuf;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> fullCloudBuf;

    PointCloud::Ptr sharpCloud;
    PointCloud::Ptr surfCloud;
    PointCloud::Ptr fullCloud;

    Eigen::Quaterniond Q_odom_curr;
    Eigen::Vector3d T_odom_curr;

    // double param_q[4];
    // double param_t[3];

    double param[7];

    Eigen::Map<Eigen::Quaterniond> Q_map_curr = Eigen::Map<Eigen::Quaterniond>(param);
    Eigen::Map<Eigen::Vector3d> T_map_curr = Eigen::Map<Eigen::Vector3d>(param + 4);

    Eigen::Quaterniond Q_map_odom;
    Eigen::Vector3d T_map_odom;

    int offsetI;
    int offsetJ;
    int offsetK;

    static const int mapCubeLength = 21;
    static const int mapCubeWidth = 21;
    static const int mapCubeHeight = 10;

    static const int mapCubeNum = mapCubeLength * mapCubeWidth * mapCubeHeight;

    PointCloud::Ptr sharpCloudMapCube[mapCubeNum];
    PointCloud::Ptr surfCloudMapCube[mapCubeNum];

    static const int subMapCubeNum = 125;
    int subSharpMapIdx[subMapCubeNum];
    int subSurfMapIdx[subMapCubeNum];

    PointCloud::Ptr subSharpMapCloud;
    PointCloud::Ptr subSurfMapCloud;

    const int optIter = 2;

    const double leafSize = 0.2;

    const double sharpDistThreshold = 1.0;
    const double surfDistThreshold = 1.0;

    pcl::VoxelGrid<PointType> downFilter;

    pcl::KdTreeFLANN<PointType>::Ptr sharpMapTree;
    pcl::KdTreeFLANN<PointType>::Ptr surfMapTree;

    int sharpCloudNum;
    int surfCloudNum;
    PointCloud::Ptr filteredSharpCloud;
    PointCloud::Ptr filteredSurfCloud;

public:
    iLaserMapping(ros::NodeHandle &nh) : inh(nh)
    {
        InitialParams();
        odomSub = nh.subscribe("/odom", 100, &iLaserMapping::odomHandler, this);
        sharpCloudSub = nh.subscribe("/sharp_cloud", 100, &iLaserMapping::sharpCloudHandler, this);
        surfCloudSub = nh.subscribe("/surf_cloud", 100, &iLaserMapping::surfCloudHandler, this);
        fullCloudSub = nh.subscribe("/sorted_cloud", 100, &iLaserMapping::fullCloudHandler, this);

        std::thread process_thread{&iLaserMapping::Process, this};
        ros::Rate rate(100);
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    void InitialParams()
    {
        //
        param[0] = 1.0;
        param[1] = 0.0;
        param[2] = 0.0;
        param[3] = 0.0;

        param[4] = 0.0;
        param[5] = 0.0;
        param[6] = 0.0;

        Q_odom_curr = {1.0, 0.0, 0.0, 0.0};
        T_odom_curr = {0.0, 0.0, 0.0};

        Q_map_odom = {1.0, 0.0, 0.0, 0.0};
        T_map_odom = {0.0, 0.0, 0.0};

        offsetI = 10;
        offsetJ = 10;
        offsetK = 5;

        for (int i = 0; i < mapCubeNum; ++i)
        {
            sharpCloudMapCube[i].reset(new PointCloud());
            surfCloudMapCube[i].reset(new PointCloud());
        }
        subSharpMapCloud.reset(new PointCloud());
        subSurfMapCloud.reset(new PointCloud());

        downFilter.setLeafSize(leafSize, leafSize, leafSize);

        sharpMapTree.reset(new pcl::KdTreeFLANN<PointType>());
        surfMapTree.reset(new pcl::KdTreeFLANN<PointType>());

        sharpCloud.reset(new PointCloud);
        surfCloud.reset(new PointCloud);
        fullCloud.reset(new PointCloud());

        sharpCloudNum = 0;
        surfCloudNum = 0;
        filteredSharpCloud.reset(new PointCloud());
        filteredSurfCloud.reset(new PointCloud());
    }

    void Process()
    {
        while (1)
        {
            while (!odomBuf.empty() && !sharpCloudBuf.empty() &&
                   !surfCloudBuf.empty() && !fullCloudBuf.empty())
            {
                bufMutex.lock();
                while (!odomBuf.empty() && odomBuf.front()->header.stamp.toSec() <
                                               sharpCloudBuf.front()->header.stamp.toSec())
                    odomBuf.pop();
                if (odomBuf.empty())
                {
                    bufMutex.unlock();
                    break;
                }

                while (!surfCloudBuf.empty() && surfCloudBuf.front()->header.stamp.toSec() <
                                                    sharpCloudBuf.front()->header.stamp.toSec())

                    surfCloudBuf.pop();
                if (surfCloudBuf.empty())
                {
                    bufMutex.unlock();
                    break;
                }

                while (!fullCloudBuf.empty() && fullCloudBuf.front()->header.stamp.toSec() <
                                                    sharpCloudBuf.front()->header.stamp.toSec())
                    fullCloudBuf.pop();
                if (fullCloudBuf.empty())
                {
                    bufMutex.unlock();
                    break;
                }

                sharpCloud = Concert2PCLCloud(sharpCloudBuf.front());
                sharpCloudBuf.pop();

                surfCloud = Concert2PCLCloud(surfCloudBuf.front());
                surfCloudBuf.pop();

                fullCloud = Concert2PCLCloud(fullCloudBuf.front());
                fullCloudBuf.pop();

                Q_odom_curr.w() = odomBuf.front()->pose.pose.orientation.w;
                Q_odom_curr.x() = odomBuf.front()->pose.pose.orientation.x;
                Q_odom_curr.y() = odomBuf.front()->pose.pose.orientation.y;
                Q_odom_curr.z() = odomBuf.front()->pose.pose.orientation.z;

                T_odom_curr.x() = odomBuf.front()->pose.pose.position.x;
                T_odom_curr.y() = odomBuf.front()->pose.pose.position.y;
                T_odom_curr.z() = odomBuf.front()->pose.pose.position.z;

                odomBuf.pop();
                while (!odomBuf.empty())
                    odomBuf.pop();
                bufMutex.unlock();

                Transform2Map();

                BuildSubMap();

                OptmizePose();

                TransformUpdate();

                UpdateMap();
            }
        }
    }

    void TransformUpdate()
    {
        Q_map_odom = Q_map_curr * Q_odom_curr.inverse();
        T_map_odom = T_map_curr - Q_map_odom * T_odom_curr;
    }

    void BuildSubMap()
    {
        int selfI = int((T_map_curr.x() + 25.0) / 50.0) + offsetI;
        int selfJ = int((T_map_curr.y() + 25.0) / 50.0) + offsetJ;
        int selfK = int((T_map_curr.z() + 25.0) / 50.0) + offsetK;

        if (T_map_curr.x() + 25.0 < 0.0)
            selfI--;
        if (T_map_curr.y() + 25.0 < 0.0)
            selfJ--;
        if (T_map_curr.z() + 25.0 < 0.0)
            selfK--;

        while (selfI < 3)
        {
            for (int k = 0; k < mapCubeHeight; ++k)
            {
                for (int j = 0; j < mapCubeWidth; ++j)
                {

                    int cubeIdxI = mapCubeLength - 1;
                    for (int i = cubeIdxI; i >= 1; --i)
                    {
                        sharpCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            sharpCloudMapCube[i - 1 + j * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                        surfCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            surfCloudMapCube[i - 1 + j * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                    }
                    sharpCloudMapCube[cubeIdxI + j * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                    surfCloudMapCube[cubeIdxI + j * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                }
            }
            selfI++;
            offsetI++;
        }

        while (selfI >= mapCubeLength - 3)
        {
            for (int k = 0; k < mapCubeHeight; ++k)
            {
                for (int j = 0; j < mapCubeWidth; ++j)
                {

                    int cubeIdxI = 0;
                    for (int i = cubeIdxI; i < mapCubeLength - 1; ++i)
                    {
                        sharpCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            sharpCloudMapCube[i + 1 + j * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                        surfCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            surfCloudMapCube[i + 1 + j * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                    }
                    sharpCloudMapCube[cubeIdxI + j * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                    surfCloudMapCube[cubeIdxI + j * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                }
            }
            selfI--;
            offsetI--;
        }

        while (selfJ < 3)
        {
            for (int k = 0; k < mapCubeHeight; ++k)
            {
                for (int i = 0; i < mapCubeLength; ++i)
                {

                    int cubeIdxJ = mapCubeWidth - 1;
                    for (int j = cubeIdxJ; j >= 1; --j)
                    {
                        sharpCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            sharpCloudMapCube[i + (j - 1) * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                        surfCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            surfCloudMapCube[i + (j - 1) * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                    }
                    sharpCloudMapCube[i + cubeIdxJ * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                    surfCloudMapCube[i + cubeIdxJ * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                }
            }
            selfJ++;
            offsetJ++;
        }

        while (selfJ >= mapCubeWidth - 3)
        {
            for (int k = 0; k < mapCubeHeight; ++k)
            {
                for (int i = 0; i < mapCubeLength; ++i)
                {

                    int cubeIdxJ = 0;
                    for (int j = cubeIdxJ; j < mapCubeWidth - 1; ++j)
                    {
                        sharpCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            sharpCloudMapCube[i + (j + 1) * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                        surfCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            surfCloudMapCube[i + (j + 1) * mapCubeLength + k * mapCubeLength * mapCubeWidth];
                    }
                    sharpCloudMapCube[i + cubeIdxJ * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                    surfCloudMapCube[i + cubeIdxJ * mapCubeLength + k * mapCubeLength * mapCubeWidth]->clear();
                }
            }
            selfJ--;
            offsetJ--;
        }

        while (selfK < 3)
        {
            for (int j = 0; j < mapCubeWidth; ++j)
            {
                for (int i = 0; i < mapCubeLength; ++i)
                {

                    int cubeIdxK = mapCubeHeight - 1;
                    for (int k = cubeIdxK; k >= 1; --k)
                    {
                        sharpCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            sharpCloudMapCube[i + j * mapCubeLength + (k - 1) * mapCubeLength * mapCubeWidth];
                        surfCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            surfCloudMapCube[i + j * mapCubeLength + (k - 1) * mapCubeLength * mapCubeWidth];
                    }
                    sharpCloudMapCube[i + j * mapCubeLength + cubeIdxK * mapCubeLength * mapCubeWidth]->clear();
                    surfCloudMapCube[i + j * mapCubeLength + cubeIdxK * mapCubeLength * mapCubeWidth]->clear();
                }
            }
            selfK++;
            offsetK++;
        }

        while (selfK >= mapCubeHeight - 3)
        {
            for (int j = 0; j < mapCubeWidth; ++j)
            {
                for (int i = 0; i < mapCubeLength; ++i)
                {

                    int cubeIdxK = 0;
                    for (int k = cubeIdxK; k < mapCubeHeight - 1; ++k)
                    {
                        sharpCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            sharpCloudMapCube[i + j * mapCubeLength + (k + 1) * mapCubeLength * mapCubeWidth];
                        surfCloudMapCube[i + j * mapCubeLength + k * mapCubeLength * mapCubeWidth] =
                            surfCloudMapCube[i + j * mapCubeLength + (k + 1) * mapCubeLength * mapCubeWidth];
                    }
                    sharpCloudMapCube[i + j * mapCubeLength + cubeIdxK * mapCubeLength * mapCubeWidth]->clear();
                    surfCloudMapCube[i + j * mapCubeLength + cubeIdxK * mapCubeLength * mapCubeWidth]->clear();
                }
            }
            selfK--;
            offsetK--;
        }

        int sharpSubMapNum = 0;
        int surfSubMapNum = 0;

        for (int i = selfI - 2; i <= selfI + 2; ++i)
        {
            for (int j = selfJ - 2; j <= selfJ + 2; ++j)
            {
                for (int k = selfK - 2; k <= selfK + 2; ++k)
                {
                    subSharpMapIdx[sharpSubMapNum++] = i + j * mapCubeLength + k * mapCubeLength * mapCubeHeight;
                    subSurfMapIdx[surfSubMapNum++] = i + j * mapCubeLength + k * mapCubeLength * mapCubeHeight;
                }
            }
        }
        subSharpMapCloud->clear();
        subSurfMapCloud->clear();
        for (const auto &ind : subSharpMapIdx)
        {
            *subSharpMapCloud += *(sharpCloudMapCube[ind]);
        }
        for (const auto &ind : subSurfMapIdx)
        {
            *subSurfMapCloud += *(surfCloudMapCube[ind]);
        }
    }

    void OptmizePose()
    {
        // PointCloud::Ptr filteredSharpCloud(new PointCloud());
        // PointCloud::Ptr filteredSurfCloud(new PointCloud());

        downFilter.setInputCloud(sharpCloud);
        downFilter.filter(*filteredSharpCloud);

        downFilter.setInputCloud(surfCloud);
        downFilter.filter(*filteredSurfCloud);

        sharpCloudNum = filteredSharpCloud->size();
        surfCloudNum = filteredSurfCloud->size();

        int sharpSubMapCloudNum = subSharpMapCloud->size();
        int surfSubMapCloudNum = subSurfMapCloud->size();

        if (sharpSubMapCloudNum > 10 && surfSubMapCloudNum > 50)
        {
            sharpMapTree->setInputCloud(subSharpMapCloud);
            surfMapTree->setInputCloud(subSurfMapCloud);

            for (int iter = 0; iter < optIter; iter++)
            {
                ceres::LossFunction *loss_func = new ceres::HuberLoss(0.1);
                ceres::LocalParameterization *q_parameterization =
                    new ceres::EigenQuaternionParameterization();
                ceres::Problem::Options problem_options;
                ceres::Problem problem(problem_options);

                problem.AddParameterBlock(param, 4, q_parameterization);
                //重构参数，优化时实际使用的是3维的等效旋转矢量
                problem.AddParameterBlock(param + 4, 3);

                int sharpCount = 0, surfCount = 0;

                for (int i = 0; i < sharpCloudNum; ++i)
                {
                    PointType sharpPoint = filteredSharpCloud->points[i];
                    PointType mapPoint;
                    Convert2SubMap(sharpPoint, mapPoint);
                    std::vector<int> sharpIndices;
                    std::vector<float> sharpDist;
                    sharpMapTree->nearestKSearch(mapPoint, 5, sharpIndices, sharpDist);

                    if (sharpDist[4] < sharpDistThreshold)
                    {
                        Eigen::Vector3d center{0, 0, 0};
                        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> nearPoints;
                        for (int idx = 0; idx < 5; idx++)
                        {
                            Eigen::Vector3d tmp_point{subSharpMapCloud->points[sharpIndices[idx]].x,
                                                      subSharpMapCloud->points[sharpIndices[idx]].y,
                                                      subSharpMapCloud->points[sharpIndices[idx]].z};
                            nearPoints.push_back(tmp_point);
                            center += tmp_point;
                        }
                        center /= 5.0;
                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                        for (int idx = 0; idx < 5; idx++)
                        {
                            Eigen::Vector3d tmp_point = nearPoints[idx] - center;
                            covMat += (tmp_point - center) * (tmp_point - center).transpose();
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
                        Eigen::Vector3d unitVec = saes.eigenvectors().col(2);

                        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
                        {
                            Eigen::Vector3d nowPoint{sharpPoint.x, sharpPoint.y, sharpPoint.z};
                            Eigen::Vector3d aPoint = center + 0.1 * unitVec;
                            Eigen::Vector3d bPoint = center - 0.1 * unitVec;
                            ceres::CostFunction *cost_func = LidarEdegFactor::Create(nowPoint, aPoint, bPoint, 1.0);
                            problem.AddResidualBlock(cost_func, loss_func, param, param + 4);
                            sharpCount++;
                        }
                    }
                }

                for (int i = 0; i < surfCloudNum; ++i)
                {
                    PointType surfPoint = filteredSurfCloud->points[i];
                    PointType mapPoint;
                    Convert2SubMap(surfPoint, mapPoint);
                    std::vector<int> surfIndices;
                    std::vector<float> surfDist;
                    surfMapTree->nearestKSearch(mapPoint, 5, surfIndices, surfDist);

                    Eigen::Matrix<double, 5, 3> pointsMat;
                    for (int j = 0; j < 5; ++j)
                    {
                        pointsMat(j, 0) = subSurfMapCloud->points[surfIndices[j]].x;
                        pointsMat(j, 1) = subSurfMapCloud->points[surfIndices[j]].y;
                        pointsMat(j, 2) = subSurfMapCloud->points[surfIndices[j]].z;
                    }

                    Eigen::Matrix<double, 5, 1> constMat = -1.0 * Eigen::Matrix<double, 5, 1>::Ones();

                    Eigen::Vector3d norm = pointsMat.colPivHouseholderQr().solve(constMat);
                    //求解 AX+BY+CZ+1=0
                    double constCoeff = 1.0 / norm.norm();
                    norm.normalize();

                    bool isValid = true;

                    for (int j = 0; j < 5; ++j)
                    {
                        //计算点到平面的距离
                        double dist = fabs(pointsMat(j, 0) * norm[0] + pointsMat(j, 1) * norm[1] +
                                           pointsMat(j, 2) * norm[2] + constCoeff);
                        if (dist > 0.2)
                        {
                            isValid = false;
                            break;
                        }
                    }

                    if (isValid)
                    {
                        Eigen::Vector3d currPoint{surfPoint.x, surfPoint.y, surfPoint.z};
                        ceres::CostFunction *cost_func = LidarNormFactor::Create(currPoint, norm, constCoeff);
                        problem.AddResidualBlock(cost_func, loss_func, param, param + 4);
                        surfCount++;
                    }
                }
                ceres::Solver::Options options;
                options.linear_solver_type = ceres::DENSE_QR;
                options.max_num_iterations = 4;
                options.minimizer_progress_to_stdout = false;
                options.check_gradients = false;
                options.gradient_check_relative_precision = 1e-4;
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);
            }
        }
        else
        {
            printf("The number of associated points is so little!\n");
        }
    }

    void UpdateMap()
    {
        for (int i = 0; i < sharpCloudNum; ++i)
        {
            PointType transedPoint;
            Convert2SubMap(filteredSharpCloud->points[i], transedPoint);
            int cubeIdxI = int((transedPoint.x + 25.0) / 50.0) + offsetI;
            int cubeIdxJ = int((transedPoint.y + 25.0) / 50.0) + offsetJ;
            int cubeIdxK = int((transedPoint.z + 25.0) / 50.0) + offsetK;

            if (transedPoint.x + 25.0 < 0.0)
                cubeIdxI--;
            if (transedPoint.y + 25.0 < 0.0)
                cubeIdxJ--;
            if (transedPoint.z + 25.0 < 0.0)
                cubeIdxK--;

            if (cubeIdxI >= 0 && cubeIdxI < mapCubeLength &&
                cubeIdxJ >= 0 && cubeIdxJ < mapCubeWidth &&
                cubeIdxK >= 0 && cubeIdxK < mapCubeHeight)
            {
                int cubeIdx = cubeIdxI + cubeIdxJ * mapCubeLength + cubeIdxK * mapCubeLength * mapCubeWidth;
                sharpCloudMapCube[cubeIdx]->push_back(transedPoint);
            }
        }
        for (int i = 0; i < surfCloudNum; ++i)
        {
            PointType transedPoint;
            Convert2SubMap(filteredSurfCloud->points[i], transedPoint);
            int cubeIdxI = int((transedPoint.x + 25.0) / 50.0) + offsetI;
            int cubeIdxJ = int((transedPoint.y + 25.0) / 50.0) + offsetJ;
            int cubeIdxK = int((transedPoint.z + 25.0) / 50.0) + offsetK;

            if (transedPoint.x + 25.0 < 0.0)
                cubeIdxI--;
            if (transedPoint.y + 25.0 < 0.0)
                cubeIdxJ--;
            if (transedPoint.z + 25.0 < 0.0)
                cubeIdxK--;

            if (cubeIdxI >= 0 && cubeIdxI < mapCubeLength &&
                cubeIdxJ >= 0 && cubeIdxJ < mapCubeWidth &&
                cubeIdxK >= 0 && cubeIdxK < mapCubeHeight)
            {
                int cubeIdx = cubeIdxI + cubeIdxJ * mapCubeLength + cubeIdxK * mapCubeLength * mapCubeWidth;
                surfCloudMapCube[cubeIdx]->push_back(transedPoint);
            }
        }

        for (int i = 0; i < subMapCubeNum; ++i)
        {

            PointCloud::Ptr tmpSharpCloud(new PointCloud());
            PointCloud::Ptr tmpSurfCloud(new PointCloud());
            downFilter.setInputCloud(sharpCloudMapCube[subSharpMapIdx[i]]);
            downFilter.filter(*tmpSharpCloud);
            sharpCloudMapCube[subSharpMapIdx[i]] = tmpSharpCloud;

            downFilter.setInputCloud(surfCloudMapCube[subSurfMapIdx[i]]);
            downFilter.filter(*tmpSurfCloud);
            surfCloudMapCube[subSurfMapIdx[i]] = tmpSurfCloud;
        }
    }

    void Convert2SubMap(const PointType &point, PointType &mapPoint)
    {
        mapPoint.intensity = point.intensity;

        Eigen::Vector3d tmp_point{point.x, point.y, point.z};
        tmp_point = Q_map_curr * tmp_point + T_map_curr;
        mapPoint.x = tmp_point.x();
        mapPoint.y = tmp_point.y();
        mapPoint.z = tmp_point.z();
    }

    void odomHandler(const nav_msgs::Odometry::ConstPtr &odom)
    {
        bufMutex.lock();
        odomBuf.push(odom);
        bufMutex.unlock();
    }

    void sharpCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &sharpCloud)
    {
        bufMutex.lock();
        sharpCloudBuf.push(sharpCloud);
        bufMutex.unlock();
    }

    void surfCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &surfCloud)
    {
        bufMutex.lock();
        surfCloudBuf.push(surfCloud);
        bufMutex.unlock();
    }

    void fullCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &fullCloud)
    {
        bufMutex.lock();
        fullCloudBuf.push(fullCloud);
        bufMutex.unlock();
    }

    void Transform2Map()
    {
        Q_map_curr = Q_map_odom * Q_odom_curr;
        T_map_curr = Q_map_odom * T_odom_curr + T_map_odom;
    }

    ~iLaserMapping()
    {
        //
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iLaserMapping");
    ros::NodeHandle nh;
    iLaserMapping iLM(nh);
    return 0;
}