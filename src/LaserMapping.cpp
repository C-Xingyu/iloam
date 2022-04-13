/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-04-06 20:54:49
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-04-08 22:15:57
 */
#include "../include/utility.h"
#include <nav_msgs/Odometry.h>
#include <queue>
#include <mutex>
#include <thread>

class iLaserMapping : BaseModule
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

    PointCloud::Ptr shapCloudMapCube[mapCubeNum];
    PointCloud::Ptr surfCloudMapCube[mapCubeNum];

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

                CalculateSelfIJK();
            }
        }
    }

    void CalculateSelfIJK()
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
            for (int j = 0; j < mapCubeWidth; ++j)
            {
                for (int k = 0; k < mapCubeHeight; ++k)
                {
                    // int cubeIdx=
                }
            }
        }
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
        T_map_curr = Q_map_odom * T_odom_curr + T_map_curr;
        //此处平移与A-LOAM不同
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