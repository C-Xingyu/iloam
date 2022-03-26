/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-24 20:25:44
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-03-25 17:13:36
 */
#include "../include/utility.h"
#include <queue>
#include <mutex>
#include <thread>
#include <pcl/kdtree/kdtree_flann.h>

class iLaserOdometry : public BaseModule
{
private:
    ros::NodeHandle inh;

    ros::Subscriber fullCloudSub;
    ros::Subscriber sharpCloudSub;
    ros::Subscriber lessSharpCloudSub;
    ros::Subscriber surfCloudSub;
    ros::Subscriber lessSurfCloudSub;

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

    double NearestThreshold;

    Eigen::Quaterniond Q_last_curr;
    Eigen::Vector3d T_last_curr;

public:
    iLaserOdometry(ros::NodeHandle &nh) : inh(nh), fullCloud(new PointCloud()),
                                          sharpCloud(new PointCloud()), lessSharpCloud(new PointCloud()),
                                          surfCloud(new PointCloud()), lessSurfCloud(new PointCloud())
    {
        InitialParams();
        fullCloudSub = inh.subscribe("/sorted_cloud", 1, &FullCloudHandler, this);
        sharpCloudSub = inh.subscribe("/sharp_cloud", 1, &SharpCloudHandler, this);
        lessSharpCloudSub = inh.subscribe("/less_sharp_cloud", 1, &LessSharpCloudHandler, this);
        surfCloudSub = inh.subscribe("/surf_cloud", 1, &SurfCloudHandler, this);
        lessSurfCloudSub = inh.subscribe("/less_surf_cloud", 1, &LessSurfCloudHandler, this);

        while (ros::ok())
        {
            ros::spinOnce();
            if (IsEmpty() && AlignTimeStamp())
            {
                Process();
            }
            else
            {
                ROS_INFO("Please ensure the buffer of point cloud isn't empty ,or align the timestamp!");
                ROS_BREAK();
            }
        }
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
        }

        lastSharpCloudTree->setInputCloud(sharpCloud);
        lastSurfCloudTree->setInputCloud(surfCloud);
    }

    void SetOdometry()
    {
        int sharpCloudNum = sharpCloud->size();
        int surfCloudNum = surfCloud->size();

        std::vector<int> sharpPointsIndices;
        std::vector<float> sharpPointsDist;

        for (int i = 0; i < sharpCloudNum; ++i)
        {
            PointType currSharpPoint = sharpCloud->points[i];
            PointType transedSharpPoint;
            Transform2Start(currSharpPoint, transedSharpPoint);
            lastSharpCloudTree->nearestKSearch(transedSharpPoint, 1, sharpPointsIndices, sharpPointsDist);

            if (sharpPointsDist[0] < NearestThreshold)
            {
                
            }
        }
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

        inh.param<int>("N_SCAN", N_SCAN, 1800);
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