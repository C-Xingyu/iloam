/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-17 21:17:17
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-04-06 14:39:50
 */
//#pragma GCC optimize(3)
#include "../include/utility.h"
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <string>
#include "../../../devel/include/iloam/ScanRegConfig.h"

class iScanRegistration : public BaseModule
{
public:
    ros::NodeHandle nh;
    int frame_count;
    ros::Subscriber sensor_cloud_sub;
    ros::Publisher sorted_cloud_pub;

    ros::Publisher sharp_cloud_pub;
    ros::Publisher less_sharp_cloud_pub;
    ros::Publisher surf_cloud_pub;
    ros::Publisher less_surf_cloud_pub;
    std::vector<ros::Publisher> each_line_pub;

    std_msgs::Header header;

    PointCloud::Ptr original_cloud; //输入点云
    PointCloud::Ptr filtered_cloud; //过滤无关点后的点云
    PointCloud::Ptr sorted_cloud;
    PointCloud::Ptr sharp_cloud;
    PointCloud::Ptr less_sharp_cloud;
    PointCloud::Ptr surf_cloud;
    PointCloud::Ptr less_surf_cloud;

    std::vector<int> scan_start_indices;
    std::vector<int> scan_end_indices;
    std::vector<PointCloud::Ptr> scan_cloud;

    std::string cloud_topic;
    double min_range;
    double max_range;
    double scan_period;
    // int LINES;
    // int N_SCAN;
    double angle_gap;
    double angle_x;
    double diff_threshold;
    bool PUB_EACH_LINE;
    int sharp_count_threshold;
    int less_sharp_count_threshold;
    int surf_count_threshold;

public:
    iScanRegistration() : original_cloud(new PointCloud()), filtered_cloud(new PointCloud()),
                          sorted_cloud(new PointCloud()), sharp_cloud(new PointCloud()),
                          surf_cloud(new PointCloud()), less_sharp_cloud(new PointCloud()),
                          less_surf_cloud(new PointCloud()), frame_count(0)

    {

        ROS_INFO("Start! ");
        InitialParams();
        dynamic_reconfigure::Server<dynamic_cfg::ScanRegConfig> server;
        dynamic_reconfigure::Server<dynamic_cfg::ScanRegConfig>::CallbackType ff;

        ff = boost::bind(&iScanRegistration::paraCallback, this, _1, _2);
        server.setCallback(ff);

        sensor_cloud_sub = nh.subscribe(cloud_topic, 100, &iScanRegistration::CloudHandler, this);
        sorted_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sorted_cloud", 100);

        sharp_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/sharp_cloud", 100);
        less_sharp_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/less_sharp_cloud", 100);
        surf_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/surf_cloud", 100);
        less_surf_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/less_surf_cloud", 100);
        each_line_pub.resize(LINES);
        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < LINES; ++i)
            {
                each_line_pub[i] = nh.advertise<sensor_msgs::PointCloud2>("/scan_" + std::to_string(i), 100);
            }
        }

        ros::spin();
    }

    void paraCallback(dynamic_cfg::ScanRegConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure Request: %lf %d %d %d %s",
                 config.diff_threshold,
                 config.sharp_count_threshold,
                 config.less_sharp_count_threshold,
                 config.surf_count_threshold,
                 config.PUB_EACH_LINE ? "True" : "False");
        diff_threshold = config.diff_threshold;
        sharp_count_threshold = config.sharp_count_threshold;
        less_sharp_count_threshold = config.less_sharp_count_threshold;
        surf_count_threshold = config.surf_count_threshold;
        PUB_EACH_LINE = config.PUB_EACH_LINE;
    }

    void InitialParams()
    {

        nh.param<std::string>("cloud_topic", cloud_topic, "/velodyne_points");
        nh.param<double>("min_range", min_range, 1.0);
        nh.param<double>("max_range", max_range, 100.0);
        nh.param<double>("scan_period", scan_period, 0.1);
        nh.param<int>("LINES", LINES, 16);
        nh.param<int>("N_SCAN", N_SCAN, 1800);
        nh.param<double>("angle_gap", angle_gap, 2.0);
        nh.param<double>("angle_x", angle_x, 0.2);
        nh.param<double>("diff_threshold", diff_threshold, 0.1);
        nh.param<bool>("PUB_EACH_LINE", PUB_EACH_LINE, true);
        nh.param<int>("sharp_count_threshold", sharp_count_threshold, 2);
        nh.param<int>("less_sharp_count_threshold", less_sharp_count_threshold, 20);
        nh.param<int>("surf_count_threshold", surf_count_threshold, 4);
        each_line_pub.resize(LINES);
        scan_start_indices.resize(LINES);
        scan_end_indices.resize(LINES);
        scan_cloud.resize(LINES);
        for (int i = 0; i < LINES; i++)
        {
            PointCloud::Ptr scan(new PointCloud());
            scan_cloud[i] = scan;
        }
    }

    void CloudHandler(const sensor_msgs::PointCloud2Ptr &cloud_msg)
    {
        auto start = std::chrono::system_clock::now();

        ROS_INFO("This is the %dst frame. ", frame_count);

        header = cloud_msg->header;

        original_cloud = Concert2PCLCloud(cloud_msg);
        RemoveSomePoints();

        SortCloud();

        ExtractFeatures();
        PublishCloud();
        AllocateMemory();
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        ROS_INFO("Time of iScanRegistration : %ld ms", duration.count());
    }

    void RemoveSomePoints()
    {

        RemoveNaNPoints(original_cloud);
        for (int i = 0; i < original_cloud->points.size(); ++i)
        {
            double range = sqrt(original_cloud->points[i].x * original_cloud->points[i].x +
                                original_cloud->points[i].y * original_cloud->points[i].y);
            if (!(range > max_range || range < min_range))
                filtered_cloud->points.push_back(original_cloud->points[i]);
        }
    }

    void SortCloud()
    {

        for (auto &point : filtered_cloud->points)
        {
            double range_xy = sqrt(point.x * point.x +
                                   point.y * point.y);

            double theta = std::atan2(abs(point.z), range_xy) * 180.0 / M_PI;
            int row_index, column_index;
            if (point.z < 0)
                row_index = 7 - int(theta / angle_gap);
            else
                row_index = 7 + int(theta / angle_gap);

            if (row_index < 0 || row_index >= LINES)
                continue;

            double horizonAngle = atan2(point.x, point.y) * 180.0 / M_PI;

            column_index = -round((horizonAngle - 90.0) / angle_x) + N_SCAN / 2;
            if (column_index >= N_SCAN)
                column_index -= N_SCAN;

            if (column_index < 0 || column_index >= N_SCAN)
                continue;
            point.intensity = float(column_index * 100.0 + row_index * 1.0) + point.intensity / 101.0;
            // ROS_INFO("point.intensity:  %f ", point.intensity);
            scan_cloud[row_index]->push_back(point);
        }

        for (int i = 0, count = 0; i < LINES; ++i)
        {
            scan_start_indices[i] = count + 5;
            count += scan_cloud[i]->size();
            scan_end_indices[i] = count - 6;
            *sorted_cloud += *scan_cloud[i];
            // std::cout << "cloudsize: " << sorted_cloud->size() << std::endl;
        }
    }

    void ExtractFeatures()
    {

        int cloud_size = sorted_cloud->size();

        std::vector<int> cloud_label(cloud_size, 0);
        std::vector<int> is_sorted(cloud_size, 0);
        std::vector<smoothness> cloud_smoothness;
        cloud_smoothness.resize(cloud_size);

        for (int i = 5; i < cloud_size - 5; ++i)
        {

            double diff_x = sorted_cloud->points[i - 5].x + sorted_cloud->points[i - 4].x +
                            sorted_cloud->points[i - 3].x + sorted_cloud->points[i - 2].x +
                            sorted_cloud->points[i - 1].x + sorted_cloud->points[i + 1].x +
                            sorted_cloud->points[i + 2].x + sorted_cloud->points[i + 3].x +
                            sorted_cloud->points[i + 4].x + sorted_cloud->points[i + 5].x -
                            10 * sorted_cloud->points[i].x;
            double diff_y = sorted_cloud->points[i - 5].y + sorted_cloud->points[i - 4].y +
                            sorted_cloud->points[i - 3].y + sorted_cloud->points[i - 2].y +
                            sorted_cloud->points[i - 1].y + sorted_cloud->points[i + 1].y +
                            sorted_cloud->points[i + 2].y + sorted_cloud->points[i + 3].y +
                            sorted_cloud->points[i + 4].y + sorted_cloud->points[i + 5].y -
                            10 * sorted_cloud->points[i].y;
            double diff_z = sorted_cloud->points[i - 5].z + sorted_cloud->points[i - 4].z +
                            sorted_cloud->points[i - 3].z + sorted_cloud->points[i - 2].z +
                            sorted_cloud->points[i - 1].z + sorted_cloud->points[i + 1].z +
                            sorted_cloud->points[i + 2].z + sorted_cloud->points[i + 3].z +
                            sorted_cloud->points[i + 4].z + sorted_cloud->points[i + 5].z -
                            10 * sorted_cloud->points[i].z;

            cloud_smoothness[i].value = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

            cloud_smoothness[i].id = i;
        }
        // ROS_INFO("Sorted!");

        for (int i = 0; i < LINES; ++i)
        {
            // ROS_INFO("sort every line");
            PointCloud::Ptr less_scan_surf_cloud(new PointCloud());
            if (scan_end_indices[i] - scan_start_indices[i] < 6)
                continue;
            for (int j = 0; j < 6; ++j) //分成六份
            {
                int start_index = scan_start_indices[i] + (scan_end_indices[i] - scan_start_indices[i]) * j / 6;
                int end_index = scan_start_indices[i] + (scan_end_indices[i] - scan_start_indices[i]) * (j + 1) / 6 - 1;
#ifdef DEBUG
                ROS_INFO("start_index: %d ", start_index);
                ROS_INFO("end_index: %d ", end_index);
#endif
                if (start_index >= end_index)
                    continue;
                std::sort(cloud_smoothness.begin() + start_index, cloud_smoothness.begin() + end_index + 1, by_value()); //升序排列

                int pick_edge_count = 0;
                for (int k = end_index; k >= start_index; --k)
                {
                    //每次ind的值就是等于k??? 有什么意义?
                    //排完序后，k不一定等于ind
                    int ind = cloud_smoothness[k].id;

                    if (cloud_smoothness[ind].value > diff_threshold && is_sorted[ind] == 0)
                    {

                        if (pick_edge_count <= sharp_count_threshold)
                        {
                            cloud_label[ind] = 2;

                            sharp_cloud->points.push_back(sorted_cloud->points[ind]);

                            less_sharp_cloud->points.push_back(sorted_cloud->points[ind]);

                            // sharp_cloud_tmp.points.push_back(sorted_cloud->points[ind]);

                            // less_sharp_cloud_tmp.points.push_back(sorted_cloud->points[ind]);
                        }
                        else if (pick_edge_count <= less_sharp_count_threshold)
                        {

                            cloud_label[ind] = 1;
                            less_sharp_cloud->points.push_back(sorted_cloud->points[ind]);
                            // less_sharp_cloud_tmp.points.push_back(sorted_cloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        is_sorted[ind] = 1;
                        pick_edge_count++;

                        for (int l = -5; l < 5; ++l)
                        {
                            double diff_x = sorted_cloud->points[ind + l].x - sorted_cloud->points[ind + l + 1].x;
                            double diff_y = sorted_cloud->points[ind + l].y - sorted_cloud->points[ind + l + 1].y;
                            double diff_z = sorted_cloud->points[ind + l].z - sorted_cloud->points[ind + l + 1].z;

                            double diff = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

                            if (diff > 0.05)
                                break;
                            is_sorted[ind + l] = 1;
                        }
                    }
                }

                int pick_surf_count = 0;
                for (int k = start_index; k <= end_index; ++k)
                {
                    int ind = cloud_smoothness[k].id;
                    if (cloud_smoothness[ind].value < diff_threshold && is_sorted[ind] == 0)
                    {
                        if (pick_surf_count <= surf_count_threshold)
                        {
                            cloud_label[ind] = -1;
                            surf_cloud->points.push_back(sorted_cloud->points[ind]);
                            // surf_cloud_tmp.points.push_back(sorted_cloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        is_sorted[ind] = 1;
                        pick_surf_count++;

                        for (int l = -5; l < 5; ++l)
                        {
                            double diff_x = sorted_cloud->points[ind + l].x - sorted_cloud->points[ind + l + 1].x;
                            double diff_y = sorted_cloud->points[ind + l].y - sorted_cloud->points[ind + l + 1].y;
                            double diff_z = sorted_cloud->points[ind + l].z - sorted_cloud->points[ind + l + 1].z;

                            double diff = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

                            if (diff > 0.05)
                                break;
                            is_sorted[ind + l] = 1;
                        }
                    }
                }

                for (int k = start_index; k <= end_index; ++k)
                {
                    int ind = cloud_smoothness[k].id;
                    if (cloud_label[ind] <= 0)
                        less_scan_surf_cloud->points.push_back(sorted_cloud->points[ind]);
                }
            }
            PointCloud::Ptr tmp_less_surf_cloud(new PointCloud());
            pcl::VoxelGrid<PointType> voxel_filter;
            voxel_filter.setInputCloud(less_scan_surf_cloud);
            voxel_filter.setLeafSize(0.2, 0.2, 0.2);
            voxel_filter.filter(*tmp_less_surf_cloud);
            *less_surf_cloud += *tmp_less_surf_cloud;
        }
    }

    void AllocateMemory()
    {
        // ROS_INFO("Allocate memory");
        original_cloud.reset(new PointCloud());
        filtered_cloud.reset(new PointCloud());
        sorted_cloud.reset(new PointCloud());
        sharp_cloud.reset(new PointCloud());
        less_sharp_cloud.reset(new PointCloud());
        surf_cloud.reset(new PointCloud());
        less_surf_cloud.reset(new PointCloud());

        // each_line_pub.resize(LINES);
        scan_start_indices.resize(LINES);
        scan_end_indices.resize(LINES);
        for (int i = 0; i < scan_cloud.size(); i++)
        {
            scan_cloud[i]->clear();
        }
        // scan_cloud.resize(LINES);
    }

    void PublishCloud()
    {
        // ROS_INFO("Publish Cloud");
        sensor_msgs::PointCloud2 tmp_sorted_cloud;
        tmp_sorted_cloud = Convert2SensorMsg(sorted_cloud);
        tmp_sorted_cloud.header = header;
        sorted_cloud_pub.publish(tmp_sorted_cloud);

        sensor_msgs::PointCloud2 tmp_sharp_cloud;
        tmp_sharp_cloud = Convert2SensorMsg(sharp_cloud);
        ROS_INFO("sharp_cloud size:  %d", sharp_cloud->size());
        tmp_sharp_cloud.header = header;
        sharp_cloud_pub.publish(tmp_sharp_cloud);

        sensor_msgs::PointCloud2 tmp_less_sharp_cloud;
        tmp_less_sharp_cloud = Convert2SensorMsg(less_sharp_cloud);
        ROS_INFO("less_sharp_cloud size:  %d", less_sharp_cloud->size());
        tmp_less_sharp_cloud.header = header;
        less_sharp_cloud_pub.publish(tmp_less_sharp_cloud);

        sensor_msgs::PointCloud2 tmp_surf_cloud;
        tmp_surf_cloud = Convert2SensorMsg(surf_cloud);
        ROS_INFO("surf_cloud size:  %d", surf_cloud->size());
        tmp_surf_cloud.header = header;
        surf_cloud_pub.publish(tmp_surf_cloud);

        sensor_msgs::PointCloud2 tmp_less_surf_cloud;
        tmp_less_surf_cloud = Convert2SensorMsg(less_surf_cloud);
        ROS_INFO("less_surf_cloud size:  %d", less_surf_cloud->size());
        tmp_less_surf_cloud.header = header;
        less_surf_cloud_pub.publish(tmp_less_surf_cloud);

        if (PUB_EACH_LINE)
        {
            for (int i = 0; i < LINES; ++i)
            {
                sensor_msgs::PointCloud2 each_sensor_cloud;
                each_sensor_cloud = Convert2SensorMsg(scan_cloud[i]);
                each_sensor_cloud.header = header;
                each_line_pub[i].publish(each_sensor_cloud);
            }
        }
    }

    ~iScanRegistration() = default;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iScanRegestration");

    iScanRegistration iSR;

    return 0;
}
