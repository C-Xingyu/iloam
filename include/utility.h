/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-17 21:48:17
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-03-22 14:59:31
 */

#pragma once

// extern const double min_range = 1.0;
// extern const double max_range = 100;
// extern const double scan_period = 0.1;
// extern const int LINES = 16;
// extern const int N_SCAN = 1800;
// extern const double angle_gap = 2.0;
// extern const double angle_x = 0.2;
// extern const double diff_threshold = 0.1;
// extern const bool PUB_EACH_LINE = true;
// extern const int sharp_count_threshold = 6;
// extern const int less_sharp_count_threshold = 20;
// extern const int surf_count_threshold = 5;

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