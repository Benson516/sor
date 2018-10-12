#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <algorithm>    // std::sort
#include <math.h>       /* isnan, isinf */

// test
#include <iostream>     // std::cout


class MEDIAN_FILTER{
public:
    // variabes
    size_t window_size;
    size_t effected_size;
    std::vector<float> buffer;
    size_t buff_idx;
    size_t filter_mid_idx;
    //
    float median_value;

    MEDIAN_FILTER(size_t window_size_in){
        window_size = window_size_in;
        effected_size = window_size;
        buffer.resize(window_size, 0.0);
        buffer_sort.resize(window_size, 0.0);
        //
        buff_idx = window_size-1;
        filter_mid_idx = size_t(window_size/2);
        //
        median_value = 0.0;
    }

    void reset(float value){
        effected_size = window_size;
        buffer.assign(window_size, value);
        buffer_sort.assign(window_size, value);
        //
        buff_idx = window_size-1;
        filter_mid_idx = size_t(window_size/2);
        //
        median_value = value;
    }

    // Core function
    float calculate_median(void){
        // using default comparison (operator <):
        std::sort(buffer_sort.begin(), buffer_sort.end());
        //
        if (effected_size == 0){
            // return "+inf"
            // return std::numeric_limits<float>::infinity();
            // return "nan"
            return std::numeric_limits<float>::quiet_NaN();
        }
        //
        if (effected_size % 2 == 1){
            // odd
            size_t half_idx = size_t(effected_size/2) + (window_size-effected_size);
            // e.g. window_size=5 --> half_idx=2
            median_value = buffer_sort[half_idx ];
        }else{
            // even
            size_t half_idx_2 = size_t(effected_size/2) + (window_size-effected_size);
            size_t half_idx_1 = half_idx_2 - 1;
            // e.g. window_size=6 --> half_idx=3
            // median_value = (buffer_sort[half_idx_1] + buffer_sort[half_idx_2])/2.0;
            median_value = buffer_sort[half_idx_1];
            // median_value = buffer_sort[half_idx_2];
        }
        return median_value;
    }

    float filter(float new_value){

        increase_buff_idx();
        increase_filter_mid_idx();
        //
        float old_value = buffer[buff_idx];
        //
        if (isnan(new_value)){
            //
            if (effected_size <= 0){
                effected_size = 0;
            }else{
                effected_size--;
            }
            // Repalce by negative infinity ("-inf"),
            // which will not present in normal data.
            new_value = -1*std::numeric_limits<float>::infinity();
        }
        //
        if (isinf(old_value) && old_value < 0.0){
            // "-inf", originally "nan"
            effected_size++;
            if (effected_size > window_size){
                effected_size = window_size;
            }
        }
        //
        buffer[buff_idx] = new_value;
        // Search for the old value in buffer_sort
        for (size_t i=0; i < window_size; ++i){
            if (old_value <= buffer_sort[i]){
                buffer_sort[i] = new_value;
                break;
            }
        }
        //
        calculate_median();
        //
        if (isinf(buffer[filter_mid_idx]) && buffer[filter_mid_idx] < 0.0){
            return std::numeric_limits<float>::quiet_NaN();
        }else{
            return median_value;
        }

    }

    //
    size_t get_idx_ahead(size_t current_idx, int increament, size_t array_size){
        int out_idx = int(current_idx) + increament;
        while (out_idx >= int(array_size)){
            out_idx -=  int(array_size);
        }
        while (out_idx < 0){
            out_idx += int(array_size);
        }
        return size_t(out_idx);
    }
private:
    //
    std::vector<float> buffer_sort;
    //
    size_t increase_buff_idx(void){
        if (buff_idx < (window_size-1)){
            buff_idx++;
        }else{
            buff_idx = 0;
        }
        return buff_idx;
    }
    size_t increase_filter_mid_idx(void){
        if (filter_mid_idx < (window_size-1)){
            filter_mid_idx++;
        }else{
            filter_mid_idx = 0;
        }
        return filter_mid_idx;
    }

};

//
ros::Publisher filtered_scan_pub;
//
size_t window_size = 5; //5;
MEDIAN_FILTER median_filter(window_size);

//
void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message)
{

    sensor_msgs::LaserScan filtered_scan = *message;
    if(message->ranges.size() < median_filter.window_size){
        // Publish the result
        filtered_scan_pub.publish(filtered_scan);
        return;
    }

    median_filter.reset(0.0);
    // TODO: reduce the length of initial scan
    // Initialize the filter with real values
    for (size_t i=(message->ranges.size()-median_filter.window_size); i < message->ranges.size(); ++i){
        median_filter.filter(message->ranges[i]); // abandon these values
    }

    size_t j = median_filter.get_idx_ahead(0, -1*int(median_filter.window_size/2), message->ranges.size());
    //
    for (size_t i=0; i < message->ranges.size(); ++i){
        // Assign the original value to the filter and do filtering
        median_filter.filter(message->ranges[i]);
        // Is it was originally nan or +inf, sinply skip them and don't filter
        if ( isnan(filtered_scan.ranges[j]) ){
            // std::cout << "nan\n";
            // Not assigned
        }else if ( isinf(filtered_scan.ranges[j]) && filtered_scan.ranges[j] > 0 ){
            // std::cout << "+inf\n";
            // Not assigned
        }else{
            if (median_filter.median_value >= filtered_scan.ranges[j]){
                // Only assigned the filtered value if this value is greater that the original one.
                // This is for preventing the route closure problem
                filtered_scan.ranges[j] = median_filter.median_value;
            }else{
                // Not assigned
            }

        }
        //
        j++;
        if(j >= filtered_scan.ranges.size()){
            j = 0;
        }
        // filtered_scan.ranges[i] = 123.0;
    }
    // Publish the result
    filtered_scan_pub.publish(filtered_scan);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "median_filter_laser");

  ros::NodeHandle nh;

  // Input laser scan
  ros::Subscriber sub = nh.subscribe("scan", 1000, laserScanCallback);
  // Output laser scan
  filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan> ("scan_filtered", 1);

  ros::spin();

  return 0;
}
