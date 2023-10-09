// #define AVERAGE //Doesnt work well with PID
#define LOGGING

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include "topics.h"

#define RANGE 4096

ros::Publisher pub;

#ifdef AVERAGE
#include <list>

#define MEMORY_SIZE 40
int sum = 0;
int total_time = 0;
std::list<std::pair<int16_t, uint64_t>> measurements;
#endif
int16_t prev = 0;
uint64_t prev_time = 0;
bool bootstart = false;

inline int dif(int v, int prev) {
    if(abs(v - prev) < std::min(v, prev) + RANGE - std::max(v, prev)) return v - prev;
    return (std::min(v, prev) + RANGE - std::max(v, prev))*(prev < v ? -1 : 1);
}

#ifdef AVERAGE
float measurement(const int16_t v) {
    int s = dif(v, prev);
    prev = v;
    sum += s;
    uint64_t cur_time = ros::Time::now().toNSec();
    uint64_t time = cur_time - prev_time;
    prev_time = cur_time;
    total_time += time;
    measurements.push_front({s, time});
    if(measurements.size() > MEMORY_SIZE) {
        sum -= measurements.back().first;
        total_time -= measurements.back().second;
        measurements.pop_back();
    }
    return ((long double)sum*60*1000000000)/RANGE/total_time;
}
#else
float measurement(int16_t v) {
    int s = dif(v, prev);
    prev = v;
    uint64_t cur_time = ros::Time::now().toNSec();
    float ans = ((float)s*60*1000000000)/RANGE/((float)(cur_time - prev_time));
    prev_time = cur_time;
    return ans;
}
#endif

void callback(const std_msgs::UInt16 val) {
    std_msgs::Float32 msg;
    msg.data = measurement(val.data); 
    #ifdef LOGGING
    ROS_INFO("%f", msg.data);
    #endif
    if(bootstart)
        pub.publish(msg);
    else
        bootstart = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rpm");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float32>(SPEED_TOPIC, 1);
    ros::Subscriber subscriber = nh.subscribe(POSITION_TOPIC, 2, callback);
    ros::spin();
    return 0;
}
