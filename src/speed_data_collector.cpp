#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include "topics.h"

const int SAMPLES = 1000;
const int beg = 0;
const int end = 101;

ros::Publisher pub;
int counter = 0;
double sum = 0;
int iter = beg;
bool measure = false;

double *ans;

void changeSpeed(int8_t vel) {
    std_msgs::Int8 msg;
    msg.data = vel;
    pub.publish(msg);
}

void finish() {
    std::string str = "{ ";
    str += std::to_string(ans[0]);
    for(int i = 1; i < end - beg; ++i) {
        str += ", ";
        str += std::to_string(ans[i]);
    }
    str += " }";
    delete[] ans;
    ROS_INFO(str.c_str());
    ros::shutdown();
}

void callback(const std_msgs::Float32 val) {
    if(++counter > SAMPLES) {
        if(measure) {
            ans[iter] = ((double)sum)/((double)SAMPLES);
            ROS_INFO("%d: %f", iter, ans[iter]);
            if(++iter == end) finish();
            changeSpeed(iter);
        }
        measure = !measure;
        counter = 0;
        sum = 0;
    }
    if(measure)
        sum += val.data;
}

int main(int argc, char **argv) {
    ans = new double[end - beg];
    ros::init(argc, argv, "speed_data_collector");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Int8>(CS_TOPIC, 1);
    changeSpeed(beg);
    ros::Subscriber subscriber = nh.subscribe(SPEED_TOPIC, 0, callback);
    ros::spin();
    return 0;
}
