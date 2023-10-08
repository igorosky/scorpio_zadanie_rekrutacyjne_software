#define USE_PREDEFINED  // This sets CS according to predefined table before activating pid (thanks to this we can quicker get desired speed without sacrificing PID correction)
#define LOGGING

// PID parameters
#define PROPORTION_FACTOR 0.17
#define INTEGRAL_FACTOR 0.11
#define DIFFERENTIATION_FACTOR 0.005

// Topic names
#define CS_TOPIC "/virtual_dc_motor/set_cs"
#define REQUIRED_SPEED_TOPIC "/virtual_dc_motor_controller/set_velocity_goal"
#define SPEED_TOPIC "/virtual_dc_motor_driver/get_velocity"

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include "pid.hpp"

#ifdef USE_PREDEFINED
#define PREDEFINED_ARRAY_LENGTH 71
const std::pair<float, uint8_t> predefined_values[PREDEFINED_ARRAY_LENGTH] = {{0, 0}, {0.000181, 7}, {0.000296, 9}, {5.861842, 15}, {5.862259, 18}, {11.721675, 19}, {11.72274, 22}, {17.582353, 23}, {17.585981, 25}, {23.444317, 28}, {29.287472, 29}, {29.312131, 31}, {35.168178, 34}, {41.011832, 35}, {41.03329, 36}, {46.893413, 37}, {46.906866, 38}, {52.753889, 39}, {52.786914, 40}, {58.634838, 42}, {64.48335, 44}, {70.365594, 46}, {76.214947, 48}, {82.060055, 49}, {87.91712, 51}, {93.762507, 52}, {93.844598, 53}, {99.671046, 54}, {105.518305, 56}, {111.366503, 57}, {117.242971, 58}, {117.258487, 59}, {122.870341, 60}, {128.909963, 61}, {134.840747, 63}, {140.635386, 64}, {146.550311, 65}, {152.295365, 66}, {152.410063, 67}, {158.138412, 68}, {164.124545, 69}, {170.006315, 70}, {175.956149, 71}, {181.681518, 73}, {187.693346, 74}, {193.548854, 75}, {199.229412, 76}, {205.218999, 77}, {211.149398, 78}, {217.00286, 79}, {222.823966, 80}, {228.680865, 81}, {234.501427, 82}, {240.354849, 83}, {246.219342, 84}, {252.057596, 85}, {257.796025, 86}, {263.725341, 87}, {269.564476, 88}, {275.503071, 89}, {281.531075, 90}, {287.275426, 91}, {293.183068, 92}, {298.958229, 93}, {310.858312, 94}, {316.632477, 95}, {322.563722, 96}, {328.167583, 97}, {334.31444, 98}, {340.164207, 99}, {351.690895, 100}};

int8_t find_predefined(const float target_speed) {
    int p = 0;
    int q = PREDEFINED_ARRAY_LENGTH;
    while(p + 1 < q) {
        const auto s = (p + q)/2;
        if(predefined_values[s].first > target_speed) q = s;
        else p = s;
    }
    if(p + 1 < PREDEFINED_ARRAY_LENGTH && target_speed - predefined_values[p].first > predefined_values[p + 1].first - target_speed)
        ++p;
    return predefined_values[p].second;
}
#endif

ros::Publisher pub;
Pid *pid;
uint64_t prev_time;

void set_cs(const int8_t cs) {
    std_msgs::Int8 msg;
    msg.data = cs;
    pub.publish(msg);
    #ifdef LOGGING
    ROS_INFO("Target speed: %f, Control signal value: %d", pid->get_target_value(), cs);
    #endif
}


void change_target_vel(const std_msgs::Float32 val) {
    #ifdef USE_PREDEFINED
    int8_t cs = find_predefined(abs((float)val.data))*(val.data < 0 ? -1 : 1);
    set_cs(cs);
    pid->set_target_value_with_default(val.data, cs);
    #else
    pid->set_target_value(val.data);
    #endif
}

void get_rmp(const std_msgs::Float32 rmp) {
    const uint64_t time = ros::Time::now().toNSec();
    set_cs(pid->calculate(rmp.data, (float)(time - prev_time)/100000000));
    prev_time = time;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "speed_controller");
    ros::NodeHandle nh;
    pid = new Pid(PROPORTION_FACTOR, INTEGRAL_FACTOR, DIFFERENTIATION_FACTOR);
    pub = nh.advertise<std_msgs::Int8>(CS_TOPIC, 2);
    ros::Subscriber vel_goal_subscriber = nh.subscribe(REQUIRED_SPEED_TOPIC, 2, change_target_vel);
    ros::Subscriber rmp_subscriber = nh.subscribe(SPEED_TOPIC, 0, get_rmp);
    set_cs(0);
    prev_time = ros::Time::now().toNSec();
    ros::spin();
    return 0;
}
