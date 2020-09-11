#pragma once

#include <stdio.h>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <nav_msgs/Odometry.h>


class Test {
    public: Test(int x);
};

// class Test {
//     public: Test(ros::NodeHandle& nodeHandle);
// };

class DVSLocalization {
    public:
        DVSLocalization(ros::NodeHandle& nodeHandle);
        void event_array_callback(const dvs_msgs::EventArray &event_array_msg);
        void localize();
    private:
        ros::Subscriber event_array_subscriber_;
        
        std::queue<dvs_msgs::EventArrayConstPtr> event_array_buf;
        std::mutex m_event_array;

        bool localizing = false;
        int num_buffer = 100;
};
