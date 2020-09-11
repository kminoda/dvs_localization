#include "dvs_vlc_ros/dvs_vlc_ros.hpp"

Test::Test(int x){
    int a = 1;
    ROS_INFO("hoge");
}

// Test::Test(ros::NodeHandle& nodeHandle) {
//     ROS_INFO("hoge");
//     event_array_subscriber_ = nodeHandle.subscribe("prophesee/camera/cd_events_buffer", 10, &DVSLocalization::event_array_callback, this);
// }

// namespace dvs_vlc {

DVSLocalization::DVSLocalization(ros::NodeHandle& nodeHandle) : 
    nodeHandle_(nodeHandle) {
        ROS_INFO("hoge");
        event_array_subscriber_ = nodeHandle_.subscribe("prophesee/camera/cd_events_buffer", 10, &DVSLocalization::event_array_callback, this);
}

void DVSLocalization::event_array_callback(const dvs_msgs::EventArray &event_array_msg)
{
    if (localizing==false)
    {
        m_event_array.lock();
        event_array_buf.push(event_array_msg);
        m_event_array.unlock();

        if (event_array_buf.size()>=num_buffer)
        {
            localize();
        }
    }
}

void DVSLocalization::localize()
{
    ROS_INFO("Start localization!")
    while (!event_array_buf.empty()) event_array_buf.pop();
}

// }