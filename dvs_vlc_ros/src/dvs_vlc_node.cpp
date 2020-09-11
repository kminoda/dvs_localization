#include <stdio.h>
#include <vector>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <nav_msgs/Odometry.h>

struct Event {
    int x;
    int y;
    double time;
    bool polarity;
    int ts_sec;
    int ts_nsec;
};

double PI = 3.1415926535;

class DVSLocalization {
    public: 
        DVSLocalization(ros::NodeHandle nodeHandle) : nodeHandle_(nodeHandle)
        {
            ROS_INFO("Node launched");
            event_array_subscriber_ = nodeHandle_.subscribe("/prophesee/camera/cd_events_buffer", 10, &DVSLocalization::event_array_callback, this);
            odom_publisher_ = nodeHandle_.advertise<nav_msgs::Odometry>("/odometry", 1000);
            load_data();
            initialize();
        }
        double prob(double dt);
        void event_array_callback(const dvs_msgs::EventArrayConstPtr &event_array_msg);
        void localize();
        void estimate_led_positions();
        bool encode_msg();
        void load_data();
        void solve_pnp();
        void publish_odom();
        void initialize();
        void undistort();
    private:
        ros::NodeHandle nodeHandle_;
        ros::Subscriber event_array_subscriber_;
        ros::Publisher odom_publisher_;

        // Parameters TBD
        double time_len = 1.0;
        double t_at = 0.5;
        double start_time = -1;
        double freq = 100;
        double sigma = 30;
        int n_led = 2;
        std::vector<std::vector<bool>> msgs = {{true, true, false, false, true, true}, {true, true, true, true, false, false}};
        int img_size_x = 1280;
        int img_size_y = 720;
        double led_r = 40;
        double thres_for_edge_detection = 50;
        int message_length = 6;
        std::vector<bool> sync = {false, false, true, false, true, true, false};
        std::string method = "xyzyaw";

        // Parameters for calculation
        bool localizing = false;
        std::queue<dvs_msgs::EventArrayConstPtr> event_array_buf;
        std::mutex m_event_array;
        std::vector<Event> event_list;
        std::vector<double> event_time_list;
        std::vector<std::vector<double>> led_pos_list = std::vector<std::vector<double>>(n_led, std::vector<double>(2));
        // std::vector<std::vector<double>> led_edges_list = std::vector<std::vector<double>>(n_led);
        // std::vector<std::vector<bool>> led_bits_list = std::vector<std::vector<bool>>(n_led);
        // std::vector<std::vector<int>> vlc_sync_idx_list = std::vector<std::vector<int>>(n_led);
        // std::vector<std::vector<bool>> vlc_msg_list = std::vector<std::vector<bool>>(n_led, std::vector<bool>(message_length));
        Eigen::MatrixXd K = Eigen::MatrixXd::Identity(3, 3);
        // int num_hist_per_sec = 2001;
        std::vector<double> dist = std::vector<double>(5);
        // double dt_for_hist = 1.0/num_hist_per_sec;
        // std::vector<double> time_bins = std::vector<double>(int(num_hist_per_sec * time_len));
        // std::vector<std::vector<int>> hist_data_pos = std::vector<std::vector<int>>(n_led, std::vector<int>(int(num_hist_per_sec * time_len)));
        // std::vector<std::vector<int>> hist_data_neg = std::vector<std::vector<int>>(n_led, std::vector<int>(int(num_hist_per_sec * time_len)));
        std::vector<std::vector<double>> ps = std::vector<std::vector<double>>(n_led);
        std::vector<std::vector<double>> ps_und = std::vector<std::vector<double>>(n_led);
        std::vector<std::vector<double>> Pws = std::vector<std::vector<double>>(n_led, std::vector<double>(3));

        // Parameters which we want to estimate
        Eigen::MatrixXd R = Eigen::MatrixXd(3, 3);
        Eigen::MatrixXd T = Eigen::MatrixXd(1, 3);
};


void DVSLocalization::event_array_callback(const dvs_msgs::EventArrayConstPtr &event_array_msg)
{
    if (localizing==false)
    {
        if (event_array_buf.empty()) 
        {
            start_time = double(event_array_msg->header.stamp.sec) + double(event_array_msg->header.stamp.nsec)*1e-9;
        }
        m_event_array.lock();
        event_array_buf.push(event_array_msg);
        m_event_array.unlock();

        if (double(event_array_msg->header.stamp.sec) + double(event_array_msg->header.stamp.nsec)*1e-9 - start_time >= time_len)
        {
            localize();
        }
    }
}
// sleep(1);
void DVSLocalization::localize()
{
    localizing = true;
    ROS_INFO("Start localization!");
    // ROS_INFO("Buffer length = %d", event_array_buf.size());

    estimate_led_positions();
    if (encode_msg()==false) 
    {
        ROS_INFO("Encoding failed."); 
    }
    else 
    {
        ROS_INFO("Encoding Success!!!");
        solve_pnp();
        publish_odom();
    }
        
    // Re-initialize
    initialize();
    ROS_INFO("Finished localization!");
}

void DVSLocalization::initialize()
{
    // all
    while (!event_array_buf.empty()) event_array_buf.pop();
    localizing = false;
    start_time = -1;

    // estimate_led_positions
    event_list.clear();
    event_time_list.clear();

    // encode_msg
    led_pos_list = std::vector<std::vector<double>>(n_led, std::vector<double>(2));
    // led_edges_list[0].clear(); led_edges_list[1].clear();  //  = std::vector<std::vector<double>>(n_led);
    // led_bits_list[0].clear(); led_bits_list[1].clear(); //  = std::vector<std::vector<bool>>(n_led);
    // vlc_sync_idx_list[0].clear(); vlc_sync_idx_list[1].clear();  //  = std::vector<std::vector<int>>(n_led);
    // vlc_msg_list = std::vector<std::vector<bool>>(n_led, std::vector<bool>(message_length));
    ps = std::vector<std::vector<double>>(n_led);
}

double DVSLocalization::prob(double dt)
{
    double res = std::exp(-std::pow((1.0/dt - freq), 2)/2/sigma/sigma) / std::sqrt(2*PI*sigma*sigma);
    return res;
}

void DVSLocalization::estimate_led_positions()
{
    // Evidence map
    int n = 5;
    std::vector<std::vector<int>> last_polarity(img_size_x, std::vector<int>(img_size_y, -1));
    std::vector<std::vector<double>> last_trans_time(img_size_x, std::vector<double>(img_size_y, -1));
    std::vector<std::vector<double>> evidence_map(img_size_x, std::vector<double>(img_size_y, 0));
    
    while (!event_array_buf.empty()) 
    {
        auto msg = event_array_buf.front(); event_array_buf.pop();
        for (int i=0; i<msg->events.size(); i++)
        {
            auto event = msg->events[i];
            Event event_to_add;
            event_to_add.x = event.x; event_to_add.y = event.y; 
            event_to_add.time = double(event.ts.sec) + double(event.ts.nsec)*1e-9; 
            event_to_add.polarity = event.polarity; 
            event_to_add.ts_sec = event.ts.sec; event_to_add.ts_nsec = event.ts.nsec;
            event_list.push_back(event_to_add);
            if (start_time<0)
            {
                start_time = double(event.ts.sec) + double(event.ts.nsec)*1e-9;
            }
            double t = double(event.ts.sec) + double(event.ts.nsec)*1e-9 - start_time;
            if (last_polarity[event.x][event.y] == -1) 
            {
                last_polarity[event.x][event.y] = event.polarity*1;
            }
            else if (last_polarity[event.x][event.y] != event.polarity*1)
            {
                double dt = t - last_trans_time[event.x][event.y];
                if (time_len-n*1.0/freq<t && t<time_len && dt!=0.0)
                {
                    evidence_map[event.x][event.y] += prob(dt);
                }
                last_trans_time[event.x][event.y] = t;
            }
            last_polarity[event.x][event.y] = event.polarity*1;
        }
    }


    // KMeans clustering from evidence_map
    double evidence_tol_rate = 0.2;
    std::vector<std::vector<int>> high_evidence_points;
    double maximum_evidence = 0;
    for (int i=0; i<img_size_x; i++) for (int j=0; j<img_size_y; j++) maximum_evidence = std::max(maximum_evidence, evidence_map[i][j]);
    for (int i=0; i<img_size_x; i++) for (int j=0; j<img_size_y; j++) if (evidence_map[i][j] > evidence_tol_rate * maximum_evidence)
    {
        std::vector<int> point;
        point.push_back(i); point.push_back(j);
        high_evidence_points.push_back(point);
    }

    cv::Mat points(high_evidence_points.size(), 1, CV_32FC2);
    for (int i=0; i<high_evidence_points.size(); i++)
    {
        points.at<cv::Point2f>(i) = cv::Point2f(high_evidence_points[i][0], high_evidence_points[i][1]);
    }
    cv::Mat centers;
    cv::Mat clusters = cv::Mat::zeros(high_evidence_points.size(), 1, CV_32FC2);

    cv::kmeans(points,
               n_led,
               clusters,
               cvTermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 10, 1.0),
               1,
               cv::KMEANS_PP_CENTERS,
               centers);
    for (int i=0; i<centers.rows; i++) for (int j=0; j<centers.cols; j++) 
    {
        // std::cout << centers.at<float>(i, j) << " ";
        led_pos_list[i][j] = centers.at<float>(i, j);
    }
    for (int ledID=0; ledID<n_led; ledID++)
    {
        ROS_INFO("LED at (x, y) = (%d, %d)", int(led_pos_list[ledID][0]), int(led_pos_list[ledID][1]));
    }
}

bool DVSLocalization::encode_msg()
{
    // int num_hist_per_sec = 2001;
    // int n_bins = num_hist_per_sec * time_len;
    // double tol_rate = 0.8;
    bool success_encoding = true;

    // // calculate time bins
    // // double time_min = 1e10;
    // // double time_max = 0;    
    // // for (int ledID; ledID<n_led; ledID++) for (int i=0; i<event_list.size(); i++)
    // // {
    // //     time_min = std::min(time_min, event_list[i].time);
    // //     time_max = std::max(time_max, event_list[i].time);
    // // }
    // std::vector<double> time_bins = std::vector<double>(n_bins);
    // for (int i=0; i<n_bins; i++)
    // {
    //     // time_bins[i] = i*(time_max - time_min)/n_bins;
    //     time_bins[i] = i*t_at/n_bins;
    // }

    // for (int ledID; ledID<n_led; ledID++)
    // {
    //     std::vector<double> led_edges_list;
    //     std::vector<bool> led_bits_list;
    //     std::vector<int> vlc_sync_idx_list;
    //     std::vector<bool> vlc_msg_list;
    //     std::vector<int> hist_data_pos = std::vector<int>(int(num_hist_per_sec * time_len));
    //     std::vector<int> hist_data_neg = std::vector<int>(int(num_hist_per_sec * time_len));

    //     // Calculate histogram 
    //     double led_x, led_y;
    //     led_x = led_pos_list[ledID][0]; led_y = led_pos_list[ledID][1];
    //     start_time = event_list[0].time;
    //     for (int i=0; i<event_list.size(); i++)
    //     {
    //         Event event = event_list[i];

    //         if (led_x - led_r < event.x && led_x + led_r > event.x && led_y - led_r < event.y && led_y + led_r > event.y)
    //         {
    //             double time;
    //             time = event.time - start_time;
    //             if (time >= t_at) break;
    //             if (event.polarity == true) 
    //             {
    //                 hist_data_pos[int(std::floor(time/t_at*n_bins))]++;
    //             }
    //             else 
    //             {
    //                 hist_data_neg[int(std::floor(time/t_at*n_bins))]++;
    //             }
    //         }
    //     }

    //     // get edges
    //     bool flag = false;
    //     for (int i=0; i<time_bins.size(); i++)
    //     {
    //         // std::cout << hist_data_pos[i] << " ";
    //         if (hist_data_pos[i] > thres_for_edge_detection && flag==false)
    //         {
    //             led_edges_list.push_back(time_bins[i]);
    //             flag = true;
    //             continue;
    //         }
    //         else if (hist_data_pos[i] < thres_for_edge_detection && flag==true)
    //         {
    //             flag = false;
    //         }
    //     }
    //     std::cout << "edges:";
    //     for (int i=0; i<20; i++) std::cout << led_edges_list[i] << " ";
    //     std::cout << std::endl;        
    //     if (led_edges_list.size()==0) {success_encoding = false; continue;}

    //     // get bits from edges
    //     led_bits_list.push_back(true);
    //     double cur_time = led_edges_list[0];
    //     int cur_idx = 1;
    //     while (true)
    //     {
    //         cur_time += 1.0/freq;
    //         // while (led_edges_list[cur_idx] > cur_time - 0.5*tol_rate/freq) cur_idx++;
    //         if(led_edges_list[cur_idx] > cur_time - 0.5*tol_rate/freq && led_edges_list[cur_idx] < cur_time + 0.5*tol_rate/freq)
    //         {
    //             led_bits_list.push_back(true);
    //             // cur_time = led_edges_list[cur_idx];
    //             cur_idx += 1;
    //         }
    //         else
    //         {
    //             led_bits_list.push_back(false);
    //         }
    //         if (cur_idx >= led_edges_list.size() || cur_time > led_edges_list[led_edges_list.size()-1])
    //         {
    //             break;
    //         }
    //     }
    //     std::cout << "bits:";
    //     for (int i=0; i<led_bits_list.size(); i++) std::cout << led_bits_list[i];
    //     std::cout << std::endl;
    //     if (led_bits_list.size()==0) {success_encoding = false; continue;}

    //     // Get sync codes from bits
    //     int n_sync = sync.size();
    //     int n_bits = led_bits_list.size();
    //     for (int i; i<n_bits-n_sync; i++)
    //     {
    //         bool is_sync = true;
    //         for (int j; j<n_sync; j++)
    //         {
    //             is_sync = (is_sync==true) && (led_bits_list[i+j]*1 == sync[j]*1);
    //         }
    //         if (is_sync == true)
    //         {
    //             vlc_sync_idx_list.push_back(i);
    //         }
    //     }
    //     std::cout << "sync_idx num:" << vlc_sync_idx_list.size() << std::endl;
    //     if (vlc_sync_idx_list.size()==0) {success_encoding = false; continue;}

    //     // Read messages
    //     for (int i; i<vlc_sync_idx_list.size(); i++)
    //     {
    //         int sum = 0;
    //         if (vlc_sync_idx_list[i] + message_length >= led_bits_list.size()) break;
    //         for (int j; j<sync.size(); j++) if (sync[j]==true) sum++;
    //         for (int j; j<message_length; j++)
    //         {
    //             vlc_msg_list[j] = led_bits_list[vlc_sync_idx_list[i]+j];
    //             if (vlc_msg_list[j]) sum++;
    //         }
    //         if (led_bits_list[i+message_length]*1==sum%2) break;
    //     }

    //     // Update ps
    //     for (int msgID=0; msgID<n_led; msgID++)
    //     { 
    //         if (msgs[msgID] == vlc_msg_list) 
    //         {
    //             ps[msgID].push_back(led_x); ps[msgID].push_back(led_y);
    //             ROS_INFO("Detected LED ID %d, (x, y) = (%d, %d)", msgID, int(led_x), int(led_y));
    //         }
    //         else success_encoding = false;
    //     }
    // }
    if (led_pos_list[0][0] < led_pos_list[1][0]) // ledID=0 comes first 
    {
        ps[0].push_back(led_pos_list[0][0]); ps[1].push_back(led_pos_list[0][1]); 
        ps[1].push_back(led_pos_list[1][0]); ps[0].push_back(led_pos_list[1][1]); 
    }
    else // ledID=1 comes first 
    {
        ps[0].push_back(led_pos_list[1][0]); ps[0].push_back(led_pos_list[1][1]); 
        ps[1].push_back(led_pos_list[0][0]); ps[1].push_back(led_pos_list[0][1]); 
    }
    return success_encoding;
}

void DVSLocalization::load_data()
{
    std::ifstream infile_calibration("/home/koji/dvs/dvs_vlc/cfg/calibration.txt");
    infile_calibration >> K(0, 0) >> K(1, 1);
    infile_calibration >> K(0, 2) >> K(1, 2); 
    infile_calibration >> dist[0] >> dist[1] >> dist[2] >> dist[3] >> dist[4]; 

    std::ifstream infile_markers("/home/koji/dvs/dvs_vlc/cfg/markers.txt");
    for (int ledID=0; ledID<n_led; ledID++) {
        infile_markers >> Pws[ledID][0] >> Pws[ledID][1] >> Pws[ledID][2];
    }
}

void DVSLocalization::undistort()
{
    cv::Mat K_cv = (cv::Mat_<double>(3, 3) 
     << K(0, 0), K(0, 1), K(0, 2), 
        K(1, 0), K(1, 1), K(1, 2), 
        K(2, 0), K(2, 1), K(2, 2)
    );
    cv::Mat distCoeffs_cv = (
        cv::Mat_<double>(5, 1) << dist[0], dist[1], dist[2], dist[3], dist[4]
    );
    // std::cout << K_cv << " " << distCoeffs_cv << std::endl;
    for (int i=0; i<ps.size(); i++)
    {
        double data[] = {ps[i][0], ps[i][1]};
        // std::cout << data[0] << " " << data[1] << std::endl;
        std::vector<cv::Point2f> src_cv;
        src_cv.push_back(cv::Point2f(ps[i][0], ps[i][1]));

        std::vector<cv::Point2f> dst;
        // std::cout << "before:" << src_cv << " and " << dst << std::endl;
        cv::Mat R;
        cv::undistortPoints(src_cv, dst, K_cv, distCoeffs_cv, R, K_cv);
        // std::cout << "after:" << src_cv << " and " << dst << std::endl;
        ps_und[i].push_back(dst[0].x);
        ps_und[i].push_back(dst[0].y);
        ROS_INFO("undistorted: (x, y): (%d, %d) -> (%d, %d)", int(ps[i][0]), int(ps[i][1]), int(ps_und[i][0]), int(ps_und[i][1]));
    }
}

void DVSLocalization::solve_pnp()
{
    undistort();
    if (method=="xyzyaw")
    {
        ROS_INFO("Use xyzyaw method");
        Eigen::MatrixXd d_theta = Eigen::MatrixXd::Zero(3, 3);
        d_theta(0, 1) = -1; d_theta(1, 0) = 1;

        Eigen::MatrixXd P1 = Eigen::MatrixXd(3, 1);
        P1(0, 0) = Pws[0][0]; P1(1, 0) = Pws[0][1]; P1(2, 0) = Pws[0][2]; 

        Eigen::MatrixXd P2 = Eigen::MatrixXd(3, 1);
        P2(0, 0) = Pws[1][0]; P2(1, 0) = Pws[1][1]; P2(2, 0) = Pws[1][2]; 

        Eigen::MatrixXd p1 = Eigen::MatrixXd(3, 1);
        p1(0, 0) = ps[0][0]; p1(1, 0) = ps[0][1]; p1(2, 0) = 1;

        Eigen::MatrixXd p2 = Eigen::MatrixXd(3, 1);
        p2(0, 0) = ps[1][0]; p2(1, 0) = ps[1][1]; p2(2, 0) = 1;

        Eigen::MatrixXd A = Eigen::MatrixXd(6, 6);
        A.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
        A.block<3, 3>(3, 0) = Eigen::MatrixXd::Identity(3, 3);
        A.block<3, 1>(0, 3) = d_theta * P1;
        A.block<3, 1>(0, 4) = -K.inverse() * p1;
        A.block<3, 1>(0, 5) = Eigen::MatrixXd::Zero(3, 1);
        A.block<3, 1>(3, 3) = d_theta * P2;
        A.block<3, 1>(3, 4) = Eigen::MatrixXd::Zero(3, 1);
        A.block<3, 1>(3, 5) = -K.inverse() * p2;

        Eigen::MatrixXd b = Eigen::MatrixXd(6, 1);
        b(0, 0) = -Pws[0][0]; b(1, 0) = -Pws[0][1]; b(2, 0) = -Pws[0][2]; 
        b(3, 0) = -Pws[1][0]; b(4, 0) = -Pws[1][1]; b(5, 0) = -Pws[1][2]; 

        Eigen::MatrixXd ans = A.inverse() * b;

        // std::cout << "A:" << A << ",b:" << b << ",ans:" << ans << std::endl;

        R(0, 0) = std::cos(ans(3, 0)); R(0, 1) = -std::sin(ans(3, 0)); 
        R(1, 0) = std::sin(ans(3, 0)); R(1, 1) = std::cos(ans(3, 0)); 
        T = ans.block<3, 1>(0, 0);
        ROS_INFO("PnP Solved. (x, y, z) = (%f, %f, %f)", ans(0, 0), ans(1, 0), ans(2, 0));
    }
    else if (method=="xyz")
    {
        // Eigen::MatrixXd A = Eigen::MatrixXd(6, 5);
        // A.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
        // A.block<3, 3>(3, 0) = Eigen::MatrixXd::Identity(3, 3);
        // A.block<3, 1>(0, 3) = ;
        // A.block<3, 1>(0, 4) = Eigen::MatrixXd::Zero(3, 1);
        // A.block<3, 1>(3, 3) = Eigen::MatrixXd::Zero(3, 1);
        // A.block<3, 1>(3, 4) = ;

        // Eigen::MatrixXd b = Eigen::MatrixXd(6, 1);
        // b(0, 0) = -Pws[0][0]; b(1, 0) = -Pws[0][1]; b(2, 0) = -Pws[0][2]; 
        // b(3, 0) = -Pws[1][0]; b(4, 0) = -Pws[1][1]; b(5, 0) = -Pws[1][2]; 

        // Eigen::MatrixXd ans = A.inverse() * b;
        
        // R(0, 0) = std::cos(ans(3, 0)); R(0, 1) = -std::sin(ans(3, 0)); 
        // R(1, 0) = std::sin(ans(3, 0)); R(1, 1) = std::cos(ans(3, 0)); 
        // T = ans.block<3, 1>(0, 0);
    }
}

void DVSLocalization::publish_odom()
{
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp.sec = event_list[event_list.size()].ts_sec;
    odom_msg.header.stamp.nsec = event_list[event_list.size()].ts_nsec;

    odom_msg.pose.pose.position.x = T(0, 0);
    odom_msg.pose.pose.position.y = T(1, 0);
    odom_msg.pose.pose.position.z = T(2, 0);
    
    // odom_msg.pose.pose.quaternion.x = ;
    // odom_msg.pose.pose.quaternion.y = ;
    // odom_msg.pose.pose.quaternion.z = ;
    // odom_msg.pose.pose.quaternion.w = ; 
    if (T(1, 0)<3 && T(1, 0)>-3) odom_publisher_.publish(odom_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dvs_vlc");
    ros::NodeHandle nodeHandle("~");
    DVSLocalization DVSLocalization(nodeHandle);
    ros::spin();
    return 0;
}