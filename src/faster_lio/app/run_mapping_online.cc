//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "laser_mapping.h"

DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

std::shared_ptr<rclcpp::Node> global_node;

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    RCLCPP_WARN(global_node->get_logger(), "Caught signal %d", sig);
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);
    global_node = std::make_shared<rclcpp::Node>("faster_lio_online");

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS2();

    signal(SIGINT, SigHandle);

    rclcpp::Rate rate(5000);  // Hz

    RCLCPP_INFO(global_node->get_logger(), "Starting Faster-LIO mapping online node...");

    while (rclcpp::ok() && !faster_lio::options::FLAG_EXIT) {
        rclcpp::spin_some(global_node);
        laser_mapping->Run();
        rate.sleep();
    }

    RCLCPP_INFO(global_node->get_logger(), "Finishing mapping...");
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();

    RCLCPP_INFO(global_node->get_logger(), "Saving trajectory to: %s", FLAGS_traj_log_file.c_str());
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    rclcpp::shutdown();
    return 0;
}
