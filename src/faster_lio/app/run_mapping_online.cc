#include <unistd.h>
#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "laser_mapping.h"

std::shared_ptr<faster_lio::LaserMapping> laser_mapping;

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    if (laser_mapping != nullptr) {
        RCLCPP_WARN(laser_mapping->get_logger(), "Caught signal %d", sig);
    } else {
        std::cerr << "Caught signal " << sig << std::endl;
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // FLAGS_stderrthreshold = google::INFO;
    // FLAGS_colorlogtostderr = true;
    // google::InitGoogleLogging(argv[0]);
    // google::ParseCommandLineFlags(&argc, &argv, true);

    laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS2();

    signal(SIGINT, SigHandle);

    rclcpp::Rate rate(5000);  // Hz

    RCLCPP_INFO(laser_mapping->get_logger(), "Starting Faster-LIO mapping online node...");

    while (rclcpp::ok() && !faster_lio::options::FLAG_EXIT) {
        rclcpp::spin_some(laser_mapping->get_node_base_interface());
        laser_mapping->Run();
        rate.sleep();
    }

    RCLCPP_INFO(laser_mapping->get_logger(), "Finishing mapping...");
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();

    std::string traj_log_file = "Log/traj.txt";
    RCLCPP_INFO(laser_mapping->get_logger(), "Saving trajectory to: %s", traj_log_file.c_str());
    laser_mapping->Savetrajectory(traj_log_file);
    laser_mapping.reset();

    rclcpp::shutdown();
    return 0;
}
