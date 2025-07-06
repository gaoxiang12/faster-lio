#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/time.h>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "laser_mapping.h"
#include "utils.h"
#include "faster_lio_interfaces/msg/custom_msg.hpp"

DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
DEFINE_string(bag_file, "/home/xiang/Data/dataset/fast_lio2/avia/2020-09-16-quick-shack.db3", "path to the ROS 2 bag");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

std::shared_ptr<rclcpp::Node> global_node;

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    RCLCPP_WARN(global_node->get_logger(), "Caught signal %d", sig);
}

template <typename MsgT>
std::shared_ptr<MsgT> deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr &msg) {
    static rclcpp::Serialization<MsgT> serializer;
    auto ros_msg = std::make_shared<MsgT>();
    rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
    serializer.deserialize_message(&extracted_serialized_msg, ros_msg.get());
    return ros_msg;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    rclcpp::init(argc, argv);

    global_node = std::make_shared<rclcpp::Node>("faster_lio_offline");

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        RCLCPP_ERROR(global_node->get_logger(), "Laser mapping init failed.");
        return -1;
    }

    signal(SIGINT, SigHandle);

    rosbag2_cpp::Reader reader;
    reader.open(FLAGS_bag_file);

    RCLCPP_INFO(global_node->get_logger(), "Opened bag: %s", FLAGS_bag_file.c_str());

    while (reader.has_next() && !faster_lio::options::FLAG_EXIT) {
        auto msg = reader.read_next();

        const std::string &topic = msg->topic_name;

        if (topic.find("livox") != std::string::npos || topic.find("custom") != std::string::npos) {
            auto livox_msg = deserialize<faster_lio_interfaces::msg::CustomMsg>(msg);
            faster_lio::Timer::Evaluate(
                [&]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
        } else if (topic.find("pointcloud") != std::string::npos || topic.find("velodyne") != std::string::npos) {
            auto pcl_msg = deserialize<sensor_msgs::msg::PointCloud2>(msg);
            faster_lio::Timer::Evaluate(
                [&]() {
                    laser_mapping->StandardPCLCallBack(pcl_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
        } else if (topic.find("imu") != std::string::npos) {
            auto imu_msg = deserialize<sensor_msgs::msg::Imu>(msg);
            laser_mapping->IMUCallBack(imu_msg);
        }
    }

    RCLCPP_INFO(global_node->get_logger(), "Finishing mapping...");
    laser_mapping->Finish();

    double fps = 1.0 / (faster_lio::Timer::GetMeanTime("Laser Mapping Single Run") / 1000.0);
    RCLCPP_INFO(global_node->get_logger(), "Faster LIO average FPS: %.2f", fps);

    RCLCPP_INFO(global_node->get_logger(), "Saving trajectory to: %s", FLAGS_traj_log_file.c_str());
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    faster_lio::Timer::PrintAll();
    faster_lio::Timer::DumpIntoFile(FLAGS_time_log_file);

    rclcpp::shutdown();
    return 0;
}
