// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 2 headers
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcpputils/filesystem_helper.hpp"

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);
    declare_parameter<double>("max_range", config_.max_range);
    declare_parameter<double>("min_range", config_.min_range);
    declare_parameter<bool>("deskew", config_.deskew);
    declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    declare_parameter<double>("map_cleanup_radius", config_.map_cleanup_radius);
    declare_parameter<double>("planarity_threshold", config_.planarity_threshold);
    declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    declare_parameter<int>("desired_num_voxelized_points", config_.desired_num_voxelized_points);
    declare_parameter<int>("max_num_iterations", config_.max_num_iterations);
    declare_parameter<double>("convergence_criterion", config_.convergence_criterion);
    declare_parameter<double>("initial_threshold", config_.initial_threshold);
    declare_parameter<double>("min_motion_th", config_.min_motion_th);
    declare_parameter<std::string>("config_file", "");

    // Load the configuration file
    std::string config_file = get_parameter("config_file").as_string();
    if (!config_file.empty()) {
        rcpputils::fs::path path(config_file);
        if (!path.is_absolute()) {
            path = rcpputils::fs::path(ament_index_cpp::get_package_share_directory("genz_icp")) / "config" / path;
        }
        YAML::Node yaml = YAML::LoadFile(path.string());

        std::vector<rclcpp::Parameter> overrides;
        for (const auto &param : yaml) {
            const auto &name = param.first.as<std::string>();
            const auto &value = param.second;

            if (value.IsScalar()) {
                // Get the declared type of the parameter
                rcl_interfaces::msg::ParameterDescriptor descriptor;
                descriptor = this->describe_parameter(name);

                using ParamType = rcl_interfaces::msg::ParameterType;
                switch (descriptor.type) {
                    case ParamType::PARAMETER_DOUBLE:
                        overrides.emplace_back(name, value.as<double>());
                        break;
                    case ParamType::PARAMETER_INTEGER:
                        overrides.emplace_back(name, value.as<int>());
                        break;
                    case ParamType::PARAMETER_BOOL:
                        overrides.emplace_back(name, value.as<bool>());
                        break;
                    case ParamType::PARAMETER_STRING:
                        overrides.emplace_back(name, value.as<std::string>());
                        break;
                    default:
                        break;
                }
            }
        }
        set_parameters(overrides);
    }

    config_.max_range = get_parameter("max_range").as_double();
    config_.min_range = get_parameter("min_range").as_double();
    config_.deskew = get_parameter("deskew").as_bool();
    config_.voxel_size = get_parameter("voxel_size").as_double();
    config_.map_cleanup_radius = get_parameter("map_cleanup_radius").as_double();
    config_.planarity_threshold = get_parameter("planarity_threshold").as_double();
    config_.max_points_per_voxel = get_parameter("max_points_per_voxel").as_int();
    config_.desired_num_voxelized_points = get_parameter("desired_num_voxelized_points").as_int();
    config_.max_num_iterations = get_parameter("max_num_iterations").as_int();
    config_.convergence_criterion = get_parameter("convergence_criterion").as_double();
    config_.initial_threshold = get_parameter("initial_threshold").as_double();
    config_.min_motion_th = get_parameter("min_motion_th").as_double();
    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    // clang-format on

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud_topic", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/genz/odometry", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/genz/trajectory", qos);
    path_msg_.header.frame_id = odom_frame_;
    if (publish_debug_clouds_) {
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/local_map", qos);
        planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/planar_points", qos);
        non_planar_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/genz/non_planar_points", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "GenZ-ICP ROS 2 odometry node initialized");
}

std::optional<Sophus::SE3d> OdometryServer::LookupTransform(
    const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &stamp) const {
    try {
        return tf2::transformToSophus(tf2_buffer_->lookupTransform(target_frame, source_frame, stamp));
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Skipping cloud: failed to find tf from %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    // Resolve the extrinsic at the scan time before updating the ICP state. A
    // missing transform must not turn a LiDAR pose into a mislabeled base pose.
    Sophus::SE3d cloud_to_base;
    if (!base_frame_.empty() && base_frame_ != cloud_frame_id) {
        const auto transform = LookupTransform(base_frame_, cloud_frame_id, msg->header.stamp);
        if (!transform) return;
        cloud_to_base = *transform;
    }

    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();

    // Register frame, main entry point to GenZ-ICP pipeline
    const auto &[planar_points, non_planar_points] = odometry_.RegisterFrame(points, timestamps);

    // P = T_L0_Lt. Anchor odom at the first accepted output frame B0:
    // T_B0_Bt = T_B0_L0 * P * T_Lt_Bt. For a rigid mounting this is X * P * X^-1.
    // Keep the left transform fixed even when base-to-LiDAR TF changes over time.
    if (!map_to_odom_) map_to_odom_ = cloud_to_base;
    const Sophus::SE3d genz_pose = odometry_.poses().back();
    const Sophus::SE3d pose = *map_to_odom_ * genz_pose * cloud_to_base.inverse();

    // Spit the current estimated pose to ROS msgs
    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);
    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(msg->header.stamp, *map_to_odom_, planar_points, non_planar_points);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Broadcast the tf ---
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(const rclcpp::Time &stamp,
                                   const Sophus::SE3d &map_to_odom,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    // The map and both correspondence clouds are already in the internal map
    // frame L0. Convert all three to odom, independently of TF broadcasting.
    const auto genz_map = odometry_.LocalMap();
    map_publisher_->publish(std::move(EigenToPointCloud2(genz_map, map_to_odom, odom_header)));
    planar_points_publisher_->publish(std::move(EigenToPointCloud2(planar_points, map_to_odom, odom_header)));
    non_planar_points_publisher_->publish(std::move(EigenToPointCloud2(non_planar_points, map_to_odom, odom_header)));
}

}  // namespace genz_icp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(genz_icp_ros::OdometryServer)
