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
#include <utility>
#include <vector>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 1 headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf2_listener_(tf2_ros::TransformListener(tf2_buffer_)) {
    pnh_.param("base_frame", base_frame_, base_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    pnh_.param("publish_odom_tf", publish_odom_tf_, false);
    pnh_.param("visualize", publish_debug_clouds_, publish_debug_clouds_);
    pnh_.param("max_range", config_.max_range, config_.max_range);
    pnh_.param("min_range", config_.min_range, config_.min_range);
    pnh_.param("deskew", config_.deskew, config_.deskew);
    pnh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    pnh_.param("map_cleanup_radius", config_.map_cleanup_radius, config_.max_range);
    pnh_.param("planarity_threshold", config_.planarity_threshold, config_.planarity_threshold);
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    pnh_.param("desired_num_voxelized_points", config_.desired_num_voxelized_points, config_.desired_num_voxelized_points);
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    pnh_.param("max_num_iterations", config_.max_num_iterations, config_.max_num_iterations);
    pnh_.param("convergence_criterion", config_.convergence_criterion, config_.convergence_criterion);
    if (config_.max_range < config_.min_range) {
        ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud_topic", queue_size_,
                                                              &OdometryServer::RegisterFrame, this);

    // Initialize publishers
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/genz/odometry", queue_size_);
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/genz/trajectory", queue_size_);
    if (publish_debug_clouds_) {
        map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/local_map", queue_size_);
        planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/planar_points", queue_size_);
        non_planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/non_planar_points", queue_size_);
    }
    // Initialize the transform buffer
    tf2_buffer_.setUsingDedicatedThread(true);
    path_msg_.header.frame_id = odom_frame_;

    // publish odometry msg
    ROS_INFO("GenZ-ICP ROS 1 Odometry Node Initialized");
}

std::optional<Sophus::SE3d> OdometryServer::LookupTransform(
    const std::string &target_frame, const std::string &source_frame, const ros::Time &stamp) const {
    try {
        return tf2::transformToSophus(tf2_buffer_.lookupTransform(target_frame, source_frame, stamp));
    } catch (const tf2::TransformException &ex) {
        ROS_WARN("Skipping cloud: failed to find tf from %s to %s: %s",
                 source_frame.c_str(), target_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg) {
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
                                     const ros::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Header for point clouds and stuff seen from desired odom_frame

    // Broadcast the tf
    if (publish_odom_tf_) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_.sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_.publish(path_msg_);

    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_.publish(odom_msg);
}

void OdometryServer::PublishClouds(const ros::Time &stamp,
                                   const Sophus::SE3d &map_to_odom,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    std_msgs::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    // The map and both correspondence clouds are already in the internal map
    // frame L0. Convert all three to odom, independently of TF broadcasting.
    const auto genz_map = odometry_.LocalMap();
    map_publisher_.publish(*EigenToPointCloud2(genz_map, map_to_odom, odom_header));
    planar_points_publisher_.publish(*EigenToPointCloud2(planar_points, map_to_odom, odom_header));
    non_planar_points_publisher_.publish(*EigenToPointCloud2(non_planar_points, map_to_odom, odom_header));
}

}  // namespace genz_icp_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "genz_icp");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    genz_icp_ros::OdometryServer node(nh, nh_private);

    ros::spin();

    return 0;
}
