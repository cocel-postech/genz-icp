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
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/Threshold.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"
#include "genz_icp/metrics/Metrics.hpp"
#include "genz_icp/pipeline/GenZICP.hpp"
#include "stl_vector_eigen.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

namespace genz_icp {
namespace {

struct Preprocessor {
    explicit Preprocessor(double max_range, double min_range, bool deskew, int max_num_threads)
        : max_range_(max_range),
          min_range_(min_range),
          deskew_(deskew),
          max_num_threads_(max_num_threads) {}

    std::vector<Eigen::Vector3d> Preprocess(const std::vector<Eigen::Vector3d> &points,
                                            const std::vector<double> &timestamps,
                                            const Sophus::SE3d &relative_motion) const {
        auto processed = genz_icp::Preprocess(points, max_range_, min_range_);
        if (!deskew_ || timestamps.empty()) {
            return processed;
        }
        (void)max_num_threads_;
        return genz_icp::DeSkewScan(processed, timestamps, Sophus::SE3d(), relative_motion);
    }

    double max_range_;
    double min_range_;
    bool deskew_;
    int max_num_threads_;
};

struct RegistrationWrapper {
    explicit RegistrationWrapper(int max_num_iterations, double convergence_criterion,
                                 int max_num_threads)
        : registration_(max_num_iterations, convergence_criterion),
          max_num_threads_(max_num_threads) {}

    Eigen::Matrix4d AlignPointsToMap(const std::vector<Eigen::Vector3d> &points,
                                     const VoxelHashMap &voxel_map,
                                     const Eigen::Matrix4d &initial_guess,
                                     double max_correspondence_distance,
                                     double kernel) {
        (void)max_num_threads_;
        Sophus::SE3d initial(initial_guess);
        const auto &[pose, planar_points, non_planar_points] =
            registration_.RegisterFrame(points, voxel_map, initial, max_correspondence_distance,
                                        kernel);
        (void)planar_points;
        (void)non_planar_points;
        return pose.matrix();
    }

    Registration registration_;
    int max_num_threads_;
};

struct GenZICPWrapper {
    explicit GenZICPWrapper(const genz_icp::pipeline::GenZConfig &config) : odometry_(config) {}

    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> RegisterFrame(
        const std::vector<Eigen::Vector3d> &points) {
        return odometry_.RegisterFrame(points);
    }

    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> RegisterFrame(
        const std::vector<Eigen::Vector3d> &points, const std::vector<double> &timestamps) {
        return odometry_.RegisterFrame(points, timestamps);
    }

    Eigen::Matrix4d LastPose() const {
        const auto poses = odometry_.poses();
        return poses.empty() ? Eigen::Matrix4d::Identity() : poses.back().matrix();
    }

    void SetTerminalStatusEnabled(bool enabled) { odometry_.SetTerminalStatusEnabled(enabled); }

    std::vector<Eigen::Vector3d> LocalMap() const { return odometry_.LocalMap(); }

    genz_icp::pipeline::GenZICP odometry_;
};

}  // namespace

PYBIND11_MODULE(genz_icp_pybind, m) {
    pybind_eigen_vector_of_vector<Eigen::Vector3d>(
        m, "_Vector3dVector", "std::vector<Eigen::Vector3d>",
        py::py_array_to_vectors_double<Eigen::Vector3d>);

    py::class_<genz_icp::pipeline::GenZConfig>(m, "_GenZConfig")
        .def(py::init<>())
        .def_readwrite("max_range", &genz_icp::pipeline::GenZConfig::max_range)
        .def_readwrite("min_range", &genz_icp::pipeline::GenZConfig::min_range)
        .def_readwrite("map_cleanup_radius", &genz_icp::pipeline::GenZConfig::map_cleanup_radius)
        .def_readwrite("max_points_per_voxel", &genz_icp::pipeline::GenZConfig::max_points_per_voxel)
        .def_readwrite("voxel_size", &genz_icp::pipeline::GenZConfig::voxel_size)
        .def_readwrite("desired_num_voxelized_points", &genz_icp::pipeline::GenZConfig::desired_num_voxelized_points)
        .def_readwrite("min_motion_th", &genz_icp::pipeline::GenZConfig::min_motion_th)
        .def_readwrite("initial_threshold", &genz_icp::pipeline::GenZConfig::initial_threshold)
        .def_readwrite("planarity_threshold", &genz_icp::pipeline::GenZConfig::planarity_threshold)
        .def_readwrite("deskew", &genz_icp::pipeline::GenZConfig::deskew)
        .def_readwrite("max_num_iterations", &genz_icp::pipeline::GenZConfig::max_num_iterations)
        .def_readwrite("convergence_criterion", &genz_icp::pipeline::GenZConfig::convergence_criterion);

    py::class_<GenZICPWrapper>(m, "_GenZICP")
        .def(py::init<const genz_icp::pipeline::GenZConfig &>(), "config"_a)
        .def("_register_frame",
             py::overload_cast<const std::vector<Eigen::Vector3d> &>(
                 &GenZICPWrapper::RegisterFrame),
             "points"_a)
        .def("_register_frame",
             py::overload_cast<const std::vector<Eigen::Vector3d> &,
                               const std::vector<double> &>(&GenZICPWrapper::RegisterFrame),
             "points"_a, "timestamps"_a)
        .def("_set_terminal_status_enabled", &GenZICPWrapper::SetTerminalStatusEnabled,
             "enabled"_a)
        .def("_last_pose", &GenZICPWrapper::LastPose)
        .def("_local_map", &GenZICPWrapper::LocalMap);

    // Map representation
    py::class_<VoxelHashMap> internal_map(m, "_VoxelHashMap", "Don't use this");
    internal_map
        .def(
            py::init([](double voxel_size, double max_distance, int max_points_per_voxel) {
                return VoxelHashMap(voxel_size, max_distance, max_distance, 0.2,
                                    max_points_per_voxel);
            }),
            "voxel_size"_a, "max_distance"_a, "max_points_per_voxel"_a)
        .def(py::init<double, double, double, double, int>(), "voxel_size"_a,
             "max_distance"_a, "map_cleanup_radius"_a, "planarity_threshold"_a,
             "max_points_per_voxel"_a)
        .def("_clear", &VoxelHashMap::Clear)
        .def("_empty", &VoxelHashMap::Empty)
        .def("_update",
             py::overload_cast<const std::vector<Eigen::Vector3d> &, const Eigen::Vector3d &>(
                 &VoxelHashMap::Update),
             "points"_a, "origin"_a)
        .def(
            "_update",
            [](VoxelHashMap &self, const std::vector<Eigen::Vector3d> &points,
               const Eigen::Matrix4d &T) {
                Sophus::SE3d pose(T);
                self.Update(points, pose);
            },
            "points"_a, "pose"_a)
        .def("_add_points", &VoxelHashMap::AddPoints, "points"_a)
        .def("_remove_far_away_points", &VoxelHashMap::RemovePointsFarFromLocation, "origin"_a)
        .def("_point_cloud", &VoxelHashMap::Pointcloud);

    py::class_<Preprocessor>(m, "_Preprocessor", "Don't use this")
        .def(py::init<double, double, bool, int>(), "max_range"_a, "min_range"_a, "deskew"_a,
             "max_num_threads"_a)
        .def(
            "_preprocess",
            [](Preprocessor &self, const std::vector<Eigen::Vector3d> &points,
               const std::vector<double> &timestamps, const Eigen::Matrix4d &relative_motion) {
                Sophus::SE3d motion(relative_motion);
                return self.Preprocess(points, timestamps, motion);
            },
            "points"_a, "timestamps"_a, "relative_motion"_a);

    py::class_<RegistrationWrapper>(m, "_Registration", "Don't use this")
        .def(py::init<int, double, int>(), "max_num_iterations"_a, "convergence_criterion"_a,
             "max_num_threads"_a)
        .def("_align_points_to_map", &RegistrationWrapper::AlignPointsToMap, "points"_a,
             "voxel_map"_a, "initial_guess"_a, "max_correspondance_distance"_a,
             "kernel"_a);

    py::class_<AdaptiveThreshold>(m, "_AdaptiveThreshold", "Don't use this")
        .def(py::init<double, double, double>(), "initial_threshold"_a, "min_motion_th"_a,
             "max_range"_a)
        .def("_compute_threshold", &AdaptiveThreshold::ComputeThreshold)
        .def(
            "_update_model_deviation",
            [](AdaptiveThreshold &self, const Eigen::Matrix4d &T) {
                Sophus::SE3d model_deviation(T);
                self.UpdateModelDeviation(model_deviation);
            },
            "model_deviation"_a);

    m.def("_voxel_down_sample", &VoxelDownsample, "frame"_a, "voxel_size"_a);
    m.def("_correct_kitti_scan", &CorrectKITTIScan, "frame"_a);

    // Metrics
    m.def("_kitti_seq_error", &metrics::SeqError, "gt_poses"_a, "results_poses"_a);
    m.def("_absolute_trajectory_error", &metrics::AbsoluteTrajectoryError, "gt_poses"_a,
          "results_poses"_a);
}

}  // namespace genz_icp
