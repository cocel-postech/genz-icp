# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
# Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import numpy as np

from genz_icp.config import GenZConfig
from genz_icp.pybind import genz_icp_pybind


class _LocalMapProxy:
    def __init__(self, odometry):
        self._odometry = odometry

    def point_cloud(self) -> np.ndarray:
        return np.asarray(self._odometry._local_map())


class GenZICP:
    def __init__(self, config: GenZConfig):
        self.config = config
        self.last_pose = np.eye(4)

        native_config = genz_icp_pybind._GenZConfig()
        native_config.max_range = self.config.data.max_range
        native_config.min_range = self.config.data.min_range
        native_config.deskew = self.config.data.deskew
        native_config.voxel_size = self.config.mapping.voxel_size
        native_config.map_cleanup_radius = self.config.mapping.map_cleanup_radius
        native_config.desired_num_voxelized_points = self.config.mapping.desired_num_voxelized_points
        native_config.planarity_threshold = self.config.mapping.planarity_threshold
        native_config.max_points_per_voxel = self.config.mapping.max_points_per_voxel
        native_config.max_num_iterations = self.config.registration.max_num_iterations
        native_config.convergence_criterion = self.config.registration.convergence_criterion
        native_config.initial_threshold = self.config.adaptive_threshold.initial_threshold
        native_config.min_motion_th = self.config.adaptive_threshold.min_motion_th

        self._odometry = genz_icp_pybind._GenZICP(native_config)
        # Python pipeline renders status in the visualizer info panel instead of terminal.
        self._odometry._set_terminal_status_enabled(False)
        self.local_map = _LocalMapProxy(self._odometry)

    @staticmethod
    def _to_local_frame(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        points_h = np.hstack((points, np.ones((points.shape[0], 1))))
        points_local_h = (np.linalg.inv(pose) @ points_h.T).T
        return points_local_h[:, :3]

    def register_frame(self, frame, timestamps):
        frame = np.asarray(frame)
        if timestamps is not None and len(timestamps) > 0:
            planar_points, non_planar_points = self._odometry._register_frame(
                genz_icp_pybind._Vector3dVector(frame),
                np.asarray(timestamps).ravel(),
            )
        else:
            planar_points, non_planar_points = self._odometry._register_frame(
                genz_icp_pybind._Vector3dVector(frame)
            )

        self.last_pose = np.asarray(self._odometry._last_pose())
        planar_points = np.asarray(planar_points)
        non_planar_points = np.asarray(non_planar_points)

        # GenZ C++ returns these sets in the global frame.
        # Visualizer expects local/body-frame points and applies pose transform by itself.
        planar_points = self._to_local_frame(planar_points, self.last_pose)
        non_planar_points = self._to_local_frame(non_planar_points, self.last_pose)

        # On the very first frame (or when correspondences are temporarily empty),
        # Registration can return no planar/non-planar points. Fallback to raw scan
        # so users still see the current scan in the visualizer.
        if planar_points.size == 0 and non_planar_points.size == 0:
            non_planar_points = frame

        return non_planar_points, planar_points
