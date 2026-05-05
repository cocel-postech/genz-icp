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


def get_voxel_hash_map(config: GenZConfig):
    return VoxelHashMap(
        voxel_size=config.mapping.voxel_size,
        max_distance=config.data.max_range,
        map_cleanup_radius=config.mapping.map_cleanup_radius,
        planarity_threshold=config.mapping.planarity_threshold,
        max_points_per_voxel=config.mapping.max_points_per_voxel,
    )


class VoxelHashMap:
    def __init__(
        self,
        voxel_size: float,
        max_distance: float,
        map_cleanup_radius: float,
        planarity_threshold: float,
        max_points_per_voxel: int,
    ):
        self._internal_map = genz_icp_pybind._VoxelHashMap(
            voxel_size=voxel_size,
            max_distance=max_distance,
            map_cleanup_radius=map_cleanup_radius,
            planarity_threshold=planarity_threshold,
            max_points_per_voxel=max_points_per_voxel,
        )

    def clear(self):
        return self._internal_map._clear()

    def empty(self):
        return self._internal_map._empty()

    def update(self, points: np.ndarray, pose: np.ndarray = np.eye(4)):
        """Add points to the inernal map representaion.

        The origin is needed to remove the far away poitns

        TODO(NACHO): Use similar overload API as we did for VDBFusion
        """
        self._internal_map._update(genz_icp_pybind._Vector3dVector(points), pose)

    def add_points(self, points):
        self._internal_map._add_points(genz_icp_pybind._Vector3dVector(points))

    def remove_far_away_points(self, origin):
        self._internal_map._remove_far_away_points(origin)

    def point_cloud(self) -> np.ndarray:
        """Return the internal representaion as a np.array (pointcloud)."""
        return np.asarray(self._internal_map._point_cloud())
