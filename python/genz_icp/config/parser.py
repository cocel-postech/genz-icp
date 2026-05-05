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
# NOTE: This module was contributed by Markus Pielmeier on PR #63
from __future__ import annotations

import importlib
import importlib.resources
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from pydantic_settings import BaseSettings, SettingsConfigDict

from genz_icp.config.config import (
    AdaptiveThresholdConfig,
    DataConfig,
    MappingConfig,
    RegistrationConfig,
)


class GenZConfig(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="genz_icp_")
    out_dir: str = "results"
    data: DataConfig = DataConfig()
    registration: RegistrationConfig = RegistrationConfig()
    mapping: MappingConfig = MappingConfig()
    adaptive_threshold: AdaptiveThresholdConfig = AdaptiveThresholdConfig()


def _resolve_config_file(config_file: Path) -> Path:
    """Resolve config file path from user path or packaged pre-tuned configs."""
    if config_file.exists():
        return config_file

    candidate_names = [config_file.name]
    if config_file.suffix == "":
        candidate_names.append(f"{config_file.name}.yaml")

    try:
        package_root = importlib.resources.files("genz_icp.config")
        search_roots = [package_root.joinpath("pretuned"), package_root.joinpath("presets")]
        for root in search_roots:
            for name in candidate_names:
                candidate = root.joinpath(name)
                if candidate.is_file():
                    return Path(str(candidate))
    except ModuleNotFoundError:
        pass

    return config_file


def _normalize_config_schema(data: Dict[str, Any]) -> Dict[str, Any]:
    """Support both nested Python config schema and flat ROS config schema."""
    normalized: Dict[str, Any] = dict(data)

    # ROS config files use top-level flat keys while Python API expects nested sections.
    flat_to_nested = {
        "deskew": ("data", "deskew"),
        "max_range": ("data", "max_range"),
        "min_range": ("data", "min_range"),
        "voxel_size": ("mapping", "voxel_size"),
        "map_cleanup_radius": ("mapping", "map_cleanup_radius"),
        "desired_num_voxelized_points": ("mapping", "desired_num_voxelized_points"),
        "planarity_threshold": ("mapping", "planarity_threshold"),
        "max_points_per_voxel": ("mapping", "max_points_per_voxel"),
        "initial_threshold": ("adaptive_threshold", "initial_threshold"),
        "min_motion_th": ("adaptive_threshold", "min_motion_th"),
        "max_num_iterations": ("registration", "max_num_iterations"),
        "convergence_criterion": ("registration", "convergence_criterion"),
    }

    for flat_key, (section_name, nested_key) in flat_to_nested.items():
        if flat_key not in normalized:
            continue

        value = normalized.pop(flat_key)
        section = normalized.get(section_name)
        if not isinstance(section, dict):
            section = {}
            normalized[section_name] = section

        # Nested schema has priority when both representations are provided.
        section.setdefault(nested_key, value)

    return normalized


def _yaml_source(config_file: Optional[Path]) -> Dict[str, Any]:
    data = None
    if config_file is not None:
        resolved_config_file = _resolve_config_file(config_file)
        try:
            yaml = importlib.import_module("yaml")
        except ModuleNotFoundError:
            print(
                "Custom configuration file specified but PyYAML is not installed on your system,"
                ' run `pip install "genz-icp[all]"`. You can also modify the config.py if your '
                "system does not support PyYaml "
            )
            sys.exit(1)
        with open(resolved_config_file) as cfg_file:
            data = yaml.safe_load(cfg_file)
    return _normalize_config_schema(data or {})


def load_config(config_file: Optional[Path]) -> GenZConfig:
    """Load configuration from an optional yaml file."""

    config = GenZConfig(**_yaml_source(config_file))

    # Check if there is a possible mistake
    if config.data.max_range < config.data.min_range:
        print("[WARNING] max_range is smaller than min_range, settng min_range to 0.0")
        config.data.min_range = 0.0

    # Use specified voxel size or compute one using the max range
    if config.mapping.voxel_size is None:
        config.mapping.voxel_size = float(config.data.max_range / 100.0)

    return config


def write_config(config: GenZConfig = GenZConfig(), filename: str = "genz_icp.yaml"):
    with open(filename, "w") as outfile:
        try:
            yaml = importlib.import_module("yaml")
            yaml.dump(config.model_dump(), outfile, default_flow_style=False)
        except ModuleNotFoundError:
            outfile.write(str(config.model_dump()))
