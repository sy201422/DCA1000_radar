import json
from copy import deepcopy
from pathlib import Path


DEFAULT_SETTINGS = {
    "config_path": "config/profile_3d.cfg",
    "cli_port": "COM5",
    "cli_baudrate": 115200,
    "network": {
        "host_ip": "192.168.33.30",
        "data_port": 4098,
        "config_port": 4096,
        "fpga_ip": "192.168.33.180",
        "fpga_port": 4096,
        "buffer_size": 2097152,
    },
    "processing": {
        "remove_static": True,
        "doppler_guard_bins": 2,
    },
    "roi": {
        "lateral_m": 1.5,
        "forward_m": 3.0,
        "min_forward_m": 0.25,
    },
    "detection": {
        "allow_strongest_fallback": False,
        "max_targets": 6,
        "display_min_confidence": 0.22,
        "cluster_min_samples": 1,
        "cluster_velocity_weight": 0.0,
        "dbscan_adaptive_eps_bands": [
            {"r_min": 0.25, "r_max": 1.0, "eps": 0.34, "min_samples": 1},
            {"r_min": 1.0, "r_max": 2.0, "eps": 0.44, "min_samples": 1},
            {"r_min": 2.0, "r_max": 3.5, "eps": 0.56, "min_samples": 1},
        ],
    },
    "tracking": {
        "confirm_hits": 3,
        "max_misses": 4,
        "process_var": 1.0,
        "measurement_var": 0.43,
        "range_measurement_scale": 0.50,
        "confidence_measurement_scale": 0.35,
        "association_gate": 5.99,
        "doppler_zero_guard_bins": 3,
        "doppler_gate_bins": 18,
        "doppler_cost_weight": 0.65,
        "report_miss_tolerance": 1,
        "lost_gate_factor": 1.3,
        "tentative_gate_factor": 0.65,
    },
    "pipeline": {
        "queue_size": 4,
        "block_track_birth_on_invalid": True,
        "invalid_policy": {
            "birth_block_gap_threshold": 16,
            "birth_block_out_of_sequence_threshold": 2,
            "birth_block_byte_mismatch_threshold": 2,
            "drop_gap_threshold": 140,
            "drop_out_of_sequence_threshold": 6,
            "drop_byte_mismatch_threshold": 6,
        },
    },
    "dca": {
        "config_timeout_s": 2.0,
        "packet_size_bytes": 1472,
        "packet_delay_us": 100,
        "packet_delay_ticks_per_us": 125,
    },
    "spatial_view": {
        "height": 180,
        "y": 42,
        "point_base_z_m": 0.10,
        "point_confidence_scale_m": 1.10,
    },
    "visualization": {
        "show_tentative_tracks": True,
        "tentative_min_confidence": 0.30,
        "tentative_min_hits": 2,
    },
}


def _deep_merge(base_value, override_value):
    if isinstance(base_value, dict) and isinstance(override_value, dict):
        merged = deepcopy(base_value)
        for key, value in override_value.items():
            if key in merged:
                merged[key] = _deep_merge(merged[key], value)
            else:
                merged[key] = deepcopy(value)
        return merged
    return deepcopy(override_value)


def load_runtime_settings(project_root, settings_path=None):
    project_root = Path(project_root)
    resolved_settings_path = Path(settings_path) if settings_path is not None else project_root / "config" / "live_motion_settings.json"

    settings = deepcopy(DEFAULT_SETTINGS)
    if resolved_settings_path.exists():
        loaded = json.loads(resolved_settings_path.read_text(encoding="utf-8"))
        settings = _deep_merge(settings, loaded)

    settings["_settings_path"] = str(resolved_settings_path)
    settings["_config_path_resolved"] = str(resolve_project_path(project_root, settings["config_path"]))
    return settings


def resolve_project_path(project_root, path_value):
    path = Path(path_value)
    if path.is_absolute():
        return path
    return Path(project_root) / path
