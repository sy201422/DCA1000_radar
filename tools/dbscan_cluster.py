"""Lightweight adaptive DBSCAN for sparse radar detections."""

from __future__ import annotations

import json
import math
from typing import Iterable, List, Mapping, Optional


def _effective_range(point: Mapping[str, object]) -> float:
    range_val = float(point.get("range", 0.0))
    if math.isfinite(range_val) and range_val > 0.0:
        return range_val
    return math.hypot(float(point["x"]), float(point["y"]))


def _normalize_band_float(value: object, field_name: str, allow_none: bool = False) -> Optional[float]:
    if value is None:
        if allow_none:
            return None
        raise ValueError(f"Adaptive DBSCAN band field '{field_name}' is required.")

    if isinstance(value, str):
        lowered = value.strip().lower()
        if allow_none and lowered in {"", "none", "null", "inf", "+inf", "infinity", "+infinity"}:
            return None

    try:
        float_value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Adaptive DBSCAN band field '{field_name}' must be numeric.") from exc

    if not math.isfinite(float_value):
        if allow_none:
            return None
        raise ValueError(f"Adaptive DBSCAN band field '{field_name}' must be finite.")

    return float_value


def _format_band_desc(r_min: float, r_max: Optional[float]) -> str:
    upper_text = "inf" if r_max is None else f"{r_max:.2f}"
    return f"{r_min:.2f}-{upper_text}m"


def normalize_adaptive_eps_bands(raw_bands: object) -> List[dict]:
    if raw_bands in (None, "", []):
        return []

    parsed_bands: object
    if isinstance(raw_bands, str):
        text = raw_bands.strip()
        if not text:
            return []
        if text.startswith("["):
            try:
                parsed_bands = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError("Invalid JSON for adaptive DBSCAN bands.") from exc
        else:
            parsed_bands = []
            for chunk in text.replace(";", ",").split(","):
                segment = chunk.strip()
                if not segment:
                    continue
                parts = [part.strip() for part in segment.split(":")]
                if len(parts) not in {3, 4}:
                    raise ValueError(
                        "Adaptive DBSCAN bands must use 'r_min:r_max:eps[:min_samples]' segments."
                    )
                band = {
                    "r_min": parts[0],
                    "r_max": parts[1],
                    "eps": parts[2],
                }
                if len(parts) == 4:
                    band["min_samples"] = parts[3]
                parsed_bands.append(band)
    else:
        parsed_bands = raw_bands

    if not isinstance(parsed_bands, (list, tuple)):
        raise ValueError("Adaptive DBSCAN bands must be a list/tuple or a compact string.")

    normalized: List[dict] = []
    previous_upper: Optional[float] = None
    for index, band in enumerate(parsed_bands):
        if not isinstance(band, Mapping):
            raise ValueError(f"Adaptive DBSCAN band #{index + 1} must be an object/dict.")

        r_min = _normalize_band_float(band.get("r_min"), "r_min")
        r_max = _normalize_band_float(band.get("r_max"), "r_max", allow_none=True)
        eps = _normalize_band_float(band.get("eps"), "eps")

        if r_min is None or r_min < 0.0:
            raise ValueError(f"Adaptive DBSCAN band #{index + 1} requires r_min >= 0.0.")
        if eps is None or eps <= 0.0:
            raise ValueError(f"Adaptive DBSCAN band #{index + 1} requires eps > 0.0.")
        if r_max is not None and r_max <= r_min:
            raise ValueError(f"Adaptive DBSCAN band #{index + 1} requires r_max > r_min.")
        if previous_upper is not None and r_min < previous_upper:
            raise ValueError("Adaptive DBSCAN bands must be sorted and non-overlapping.")

        min_samples = band.get("min_samples")
        if min_samples is not None:
            try:
                min_samples = int(min_samples)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Adaptive DBSCAN band #{index + 1} has invalid min_samples."
                ) from exc
            if min_samples < 1:
                raise ValueError(f"Adaptive DBSCAN band #{index + 1} requires min_samples >= 1.")

        normalized_band = {
            "r_min": float(r_min),
            "r_max": None if r_max is None else float(r_max),
            "eps": float(eps),
            "description": _format_band_desc(float(r_min), None if r_max is None else float(r_max)),
        }
        if min_samples is not None:
            normalized_band["min_samples"] = min_samples
        normalized.append(normalized_band)

        previous_upper = None if r_max is None else float(r_max)
        if previous_upper is None and index != len(parsed_bands) - 1:
            raise ValueError("An adaptive DBSCAN band with r_max=None must be the last band.")

    return normalized


def _range_matches_band(range_val: float, band: Mapping[str, object]) -> bool:
    band_min = float(band["r_min"])
    band_max = band.get("r_max")
    if range_val < band_min:
        return False
    if band_max is None:
        return True
    return range_val < float(band_max)


def _build_feature_matrix(point_list: List[dict], use_velocity_feature: bool, velocity_weight: float):
    if use_velocity_feature:
        return [
            (
                float(point["x"]),
                float(point["y"]),
                float(point.get("v", 0.0)) * velocity_weight,
            )
            for point in point_list
        ]
    return [
        (
            float(point["x"]),
            float(point["y"]),
        )
        for point in point_list
    ]


def _region_query(features, point_index, eps):
    eps_squared = eps * eps
    query_point = features[point_index]
    neighbors = []
    for candidate_index, candidate_point in enumerate(features):
        distance_squared = 0.0
        for dimension, value in enumerate(query_point):
            delta = value - candidate_point[dimension]
            distance_squared += delta * delta
        if distance_squared <= eps_squared:
            neighbors.append(candidate_index)
    return neighbors


def _dbscan_labels(features, eps, min_samples):
    labels = [-1] * len(features)
    visited = [False] * len(features)
    cluster_id = 0

    for point_index in range(len(features)):
        if visited[point_index]:
            continue

        visited[point_index] = True
        neighbors = _region_query(features, point_index, eps)
        if len(neighbors) < min_samples:
            continue

        labels[point_index] = cluster_id
        seeds = list(neighbors)
        seed_index = 0
        seed_set = set(seeds)

        while seed_index < len(seeds):
            candidate_index = seeds[seed_index]
            seed_index += 1

            if not visited[candidate_index]:
                visited[candidate_index] = True
                candidate_neighbors = _region_query(features, candidate_index, eps)
                if len(candidate_neighbors) >= min_samples:
                    for neighbor_index in candidate_neighbors:
                        if neighbor_index not in seed_set:
                            seed_set.add(neighbor_index)
                            seeds.append(neighbor_index)

            if labels[candidate_index] == -1:
                labels[candidate_index] = cluster_id

        cluster_id += 1

    return labels


def _shared_band_boundary(cluster_a: Mapping[str, object], cluster_b: Mapping[str, object]) -> Optional[float]:
    tolerance = 1e-6
    a_min = cluster_a.get("_band_r_min")
    a_max = cluster_a.get("_band_r_max")
    b_min = cluster_b.get("_band_r_min")
    b_max = cluster_b.get("_band_r_max")

    if a_max is not None and b_min is not None and abs(float(a_max) - float(b_min)) <= tolerance:
        return float(a_max)
    if b_max is not None and a_min is not None and abs(float(b_max) - float(a_min)) <= tolerance:
        return float(b_max)
    return None


def _merge_band_description(desc_a: Optional[str], desc_b: Optional[str]) -> Optional[str]:
    descriptions = [desc for desc in (desc_a, desc_b) if desc]
    if not descriptions:
        return None

    unique_descriptions = []
    for description in descriptions:
        if description not in unique_descriptions:
            unique_descriptions.append(description)
    return "|".join(unique_descriptions)


def _merge_band_upper(a_max: Optional[float], b_max: Optional[float]) -> Optional[float]:
    if a_max is None or b_max is None:
        return None
    return max(float(a_max), float(b_max))


def _summarize_cluster_points(
    c_points: List[dict],
    label: int,
    eps: float,
    min_samples: int,
    range_band_desc: Optional[str] = None,
    band_r_min: Optional[float] = None,
    band_r_max: Optional[float] = None,
    boundary_merged: bool = False,
) -> dict:
    size = len(c_points)
    if size == 0:
        raise ValueError("Cannot summarize an empty cluster.")

    weights = [max(float(point.get("score", 0.0)), 0.0) for point in c_points]
    weight_sum = sum(weights)
    if weight_sum <= 0.0:
        weights = [1.0] * size
        weight_sum = float(size)

    x_mean = sum(float(point["x"]) * weight for point, weight in zip(c_points, weights)) / weight_sum
    y_mean = sum(float(point["y"]) * weight for point, weight in zip(c_points, weights)) / weight_sum
    v_mean = sum(float(point.get("v", 0.0)) * weight for point, weight in zip(c_points, weights)) / weight_sum
    range_vals = [_effective_range(point) for point in c_points]
    spread_xy = math.sqrt(
        sum(
            (math.hypot(float(point["x"]) - x_mean, float(point["y"]) - y_mean) ** 2)
            for point in c_points
        ) / float(size)
    )
    peak_score = max(float(point.get("score", 0.0)) for point in c_points)
    mean_score = sum(float(point.get("score", 0.0)) for point in c_points) / float(size)

    size_score = min(1.0, float(size) / max(float(min_samples), 1.0))
    spread_score = max(0.0, 1.0 - (spread_xy / max(eps, 1e-6)))
    score_strength = min(1.0, peak_score / 3.0)
    confidence = max(0.0, min(1.0, (0.45 * size_score) + (0.3 * score_strength) + (0.25 * spread_score)))

    cluster = {
        "x": float(x_mean),
        "y": float(y_mean),
        "v": float(v_mean),
        "size": size,
        "confidence": float(confidence),
        "label": label,
        "spread_xy": float(spread_xy),
        "mean_score": float(mean_score),
        "peak_score": float(peak_score),
        "eps_used": float(eps),
        "min_samples_used": int(min_samples),
        "member_points": list(c_points),
        "_member_points": list(c_points),
        "_band_r_min": band_r_min,
        "_band_r_max": band_r_max,
        "_point_range_min": float(min(range_vals)),
        "_point_range_max": float(max(range_vals)),
    }
    if range_band_desc is not None:
        cluster["range_band"] = range_band_desc
    if boundary_merged:
        cluster["boundary_merged"] = True
    return cluster


def _cluster_single_batch(
    point_list: List[dict],
    eps: float,
    min_samples: int,
    use_velocity_feature: bool,
    velocity_weight: float,
    label_offset: int = 0,
    range_band_desc: Optional[str] = None,
    band_r_min: Optional[float] = None,
    band_r_max: Optional[float] = None,
) -> tuple[List[dict], int]:
    if not point_list:
        return [], label_offset

    features = _build_feature_matrix(point_list, use_velocity_feature, velocity_weight)
    labels = _dbscan_labels(features, eps, min_samples)

    clusters: List[dict] = []
    next_label = label_offset
    unique_labels = sorted(set(labels))
    for label in unique_labels:
        if label == -1:
            continue

        member_indices = [index for index, current_label in enumerate(labels) if current_label == label]
        if not member_indices:
            continue

        c_points = [point_list[index] for index in member_indices]
        cluster = _summarize_cluster_points(
            c_points=c_points,
            label=next_label,
            eps=eps,
            min_samples=min_samples,
            range_band_desc=range_band_desc,
            band_r_min=band_r_min,
            band_r_max=band_r_max,
        )
        clusters.append(cluster)
        next_label += 1

    return clusters, next_label


def _merge_adaptive_boundary_clusters(clusters: List[dict]) -> List[dict]:
    if len(clusters) < 2:
        return clusters

    merged_clusters = list(clusters)
    while True:
        best_pair: Optional[tuple[int, int]] = None
        best_distance: Optional[float] = None

        for left_index in range(len(merged_clusters)):
            for right_index in range(left_index + 1, len(merged_clusters)):
                left_cluster = merged_clusters[left_index]
                right_cluster = merged_clusters[right_index]
                shared_boundary = _shared_band_boundary(left_cluster, right_cluster)
                if shared_boundary is None:
                    continue

                merge_threshold = max(float(left_cluster["eps_used"]), float(right_cluster["eps_used"]))
                centroid_distance = math.hypot(
                    float(left_cluster["x"]) - float(right_cluster["x"]),
                    float(left_cluster["y"]) - float(right_cluster["y"]),
                )
                if centroid_distance > merge_threshold:
                    continue

                if float(left_cluster["_point_range_max"]) <= float(right_cluster["_point_range_min"]):
                    lower_cluster = left_cluster
                    upper_cluster = right_cluster
                else:
                    lower_cluster = right_cluster
                    upper_cluster = left_cluster

                lower_near_boundary = float(lower_cluster["_point_range_max"]) >= shared_boundary - merge_threshold
                upper_near_boundary = float(upper_cluster["_point_range_min"]) <= shared_boundary + merge_threshold
                if not (lower_near_boundary and upper_near_boundary):
                    continue

                if best_distance is None or centroid_distance < best_distance:
                    best_distance = centroid_distance
                    best_pair = (left_index, right_index)

        if best_pair is None:
            break

        left_index, right_index = best_pair
        left_cluster = merged_clusters[left_index]
        right_cluster = merged_clusters[right_index]
        combined_points = list(left_cluster["_member_points"]) + list(right_cluster["_member_points"])
        merged_cluster = _summarize_cluster_points(
            c_points=combined_points,
            label=min(int(left_cluster["label"]), int(right_cluster["label"])),
            eps=max(float(left_cluster["eps_used"]), float(right_cluster["eps_used"])),
            min_samples=max(int(left_cluster["min_samples_used"]), int(right_cluster["min_samples_used"])),
            range_band_desc=_merge_band_description(left_cluster.get("range_band"), right_cluster.get("range_band")),
            band_r_min=min(float(left_cluster["_band_r_min"]), float(right_cluster["_band_r_min"])),
            band_r_max=_merge_band_upper(left_cluster.get("_band_r_max"), right_cluster.get("_band_r_max")),
            boundary_merged=True,
        )
        merged_clusters[left_index] = merged_cluster
        del merged_clusters[right_index]

    return merged_clusters


def _strip_internal_cluster_fields(clusters: List[dict]) -> List[dict]:
    public_clusters = []
    for cluster in clusters:
        public_clusters.append({key: value for key, value in cluster.items() if not key.startswith("_")})
    return public_clusters


def cluster_points(
    points: Iterable[dict],
    eps: float = 0.35,
    min_samples: int = 1,
    use_velocity_feature: bool = False,
    velocity_weight: float = 0.25,
    adaptive_eps_bands: object = None,
) -> List[dict]:
    if velocity_weight < 0.0:
        raise ValueError("velocity_weight must be >= 0.0")
    if eps <= 0.0:
        raise ValueError("eps must be > 0.0")
    if min_samples < 1:
        raise ValueError("min_samples must be >= 1")

    point_list: List[dict] = []
    for index, point in enumerate(points):
        try:
            x_val = float(point["x"])
            y_val = float(point["y"])
        except (KeyError, TypeError, ValueError):
            continue

        if not math.isfinite(x_val) or not math.isfinite(y_val):
            continue

        clean_point = dict(point)
        clean_point["x"] = x_val
        clean_point["y"] = y_val
        clean_point["cluster_index"] = int(point.get("cluster_index", index))
        for key in ("v", "score", "range"):
            try:
                value = float(clean_point.get(key, 0.0))
            except (TypeError, ValueError):
                value = 0.0
            clean_point[key] = value if math.isfinite(value) else 0.0
        point_list.append(clean_point)

    if not point_list:
        return []

    normalized_bands = normalize_adaptive_eps_bands(adaptive_eps_bands)
    if not normalized_bands:
        clusters, _ = _cluster_single_batch(
            point_list=point_list,
            eps=eps,
            min_samples=min_samples,
            use_velocity_feature=use_velocity_feature,
            velocity_weight=velocity_weight,
        )
        return _strip_internal_cluster_fields(clusters)

    band_point_lists: List[List[dict]] = [[] for _ in normalized_bands]
    fallback_points: List[dict] = []
    for point in point_list:
        point_range = _effective_range(point)
        matched = False
        for band_index, band in enumerate(normalized_bands):
            if _range_matches_band(point_range, band):
                band_point_lists[band_index].append(point)
                matched = True
                break
        if not matched:
            fallback_points.append(point)

    clusters: List[dict] = []
    next_label = 0
    for band, band_points in zip(normalized_bands, band_point_lists):
        if not band_points:
            continue

        band_clusters, next_label = _cluster_single_batch(
            point_list=band_points,
            eps=float(band["eps"]),
            min_samples=int(band.get("min_samples", min_samples)),
            use_velocity_feature=use_velocity_feature,
            velocity_weight=velocity_weight,
            label_offset=next_label,
            range_band_desc=str(band["description"]),
            band_r_min=float(band["r_min"]),
            band_r_max=None if band.get("r_max") is None else float(band["r_max"]),
        )
        clusters.extend(band_clusters)

    if fallback_points:
        fallback_clusters, _ = _cluster_single_batch(
            point_list=fallback_points,
            eps=eps,
            min_samples=min_samples,
            use_velocity_feature=use_velocity_feature,
            velocity_weight=velocity_weight,
            label_offset=next_label,
            range_band_desc="fallback",
        )
        clusters.extend(fallback_clusters)

    clusters = _merge_adaptive_boundary_clusters(clusters)
    return _strip_internal_cluster_fields(clusters)
