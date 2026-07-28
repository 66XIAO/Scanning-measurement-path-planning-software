import csv
import json

from speed_planning_core import result_rows


CSV_HEADER = ["Optimal Viewpoint", "X", "Y", "Z", "w", "x", "y", "z"]


def build_path_pose_rows(optimal_viewpoints_with_pose, optimal_path=None):
    if not optimal_viewpoints_with_pose:
        raise ValueError("No optimal viewpoints with pose are available")

    if optimal_path:
        ordered_indices = list(optimal_path)
    else:
        ordered_indices = list(range(len(optimal_viewpoints_with_pose)))

    rows = []
    for row_index, viewpoint_index in enumerate(ordered_indices, start=1):
        if viewpoint_index < 0 or viewpoint_index >= len(optimal_viewpoints_with_pose):
            raise IndexError("Path index is outside optimal viewpoint range: {}".format(viewpoint_index))

        viewpoint, pose = optimal_viewpoints_with_pose[viewpoint_index]
        w, x_quat, y_quat, z_quat = pose
        rows.append([
            row_index,
            viewpoint.X(),
            viewpoint.Y(),
            viewpoint.Z(),
            w,
            x_quat,
            y_quat,
            z_quat,
        ])

    return rows


def write_path_pose_csv(file_path, optimal_viewpoints_with_pose, optimal_path=None):
    rows = build_path_pose_rows(optimal_viewpoints_with_pose, optimal_path)
    with open(file_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(CSV_HEADER)
        writer.writerows(rows)
    return len(rows)


SPEED_CSV_HEADER = [
    "index", "X", "Y", "Z", "w", "x", "y", "z",
    "source_X", "source_Y", "source_Z", "source_w", "source_x", "source_y", "source_z",
    "source_pose_frame", "command_pose_frame", "extrinsic_applied",
    "extrinsic_validated", "extrinsic_config_id",
    "path_s_mm", "segment_length_mm", "curvature_1_mm",
    "orientation_delta_deg", "linear_speed", "linear_accel",
    "joint_speed", "joint_accel",
    "tcp_angular_speed_deg_s", "tcp_angular_accel_deg_s2", "dt_to_next_s",
    "max_linear_speed_mm_s", "feasible", "violation_codes",
]


def write_speed_plan_csv(file_path, result):
    rows = result_rows(result)
    diagnostics = result.diagnostics or {}
    source_records = diagnostics.get("source_pose_records") or []
    with open(file_path, "w", encoding="utf-8-sig", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(SPEED_CSV_HEADER)
        for row_index, row in enumerate(rows):
            source = source_records[row_index] if row_index < len(source_records) else {
                "x": row["x"], "y": row["y"], "z": row["z"],
                "qw": row["qw"], "qx": row["qx"], "qy": row["qy"], "qz": row["qz"],
            }
            writer.writerow([
                row["index"], row["x"], row["y"], row["z"],
                row["qw"], row["qx"], row["qy"], row["qz"],
                source["x"], source["y"], source["z"],
                source["qw"], source["qx"], source["qy"], source["qz"],
                diagnostics.get("source_pose_frame", "unknown"),
                diagnostics.get("command_pose_frame", "unknown"),
                diagnostics.get("extrinsic_applied", False),
                diagnostics.get("extrinsic_validated", False),
                diagnostics.get("extrinsic_config_id", ""),
                row["path_s"], row["segment_length"], row["curvature"],
                row["orientation_delta_deg"], row["linear_speed"], row["linear_accel"],
                row["joint_speed"], row["joint_accel"],
                row["angular_speed"], row["angular_accel"], row["dt_to_next"],
                row["max_linear_speed"], row["feasible"], row["violation_codes"],
            ])
    metadata = {
        "schema_version": "1.0",
        "algorithm": result.algorithm,
        "total_time_s": result.total_time,
        "feasible": result.feasible,
        "warnings": list(result.warnings),
        "pose_count": len(rows),
        "diagnostics": {key: value for key, value in diagnostics.items()
                        if key != "source_pose_records"},
    }
    with open(file_path + ".metadata.json", "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)
    return len(rows)
