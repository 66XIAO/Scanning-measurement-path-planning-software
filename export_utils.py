import csv


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
