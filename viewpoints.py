"""Viewpoint generation and pose computation.

All functions are *pure* — they receive geometry data as arguments and return
results without touching any global state or the OCC viewer.
"""

import math
from OCC.Core.gp import (
    gp_Pnt, gp_Vec, gp_Dir, gp_Ax1, gp_Ax3, gp_Trsf,
)
from OCC.Display.OCCViewer import rgb_color

from geometry import generate_viewpoint, calculate_face_normal, display_coordinate_system
from config import VIEWPOINT_DISTANCE, NUM_CANDIDATES_PER_FACE, ZENITH_ANGLE_DEG


# ---------------------------------------------------------------------------
# Pose (quaternion) calculation
# ---------------------------------------------------------------------------

def calculate_viewpoint_pose(viewpoint, normal, center=None):
    """Compute the quaternion (w, x, y, z) for *viewpoint*.

    The local Z axis points from *viewpoint* towards *center* (if given),
    otherwise along *normal*.  X is derived via Gram-Schmidt.

    Returns
    -------
    tuple of (w, x, y, z)
    """
    try:
        z_vec = _pose_z_axis(viewpoint, normal, center)
        x_vec = _pose_x_axis(z_vec)
        y_vec = z_vec.Crossed(x_vec)
        if y_vec.Magnitude() > 0:
            y_vec.Normalize()

        local_cs = gp_Ax3(viewpoint, gp_Dir(z_vec), gp_Dir(x_vec))
        world_cs = gp_Ax3(gp_Pnt(), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))

        tr = gp_Trsf()
        tr.SetTransformation(local_cs, world_cs)
        q = tr.GetRotation()
        return (q.W(), q.X(), q.Y(), q.Z())
    except Exception as e:
        print("Error calculating viewpoint pose: {}".format(str(e)))
        return (1.0, 0.0, 0.0, 0.0)


def _pose_z_axis(viewpoint, normal, center):
    if center is not None:
        z = gp_Vec(viewpoint, center)
        if z.Magnitude() > 0:
            z.Normalize()
            return z
    if isinstance(normal, gp_Vec):
        return normal.Normalized()
    if hasattr(normal, 'X'):
        return gp_Vec(normal.X(), normal.Y(), normal.Z()).Normalized()
    try:
        x, y, z = normal
        return gp_Vec(x, y, z).Normalized()
    except Exception:
        return gp_Vec(0, 0, 1).Normalized()


def _pose_x_axis(z_vec):
    ref = gp_Vec(1, 0, 0)
    if abs(ref.Dot(z_vec)) > 0.999:
        ref = gp_Vec(0, 1, 0)
    proj = gp_Vec(z_vec.X() * ref.Dot(z_vec),
                   z_vec.Y() * ref.Dot(z_vec),
                   z_vec.Z() * ref.Dot(z_vec))
    x = ref - proj
    if x.Magnitude() > 0:
        x.Normalize()
        return x
    return gp_Vec(1, 0, 0)


# ---------------------------------------------------------------------------
# Center viewpoint generation
# ---------------------------------------------------------------------------

def build_center_viewpoints(display, faces, centers, normals,
                            distance=VIEWPOINT_DISTANCE):
    """Generate one center viewpoint per face.

    Parameters
    ----------
    display : OCC viewer
    faces, centers, normals : lists of equal length

    Returns
    -------
    (center_vps, center_vps_with_pose, normal_line_objects)
        center_vps : list of gp_Pnt
        center_vps_with_pose : list of (gp_Pnt, (w,x,y,z))
        normal_line_objects : list of display handles for the centre-to-vp lines
    """
    center_vps = []
    center_vps_with_pose = []
    normal_line_objects = []

    count = min(len(faces), len(centers))

    for i in range(count):
        center = centers[i]
        normal = normals[i] if i < len(normals) else calculate_face_normal(faces[i])

        vp = generate_viewpoint(center, normal, distance)
        center_vps.append(vp)

        pose = calculate_viewpoint_pose(vp, normal, center)
        center_vps_with_pose.append((vp, pose))

        # display blue dot
        display.DisplayShape(vp, color=rgb_color(0, 0, 1), update=False)

        # display line centre -> viewpoint
        from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
        edge = BRepBuilderAPI_MakeEdge(center, vp)
        line_obj = display.DisplayShape(edge.Edge(), color=rgb_color(0.5, 0.5, 1.0), update=False)
        if line_obj:
            normal_line_objects.append(line_obj)

        # display trihedron
        display_coordinate_system(display, vp, normal, size=50, center=center)

    return center_vps, center_vps_with_pose, normal_line_objects


# ---------------------------------------------------------------------------
# Candidate viewpoint generation (center + N tilted per face)
# ---------------------------------------------------------------------------

def build_candidate_viewpoints(display, faces, centers, normals,
                               distance=VIEWPOINT_DISTANCE,
                               num_candidates=NUM_CANDIDATES_PER_FACE,
                               zenith_deg=ZENITH_ANGLE_DEG):
    """Generate center + *num_candidates* tilted viewpoints per face.

    Returns
    -------
    (all_vps, all_vps_with_pose, center_vps, normal_line_objects, coordinate_systems)
    """
    all_center_vps = []
    center_poses = []
    all_non_center_vps = []
    non_center_poses = []
    normal_line_objects = []
    coord_systems = []

    count = min(len(faces), len(centers))
    zenith_rad = math.radians(zenith_deg)

    for i in range(count):
        center = centers[i]
        main_normal = normals[i] if i < len(normals) else calculate_face_normal(faces[i])
        if main_normal.Magnitude() > 0:
            main_normal = gp_Vec(main_normal.X(), main_normal.Y(), main_normal.Z())
            main_normal.Normalize()

        # --- centre viewpoint ---
        cv = generate_viewpoint(center, main_normal, distance)
        all_center_vps.append(cv)
        cp = calculate_viewpoint_pose(cv, main_normal, center)
        center_poses.append(cp)

        display.DisplayShape(cv, color=rgb_color(0, 0, 1), update=False)
        tri = display_coordinate_system(display, cv, main_normal, size=8.0, center=center)
        if tri:
            coord_systems.append(tri)

        # --- normal line (centre -> centre viewpoint) ---
        from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
        edge = BRepBuilderAPI_MakeEdge(center, cv)
        lo = display.DisplayShape(edge.Edge(), color=rgb_color(0.5, 0.5, 1.0), update=False)
        if lo:
            normal_line_objects.append(lo)

        # --- tilted candidates ---
        x_vec = _perpendicular_vector(main_normal)
        x_rot_axis = gp_Ax1(center, gp_Dir(x_vec))
        z_rot_axis = gp_Ax1(center, gp_Dir(main_normal))

        for az_idx in range(num_candidates):
            az_deg = 360.0 / num_candidates * az_idx
            az_rad = math.radians(az_deg)
            try:
                r1 = gp_Trsf()
                r1.SetRotation(x_rot_axis, zenith_rad)
                r2 = gp_Trsf()
                r2.SetRotation(z_rot_axis, az_rad)
                combined = gp_Trsf()
                combined.Multiply(r2)
                combined.Multiply(r1)

                rot_normal = gp_Vec(main_normal.X(), main_normal.Y(), main_normal.Z())
                rot_normal.Transform(combined)
                if rot_normal.Magnitude() > 0:
                    rot_normal.Normalize()

                cand = generate_viewpoint(center, rot_normal, distance)
                all_non_center_vps.append(cand)

                cp2 = calculate_viewpoint_pose(cand, rot_normal, center)
                non_center_poses.append(cp2)

                display.DisplayShape(cand, color=rgb_color(0, 0.5, 0.5), update=False)
                tri2 = display_coordinate_system(display, cand, rot_normal, size=6.0, center=center)
                if tri2:
                    coord_systems.append(tri2)
            except Exception as e:
                print("Error generating candidate viewpoint: {}".format(str(e)))

    # Merge: centres first, then non-centres
    all_vps = list(all_center_vps) + list(all_non_center_vps)
    all_poses = list(center_poses) + list(non_center_poses)
    all_with_pose = list(zip(all_vps, all_poses))

    return all_vps, all_with_pose, all_center_vps, normal_line_objects, coord_systems


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _perpendicular_vector(normal):
    """Return a unit vector perpendicular to *normal*."""
    if abs(normal.X()) < abs(normal.Y()):
        ref = gp_Vec(1, 0, 0)
    else:
        ref = gp_Vec(0, 1, 0)
    dot = ref.Dot(normal)
    proj = gp_Vec(normal.X() * dot, normal.Y() * dot, normal.Z() * dot)
    v = ref - proj
    if v.Magnitude() > 0:
        v.Normalize()
        return v
    return gp_Vec(1, 0, 0)


# ---------------------------------------------------------------------------
# Filtering helpers
# ---------------------------------------------------------------------------

def ordered_face_candidates(records, face_index, face_count,
                            candidates_per_face=NUM_CANDIDATES_PER_FACE):
    """Extract centre + non-centre candidate records for one face."""
    result = []
    if face_index < len(records):
        result.append(records[face_index])
    start = face_count + face_index * candidates_per_face
    end = start + candidates_per_face
    result.extend(records[start:end])
    return result


def choose_closest_to_distance(candidates, face_center, target_distance,
                               distance_fn):
    """Pick the candidate whose viewpoint is closest to *target_distance*."""
    if not candidates:
        return None
    return min(candidates,
               key=lambda item: abs(distance_fn(item[0], face_center) - target_distance))
