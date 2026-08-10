"""Scene rendering: unified redraw based on visibility flags.

Centralises the ``render_scene`` logic so that every toggle action goes
through one code-path instead of duplicating the display calls.
"""

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.gp import gp_Pnt
from OCC.Display.OCCViewer import rgb_color
from dataclasses import dataclass

from geometry import calculate_face_normal, display_coordinate_system, ConvertBndToShape
from config import NUM_CANDIDATES_PER_FACE
from surface_segmentation import is_surface_patch


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class LayerVisibility:
    model: bool = True
    face_centers: bool = True
    normal_lines: bool = True
    all_viewpoints: bool = True
    optimal_viewpoints: bool = True
    planned_path: bool = True
    sensor_volumes: bool = True
    obb_boxes: bool = True


class RenderRequest:
    def __init__(self, visibility=None, fit_all=True):
        self.visibility = visibility or LayerVisibility()
        self.fit_all = fit_all


# ---------------------------------------------------------------------------
# Scene rendering
# ---------------------------------------------------------------------------

def render_scene(display, vis, current_shape, current_faces,
                 face_centers, face_normals, center_view_points,
                 view_points, optimal_viewpoints, optimal_path,
                 sensor_volumes_list, face_obbs,
                 coordinate_systems, optimal_path_objects,
                 sensor_volume_objects, obb_visualizations,
                 fit_all=True):
    """Redraw the entire scene from current state and visibility flags.

    All mutable collection handles are cleared and re-populated in-place so
    that the caller's references remain valid.

    Returns
    -------
    tuple of (coordinate_systems, optimal_path_objects, sensor_volume_objects, obb_visualizations)
    """
    try:
        display.EraseAll()
        coordinate_systems.clear()
        optimal_path_objects.clear()
        sensor_volume_objects.clear()
        obb_visualizations.clear()

        # --- model / patches ---
        if vis.model and current_shape is not None:
            if current_faces:
                colors = [
                    rgb_color(0.8, 0.8, 1.0),
                    rgb_color(1.0, 0.8, 0.8),
                    rgb_color(0.8, 1.0, 0.8),
                    rgb_color(1.0, 1.0, 0.8),
                ]
                if any(is_surface_patch(face) for face in current_faces):
                    display.DisplayShape(
                        current_shape, color=rgb_color(0.7, 0.7, 0.7),
                        transparency=0.35, update=False)
                    for i, patch in enumerate(current_faces):
                        if is_surface_patch(patch):
                            display.DisplayShape(
                                patch.center, color=colors[i % len(colors)], update=False)
                else:
                    for i, face in enumerate(current_faces):
                        display.DisplayShape(face, color=colors[i % len(colors)], update=False)
            else:
                display.DisplayShape(current_shape, color=rgb_color(0.7, 0.7, 0.7), update=False)

        # --- face centres ---
        if vis.face_centers:
            for i, center in enumerate(face_centers):
                display.DisplayShape(center, color=rgb_color(1, 0, 0), update=False)
                if i < len(current_faces):
                    normal = face_normals[i] if i < len(face_normals) else calculate_face_normal(current_faces[i])
                    tri = display_coordinate_system(display, center, normal, size=10.0)
                    if tri:
                        coordinate_systems.append(tri)

        # --- normal lines ---
        if vis.normal_lines:
            for i in range(min(len(face_centers), len(center_view_points))):
                edge = BRepBuilderAPI_MakeEdge(face_centers[i], center_view_points[i])
                display.DisplayShape(edge.Edge(), color=rgb_color(0.5, 0.5, 1.0), update=False)

        # --- viewpoints ---
        num_centers = len(face_centers)
        if vis.all_viewpoints:
            for i in range(min(num_centers, len(center_view_points))):
                vp = center_view_points[i]
                display.DisplayShape(vp, color=rgb_color(0, 0, 1), update=False)
                if i < len(current_faces) and i < len(face_centers):
                    normal = face_normals[i] if i < len(face_normals) else calculate_face_normal(current_faces[i])
                    tri = display_coordinate_system(display, vp, normal, size=8.0, center=face_centers[i])
                    if tri:
                        coordinate_systems.append(tri)

            for i in range(num_centers, len(view_points)):
                vp = view_points[i]
                display.DisplayShape(vp, color=rgb_color(0, 0.5, 0.5), update=False)
                face_idx = (i - num_centers) // NUM_CANDIDATES_PER_FACE
                if face_idx < len(current_faces) and face_idx < len(face_centers):
                    normal = face_normals[face_idx] if face_idx < len(face_normals) else calculate_face_normal(current_faces[face_idx])
                    tri = display_coordinate_system(display, vp, normal, size=6.0, center=face_centers[face_idx])
                    if tri:
                        coordinate_systems.append(tri)
        elif vis.optimal_viewpoints:
            for vp in optimal_viewpoints:
                display.DisplayShape(vp, color=rgb_color(0, 1, 0), update=False)

        # --- optimal viewpoints (always drawn on top when flag set) ---
        if vis.optimal_viewpoints:
            for vp in optimal_viewpoints:
                display.DisplayShape(vp, color=rgb_color(0, 1, 0), update=False)

        # --- sensor volumes ---
        if vis.sensor_volumes:
            for box, _, color in sensor_volumes_list:
                obj = display.DisplayShape(box, color=color, transparency=0.3, update=False)
                if obj:
                    sensor_volume_objects.append(obj)

        # --- OBB boxes ---
        if vis.obb_boxes:
            for obb in face_obbs:
                shape = ConvertBndToShape(obb)
                if shape:
                    obj = display.DisplayShape(shape, color=rgb_color(0, 0, 1),
                                               transparency=0.7, update=False)
                    if obj:
                        obb_visualizations.append(obj)

        # --- planned path ---
        if vis.planned_path and optimal_path:
            _draw_path_edges(display, optimal_path, optimal_viewpoints, optimal_path_objects)

        if fit_all:
            display.FitAll()
        display.Repaint()

    except Exception as e:
        print("Error rendering scene: {}".format(str(e)))
        import traceback
        traceback.print_exc()


# ---------------------------------------------------------------------------
# Path edge drawing
# ---------------------------------------------------------------------------

def draw_path_edges(display, path, points, path_objects):
    """Draw path edges and append handles to *path_objects*."""
    _draw_path_edges(display, path, points, path_objects)


def _draw_path_edges(display, path, points, path_objects):
    for i in range(len(path) - 1):
        start = points[path[i]]
        end = points[path[i + 1]]

        if isinstance(start, tuple) and isinstance(end, tuple):
            p1 = gp_Pnt(start[0], start[1], start[2])
            p2 = gp_Pnt(end[0], end[1], end[2])
        else:
            p1, p2 = start, end

        edge = BRepBuilderAPI_MakeEdge(p1, p2)
        obj = display.DisplayShape(edge.Edge(), color=rgb_color(1, 0, 1), update=False)
        if obj:
            path_objects.append(obj)
