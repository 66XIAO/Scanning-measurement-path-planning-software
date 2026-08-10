"""Trim-aware surface partitioning without destructive BRep splitting.

The legacy equal-parameter strategy creates new ``TopoDS_Face`` objects with
Boolean split operations.  That is appropriate for simple, untrimmed faces but
becomes both fragile and slow for a large free-form face with many trimming
wires.  This module uses an OCC triangulation only to partition the material in
the global XOY plane.  Patch centres and normals are then evaluated on the
original CAD surface, so the mesh does not replace the source geometry.
"""

from dataclasses import dataclass
import math
import time

from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRepBndLib import brepbndlib
import OCC.Core.BRepCheck as BRepCheckModule
from OCC.Core.BRepCheck import BRepCheck_Analyzer
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRepTools import breptools
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.GProp import GProp_GProps
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_WIRE
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.gp import gp_Pnt, gp_Vec


@dataclass
class SurfacePatch:
    """A connected mesh-grid region backed by the original CAD face."""

    source_face: object
    source_face_index: int
    grid_u_index: int
    grid_v_index: int
    component_index: int
    center: object
    normal: object
    area: float
    triangle_count: int
    bounds: tuple
    representative_uv: tuple
    strategy: str = "mesh_grid"

    def summary(self):
        return {
            "source_face_index": self.source_face_index,
            "grid_u_index": self.grid_u_index,
            "grid_v_index": self.grid_v_index,
            "component_index": self.component_index,
            "center": [self.center.X(), self.center.Y(), self.center.Z()],
            "normal": [self.normal.X(), self.normal.Y(), self.normal.Z()],
            "area": self.area,
            "triangle_count": self.triangle_count,
            "bounds": list(self.bounds),
            "representative_uv": list(self.representative_uv),
            "strategy": self.strategy,
        }


@dataclass
class _TriangleSample:
    nodes: tuple
    area: float
    centroid: tuple
    normal: tuple
    uv: tuple
    bounds: tuple


def is_surface_patch(value):
    return isinstance(value, SurfacePatch)


def inspect_shape_topology(shape):
    """Return inexpensive face/wire counts used by automatic routing."""
    face_count = 0
    trimmed_face_count = 0
    max_wire_count = 0
    total_wire_count = 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face_count += 1
        face = explorer.Current()
        wire_count = 0
        wire_explorer = TopExp_Explorer(face, TopAbs_WIRE)
        while wire_explorer.More():
            wire_count += 1
            wire_explorer.Next()
        total_wire_count += wire_count
        max_wire_count = max(max_wire_count, wire_count)
        if wire_count > 1:
            trimmed_face_count += 1
        explorer.Next()
    return {
        "face_count": face_count,
        "trimmed_face_count": trimmed_face_count,
        "max_wire_count": max_wire_count,
        "total_wire_count": total_wire_count,
        "has_trimmed_faces": trimmed_face_count > 0,
    }


def choose_mesh_deflection(shape, minimum=0.5, maximum=5.0, divisions=250.0):
    """Choose a bounded absolute mesh deflection from the model diagonal."""
    bounds = _shape_bounds(shape)
    diagonal = math.sqrt(
        (bounds[3] - bounds[0]) ** 2
        + (bounds[4] - bounds[1]) ** 2
        + (bounds[5] - bounds[2]) ** 2)
    return max(minimum, min(maximum, diagonal / max(divisions, 1.0)))


def segment_shape_mesh_grid(shape, u=8, v=6, linear_deflection=None,
                            angular_deflection=0.5, area_tolerance=0.02):
    """Partition a trimmed CAD shape by connected mesh regions in XOY.

    Each source triangle is assigned exactly once using its XOY centroid.  A
    connected-component pass prevents material on opposite sides of a hole
    from becoming one patch merely because it occupies the same grid cell.
    """
    if u < 1 or v < 1:
        raise ValueError("Grid counts must be positive")
    topology = inspect_shape_topology(shape)
    if topology["face_count"] == 0:
        raise ValueError("Shape contains no faces")

    bounds = _shape_bounds(shape)
    x_span = bounds[3] - bounds[0]
    y_span = bounds[4] - bounds[1]
    if x_span <= 1e-12 or y_span <= 1e-12:
        raise ValueError("XOY projection has zero width or height")
    if linear_deflection is None:
        linear_deflection = choose_mesh_deflection(shape)

    # Viewer triangulations can be much finer than this workflow requires.
    # Removing only cached polygonal data makes the runtime deterministic; the
    # exact BRep geometry and trimming wires remain unchanged.
    breptools.Clean(shape, True)
    mesh_started = time.perf_counter()
    mesher = BRepMesh_IncrementalMesh(
        shape, float(linear_deflection), False,
        float(angular_deflection), True)
    mesher.Perform()
    if not mesher.IsDone():
        raise RuntimeError("OpenCascade surface meshing did not complete")
    mesh_seconds = time.perf_counter() - mesh_started

    extraction_started = time.perf_counter()
    grouped = {}
    face_context = {}
    total_nodes = 0
    source_triangle_count = 0
    partition_triangle_count = 0
    source_mesh_area = 0.0
    mesh_area = 0.0

    face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_index = 0
    while face_explorer.More():
        face = face_explorer.Current()
        face_index += 1
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation(face, location)
        if triangulation is None or triangulation.NbTriangles() == 0:
            face_explorer.Next()
            continue
        if not triangulation.HasUVNodes():
            raise RuntimeError(
                "Face {} triangulation has no UV nodes".format(face_index))

        transformation = location.Transformation()
        nodes = [None]
        uv_nodes = [None]
        for node_index in range(1, triangulation.NbNodes() + 1):
            point = triangulation.Node(node_index).Transformed(transformation)
            uv_point = triangulation.UVNode(node_index)
            nodes.append((point.X(), point.Y(), point.Z()))
            uv_nodes.append((uv_point.X(), uv_point.Y()))
        total_nodes += triangulation.NbNodes()
        face_context[face_index] = {
            "face": face,
            "nodes": nodes,
            "surface": BRep_Tool.Surface(face),
            "adaptor": BRepAdaptor_Surface(face),
        }

        reverse = face.Orientation() == TopAbs_REVERSED
        for triangle_index in range(1, triangulation.NbTriangles() + 1):
            n1, n2, n3 = triangulation.Triangle(triangle_index).Get()
            p1, p2, p3 = nodes[n1], nodes[n2], nodes[n3]
            source_cross = _cross(_subtract(p2, p1), _subtract(p3, p1))
            source_area = 0.5 * _length(source_cross)
            if source_area <= 1e-14:
                continue
            source_triangle_count += 1
            source_mesh_area += source_area
            vertices = [
                p1 + uv_nodes[n1],
                p2 + uv_nodes[n2],
                p3 + uv_nodes[n3],
            ]
            triangle_xmin = min(point[0] for point in vertices)
            triangle_xmax = max(point[0] for point in vertices)
            triangle_ymin = min(point[1] for point in vertices)
            triangle_ymax = max(point[1] for point in vertices)
            first_u = min(u - 1, max(0, int((triangle_xmin - bounds[0]) / x_span * u)))
            last_u = min(u - 1, max(0, int((triangle_xmax - bounds[0]) / x_span * u)))
            first_v = min(v - 1, max(0, int((triangle_ymin - bounds[1]) / y_span * v)))
            last_v = min(v - 1, max(0, int((triangle_ymax - bounds[1]) / y_span * v)))

            for grid_u in range(first_u, last_u + 1):
                cell_xmin = bounds[0] + x_span * grid_u / u
                cell_xmax = bounds[0] + x_span * (grid_u + 1) / u
                for grid_v in range(first_v, last_v + 1):
                    cell_ymin = bounds[1] + y_span * grid_v / v
                    cell_ymax = bounds[1] + y_span * (grid_v + 1) / v
                    clipped = _clip_polygon_to_rect(
                        vertices, cell_xmin, cell_xmax,
                        cell_ymin, cell_ymax)
                    if len(clipped) < 3:
                        continue
                    for polygon_index in range(1, len(clipped) - 1):
                        sample = _make_triangle_sample(
                            clipped[0], clipped[polygon_index],
                            clipped[polygon_index + 1], reverse)
                        if sample is None:
                            continue
                        grouped.setdefault(
                            (face_index, grid_u, grid_v), []).append(sample)
                        partition_triangle_count += 1
                        mesh_area += sample.area
        face_explorer.Next()

    patches = []
    connected_component_splits = 0
    for key in sorted(grouped):
        samples = grouped[key]
        components = _connected_components(samples)
        connected_component_splits += max(0, len(components) - 1)
        for component_index, component in enumerate(components):
            patches.append(_build_patch(
                key, component_index, component, samples,
                face_context[key[0]]))

    extraction_seconds = time.perf_counter() - extraction_started
    original_area = _surface_area(shape)
    area_ratio = mesh_area / source_mesh_area if source_mesh_area > 1e-12 else 0.0
    cad_mesh_area_ratio = (
        source_mesh_area / original_area if original_area > 1e-12 else 0.0)
    brep_valid, brep_status = _shape_validity(shape)
    occupied_cells = len({(key[1], key[2]) for key in grouped})
    warnings = []
    input_warnings = []
    if not patches:
        warnings.append("Mesh-grid segmentation produced no patches")
    if source_mesh_area <= 1e-12:
        warnings.append("Source mesh area is zero or unavailable")
    elif abs(area_ratio - 1.0) > 1e-6:
        warnings.append(
            "Partition area does not conserve the source mesh: "
            "partition/source={:.6f}".format(area_ratio))
    if not brep_valid:
        input_warnings.append(
            "Input BRep is invalid ({}); CAD surface area is not used as the "
            "mesh-grid conservation reference".format(
                ", ".join(brep_status) if brep_status else "unknown status"))
    elif original_area <= 1e-12:
        warnings.append("Original CAD surface area is zero or unavailable")
    elif abs(cad_mesh_area_ratio - 1.0) > area_tolerance:
        warnings.append(
            "Source mesh differs from valid CAD area: mesh/CAD={:.6f} "
            "(tolerance={:.3%})".format(cad_mesh_area_ratio, area_tolerance))

    diagnostics = {
        "strategy": "mesh_grid",
        "strategy_reason": "trimmed_face_detected",
        "projection_plane": "XOY",
        "grid_u": u,
        "grid_v": v,
        "original_face_count": topology["face_count"],
        "trimmed_face_count": topology["trimmed_face_count"],
        "max_wire_count": topology["max_wire_count"],
        "patch_count": len(patches),
        "original_area": original_area,
        "source_mesh_area": source_mesh_area,
        "mesh_area": mesh_area,
        "patch_area_sum": mesh_area,
        "area_ratio": area_ratio,
        "cad_mesh_area_ratio": cad_mesh_area_ratio,
        "input_brep_valid": brep_valid,
        "input_brep_status": brep_status,
        "mesh_linear_deflection": float(linear_deflection),
        "mesh_angular_deflection": float(angular_deflection),
        "mesh_node_count": total_nodes,
        "mesh_triangle_count": source_triangle_count,
        "partition_triangle_count": partition_triangle_count,
        "occupied_grid_cells": occupied_cells,
        "empty_grid_cells": u * v - occupied_cells,
        "connected_component_splits": connected_component_splits,
        "mesh_seconds": mesh_seconds,
        "partition_seconds": extraction_seconds,
        "split_attempts": 0,
        "effective_splits": 0,
        "split_no_effect": 0,
        "split_exceptions": 0,
        "area_rejected_splits": 0,
        "fallback_faces": 0,
        "periodic_faces": 0,
        "warnings": warnings,
        "input_warnings": input_warnings,
    }
    return patches, diagnostics


def _connected_components(samples):
    parent = list(range(len(samples)))
    ranks = [0] * len(samples)
    edge_owner = {}

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        root_first = find(first)
        root_second = find(second)
        if root_first == root_second:
            return
        if ranks[root_first] < ranks[root_second]:
            root_first, root_second = root_second, root_first
        parent[root_second] = root_first
        if ranks[root_first] == ranks[root_second]:
            ranks[root_first] += 1

    for index, sample in enumerate(samples):
        n1, n2, n3 = sample.nodes
        for first, second in ((n1, n2), (n2, n3), (n3, n1)):
            edge = (min(first, second), max(first, second))
            previous = edge_owner.get(edge)
            if previous is None:
                edge_owner[edge] = index
            else:
                union(index, previous)

    grouped = {}
    for index in range(len(samples)):
        grouped.setdefault(find(index), []).append(index)
    return sorted(grouped.values(), key=lambda values: values[0])


def _clip_polygon_to_rect(vertices, xmin, xmax, ymin, ymax):
    polygon = list(vertices)
    polygon = _clip_polygon_half_plane(
        polygon, axis=0, limit=xmin, keep_greater=True)
    polygon = _clip_polygon_half_plane(
        polygon, axis=0, limit=xmax, keep_greater=False)
    polygon = _clip_polygon_half_plane(
        polygon, axis=1, limit=ymin, keep_greater=True)
    polygon = _clip_polygon_half_plane(
        polygon, axis=1, limit=ymax, keep_greater=False)
    return polygon


def _clip_polygon_half_plane(vertices, axis, limit, keep_greater):
    if not vertices:
        return []

    def is_inside(vertex):
        if keep_greater:
            return vertex[axis] >= limit - 1e-12
        return vertex[axis] <= limit + 1e-12

    result = []
    previous = vertices[-1]
    previous_inside = is_inside(previous)
    for current in vertices:
        current_inside = is_inside(current)
        if current_inside != previous_inside:
            denominator = current[axis] - previous[axis]
            if abs(denominator) > 1e-15:
                ratio = (limit - previous[axis]) / denominator
                result.append(tuple(
                    previous[index] + ratio * (current[index] - previous[index])
                    for index in range(len(current))))
        if current_inside:
            result.append(current)
        previous = current
        previous_inside = current_inside
    return result


def _make_triangle_sample(first, second, third, reverse):
    point_first = first[:3]
    point_second = second[:3]
    point_third = third[:3]
    cross = _cross(
        _subtract(point_second, point_first),
        _subtract(point_third, point_first))
    cross_length = _length(cross)
    if cross_length <= 1e-14:
        return None
    direction = tuple(value / cross_length for value in cross)
    if reverse:
        direction = tuple(-value for value in direction)
    centroid = tuple(
        (point_first[index] + point_second[index] + point_third[index]) / 3.0
        for index in range(3))
    uv = tuple((first[index] + second[index] + third[index]) / 3.0
               for index in (3, 4))
    return _TriangleSample(
        nodes=tuple(_vertex_key(vertex) for vertex in (first, second, third)),
        area=0.5 * cross_length,
        centroid=centroid,
        normal=direction,
        uv=uv,
        bounds=(
            min(point_first[0], point_second[0], point_third[0]),
            min(point_first[1], point_second[1], point_third[1]),
            min(point_first[2], point_second[2], point_third[2]),
            max(point_first[0], point_second[0], point_third[0]),
            max(point_first[1], point_second[1], point_third[1]),
            max(point_first[2], point_second[2], point_third[2]),
        ))


def _vertex_key(vertex, tolerance=1e-7):
    return tuple(int(round(value / tolerance)) for value in vertex[:3])


def _build_patch(key, component_index, component, samples, context):
    selected = [samples[index] for index in component]
    total_area = sum(sample.area for sample in selected)
    weighted_center = tuple(
        sum(sample.area * sample.centroid[axis] for sample in selected) / total_area
        for axis in range(3))
    representative = min(
        selected,
        key=lambda sample: sum(
            (sample.centroid[axis] - weighted_center[axis]) ** 2
            for axis in range(3)))

    u_value, v_value = representative.uv
    try:
        center = context["surface"].Value(u_value, v_value)
        normal = _surface_normal(context["adaptor"], u_value, v_value)
    except Exception:
        center = gp_Pnt(*representative.centroid)
        normal = gp_Vec(*representative.normal)
    if normal.Magnitude() <= 1e-12:
        normal = gp_Vec(*representative.normal)
    if normal.Magnitude() > 0:
        normal.Normalize()
    mesh_normal = gp_Vec(*representative.normal)
    if mesh_normal.Magnitude() > 0 and normal.Dot(mesh_normal) < 0:
        normal.Reverse()
    # Preserve the original application's default normal convention.
    if normal.Dot(gp_Vec(0, 0, 1)) < 0:
        normal.Reverse()

    patch_bounds = (
        min(sample.bounds[0] for sample in selected),
        min(sample.bounds[1] for sample in selected),
        min(sample.bounds[2] for sample in selected),
        max(sample.bounds[3] for sample in selected),
        max(sample.bounds[4] for sample in selected),
        max(sample.bounds[5] for sample in selected),
    )
    return SurfacePatch(
        source_face=context["face"],
        source_face_index=key[0],
        grid_u_index=key[1],
        grid_v_index=key[2],
        component_index=component_index,
        center=center,
        normal=normal,
        area=total_area,
        triangle_count=len(selected),
        bounds=patch_bounds,
        representative_uv=representative.uv,
    )


def _surface_normal(adaptor, u_value, v_value):
    point = gp_Pnt()
    derivative_u = gp_Vec()
    derivative_v = gp_Vec()
    adaptor.D1(u_value, v_value, point, derivative_u, derivative_v)
    return derivative_u.Crossed(derivative_v)


def _shape_bounds(shape):
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    if box.IsVoid():
        raise ValueError("Could not calculate model bounds")
    return tuple(float(value) for value in box.Get())


def _surface_area(shape):
    properties = GProp_GProps()
    brepgprop.SurfaceProperties(shape, properties)
    return float(properties.Mass())


def _shape_validity(shape):
    analyzer = BRepCheck_Analyzer(shape, True, True, True)
    valid = analyzer.IsValid()
    if valid:
        return True, []
    status_names = {}
    for name in dir(BRepCheckModule):
        if not name.startswith("BRepCheck_") or name == "BRepCheck_Status":
            continue
        value = getattr(BRepCheckModule, name)
        try:
            status_names[int(value)] = name.replace("BRepCheck_", "")
        except (TypeError, ValueError):
            pass
    found = set()
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        result = analyzer.Result(explorer.Current())
        for status in result.Status():
            status_value = int(status)
            if status_value != int(BRepCheckModule.BRepCheck_NoError):
                found.add(status_names.get(status_value, str(status_value)))
        explorer.Next()
    return False, sorted(found)


def _subtract(first, second):
    return tuple(first[index] - second[index] for index in range(3))


def _cross(first, second):
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _length(vector):
    return math.sqrt(sum(value * value for value in vector))
