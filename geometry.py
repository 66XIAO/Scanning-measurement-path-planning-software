"""Geometry operations: face normals, segmentation, OBB, viewpoints, coordinate systems.

All functions in this module are *pure* or receive the OCC viewer (``display``)
as an explicit parameter so that no global state is required.
"""

from dataclasses import asdict, dataclass, field
import math
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir, gp_Ax1, gp_Ax2, gp_Ax3, gp_Trsf, gp_XYZ
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepBndLib import brepbndlib, brepbndlib_AddOBB
from OCC.Core.Bnd import Bnd_Box, Bnd_OBB
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeSphere
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.ShapeAnalysis import ShapeAnalysis_Surface
from OCC.Core.BOPAlgo import BOPAlgo_Splitter
from OCC.Extend.TopologyUtils import TopologyExplorer
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.Geom import Geom_Axis2Placement
from OCC.Core.AIS import AIS_Trihedron
from OCC.Display.OCCViewer import rgb_color

from surface_segmentation import (
    inspect_shape_topology,
    is_surface_patch,
    segment_shape_mesh_grid,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clamp_segments(value, minimum=1, maximum=100):
    """Clamp segmentation count to a sane range."""
    return max(minimum, min(maximum, int(value)))


# ---------------------------------------------------------------------------
# Face normal
# ---------------------------------------------------------------------------

def calculate_face_normal(face, reference_normal=None):
    """Calculate the outward-pointing normal of *face*.

    Parameters
    ----------
    face : TopoDS_Face
    reference_normal : gp_Vec or None
        When given, the returned normal is flipped (if necessary) so that
        its dot-product with *reference_normal* is positive.  This keeps
        normals consistent across split patches of the same original face.

    Returns
    -------
    gp_Vec
    """
    if is_surface_patch(face):
        normal = gp_Vec(face.normal.X(), face.normal.Y(), face.normal.Z())
        return _align_normal(normal, reference_normal)
    try:
        adaptor = BRepAdaptor_Surface(face)

        try:
            u_min = adaptor.FirstUParameter()
            u_max = adaptor.LastUParameter()
            v_min = adaptor.FirstVParameter()
            v_max = adaptor.LastVParameter()
        except Exception:
            u_min, u_max, v_min, v_max = 0.0, 1.0, 0.0, 1.0

        u_mid = (u_min + u_max) / 2.0
        v_mid = (v_min + v_max) / 2.0

        # --- primary method: partial derivatives at centre ---
        normal = _normal_from_derivatives(adaptor, u_mid, v_mid, reference_normal)

        if normal is None:
            # --- fallback: three surface points ---
            normal = _normal_from_points(face, u_min, u_max, v_min, v_max, reference_normal)

        if normal is None:
            normal = _default_normal(reference_normal)

        return normal

    except Exception as e:
        print("Error calculating normal: {}".format(str(e)))
        return _default_normal(reference_normal)


def _normal_from_derivatives(adaptor, u, v, reference_normal):
    """Compute normal via D1 partial derivatives."""
    try:
        d1u = gp_Vec()
        d1v = gp_Vec()
        p = gp_Pnt()
        adaptor.D1(u, v, p, d1u, d1v)
        normal = d1u.Crossed(d1v)
        if normal.SquareMagnitude() > 1e-10:
            normal.Normalize()
            return _align_normal(normal, reference_normal)
    except Exception:
        pass
    return None


def _normal_from_points(face, u_min, u_max, v_min, v_max, reference_normal):
    """Compute normal from three sample points on the surface."""
    try:
        p1 = BRep_Tool.Value(u_min, v_min, face)
        p2 = BRep_Tool.Value(u_max, v_min, face)
        p3 = BRep_Tool.Value(u_min, v_max, face)
        vec1 = gp_Vec(p1, p2)
        vec2 = gp_Vec(p1, p3)
        normal = vec1.Crossed(vec2)
        if normal.SquareMagnitude() > 1e-10:
            normal.Normalize()
            return _align_normal(normal, reference_normal)
    except Exception:
        pass
    return None


def _align_normal(normal, reference_normal):
    """Flip *normal* so it points in the same hemisphere as *reference_normal*."""
    if reference_normal is not None:
        if normal.Dot(reference_normal) < 0:
            normal = normal.Reversed()
    else:
        up = gp_Vec(0, 0, 1)
        if normal.Dot(up) < 0:
            normal = normal.Reversed()
    return normal


def _default_normal(reference_normal):
    if reference_normal is not None:
        return reference_normal.Normalized()
    return gp_Vec(0, 0, 1)


# ---------------------------------------------------------------------------
# Viewpoint generation
# ---------------------------------------------------------------------------

def generate_viewpoint(center, normal, distance=300.0):
    """Offset *center* along *normal* by *distance* and return the new point."""
    try:
        if normal.Magnitude() > 0:
            n = gp_Vec(normal.X(), normal.Y(), normal.Z())
            n.Normalize()
            scaled = gp_Vec(n.X() * distance, n.Y() * distance, n.Z() * distance)
            vp = gp_Vec(center.X(), center.Y(), center.Z()) + scaled
            return gp_Pnt(vp.X(), vp.Y(), vp.Z())
    except Exception as e:
        print("Error generating viewpoint: {}".format(str(e)))
    return center


# ---------------------------------------------------------------------------
# Coordinate-system display
# ---------------------------------------------------------------------------

def calculate_workpiece_coordinate_size(shape, ratio=0.08, fallback=50.0):
    """Return an axis size proportional to the imported CAD bounding box.

    The value uses the same units as the CAD model. No absolute clamp is
    applied, so the visual remains scale-independent for models authored in
    millimetres, metres, or another consistent unit system.
    """
    try:
        if shape is None:
            return float(fallback)
        bounds = Bnd_Box()
        brepbndlib.Add(shape, bounds)
        if bounds.IsVoid():
            return float(fallback)
        x_min, y_min, z_min, x_max, y_max, z_max = bounds.Get()
        diagonal = math.sqrt(
            (x_max - x_min) ** 2
            + (y_max - y_min) ** 2
            + (z_max - z_min) ** 2)
        size = diagonal * float(ratio)
        if math.isfinite(size) and size > 1.0e-9:
            return size
    except Exception as e:
        print("Error calculating workpiece coordinate size: {}".format(str(e)))
    return float(fallback)


def display_workpiece_coordinate_system(display, shape=None, size=None):
    """Display the imported CAD global frame at the true origin.

    This trihedron is fixed at ``(0, 0, 0)`` with canonical X/Y/Z axes. It is
    marked infinite so it does not enlarge ``FitAll`` bounds, and its selection
    modes are deactivated without changing the viewer's global auto-selection
    policy.
    """
    try:
        axis_size = (
            calculate_workpiece_coordinate_size(shape)
            if size is None else float(size))
        ax2 = gp_Ax2(gp_Pnt(0.0, 0.0, 0.0),
                     gp_Dir(0.0, 0.0, 1.0),
                     gp_Dir(1.0, 0.0, 0.0))
        trihedron = AIS_Trihedron(Geom_Axis2Placement(ax2))
        trihedron.SetSize(axis_size)
        trihedron.SetInfiniteState(True)

        ctx = display.Context
        ctx.Display(trihedron, False)
        ctx.Deactivate(trihedron)
        return trihedron
    except Exception as e:
        print("Error creating workpiece coordinate system: {}".format(str(e)))
        return None

def display_coordinate_system(display, position, normal, size=50.0, center=None):
    """Display a small trihedron at *position* and return its AIS handle.

    Parameters
    ----------
    display : OCC viewer handle
    position : gp_Pnt
    normal : gp_Vec
    size : float
    center : gp_Pnt or None
        When given, the Z axis of the trihedron points from *position*
        towards *center*.
    """
    try:
        z_vec = _compute_z_axis(position, normal, center)
        x_vec = _orthogonal_x(z_vec)

        ax2 = gp_Ax2(position, gp_Dir(z_vec), gp_Dir(x_vec))
        axis_placement = Geom_Axis2Placement(ax2)
        trihedron = AIS_Trihedron(axis_placement)
        trihedron.SetSize(size)

        ctx = display.Context
        ctx.SetAutoActivateSelection(False)
        ctx.Display(trihedron, True)
        return trihedron
    except Exception as e:
        print("Error creating coordinate system: {}".format(str(e)))
        return None


def _compute_z_axis(position, normal, center):
    """Return the Z-axis direction vector."""
    if center is not None:
        z = gp_Vec(position, center)
        if z.Magnitude() > 0:
            z.Normalize()
            return z
    if isinstance(normal, gp_Vec):
        return normal.Normalized()
    return gp_Vec(normal.X(), normal.Y(), normal.Z()).Normalized()


def _orthogonal_x(z_vec):
    """Gram-Schmidt: find an X axis perpendicular to *z_vec*."""
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
# OBB
# ---------------------------------------------------------------------------

def ConvertBndToShape(theBox):
    """Convert a ``Bnd_OBB`` to a ``TopoDS_Shape`` box for visualisation."""
    try:
        c = theBox.Center()
        xd = theBox.XDirection()
        yd = theBox.YDirection()
        zd = theBox.ZDirection()
        hx = theBox.XHSize()
        hy = theBox.YHSize()
        hz = theBox.ZHSize()

        ax = gp_XYZ(xd.X(), xd.Y(), xd.Z())
        ay = gp_XYZ(yd.X(), yd.Y(), yd.Z())
        az = gp_XYZ(zd.X(), zd.Y(), zd.Z())

        p = gp_Pnt(c.X(), c.Y(), c.Z())
        axes = gp_Ax2(p, gp_Dir(zd), gp_Dir(xd))
        axes.SetLocation(gp_Pnt(p.XYZ() - ax * hx - ay * hy - az * hz))

        return BRepPrimAPI_MakeBox(axes, 2.0 * hx, 2.0 * hy, 2.0 * hz).Shape()
    except Exception as e:
        print("Error converting OBB to shape: {}".format(str(e)))
        import traceback
        traceback.print_exc()
        return None


def generate_face_obb(face):
    """Generate an OBB bounding box for *face*.

    Returns
    -------
    (obb, obb_shape, obb_color) or (None, None, None)
    """
    if is_surface_patch(face):
        try:
            xmin, ymin, zmin, xmax, ymax, zmax = face.bounds
            obb = Bnd_OBB()
            obb.SetCenter(gp_Pnt(
                (xmin + xmax) / 2.0,
                (ymin + ymax) / 2.0,
                (zmin + zmax) / 2.0))
            obb.SetXComponent(gp_Dir(1, 0, 0), max((xmax - xmin) / 2.0, 1e-6))
            obb.SetYComponent(gp_Dir(0, 1, 0), max((ymax - ymin) / 2.0, 1e-6))
            obb.SetZComponent(gp_Dir(0, 0, 1), max((zmax - zmin) / 2.0, 1e-6))
            return obb, ConvertBndToShape(obb), rgb_color(0, 0, 1)
        except Exception as e:
            print("Error generating mesh-grid patch OBB: {}".format(str(e)))
            return None, None, None
    try:
        obb = Bnd_OBB()
        brepbndlib_AddOBB(face, obb)
        obb_shape = ConvertBndToShape(obb)
        obb_color = rgb_color(0, 0, 1)
        return obb, obb_shape, obb_color
    except Exception as e:
        print("Error generating OBB: {}".format(str(e)))
        import traceback
        traceback.print_exc()
        return None, None, None


# ---------------------------------------------------------------------------
# Surface segmentation
# ---------------------------------------------------------------------------


@dataclass
class SegmentationDiagnostics:
    strategy: str = "equal_param"
    original_face_count: int = 0
    patch_count: int = 0
    original_area: float = 0.0
    patch_area_sum: float = 0.0
    area_ratio: float = 1.0
    split_attempts: int = 0
    effective_splits: int = 0
    split_no_effect: int = 0
    split_exceptions: int = 0
    area_rejected_splits: int = 0
    fallback_faces: int = 0
    periodic_faces: int = 0
    warnings: list = field(default_factory=list)


def _surface_area(shape):
    props = GProp_GProps()
    brepgprop.SurfaceProperties(shape, props)
    return float(props.Mass())


def segment_model(shape, u=6, v=4, return_diagnostics=False, area_tolerance=0.01,
                  strategy="equal_param", mesh_linear_deflection=None,
                  mesh_angular_deflection=0.5):
    """Segment *shape* into ``u * v`` patches per original face.

    Uses iso-parametric lines (``ShapeAnalysis_Surface``) and
    ``BOPAlgo_Splitter`` for equally-spaced splitting.

    Returns
    -------
    list of TopoDS_Face, or ``(patches, diagnostics_dict)`` when
    ``return_diagnostics`` is true.
    """
    requested_strategy = strategy
    if strategy == "auto":
        topology = inspect_shape_topology(shape)
        strategy = "mesh_grid" if topology["has_trimmed_faces"] else "equal_param"
    if strategy == "mesh_grid":
        patches, diagnostics = segment_shape_mesh_grid(
            shape, u, v,
            linear_deflection=mesh_linear_deflection,
            angular_deflection=mesh_angular_deflection,
            area_tolerance=max(area_tolerance, 0.02))
        if requested_strategy == "mesh_grid":
            diagnostics["strategy_reason"] = "explicit_request"
        if return_diagnostics:
            return patches, diagnostics
        return patches
    if strategy != "equal_param":
        raise ValueError("Unsupported segmentation strategy: {}".format(strategy))

    try:
        diagnostics = SegmentationDiagnostics()
        patches = []
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            face = exp.Current()
            diagnostics.original_face_count += 1
            try:
                diagnostics.original_area += _surface_area(face)
                adaptor = BRepAdaptor_Surface(face)
                if adaptor.IsUPeriodic() or adaptor.IsVPeriodic():
                    diagnostics.periodic_faces += 1
            except Exception as e:
                diagnostics.warnings.append("Could not inspect face {}: {}".format(
                    diagnostics.original_face_count, e))
            try:
                face_patches = _segment_single_face(face, u, v, diagnostics)
                patches.extend(face_patches)
            except Exception as e:
                print("Error splitting single face: {}".format(str(e)))
                diagnostics.fallback_faces += 1
                patches.append(face)
            exp.Next()

        if not patches:
            exp = TopExp_Explorer(shape, TopAbs_FACE)
            while exp.More():
                patches.append(exp.Current())
                exp.Next()

        diagnostics.patch_count = len(patches)
        for patch in patches:
            try:
                diagnostics.patch_area_sum += _surface_area(patch)
            except Exception as e:
                diagnostics.warnings.append("Could not calculate patch area: {}".format(e))
        if diagnostics.original_area > 1e-12:
            diagnostics.area_ratio = diagnostics.patch_area_sum / diagnostics.original_area
            if abs(diagnostics.area_ratio - 1.0) > area_tolerance:
                diagnostics.warnings.append(
                    "Area conservation failed: patch/original={:.6f} (tolerance={:.3%})".format(
                        diagnostics.area_ratio, area_tolerance))
        else:
            diagnostics.area_ratio = 0.0
            diagnostics.warnings.append("Original surface area is zero or unavailable")
        if diagnostics.split_exceptions:
            diagnostics.warnings.append("{} iso split operations raised exceptions".format(
                diagnostics.split_exceptions))
        if diagnostics.area_rejected_splits:
            diagnostics.warnings.append(
                "{} iso split operations were rejected because their areas did not conserve".format(
                    diagnostics.area_rejected_splits))
        if diagnostics.fallback_faces:
            diagnostics.warnings.append("{} original faces used fallback output".format(
                diagnostics.fallback_faces))
        if diagnostics.periodic_faces:
            diagnostics.warnings.append(
                "{} periodic faces detected; inspect seam-adjacent patches".format(
                    diagnostics.periodic_faces))

        print("Model segmentation complete, total {} patches (u={}, v={}), area ratio {:.6f}".format(
            len(patches), u, v, diagnostics.area_ratio))
        if return_diagnostics:
            result = asdict(diagnostics)
            if requested_strategy == "auto":
                result["strategy_reason"] = "untrimmed_faces"
            return patches, result
        return patches
    except Exception as e:
        print("Error segmenting model: {}".format(str(e)))
        import traceback
        traceback.print_exc()
        return []


def _segment_single_face(face, u, v, diagnostics=None):
    """Segment one face into u*v patches."""
    surf = BRep_Tool.Surface(face)
    adaptor = BRepAdaptor_Surface(face)
    try:
        u_min = adaptor.FirstUParameter()
        u_max = adaptor.LastUParameter()
        v_min = adaptor.FirstVParameter()
        v_max = adaptor.LastVParameter()
    except Exception:
        u_min, u_max, v_min, v_max = 0.0, 1.0, 0.0, 1.0

    sas = ShapeAnalysis_Surface(surf)

    if u == 1 and v == 1:
        return [face]

    # --- split in V direction ---
    v_faces = [face]
    if v > 1:
        for level in range(v - 1):
            new_v = []
            any_split = False
            for vf in v_faces:
                v_pos = v_min + (v_max - v_min) * (level + 1) / v
                result = _split_face_iso(sas, vf, v_pos, axis="v",
                                         diagnostics=diagnostics)
                if len(result) >= 2:
                    new_v.extend(result)
                    any_split = True
                else:
                    new_v.append(vf)
            if any_split:
                v_faces = new_v
            else:
                break

    # --- split in U direction ---
    patches = []
    for vf in v_faces:
        if u > 1:
            u_faces = [vf]
            for level in range(u - 1):
                new_u = []
                any_split = False
                for uf in u_faces:
                    u_pos = u_min + (u_max - u_min) * (level + 1) / u
                    result = _split_face_iso(sas, uf, u_pos, axis="u",
                                             diagnostics=diagnostics)
                    if len(result) >= 2:
                        new_u.extend(result)
                        any_split = True
                    else:
                        new_u.append(uf)
                if any_split:
                    u_faces = new_u
                else:
                    break
            patches.extend(u_faces)
        else:
            patches.append(vf)

    return patches


def _split_face_iso(sas, face, param, axis="u", diagnostics=None):
    """Split *face* along one iso-parametric line."""
    if diagnostics is not None:
        diagnostics.split_attempts += 1
    try:
        if axis == "u":
            iso = sas.UIso(param)
        else:
            iso = sas.VIso(param)
        edge = BRepBuilderAPI_MakeEdge(iso).Edge()

        splitter = BOPAlgo_Splitter()
        splitter.AddArgument(face)
        splitter.AddTool(edge)
        splitter.Perform()

        faces = _collect_faces(splitter.Shape())
        if len(faces) >= 2:
            input_area = _surface_area(face)
            output_area = sum(_surface_area(result_face) for result_face in faces)
            relative_error = abs(output_area - input_area) / max(input_area, 1e-12)
            if relative_error > 1e-6:
                if diagnostics is not None:
                    diagnostics.area_rejected_splits += 1
                return [face]
        if diagnostics is not None:
            if len(faces) >= 2:
                diagnostics.effective_splits += 1
            else:
                diagnostics.split_no_effect += 1
        return faces or [face]
    except Exception:
        if diagnostics is not None:
            diagnostics.split_exceptions += 1
        return [face]


def _collect_faces(shape):
    """Collect all faces from *shape*."""
    faces = []
    explorer = TopologyExplorer(shape)
    for f in explorer.faces():
        faces.append(f)
    return faces
