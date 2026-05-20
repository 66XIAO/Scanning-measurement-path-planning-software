"""Collision detection: sensor volumes, OBB generation, sweep tests.

All functions are *pure* — they receive geometry data as arguments and return
results without touching any global state or the OCC viewer.
"""

from dataclasses import dataclass
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Dir, gp_Ax2
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.Bnd import Bnd_OBB
from OCC.Display.OCCViewer import rgb_color

from geometry import generate_face_obb, ConvertBndToShape


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class CollisionSummary:
    total: int
    collisions: int

    @property
    def clear(self):
        return self.total - self.collisions


# ---------------------------------------------------------------------------
# Sensor volume creation
# ---------------------------------------------------------------------------

def create_sensor_volume(viewpoint, orientation, size_config):
    """Create a box-shaped sensor volume at *viewpoint*.

    Parameters
    ----------
    viewpoint : gp_Pnt
    orientation : tuple of (w, x, y, z) quaternion — **required**
    size_config : dict with keys 'width', 'height', 'depth'

    Returns
    -------
    (sensor_box_shape, world_vertices, sensor_color) or (None, None, None)
    """
    try:
        if orientation is None:
            raise ValueError("Missing pose info. Sensor volume creation requires valid quaternion pose")
        if not (isinstance(orientation, (tuple, list)) and len(orientation) >= 4):
            raise ValueError("Invalid pose info: {}. Expected quaternion (w, x, y, z)".format(orientation))

        w, x, y, z = orientation

        # quaternion -> rotation matrix
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        m11 = 1 - 2 * (yy + zz)
        m12 = 2 * (xy - wz)
        m13 = 2 * (xz + wy)
        m21 = 2 * (xy + wz)
        m22 = 1 - 2 * (xx + zz)
        m23 = 2 * (yz - wx)
        m31 = 2 * (xz - wy)
        m32 = 2 * (yz + wx)
        m33 = 1 - 2 * (xx + yy)

        x_axis = gp_Dir(m11, m21, m31)
        y_axis = gp_Dir(m12, m22, m32)
        z_axis = gp_Dir(m13, m23, m33)

        # Flip Z so the box grows opposite to the viewing direction
        z_rev = gp_Dir(-z_axis.X(), -z_axis.Y(), -z_axis.Z())

        sw = size_config.get('width', 80)
        sh = size_config.get('height', 60)
        sd = size_config.get('depth', 80)
        half_w = sw / 2
        half_d = sd / 2

        sensor_ax2 = gp_Ax2(viewpoint, z_rev, x_axis)

        x_dir = sensor_ax2.XDirection()
        y_dir = sensor_ax2.YDirection()
        offset = gp_Vec(x_dir).Multiplied(-half_w).Added(gp_Vec(y_dir).Multiplied(-half_d))

        box_origin = viewpoint.Translated(offset)
        box_ax2 = gp_Ax2(box_origin, z_rev, x_axis)

        sensor_box = BRepPrimAPI_MakeBox(box_ax2, sw, sd, sh).Shape()

        # 8 world-space vertices
        world_vertices = []
        for lz in [0, sh]:
            for ly in [-half_d, half_d]:
                for lx in [-half_w, half_w]:
                    vp = viewpoint \
                        .Translated(gp_Vec(x_dir).Multiplied(lx)) \
                        .Translated(gp_Vec(y_dir).Multiplied(ly)) \
                        .Translated(gp_Vec(z_rev).Multiplied(lz))
                    world_vertices.append(vp)

        sensor_color = rgb_color(0.2, 0.8, 0.2)
        return sensor_box, world_vertices, sensor_color

    except Exception as e:
        print("Error creating sensor volume: {}".format(str(e)))
        return None, None, None


# ---------------------------------------------------------------------------
# OBB-vs-vertices collision check
# ---------------------------------------------------------------------------

def check_collision_obb_vertices(face_obb, sensor_vertices):
    """Return True if *face_obb* intersects the OBB built from *sensor_vertices*."""
    try:
        sensor_obb = Bnd_OBB()
        for v in sensor_vertices:
            sensor_obb.Add(v)
        return not face_obb.IsOut(sensor_obb)
    except Exception as e:
        print("Collision detection error: {}".format(str(e)))
        import traceback
        traceback.print_exc()
        return False


def check_collision_sweep(obb_list, sensor_volumes):
    """Batch collision detection.

    Parameters
    ----------
    obb_list : list of Bnd_OBB
    sensor_volumes : list of (box_shape, vertices, color)

    Returns
    -------
    list of bool — True means collision detected
    """
    results = []
    for box, vertices, _ in sensor_volumes:
        hit = False
        for obb in obb_list:
            if check_collision_obb_vertices(obb, vertices):
                hit = True
                break
        results.append(hit)
    return results


def summarize_collision_results(results):
    """Return a CollisionSummary from a list of booleans."""
    return CollisionSummary(total=len(results), collisions=sum(1 for r in results if r))


# ---------------------------------------------------------------------------
# Display helpers for sensor volumes and OBBs
# ---------------------------------------------------------------------------

def display_sensor_volumes(display, sensor_volumes_list, transparency=0.3):
    """Display all sensor volumes and return display handles."""
    handles = []
    for box, _, color in sensor_volumes_list:
        obj = display.DisplayShape(box, color=color, transparency=transparency, update=False)
        if obj:
            handles.append(obj)
    return handles


def display_obb_boxes(display, face_obbs, transparency=0.7):
    """Display all OBB boxes and return display handles."""
    handles = []
    for obb in face_obbs:
        shape = ConvertBndToShape(obb)
        if shape:
            obj = display.DisplayShape(shape, color=rgb_color(0, 0, 1),
                                       transparency=transparency, update=False)
            if obj:
                handles.append(obj)
    return handles
