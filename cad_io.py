"""CAD loading with a stable single-shape contract across pythonOCC versions."""

import os

from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Extend.DataExchange import read_iges_file, read_step_file


def combine_shapes(value):
    """Return one TopoDS shape when a reader returns one or many roots."""
    if value is None:
        raise ValueError("CAD reader returned no shape")
    if isinstance(value, (list, tuple)):
        shapes = [shape for shape in value
                  if shape is not None and not shape.IsNull()]
        if not shapes:
            raise ValueError("CAD reader returned no valid shapes")
        if len(shapes) == 1:
            return shapes[0]
        compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(compound)
        for shape in shapes:
            builder.Add(compound, shape)
        return compound
    if value.IsNull():
        raise ValueError("CAD reader returned a null shape")
    return value


def load_cad_shape(file_path):
    extension = os.path.splitext(file_path)[1].lower()
    if extension in (".step", ".stp"):
        return combine_shapes(read_step_file(file_path))
    if extension in (".iges", ".igs"):
        return combine_shapes(read_iges_file(file_path))
    raise ValueError("Unsupported CAD extension: {}".format(extension))
