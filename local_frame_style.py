"""Validated view preferences for the small face and viewpoint frames."""
import math
import re


DEFAULT_LOCAL_FRAME_STYLE = {
    "face_center_size": 10.0,
    "center_view_size": 8.0,
    "candidate_view_size": 6.0,
    "labels_visible": True,
    "label_height": 11.0,
    "label_color": "#FFD24D",
    "label_font": "",
}


def normalize_local_frame_style(values):
    if not isinstance(values, dict):
        raise ValueError("Local frame style must be an object")
    result = dict(DEFAULT_LOCAL_FRAME_STYLE)
    for key in ("face_center_size", "center_view_size", "candidate_view_size"):
        if key in values:
            value = float(values[key])
            if not math.isfinite(value) or not 0.1 <= value <= 100000.0:
                raise ValueError("Local frame size is out of range")
            result[key] = value
    if "labels_visible" in values:
        value = values["labels_visible"]
        if not isinstance(value, bool):
            raise ValueError("labels_visible must be a boolean")
        result["labels_visible"] = value
    if "label_height" in values:
        value = float(values["label_height"])
        if not math.isfinite(value) or not 6.0 <= value <= 48.0:
            raise ValueError("Label height is out of range")
        result["label_height"] = value
    if "label_color" in values:
        value = values["label_color"]
        if not isinstance(value, str) or not re.fullmatch(r"#[0-9a-fA-F]{6}", value):
            raise ValueError("Label color must be #RRGGBB")
        result["label_color"] = value.upper()
    if "label_font" in values:
        value = values["label_font"]
        if not isinstance(value, str) or len(value) > 80 or "\x00" in value:
            raise ValueError("Invalid label font")
        result["label_font"] = value
    return result


def read_local_frame_style(settings):
    values = {}
    for key, default in DEFAULT_LOCAL_FRAME_STYLE.items():
        stored = settings.value("localFrames/" + key, default)
        if key == "labels_visible" and isinstance(stored, str):
            stored = stored.lower() in ("true", "1", "yes")
        values[key] = stored
    try:
        return normalize_local_frame_style(values)
    except (TypeError, ValueError):
        return dict(DEFAULT_LOCAL_FRAME_STYLE)


def write_local_frame_style(settings, values):
    normalized = normalize_local_frame_style(values)
    for key, value in normalized.items():
        settings.setValue("localFrames/" + key, value)
    return normalized
