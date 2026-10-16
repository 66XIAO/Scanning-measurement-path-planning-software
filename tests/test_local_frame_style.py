import unittest

from local_frame_style import (DEFAULT_LOCAL_FRAME_STYLE,
                               normalize_local_frame_style,
                               read_local_frame_style, write_local_frame_style)


class FakeSettings:
    def __init__(self):
        self.items = {}

    def value(self, key, default=None):
        return self.items.get(key, default)

    def setValue(self, key, value):
        self.items[key] = value


class LocalFrameStyleTests(unittest.TestCase):
    def test_settings_round_trip_and_invalid_saved_value_falls_back(self):
        settings = FakeSettings()
        chosen = write_local_frame_style(settings, {
            "face_center_size": 15,
            "labels_visible": False,
            "label_color": "#11aabb",
        })
        self.assertEqual(read_local_frame_style(settings), chosen)
        settings.setValue("localFrames/label_height", "not a number")
        self.assertEqual(read_local_frame_style(settings), DEFAULT_LOCAL_FRAME_STYLE)

    def test_bad_sizes_and_label_options_are_rejected(self):
        for values in ({"face_center_size": 0},
                       {"candidate_view_size": float("nan")},
                       {"label_height": 50},
                       {"labels_visible": "false"},
                       {"label_color": "yellow"}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                normalize_local_frame_style(values)
