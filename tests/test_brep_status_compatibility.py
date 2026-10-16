import types
import unittest
from unittest import mock

import surface_segmentation as segmentation
from OCC.Core import BRepCheck


class BRepStatusCompatibilityTests(unittest.TestCase):
    def test_installed_native_status_list_can_be_read_twice_without_mutation(self):
        statuses = BRepCheck.BRepCheck_ListOfStatus()
        statuses.Append(BRepCheck.BRepCheck_NoError)
        statuses.Append(BRepCheck.BRepCheck_UnorientableShape)
        expected = [BRepCheck.BRepCheck_NoError, BRepCheck.BRepCheck_UnorientableShape]
        self.assertEqual(list(segmentation._iter_brep_statuses(statuses)), expected)
        self.assertEqual(list(segmentation._iter_brep_statuses(statuses)), expected)
        self.assertEqual(statuses.Size(), 2)

    def test_python_iterable_statuses_remain_supported(self):
        statuses = [BRepCheck.BRepCheck_UnorientableShape]
        self.assertEqual(list(segmentation._iter_brep_statuses(statuses)), statuses)

    def test_non_iterable_statuses_keep_invalid_shape_diagnostics_and_legacy_constructor(self):
        # Model a binding with no __iter__, and an old two-argument analyzer.
        native_values = [BRepCheck.BRepCheck_NoError, BRepCheck.BRepCheck_UnorientableShape]
        statuses = types.SimpleNamespace(this=native_values)

        class NativeIterator:
            def __init__(self, values):
                self.values = values
                self.index = 0

            def More(self):
                return self.index < len(self.values)

            def Value(self):
                return self.values[self.index]

            def Next(self):
                self.index += 1

        class FaceExplorer:
            def __init__(self, *args):
                self.more = True

            def More(self):
                return self.more

            def Current(self):
                return object()

            def Next(self):
                self.more = False

        result = types.SimpleNamespace(Status=lambda: statuses)
        analyzer = types.SimpleNamespace(IsValid=lambda: False, Result=lambda face: result)
        shape = object()
        with mock.patch.object(segmentation, 'BRepCheck_Analyzer', side_effect=[TypeError(), analyzer]) as factory, \
                mock.patch.object(segmentation, 'TopExp_Explorer', FaceExplorer), \
                mock.patch.object(BRepCheck, 'BRepCheck_ListIteratorOfListOfStatus', NativeIterator):
            self.assertEqual(segmentation._shape_validity(shape), (False, ['UnorientableShape']))
            self.assertEqual(factory.call_args_list, [mock.call(shape, True, True, True),
                                                     mock.call(shape, True)])
        self.assertEqual(len(native_values), 2)


if __name__ == '__main__':
    unittest.main()
