import ast
from pathlib import Path
import unittest

from OCC.Core.gp import gp_Pnt, gp_Vec

from viewpoints import build_candidate_viewpoints, build_center_viewpoints


class _NoDisplayCalls:
    def __getattr__(self, name):
        raise AssertionError("render=False unexpectedly used display.{}".format(name))


class ViewpointPureGenerationTests(unittest.TestCase):
    def test_center_generation_can_run_without_display_side_effects(self):
        result = build_center_viewpoints(
            _NoDisplayCalls(), [object()], [gp_Pnt(0, 0, 0)],
            [gp_Vec(0, 0, 1)], render=False)
        self.assertEqual(len(result[0]), 1)
        self.assertEqual(result[2], [])

    def test_candidate_generation_can_run_without_display_side_effects(self):
        result = build_candidate_viewpoints(
            _NoDisplayCalls(), [object()], [gp_Pnt(0, 0, 0)],
            [gp_Vec(0, 0, 1)], num_candidates=2, render=False)
        self.assertEqual(len(result[0]), 3)
        self.assertEqual(result[3], [])
        self.assertEqual(result[4], [])


class MainRenderPipelineSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main_path = Path(__file__).resolve().parents[1] / "main.py"
        cls.tree = ast.parse(cls.main_path.read_text(encoding="utf-8"))
        cls.functions = {
            node.name: node for node in cls.tree.body
            if isinstance(node, ast.FunctionDef)}

    def test_model_processing_handlers_use_one_unified_render_call(self):
        for name in (
                "get_centers", "generate_center_viewpoints",
                "generate_candidate_viewpoints", "filter_optimal_viewpoints"):
            node = self.functions[name]
            unified_renders = [
                call for call in ast.walk(node)
                if isinstance(call, ast.Call)
                and isinstance(call.func, ast.Name)
                and call.func.id == "_do_render"]
            direct_display_calls = [
                call for call in ast.walk(node)
                if isinstance(call, ast.Call)
                and isinstance(call.func, ast.Attribute)
                and call.func.attr in ("DisplayShape", "Repaint", "FitAll")]
            self.assertEqual(len(unified_renders), 1, name)
            self.assertEqual(direct_display_calls, [], name)

    def test_drop_dispatch_has_separate_workstation_and_cad_targets(self):
        node = self.functions["open_dropped_file"]
        called_names = {
            call.func.id for call in ast.walk(node)
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)}
        self.assertIn("open_workstation_from_path", called_names)
        self.assertIn("import_model_from_path", called_names)


if __name__ == "__main__":
    unittest.main()
