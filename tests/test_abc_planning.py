"""Contract and regression checks for the bundled ABC path planner."""

from dataclasses import replace
import importlib.util
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

import ABC
from config import ABCConfig
from planning import run_abc_solver


POINTS = np.array([[0, 0, 0], [2, 1, 1], [1, 3, 0], [4, 2, 2], [5, 0, 1]], float)


def configuration(points=POINTS, **overrides):
    parameters = dict(points=points, FOOD_NUMBER=4, LIMIT=8,
                      MAXIMUM_EVALUATION=80, SHOW_PROGRESS=False,
                      VISUALIZE=False, PLOT_HISTORY=False, SEED=2025)
    parameters.update(overrides)
    return ABC.TSPConfig(**parameters)


class ABCPlanningTests(unittest.TestCase):
    def assert_result(self, solver, result):
        tour = result["best_tour"]
        self.assertEqual(sorted(tour), list(range(solver.conf.N)))
        edges = np.diff(solver.conf.points[tour], axis=0)
        length = float(np.linalg.norm(edges, axis=1).sum())
        if solver.conf.CLOSED_TOUR:
            length += np.linalg.norm(solver.conf.points[tour[-1]] - solver.conf.points[tour[0]])
        self.assertAlmostEqual(result["best_length"], length)
        objective, _, corners = solver._objective(tour)
        self.assertAlmostEqual(result["best_objective"], objective)
        self.assertEqual(result["best_corners"], corners)
        self.assertLessEqual(solver.evalCount, solver.conf.MAXIMUM_EVALUATION)
        self.assertEqual(len(result["history_length"]), len(result["history_objective"]))
        self.assertEqual(len(result["history_corners"]), len(result["history_objective"]))
        self.assertTrue(all(a >= b for a, b in zip(
            result["history_objective"], result["history_objective"][1:])))

    def test_project_entrypoint_returns_expected_contract(self):
        cfg = replace(ABCConfig(), food_number=4, maximum_evaluation=80,
                      show_progress=False, visualize=False, plot_history=False)
        result = run_abc_solver(POINTS, cfg)
        self.assertEqual(set(result), {"best_tour", "best_length", "best_corners",
                                      "best_objective", "history_objective",
                                      "history_length", "history_corners"})
        self.assertEqual(sorted(result["best_tour"]), list(range(len(POINTS))))
        self.assertAlmostEqual(result["best_length"], float(np.linalg.norm(
            np.diff(POINTS[result["best_tour"]], axis=0), axis=1).sum()))

    def test_open_closed_and_fixed_endpoints(self):
        for options in ({}, {"CLOSED_TOUR": True}, {"start_index": 2},
                        {"end_index": 1}, {"start_index": 2, "end_index": 1}):
            with self.subTest(options=options):
                solver = ABC.ABC_TSP3D(configuration(**options))
                result = solver.run()
                self.assert_result(solver, result)
                if "start_index" in options:
                    self.assertEqual(result["best_tour"][0], options["start_index"])
                if "end_index" in options:
                    self.assertEqual(result["best_tour"][-1], options["end_index"])

    def test_small_and_coincident_point_sets(self):
        for count in (1, 2, 3):
            for fixed in (False, True):
                if count == 1 and fixed:
                    continue
                options = {"start_index": 0, "end_index": count - 1} if fixed else {}
                with self.subTest(count=count, fixed=fixed):
                    solver = ABC.ABC_TSP3D(configuration(np.zeros((count, 3)), **options))
                    self.assert_result(solver, solver.run())

    def test_budget_can_end_after_initialization(self):
        solver = ABC.ABC_TSP3D(configuration(MAXIMUM_EVALUATION=4))
        self.assert_result(solver, solver.run())
        self.assertEqual(solver.evalCount, 4)

    def test_final_scout_improvement_is_returned(self):
        points = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], float)
        solver = ABC.ABC_TSP3D(configuration(points, FOOD_NUMBER=2, LIMIT=1,
                                            MAXIMUM_EVALUATION=7, weights=(1, 0)))
        bad, good = np.array([0, 2, 1, 3]), np.arange(4)
        with mock.patch.object(solver, "_random_perm_vars", side_effect=[bad, bad, good]), \
                mock.patch.object(solver, "_neighbor", side_effect=lambda perm: perm.copy()):
            result = solver.run()
        self.assertEqual(solver.evalCount, 7)
        self.assertEqual(result["best_length"], 3)
        np.testing.assert_array_equal(result["best_tour"], good)
        self.assertEqual(result["history_length"][-1], 3)

    def test_length_is_geometric_even_when_objective_has_crossing_penalty(self):
        points = np.array([[0, 0, 0], [2, 2, 0], [0, 2, 0], [2, 0, 0], [3, 0, 0]], float)
        solver = ABC.ABC_TSP3D(configuration(points, weights=(1, 0)))
        tour = np.arange(5)
        self.assertTrue(solver._has_intersections(tour))
        self.assertAlmostEqual(solver._objective(tour)[0], solver._tour_length(tour) * 1.5)

    def test_seed_reproducible_and_global_random_states_unchanged(self):
        before_random, before_numpy = random.getstate(), np.random.get_state()
        first = ABC.ABC_TSP3D(configuration()).run()
        second = ABC.ABC_TSP3D(configuration()).run()
        np.testing.assert_array_equal(first["best_tour"], second["best_tour"])
        self.assertEqual(first["history_objective"], second["history_objective"])
        self.assertEqual(random.getstate(), before_random)
        after_numpy = np.random.get_state()
        self.assertEqual(after_numpy[0], before_numpy[0])
        np.testing.assert_array_equal(after_numpy[1], before_numpy[1])
        self.assertEqual(after_numpy[2:], before_numpy[2:])

    def test_instances_can_be_interleaved_without_changing_seeded_run(self):
        baseline = ABC.ABC_TSP3D(configuration()).run()
        solver = ABC.ABC_TSP3D(configuration())
        ABC.ABC_TSP3D(configuration(SEED=123)).run()
        result = solver.run()
        np.testing.assert_array_equal(baseline["best_tour"], result["best_tour"])
        self.assertEqual(baseline["history_objective"], result["history_objective"])

    def test_invalid_points_and_parameters_fail_before_search(self):
        cases = ({"points": []}, {"points": [[1, 2]]},
                 {"points": [[0, float("nan"), 0]]}, {"points": [[0, 0, float("inf")]]},
                 {"FOOD_NUMBER": 0}, {"FOOD_NUMBER": 1.5}, {"LIMIT": 0},
                 {"MAXIMUM_EVALUATION": 0}, {"MAXIMUM_EVALUATION": 3},
                 {"start_index": -1}, {"end_index": len(POINTS)},
                 {"start_index": 0, "end_index": 0},
                 {"CLOSED_TOUR": True, "start_index": 0},
                 {"weights": (0, 0)}, {"weights": (-1, 0)},
                 {"weights": (1, float("nan"))}, {"weights": (1,)},
                 {"angle_threshold_deg": -1}, {"VIS_INTERVAL_EVALS": 0})
        for options in cases:
            with self.subTest(options=options), self.assertRaises(ValueError):
                configuration(**options)

    def test_import_and_headless_solver_need_no_optional_modules_or_files(self):
        root = str(Path(__file__).resolve().parents[1])
        script = '''
import builtins, os, sys
sys.path.insert(0, sys.argv[1])
from runtime_environment import configure_runtime
configure_runtime()
original_import = builtins.__import__
def guarded_import(name, *args, **kwargs):
    if name.split('.')[0] in {'deap', 'matplotlib', 'progressbar', 'progressbar2', 'OCC', 'PyQt5', 'PySide6'}:
        raise ImportError('optional module deliberately absent: ' + name)
    return original_import(name, *args, **kwargs)
builtins.__import__ = guarded_import
import ABC
from planning import run_abc_solver
from config import ABCConfig
result = run_abc_solver([[0,0,0], [1,0,0]], ABCConfig(food_number=2, maximum_evaluation=8))
assert sorted(result['best_tour']) == [0,1]
assert result['best_length'] == 1
assert os.listdir('.') == []
assert 'matplotlib.pyplot' not in sys.modules
'''
        with tempfile.TemporaryDirectory() as directory:
            process = subprocess.run([sys.executable, "-B", "-c", script, root],
                                     cwd=directory, capture_output=True, text=True)
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional matplotlib not installed")
    def test_history_only_creates_directory_and_agg_plots_preserve_backend(self):
        import matplotlib
        backend = matplotlib.get_backend()
        with tempfile.TemporaryDirectory() as directory:
            for visualize in (False, True):
                target = Path(directory) / str(visualize)
                conf = configuration(MAXIMUM_EVALUATION=12, VISUALIZE=visualize,
                                     PLOT_HISTORY=True, VIS_SAVE_DIR=str(target))
                ABC.ABC_TSP3D(conf).run()
                self.assertTrue((target / "history.png").is_file())
                self.assertEqual(bool(list(target.glob("frame_*.png"))), visualize)
                for path in target.glob("*.png"):
                    self.assertGreater(path.stat().st_size, 100)
        self.assertEqual(matplotlib.get_backend(), backend)


if __name__ == "__main__":
    unittest.main()
