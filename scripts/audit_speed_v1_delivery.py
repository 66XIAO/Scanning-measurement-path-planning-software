"""Audit all required V1 delivery artifacts and the final live station inventory."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robodk_bridge as bridge


ROOT = Path(__file__).resolve().parents[1]
CASE = ROOT / "artifacts" / "speed_v1" / "fair_playback_20260919_093222"
EXPECTED_PROGRAMS = {
    "A",
    "V1_A_stop_20260917_235523",
    "V1_Joint_20260918_182439",
    "A_stop_fair_playback_20260919_093222",
    "B_fixed_fair_playback_20260919_093222",
    "C_variable_fair_playback_20260919_093222",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    report = json.loads((CASE / "final_report.json").read_text(encoding="utf-8"))
    a, b, c = (report[k] for k in ("A_stop", "B_fixed", "C_variable"))
    require(a["wall_time_s"] > b["wall_time_s"] > c["wall_time_s"], "Playback ordering is not A > B > C")
    require(math.isclose(report["comparison"]["C_vs_A_time_reduction_percent"], 19.31999324181576), "C/A result drifted")
    require(report["comparison"]["all_native_constraints_pass"], "Native constraint audit failed")
    require(report["comparison"]["continuous_arms_structurally_pass"], "Continuous structure audit failed")
    for arm in (a, b, c):
        require(arm["update_valid_ratio"] == 1.0, f"Update invalid for {arm['program']}")
        require(arm["trajectory_message"] == "Success", f"Trajectory failed for {arm['program']}")
        require(arm["trajectory_error_codes"] == [0], f"Trajectory error for {arm['program']}")
        require(arm["native_joint_limits_pass"], f"Native limits failed for {arm['program']}")
        require(arm["export_rounding_matches_commands"], f"Export mismatch for {arm['program']}")
    for arm in (b, c):
        require(arm["max_position_error_mm"] <= 1.5, "Position tolerance failed")
        require(arm["max_orientation_error_deg"] <= 2.0, "Orientation tolerance failed")
        require(arm["internal_rounding_nonzero"], "Internal rounding is zero")
        require(arm["endpoint_rounding_zero"], "Endpoint does not stop")

    required_files = [
        ROOT / "docs" / "user" / "continuous_scan_v1.md",
        ROOT / "docs" / "reports" / "speed_planning_v1_final.md",
        CASE / "summary.csv",
        CASE / "C_variable" / "C_variable_fair_playback_20260919_093222.script",
        CASE / "final_figure_20260919" / "08_previews" / "speed_v1_final.png",
        CASE / "final_figure_20260919" / "07_svg_export" / "speed_v1_final.svg",
        CASE / "final_figure_20260919" / "source_data.csv",
        ROOT / "artifacts" / "speed_v1" / "station_backups" / "20260919_final" / "before_final_cleanup.rdk",
        ROOT / "artifacts" / "speed_v1" / "station_backups" / "20260919_final" / "after_final_cleanup.rdk",
    ]
    for path in required_files:
        require(path.is_file() and path.stat().st_size > 0, f"Missing delivery artifact: {path}")

    api = bridge._import_api()
    rdk = api["Robolink"]()
    require(rdk.ActiveStation().Name() == "UR10_大曲面_MSCGA_corrected", "Wrong active station")
    programs = set(rdk.ItemList(8, True))
    require(programs == EXPECTED_PROGRAMS, f"Unexpected live program inventory: {sorted(programs)}")
    require(len(rdk.ItemList(6, True)) == 660, "Live target count changed")

    audit = {
        "status": "pass",
        "requirements": {
            "software_integration": True,
            "robodk_native_movel_import": True,
            "high_precision_program_export": True,
            "three_arm_reproducible_comparison": True,
            "position_orientation_tolerance": True,
            "native_joint_simulation_limits": True,
            "operation_guide": True,
            "station_backup_and_cleanup": True,
        },
        "playback_time_s": {"A_stop": a["wall_time_s"], "B_fixed": b["wall_time_s"], "C_variable": c["wall_time_s"]},
        "remaining_programs": sorted(programs),
        "remaining_targets": 660,
        "scope_excluded": ["UR10 hardware execution", "physical calibration", "scan quality", "collision", "torque"],
    }
    output = CASE / "delivery_audit.json"
    output.write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(audit, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
