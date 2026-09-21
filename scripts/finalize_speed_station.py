"""Back up the live station and remove only superseded V1 programs by exact name."""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robodk_bridge as bridge


STATION = "UR10_大曲面_MSCGA_corrected"
RETIRED_PROGRAMS = {
    "A_stop_fair_playback_20260918_080917",
    "B_fixed_fair_playback_20260918_080917",
    "C_variable_fair_playback_20260918_080917",
    "V1_Joint_20260918_080248",
}
REQUIRED_PROGRAMS = {
    "A",
    "V1_A_stop_20260917_235523",
    "V1_Joint_20260918_182439",
    "A_stop_fair_playback_20260919_093222",
    "B_fixed_fair_playback_20260919_093222",
    "C_variable_fair_playback_20260919_093222",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    output = Path(sys.argv[1]).resolve()
    output.mkdir(parents=True, exist_ok=False)
    api = bridge._import_api()
    rdk = api["Robolink"]()
    if rdk.ActiveStation().Name() != STATION:
        raise RuntimeError("Active station is not the accepted V1 station")

    program_names = rdk.ItemList(8, True)
    target_names = rdk.ItemList(6, True)
    if len(program_names) != len(set(program_names)):
        raise RuntimeError("Duplicate program names prevent exact cleanup")
    missing_required = REQUIRED_PROGRAMS - set(program_names)
    missing_retired = RETIRED_PROGRAMS - set(program_names)
    if missing_required or missing_retired:
        raise RuntimeError(
            f"Unexpected program inventory; missing required={sorted(missing_required)}, "
            f"missing retired={sorted(missing_retired)}"
        )

    handles = {item.Name(): item for item in rdk.ItemList(8)}
    for name in RETIRED_PROGRAMS | REQUIRED_PROGRAMS:
        if handles[name].Busy():
            raise RuntimeError(f"Program is still running: {name}")

    before = output / "before_final_cleanup.rdk"
    rdk.Save(str(before))
    if not before.is_file() or before.stat().st_size < 1000:
        raise RuntimeError("Full station backup was not created")

    ledger = {
        "station": STATION,
        "backup": str(before),
        "backup_sha256": _sha256(before),
        "programs_before": program_names,
        "targets_before_count": len(target_names),
        "retired_programs": sorted(RETIRED_PROGRAMS),
        "deleted_programs": [],
    }
    report = output / "final_cleanup_report.json"
    report.write_text(json.dumps(ledger, ensure_ascii=False, indent=2), encoding="utf-8")

    rdk.Render(False)
    try:
        for name in sorted(RETIRED_PROGRAMS):
            item = handles[name]
            if item.Name() != name:
                raise RuntimeError(f"Program identity changed: {name}")
            item.Delete()
            ledger["deleted_programs"].append(name)
            report.write_text(json.dumps(ledger, ensure_ascii=False, indent=2), encoding="utf-8")
    finally:
        rdk.Render(True)

    programs_after = rdk.ItemList(8, True)
    targets_after = rdk.ItemList(6, True)
    if set(programs_after) != set(program_names) - RETIRED_PROGRAMS:
        raise RuntimeError("Program inventory differs from exact deletion set")
    if targets_after != target_names:
        raise RuntimeError("Target inventory changed during program-only cleanup")

    update_results = {}
    for name in programs_after:
        result = list(rdk.Item(name, 8).Update())
        update_results[name] = result
        if len(result) < 4 or float(result[3]) != 1.0:
            raise RuntimeError(f"Retained program failed Update: {name}: {result}")

    after = output / "after_final_cleanup.rdk"
    rdk.Save(str(after))
    ledger.update(
        programs_after=programs_after,
        targets_after_count=len(targets_after),
        target_inventory_unchanged=True,
        update_results=update_results,
        after_backup=str(after),
        after_backup_sha256=_sha256(after),
    )
    report.write_text(json.dumps(ledger, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({
        "program_count": len(programs_after),
        "target_count": len(targets_after),
        "remaining_programs": programs_after,
        "report": str(report),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
