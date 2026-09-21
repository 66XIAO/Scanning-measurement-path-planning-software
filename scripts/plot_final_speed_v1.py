"""Create the final, data-driven V1 comparison figure and its QA manifest."""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _distance(commands: list[dict]) -> list[float]:
    result = [0.0]
    for before, after in zip(commands, commands[1:]):
        delta = math.sqrt(sum((after[k] - before[k]) ** 2 for k in ("x", "y", "z")))
        result.append(result[-1] + delta)
    return result


def main() -> None:
    root = Path(sys.argv[1]).resolve()
    package = root / "final_figure_20260919"
    preview_dir = package / "08_previews"
    svg_dir = package / "07_svg_export"
    manifest_dir = package / "09_manifests"
    qa_dir = package / "10_qa"
    request_dir = package / "00_request"
    for folder in (preview_dir, svg_dir, manifest_dir, qa_dir, request_dir):
        folder.mkdir(parents=True, exist_ok=True)

    report = json.loads((root / "final_report.json").read_text(encoding="utf-8"))
    comparison = json.loads((root / "comparison.json").read_text(encoding="utf-8"))
    labels = ["Point stop", "Smoothed fixed", "Smoothed variable"]
    keys = ["A_stop", "B_fixed", "C_variable"]
    times = [report[k]["wall_time_s"] for k in keys]

    # The first command is the common approach MoveJ.  The speed plot begins
    # with the first incoming scan segment so it represents scanning only.
    b_commands = comparison["B_fixed"]["incoming_commands"][1:]
    c_commands = comparison["C_variable"]["incoming_commands"][1:]
    b_s, c_s = _distance(b_commands), _distance(c_commands)
    b_v = [row["linear_speed"] for row in b_commands]
    c_v = [row["linear_speed"] for row in c_commands]

    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.linewidth": 0.8,
        "svg.fonttype": "none",
    })
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 3.7), constrained_layout=True)

    ax = axes[0]
    bars = ax.bar(labels, times, color=["#8a8a8a", "#d0d0d0", "#0072B2"], edgecolor="black", linewidth=0.8)
    for bar, value in zip(bars, times):
        ax.text(bar.get_x() + bar.get_width() / 2, value + 5, f"{value:.1f}", ha="center", va="bottom")
    ax.set_ylabel("Measured RoboDK playback time (s)")
    ax.set_ylim(0, max(times) * 1.16)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(axis="x", rotation=12)
    ax.text(0.02, 0.97, "(a)", transform=ax.transAxes, va="top", fontweight="bold")

    ax = axes[1]
    ax.plot(b_s, b_v, color="#777777", linewidth=1.0, label="Smoothed fixed")
    ax.plot(c_s, c_v, color="#0072B2", linewidth=1.0, label="Smoothed variable")
    ax.set_xlabel("Command-path distance (mm)")
    ax.set_ylabel("Commanded incoming-segment speed (mm/s)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, loc="upper right")
    ax.text(0.02, 0.97, "(b)", transform=ax.transAxes, va="top", fontweight="bold")

    png = preview_dir / "speed_v1_final.png"
    svg = svg_dir / "speed_v1_final.svg"
    fig.savefig(png, dpi=240, facecolor="white")
    fig.savefig(svg, facecolor="white")
    plt.close(fig)

    with (package / "source_data.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["arm", "path_distance_mm", "command_speed_mm_s", "playback_time_s"])
        for arm, distances, speeds, playback in (
            ("B_fixed", b_s, b_v, times[1]),
            ("C_variable", c_s, c_v, times[2]),
        ):
            writer.writerows((arm, distance, speed, playback) for distance, speed in zip(distances, speeds))

    preflight = {
        "mode": "data-driven scientific chart",
        "purpose": "internal V1 acceptance report",
        "figure_type": "two-panel comparison",
        "palette": "white background, black text, one blue accent",
        "standard": "minimal journal style",
        "editability": "SVG text and paths plus source CSV",
        "language": "English",
        "data_sources": ["final_report.json", "comparison.json"],
    }
    (request_dir / "preflight_choices.json").write_text(json.dumps(preflight, indent=2), encoding="utf-8")
    index = {
        "png": str(png),
        "svg": str(svg),
        "source_csv": str(package / "source_data.csv"),
        "data_sources": [str(root / "final_report.json"), str(root / "comparison.json")],
    }
    (manifest_dir / "output_index.json").write_text(json.dumps(index, indent=2), encoding="utf-8")
    (qa_dir / "QA_notes.md").write_text(
        "# QA notes\n\n"
        "- Values are loaded directly from the final machine-readable reports.\n"
        "- PNG and editable SVG were generated from the same plotting call.\n"
        "- The speed panel shows commanded incoming-segment speed, not measured instantaneous TCP speed.\n"
        "- Playback bars show 1x RoboDK wall-clock simulation measurements.\n",
        encoding="utf-8",
    )
    print(json.dumps(index, indent=2))


if __name__ == "__main__":
    main()
