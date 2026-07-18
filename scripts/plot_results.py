#!/usr/bin/env python3
"""Plot experiment CSV files and print reproducible acceptance metrics."""

import argparse
import csv
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


SCENARIO_BOXES = {
    "five": [
        (1.5, 0.0, 1.2, 0.6, 2.0, 2.4),
        (3.0, -1.2, 2.0, 0.8, 1.6, 4.0),
        (4.0, 1.2, 2.0, 0.8, 1.6, 4.0),
        (5.4, 0.0, 3.7, 0.8, 3.5, 1.0),
        (6.8, 0.0, 1.3, 0.8, 4.5, 2.6),
    ],
    "narrow": [
        (4.0, 0.0, 1.5, 0.6, 8.0, 3.0),
        (2.0, -2.8, 2.5, 0.6, 1.0, 5.0),
        (2.0, 2.8, 2.5, 0.6, 1.0, 5.0),
        (6.0, -2.8, 2.5, 0.6, 1.0, 5.0),
        (6.0, 2.8, 2.5, 0.6, 1.0, 5.0),
    ],
}


def point_to_aabb_distance(point, box):
    return math.sqrt(
        sum(
            max(abs(point[index] - box[index]) - box[index + 3] / 2.0, 0.0) ** 2
            for index in range(3)
        )
    )


def load_rows(path):
    with path.open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"{path} contains no telemetry rows")
    required = {"t", "x", "y", "z", "rpm1", "rpm2", "rpm3", "rpm4"}
    missing = required.difference(rows[0])
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(sorted(missing))}")
    return rows


def values(rows, key):
    return [float(row[key]) for row in rows]


def save_figure(figure, path):
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Generate plots and metrics from run_experiment/waypoint_mission CSV."
    )
    parser.add_argument("csv")
    parser.add_argument("--prefix", default="experiment")
    parser.add_argument(
        "--goal",
        nargs=3,
        type=float,
        help="override CSV goal; required only for legacy CSV files",
    )
    parser.add_argument(
        "--scenario", choices=["none", "five", "narrow"], default="none"
    )
    parser.add_argument("--safety-distance", type=float, default=0.5)
    parser.add_argument("--vehicle-radius", type=float, default=0.18)
    parser.add_argument("--max-rpm", type=float, default=9000.0)
    args = parser.parse_args()

    csv_path = Path(args.csv).expanduser()
    prefix = Path(args.prefix).expanduser()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    try:
        rows = load_rows(csv_path)
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    raw_time = values(rows, "t")
    # CSV timestamps may include DDS discovery time when the recorder is started
    # before launch. Metrics and plots use time since the first received sample.
    time_values = [sample - raw_time[0] for sample in raw_time]
    xyz = [values(rows, axis) for axis in ("x", "y", "z")]
    positions = list(zip(*xyz))
    rpm = [values(rows, f"rpm{index}") for index in range(1, 5)]

    if args.goal is not None:
        goals = [tuple(args.goal)] * len(rows)
    elif {"goal_x", "goal_y", "goal_z"}.issubset(rows[0]):
        goals = [
            (float(row["goal_x"]), float(row["goal_y"]), float(row["goal_z"]))
            for row in rows
        ]
    else:
        print(
            "ERROR: legacy CSV has no goal columns; pass --goal X Y Z",
            file=sys.stderr,
        )
        return 3

    error = [math.dist(position, goal) for position, goal in zip(positions, goals)]
    path_length = sum(
        math.dist(positions[index - 1], positions[index])
        for index in range(1, len(positions))
    )
    final_error = error[-1]
    final_window_start = max(time_values[-1] - 2.0, time_values[0])
    steady_errors = [
        sample for sample_time, sample in zip(time_values, error)
        if sample_time >= final_window_start
    ]
    steady_mean = sum(steady_errors) / len(steady_errors)
    maximum_rpm = max(max(motor) for motor in rpm)
    saturation_samples = sum(
        any(motor[index] >= args.max_rpm - 1.0 for motor in rpm)
        for index in range(len(rows))
    )

    constant_goal = all(math.dist(goals[0], goal) < 1e-9 for goal in goals)
    arrival_time = None
    overshoot = None
    if constant_goal:
        arrival_time = next(
            (sample_time for sample_time, sample in zip(time_values, error) if sample < 0.3),
            None,
        )
        start = positions[0]
        direction = tuple(goals[0][i] - start[i] for i in range(3))
        distance = math.sqrt(sum(component * component for component in direction))
        if distance > 1e-9:
            unit = tuple(component / distance for component in direction)
            projections = [
                sum((position[i] - start[i]) * unit[i] for i in range(3))
                for position in positions
            ]
            overshoot = max(0.0, max(projections) - distance)

    figure, axis = plt.subplots()
    for key, series in zip(("x", "y", "z"), xyz):
        axis.plot(time_values, series, label=key)
    axis.legend()
    axis.set(xlabel="time [s]", ylabel="position [m]")
    save_figure(figure, f"{prefix}_position.png")

    figure, axis = plt.subplots()
    for index, series in enumerate(rpm, start=1):
        axis.plot(time_values, series, label=f"rpm{index}")
    axis.axhline(args.max_rpm, color="r", linestyle="--", label="RPM limit")
    axis.legend()
    axis.set(xlabel="time [s]", ylabel="RPM")
    save_figure(figure, f"{prefix}_rpm.png")

    figure, axis = plt.subplots()
    axis.plot(time_values, error)
    axis.axhline(0.3, color="r", linestyle="--", label="0.3 m acceptance")
    axis.legend()
    axis.set(xlabel="time [s]", ylabel="active-goal error [m]")
    save_figure(figure, f"{prefix}_error.png")

    figure, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].plot(xyz[0], xyz[1], label="actual")
    axes[0].set(xlabel="x [m]", ylabel="y [m]", aspect="equal")
    axes[1].plot(xyz[0], xyz[2], label="actual")
    axes[1].set(xlabel="x [m]", ylabel="z [m]")
    save_figure(figure, f"{prefix}_trajectory.png")

    attitude_peak = None
    if {"roll", "pitch", "yaw"}.issubset(rows[0]):
        attitude = [values(rows, axis) for axis in ("roll", "pitch", "yaw")]
        figure, axis = plt.subplots()
        for label, series in zip(("roll", "pitch", "yaw"), attitude):
            axis.plot(time_values, [math.degrees(value) for value in series], label=label)
        axis.legend()
        axis.set(xlabel="time [s]", ylabel="attitude [deg]")
        save_figure(figure, f"{prefix}_attitude.png")
        attitude_peak = max(
            abs(math.degrees(value)) for series in attitude[:2] for value in series
        )

    minimum_clearance = None
    minimum_body_clearance = None
    if args.scenario != "none":
        boxes = SCENARIO_BOXES[args.scenario]
        clearance = [
            min(point_to_aabb_distance(position, box) for box in boxes)
            for position in positions
        ]
        minimum_clearance = min(clearance)
        minimum_body_clearance = minimum_clearance - args.vehicle_radius
        required_center_clearance = args.safety_distance + args.vehicle_radius
        figure, axis = plt.subplots()
        axis.plot(time_values, clearance)
        axis.axhline(
            required_center_clearance,
            color="r",
            linestyle="--",
            label=(
                f"required center clearance {required_center_clearance:.2f} m "
                f"({args.safety_distance:.2f} m safety + body radius)"
            ),
        )
        axis.legend()
        axis.set(xlabel="time [s]", ylabel="center-to-AABB clearance [m]")
        save_figure(figure, f"{prefix}_clearance.png")

    metric_lines = [
        f"samples: {len(rows)}",
        f"duration_s: {time_values[-1]:.3f}",
        f"final_error_m: {final_error:.4f}",
        f"path_length_m: {path_length:.4f}",
        f"maximum_rpm: {maximum_rpm:.1f}",
        f"rpm_saturation_samples: {saturation_samples}",
    ]
    if constant_goal:
        metric_lines.append(f"steady_mean_error_last_2s_m: {steady_mean:.4f}")
    else:
        metric_lines.append(f"active_goal_mean_error_last_2s_m: {steady_mean:.4f}")
    if arrival_time is not None:
        metric_lines.append(f"first_below_0.3m_s: {arrival_time:.3f}")
    elif constant_goal:
        metric_lines.append("first_below_0.3m_s: NOT_REACHED")
    if overshoot is not None:
        metric_lines.append(f"maximum_overshoot_m: {overshoot:.4f}")
    if attitude_peak is not None:
        metric_lines.append(f"maximum_abs_roll_pitch_deg: {attitude_peak:.3f}")
    if minimum_clearance is not None:
        metric_lines.append(f"minimum_center_aabb_clearance_m: {minimum_clearance:.4f}")
        metric_lines.append(f"vehicle_radius_m: {args.vehicle_radius:.4f}")
        metric_lines.append(
            f"minimum_body_aabb_clearance_m: {minimum_body_clearance:.4f}"
        )
        metric_lines.append(f"required_safety_distance_m: {args.safety_distance:.4f}")
        metric_lines.append(
            "clearance_acceptance: "
            + ("PASS" if minimum_body_clearance > args.safety_distance else "FAIL")
        )
    if "waypoint_index" in rows[0]:
        waypoint_indices = [int(row["waypoint_index"]) for row in rows]
        unique_indices = sorted(set(waypoint_indices))
        metric_lines.append(f"waypoint_segments_recorded: {len(unique_indices)}")
        for waypoint_index in unique_indices:
            segment_errors = [
                sample for sample, index in zip(error, waypoint_indices)
                if index == waypoint_index
            ]
            metric_lines.append(
                f"waypoint_{waypoint_index}_minimum_error_m: {min(segment_errors):.4f}"
            )

    metrics_path = Path(f"{prefix}_metrics.txt")
    metrics_path.write_text("\n".join(metric_lines) + "\n", encoding="utf-8")
    print("\n".join(metric_lines))
    print(f"plots and metrics written with prefix {prefix}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
