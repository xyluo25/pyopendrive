"""Analysis utilities for SUMO mobility-energy outputs and CARLA telemetry."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import statistics
from typing import Any
import xml.etree.ElementTree as ET

FIGURE_DPI = 1200
FUEL_LOWER_HEATING_VALUE_MJ_PER_KG = 44.0
KWH_PER_MJ = 1.0 / 3.6
WH_PER_KWH = 1000.0
STRING_VALUE_FIELDS = {
    "actor_id",
    "arrivalLane",
    "departLane",
    "devices",
    "id",
    "is_ego",
    "role_name",
    "sumo_id",
    "type_id",
    "vType",
    "vaporized",
}


class FigureGenerationError(RuntimeError):
    """Raised when figure rendering dependencies are unavailable."""


def read_tripinfo(path: str | Path) -> list[dict[str, object]]:
    """Read SUMO ``tripinfo.xml`` into row dictionaries.

    Nested ``emissions`` and ``battery`` elements are flattened with prefixes so
    missing optional SUMO outputs never break parsing.
    """

    tripinfo_path = Path(path)
    if not tripinfo_path.exists():
        raise FileNotFoundError(f"SUMO tripinfo file not found: {tripinfo_path}")

    rows: list[dict[str, object]] = []
    for _event, elem in ET.iterparse(tripinfo_path, events=("end",)):
        if elem.tag != "tripinfo":
            continue
        row: dict[str, object] = {
            key: _coerce_field_value(key, value) for key, value in elem.attrib.items()
        }
        for child in list(elem):
            for key, value in child.attrib.items():
                prefixed_key = f"{child.tag}_{key}"
                row[prefixed_key] = _coerce_field_value(prefixed_key, value)
        rows.append(row)
        elem.clear()
    return rows


def summarize_tripinfo(
    rows: list[dict[str, object]],
    scenario_id: str,
) -> dict[str, object]:
    """Create scenario-level mobility and energy-consumption metrics."""

    completed_vehicles = len(rows)
    durations = _numbers(rows, "duration")
    route_lengths = _numbers(rows, "routeLength")
    waiting_times = _numbers(rows, "waitingTime")
    waiting_counts = _numbers(rows, "waitingCount")
    time_losses = _numbers(rows, "timeLoss")
    departures = _numbers(rows, "depart")
    arrivals = _numbers(rows, "arrival")

    total_distance_m = sum(route_lengths)
    total_duration_s = sum(durations)
    total_distance_km = total_distance_m / 1000.0
    observed_hours = _observed_hours(departures, arrivals, durations)

    # SUMO stores fuel and electricity use on the XML element named
    # ``emissions``. The analysis reports those fields as energy consumption,
    # not as pollutant-output metrics.
    total_fuel_mg = _sum_first_available(rows, ["emissions_fuel_abs", "fuel_abs"])
    total_electricity_wh = _sum_first_available(
        rows,
        ["emissions_electricity_abs", "electricity_abs"],
    )
    total_battery_consumed_wh = _sum_first_available(
        rows,
        ["battery_totalEnergyConsumed", "totalEnergyConsumed"],
    )
    total_battery_regenerated_wh = _sum_first_available(
        rows,
        ["battery_totalEnergyRegenerated", "totalEnergyRegenerated"],
    )
    final_battery_values = _numbers(rows, "battery_actualBatteryCapacity")
    fuel_energy_kwh = _fuel_mg_to_kwh(total_fuel_mg)
    electricity_energy_kwh = total_electricity_wh / WH_PER_KWH
    battery_consumed_kwh = total_battery_consumed_wh / WH_PER_KWH
    battery_regenerated_kwh = total_battery_regenerated_wh / WH_PER_KWH
    electric_drive_energy_kwh = (
        battery_consumed_kwh if battery_consumed_kwh > 0 else electricity_energy_kwh
    )
    total_energy_kwh = fuel_energy_kwh + electric_drive_energy_kwh
    net_energy_kwh = max(total_energy_kwh - battery_regenerated_kwh, 0.0)

    return {
        "scenario_id": scenario_id,
        "completed_vehicles": completed_vehicles,
        "avg_travel_time_s": _mean(durations),
        "avg_route_length_m": _mean(route_lengths),
        "avg_speed_mps": total_distance_m / total_duration_s
        if total_duration_s > 0
        else 0.0,
        "avg_waiting_time_s": _mean(waiting_times),
        "avg_waiting_count": _mean(waiting_counts),
        "avg_time_loss_s": _mean(time_losses),
        "throughput_veh_per_hour": completed_vehicles / observed_hours
        if observed_hours > 0
        else 0.0,
        "total_distance_km": total_distance_km,
        "total_fuel_mg": total_fuel_mg,
        "fuel_energy_kwh": fuel_energy_kwh,
        "electricity_energy_kwh": electricity_energy_kwh,
        "battery_consumed_kwh": battery_consumed_kwh,
        "battery_regenerated_kwh": battery_regenerated_kwh,
        "total_energy_kwh": total_energy_kwh,
        "net_energy_kwh": net_energy_kwh,
        "avg_final_battery_capacity_wh": _mean(final_battery_values),
        "electricity_kwh_per_km": electricity_energy_kwh / total_distance_km
        if total_distance_km > 0
        else 0.0,
        "energy_kwh_per_km": net_energy_kwh / total_distance_km
        if total_distance_km > 0
        else 0.0,
    }


def read_carla_telemetry(path: str | Path) -> list[dict[str, object]]:
    """Read CARLA telemetry CSV into row dictionaries."""

    telemetry_path = Path(path)
    if not telemetry_path.exists():
        raise FileNotFoundError(f"CARLA telemetry file not found: {telemetry_path}")

    rows: list[dict[str, object]] = []
    with telemetry_path.open("r", newline="", encoding="utf-8") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            rows.append(
                {key: _coerce_field_value(key, value) for key, value in row.items()}
            )
    return rows


def summarize_carla_telemetry(
    rows: list[dict[str, object]],
    scenario_id: str,
) -> dict[str, object]:
    """Create CARLA telemetry summary metrics."""

    speeds = _numbers(rows, "speed_mps")
    accelerations = _numbers(rows, "accel_mps2")
    jerks = _numbers(rows, "jerk_mps3")
    times = _numbers(rows, "time_s")
    actor_ids = {
        str(row.get("actor_id"))
        for row in rows
        if row.get("actor_id") not in {None, ""}
    }

    return {
        "scenario_id": scenario_id,
        "carla_available": bool(rows),
        "avg_speed_mps": _mean(speeds),
        "max_speed_mps": max(speeds) if speeds else 0.0,
        "avg_accel_mps2": _mean(accelerations),
        "max_accel_mps2": max(accelerations) if accelerations else 0.0,
        "avg_abs_accel_mps2": _mean([abs(value) for value in accelerations]),
        "avg_jerk_mps3": _mean(jerks),
        "stop_episode_count": _count_stop_episodes(rows),
        "vehicle_count": len(actor_ids),
        "telemetry_duration_s": max(times) - min(times) if len(times) >= 2 else 0.0,
    }


def write_summary_csv(rows: list[dict[str, object]], path: str | Path) -> None:
    """Write dictionaries to CSV with stable column order."""

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = _fieldnames(rows)
    with output_path.open("w", newline="", encoding="utf-8") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_result_figures(
    *,
    trip_rows: list[dict[str, object]],
    carla_rows: list[dict[str, object]],
    scenario_summary: dict[str, object],
    out_dir: str | Path,
    dpi: int = FIGURE_DPI,
) -> tuple[dict[str, Path], str]:
    """Write high-resolution matplotlib result figures."""

    figures_dir = Path(out_dir) / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    plotter = _load_matplotlib_pyplot()

    outputs: dict[str, Path] = {}
    mobility_path = figures_dir / "mobility_energy_summary.png"
    _plot_mobility_energy_summary(plotter, scenario_summary, mobility_path, dpi)
    outputs["figure_mobility_energy_summary"] = mobility_path

    if trip_rows:
        travel_time_path = figures_dir / "travel_time_distribution.png"
        _plot_histogram(
            plotter,
            _numbers(trip_rows, "duration"),
            title="Travel Time Distribution",
            x_label="Travel time (s)",
            output_path=travel_time_path,
            dpi=dpi,
        )
        outputs["figure_travel_time_distribution"] = travel_time_path

        route_time_path = figures_dir / "route_length_vs_travel_time.png"
        _plot_route_length_vs_travel_time(
            plotter,
            trip_rows,
            route_time_path,
            dpi,
        )
        outputs["figure_route_length_vs_travel_time"] = route_time_path

    if carla_rows:
        carla_speed_path = figures_dir / "carla_speed_profile.png"
        _plot_carla_speed_profile(plotter, carla_rows, carla_speed_path, dpi)
        outputs["figure_carla_speed_profile"] = carla_speed_path

        speed_accel_path = figures_dir / "speed_acceleration_profile.png"
        _plot_speed_acceleration_profile(plotter, carla_rows, speed_accel_path, dpi)
        outputs["figure_speed_acceleration_profile"] = speed_accel_path

    return outputs, "generated"


def analyze_project(
    *,
    project_dir: str | Path | None = None,
    out_dir: str | Path,
    scenario_id: str | None = None,
    tripinfo_path: str | Path | None = None,
    carla_telemetry_path: str | Path | None = None,
    skip_carla: bool = False,
    ego_vehicle_ids: list[str] | None = None,
) -> dict[str, Path]:
    """Analyze SUMO outputs and optional CARLA telemetry.

    CARLA is intentionally optional. When ``skip_carla`` is true or no telemetry
    file exists, only SUMO-derived result files are written.
    """

    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    inferred_scenario_id = scenario_id or _scenario_id_from_project(project_dir)
    ego_filter = _ego_filter_from_project(project_dir)
    if ego_vehicle_ids is not None:
        ego_filter["vehicle_ids"] = [str(vehicle_id) for vehicle_id in ego_vehicle_ids]
        ego_filter["enabled"] = bool(ego_vehicle_ids)
        ego_filter["analyze_only_ego"] = bool(ego_vehicle_ids)
    if project_dir is not None:
        project_path = Path(project_dir)
        tripinfo_path = (
            tripinfo_path or project_path / "sumo" / "outputs" / "tripinfo.xml"
        )
        carla_telemetry_path = (
            carla_telemetry_path
            or project_path / "carla" / "outputs" / "carla_telemetry.csv"
        )

    trip_rows_for_figures: list[dict[str, object]] = []
    carla_rows_for_figures: list[dict[str, object]] = []
    trip_summary = _unavailable_tripinfo_summary(inferred_scenario_id)
    if tripinfo_path is not None and _has_content(tripinfo_path):
        trip_rows = read_tripinfo(tripinfo_path)
        if _should_filter_ego(ego_filter):
            trip_rows = _filter_tripinfo_to_ego(trip_rows, ego_filter)
            write_summary_csv(trip_rows, out_path / "ego_tripinfo.csv")
        trip_rows_for_figures = trip_rows
        trip_summary = summarize_tripinfo(trip_rows, inferred_scenario_id)

    carla_summary: dict[str, object] | None = None
    carla_analysis_status = (
        "skipped_by_request" if skip_carla else "skipped_no_telemetry"
    )
    if (
        not skip_carla
        and carla_telemetry_path is not None
        and _has_content(carla_telemetry_path)
    ):
        carla_rows = read_carla_telemetry(carla_telemetry_path)
        if _should_filter_ego(ego_filter):
            carla_rows = _filter_carla_rows_to_ego(carla_rows, ego_filter)
            write_summary_csv(carla_rows, out_path / "ego_carla_telemetry.csv")
        carla_rows_for_figures = carla_rows
        if carla_rows:
            carla_summary = summarize_carla_telemetry(carla_rows, inferred_scenario_id)
            carla_analysis_status = "analyzed"
        else:
            carla_analysis_status = "skipped_empty_telemetry"

    scenario_summary = dict(trip_summary)
    if carla_summary is not None:
        for key, value in carla_summary.items():
            if key == "scenario_id":
                continue
            scenario_summary[f"carla_{key}" if key != "carla_available" else key] = (
                value
            )
    else:
        scenario_summary["carla_analysis_status"] = carla_analysis_status

    scenario_summary_path = out_path / "scenario_summary.csv"
    mobility_path = out_path / "mobility_energy_table.csv"
    metadata_path = out_path / "run_metadata.json"
    try:
        figure_outputs, figure_generation_status = write_result_figures(
            trip_rows=trip_rows_for_figures,
            carla_rows=carla_rows_for_figures,
            scenario_summary=scenario_summary,
            out_dir=out_path,
        )
    except FigureGenerationError as exc:
        figure_outputs = {}
        figure_generation_status = f"skipped: {exc}"

    write_summary_csv([scenario_summary], scenario_summary_path)
    write_summary_csv([trip_summary], mobility_path)
    metadata_path.write_text(
        json.dumps(
            {
                "project_dir": str(project_dir) if project_dir is not None else None,
                "tripinfo_path": str(tripinfo_path)
                if tripinfo_path is not None
                else None,
                "carla_telemetry_path": str(carla_telemetry_path)
                if carla_telemetry_path is not None
                else None,
                "scenario_id": inferred_scenario_id,
                "carla_analysis_status": carla_analysis_status,
                "ego_filter": ego_filter,
                "figure_generation_status": figure_generation_status,
                "figures": {name: str(path) for name, path in figure_outputs.items()},
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    outputs = {
        "scenario_summary": scenario_summary_path,
        "mobility_energy_table": mobility_path,
        "run_metadata": metadata_path,
    }
    outputs.update(figure_outputs)
    if _should_filter_ego(ego_filter):
        ego_tripinfo_path = out_path / "ego_tripinfo.csv"
        if ego_tripinfo_path.exists():
            outputs["ego_tripinfo"] = ego_tripinfo_path
        ego_carla_path = out_path / "ego_carla_telemetry.csv"
        if ego_carla_path.exists():
            outputs["ego_carla_telemetry"] = ego_carla_path
    if carla_summary is not None:
        carla_path = out_path / "carla_telemetry_summary.csv"
        write_summary_csv([carla_summary], carla_path)
        outputs["carla_telemetry_summary"] = carla_path
    return outputs


def summarize_replicates(
    summary_files: list[Path], out_dir: str | Path
) -> dict[str, Path]:
    """Combine scenario summaries and write CIs plus energy-savings tables."""

    rows: list[dict[str, object]] = []
    for summary_file in summary_files:
        with Path(summary_file).open("r", newline="", encoding="utf-8") as file_obj:
            rows.extend(dict(row) for row in csv.DictReader(file_obj))

    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    write_summary_csv(rows, out_path / "batch_summary.csv")

    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row.get("scenario_id", "unknown")), []).append(row)

    ci_rows: list[dict[str, object]] = []
    for scenario_id, scenario_rows in grouped.items():
        numeric_keys = sorted(
            key
            for key in scenario_rows[0]
            if key != "scenario_id"
            and _maybe_float(scenario_rows[0].get(key)) is not None
        )
        for key in numeric_keys:
            values = [
                float(value)
                for row in scenario_rows
                if (value := _maybe_float(row.get(key))) is not None
            ]
            if not values:
                continue
            mean_value = statistics.fmean(values)
            std_value = statistics.stdev(values) if len(values) > 1 else 0.0
            half_width = 1.96 * std_value / math.sqrt(len(values))
            ci_rows.append(
                {
                    "scenario_id": scenario_id,
                    "metric": key,
                    "mean": mean_value,
                    "std": std_value,
                    "n": len(values),
                    "ci95_low": mean_value - half_width,
                    "ci95_high": mean_value + half_width,
                }
            )

    ci_path = out_path / "confidence_intervals.csv"
    write_summary_csv(ci_rows, ci_path)
    energy_savings_path = out_path / "energy_savings.csv"
    write_summary_csv(_energy_savings_rows(grouped), energy_savings_path)
    return {
        "batch_summary": out_path / "batch_summary.csv",
        "confidence_intervals": ci_path,
        "energy_savings": energy_savings_path,
    }


def _energy_savings_rows(
    grouped: dict[str, list[dict[str, object]]],
) -> list[dict[str, object]]:
    """Compare each scenario's energy use with the baseline scenario."""

    if not grouped:
        return []

    baseline_id = _baseline_scenario_id(grouped)
    baseline_rows = grouped[baseline_id]
    baseline_energy = _mean_numeric_metric(baseline_rows, "net_energy_kwh")
    baseline_energy_intensity = _mean_numeric_metric(
        baseline_rows,
        "energy_kwh_per_km",
    )
    if baseline_energy is None:
        return []

    savings_rows: list[dict[str, object]] = []
    for scenario_id, scenario_rows in grouped.items():
        scenario_energy = _mean_numeric_metric(scenario_rows, "net_energy_kwh")
        if scenario_energy is None:
            continue
        scenario_energy_intensity = _mean_numeric_metric(
            scenario_rows,
            "energy_kwh_per_km",
        )
        energy_saved = baseline_energy - scenario_energy
        intensity_saved = (
            baseline_energy_intensity - scenario_energy_intensity
            if baseline_energy_intensity is not None
            and scenario_energy_intensity is not None
            else 0.0
        )
        savings_rows.append(
            {
                "baseline_scenario_id": baseline_id,
                "scenario_id": scenario_id,
                "n": len(scenario_rows),
                "baseline_net_energy_kwh": baseline_energy,
                "scenario_net_energy_kwh": scenario_energy,
                "energy_saved_kwh": energy_saved,
                "energy_saved_percent": _percent_saved(
                    baseline_energy,
                    energy_saved,
                ),
                "baseline_energy_kwh_per_km": baseline_energy_intensity or 0.0,
                "scenario_energy_kwh_per_km": scenario_energy_intensity or 0.0,
                "energy_intensity_saved_kwh_per_km": intensity_saved,
                "energy_intensity_saved_percent": _percent_saved(
                    baseline_energy_intensity,
                    intensity_saved,
                ),
            }
        )
    return savings_rows


def _baseline_scenario_id(grouped: dict[str, list[dict[str, object]]]) -> str:
    """Choose the baseline scenario id used for energy-savings comparisons."""

    for scenario_id in grouped:
        if "baseline" in scenario_id.lower():
            return scenario_id
    return next(iter(grouped))


def _mean_numeric_metric(rows: list[dict[str, object]], key: str) -> float | None:
    """Return the mean numeric value for one metric, or ``None`` if missing."""

    values = [
        float(value)
        for row in rows
        if (value := _maybe_float(row.get(key))) is not None
    ]
    if not values:
        return None
    return statistics.fmean(values)


def _percent_saved(baseline_value: float | None, saved_value: float) -> float:
    """Return percent savings with safe handling for missing or zero baselines."""

    if baseline_value in {None, 0.0}:
        return 0.0
    return 100.0 * saved_value / float(baseline_value)


def _load_matplotlib_pyplot() -> Any:
    try:
        import matplotlib

        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as pyplot
    except ImportError as exc:
        raise FigureGenerationError(
            "Matplotlib could not be imported, so result figures cannot be "
            "written. Reinstall NumPy and Matplotlib for this Python "
            "environment, then rerun the analysis command."
        ) from exc
    return pyplot


def _plot_mobility_energy_summary(
    plt: Any,
    scenario_summary: dict[str, object],
    output_path: Path,
    dpi: int,
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.8))
    fig.suptitle(
        f"Scenario Summary: {scenario_summary.get('scenario_id', 'scenario')}",
        fontsize=13,
        fontweight="bold",
    )

    mobility_metrics = [
        ("Vehicles", "completed_vehicles"),
        ("Travel\nTime (s)", "avg_travel_time_s"),
        ("Waiting\nTime (s)", "avg_waiting_time_s"),
        ("Time\nLoss (s)", "avg_time_loss_s"),
    ]
    energy_metrics = [
        ("Distance\n(km)", "total_distance_km"),
        ("Net Energy\n(kWh)", "net_energy_kwh"),
        ("Energy\n(kWh/km)", "energy_kwh_per_km"),
    ]
    _draw_metric_bars(axes[0], scenario_summary, mobility_metrics, "Mobility")
    _draw_metric_bars(axes[1], scenario_summary, energy_metrics, "Energy Consumption")
    fig.tight_layout()
    _save_figure(plt, fig, output_path, dpi)


def _draw_metric_bars(
    axis: Any,
    summary: dict[str, object],
    metrics: list[tuple[str, str]],
    title: str,
) -> None:
    labels = [label for label, _key in metrics]
    values = [_maybe_float(summary.get(key)) or 0.0 for _label, key in metrics]
    colors = ["#2f6f9f", "#7aa35a", "#d28b2c", "#9b5f91"]
    bars = axis.bar(labels, values, color=colors[: len(values)])
    axis.set_title(title)
    axis.set_ylabel("Value")
    axis.grid(axis="y", alpha=0.25)
    axis.bar_label(bars, labels=[_format_number(value) for value in values], padding=3)
    axis.tick_params(axis="x", labelrotation=0)


def _plot_histogram(
    plt: Any,
    values: list[float],
    *,
    title: str,
    x_label: str,
    output_path: Path,
    dpi: int,
) -> None:
    fig, axis = plt.subplots(figsize=(8.2, 4.8))
    if values:
        bin_count = min(30, max(5, int(math.sqrt(len(values)))))
        axis.hist(values, bins=bin_count, color="#2f6f9f", edgecolor="white")
    else:
        axis.text(0.5, 0.5, "No data", ha="center", va="center")
    axis.set_title(title, fontsize=13, fontweight="bold")
    axis.set_xlabel(x_label)
    axis.set_ylabel("Vehicle count")
    axis.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    _save_figure(plt, fig, output_path, dpi)


def _plot_route_length_vs_travel_time(
    plt: Any,
    trip_rows: list[dict[str, object]],
    output_path: Path,
    dpi: int,
) -> None:
    points = [
        (route_length, duration)
        for row in trip_rows
        if (route_length := _maybe_float(row.get("routeLength"))) is not None
        and (duration := _maybe_float(row.get("duration"))) is not None
    ]
    sampled_points = _sample_sequence(points, max_items=12000)

    fig, axis = plt.subplots(figsize=(8.2, 5.2))
    if sampled_points:
        x_values = [point[0] for point in sampled_points]
        y_values = [point[1] for point in sampled_points]
        axis.scatter(x_values, y_values, s=12, alpha=0.55, color="#2f6f9f")
    else:
        axis.text(0.5, 0.5, "No data", ha="center", va="center")
    axis.set_title("Route Length vs Travel Time", fontsize=13, fontweight="bold")
    axis.set_xlabel("Route length (m)")
    axis.set_ylabel("Travel time (s)")
    axis.grid(alpha=0.25)
    fig.tight_layout()
    _save_figure(plt, fig, output_path, dpi)


def _plot_carla_speed_profile(
    plt: Any,
    carla_rows: list[dict[str, object]],
    output_path: Path,
    dpi: int,
) -> None:
    rows_by_actor: dict[str, list[dict[str, object]]] = {}
    for row in carla_rows:
        actor_id = str(row.get("actor_id", ""))
        if actor_id:
            rows_by_actor.setdefault(actor_id, []).append(row)

    fig, axis = plt.subplots(figsize=(9.0, 5.0))
    for actor_id, actor_rows in list(rows_by_actor.items())[:8]:
        actor_rows.sort(key=lambda row: _maybe_float(row.get("time_s")) or 0.0)
        sampled_rows = _sample_sequence(actor_rows, max_items=2500)
        times = [_maybe_float(row.get("time_s")) for row in sampled_rows]
        speeds = [_maybe_float(row.get("speed_mps")) for row in sampled_rows]
        points = [
            (time_value, speed_value)
            for time_value, speed_value in zip(times, speeds, strict=False)
            if time_value is not None and speed_value is not None
        ]
        if points:
            axis.plot(
                [point[0] for point in points],
                [point[1] for point in points],
                linewidth=1.6,
                label=f"actor {actor_id}",
            )

    if not axis.lines:
        axis.text(0.5, 0.5, "No data", ha="center", va="center")
    else:
        axis.legend(loc="best", fontsize=8)
    axis.set_title("CARLA Speed Profile", fontsize=13, fontweight="bold")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Speed (m/s)")
    axis.grid(alpha=0.25)
    fig.tight_layout()
    _save_figure(plt, fig, output_path, dpi)


def _plot_speed_acceleration_profile(
    plt: Any,
    carla_rows: list[dict[str, object]],
    output_path: Path,
    dpi: int,
) -> None:
    points = [
        (speed, acceleration)
        for row in carla_rows
        if (speed := _maybe_float(row.get("speed_mps"))) is not None
        and (acceleration := _maybe_float(row.get("accel_mps2"))) is not None
    ]
    sampled_points = _sample_sequence(points, max_items=12000)

    fig, axis = plt.subplots(figsize=(8.2, 5.2))
    if sampled_points:
        axis.scatter(
            [point[0] for point in sampled_points],
            [point[1] for point in sampled_points],
            s=12,
            alpha=0.5,
            color="#7aa35a",
        )
    else:
        axis.text(0.5, 0.5, "No data", ha="center", va="center")
    axis.set_title("Speed-Acceleration Profile", fontsize=13, fontweight="bold")
    axis.set_xlabel("Speed (m/s)")
    axis.set_ylabel("Acceleration (m/s2)")
    axis.grid(alpha=0.25)
    fig.tight_layout()
    _save_figure(plt, fig, output_path, dpi)


def _save_figure(plt: Any, fig: Any, output_path: Path, dpi: int) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def _fuel_mg_to_kwh(fuel_mg: float) -> float:
    """Convert SUMO's fuel mass to energy using a gasoline LHV estimate."""

    fuel_kg = fuel_mg / 1_000_000.0
    return fuel_kg * FUEL_LOWER_HEATING_VALUE_MJ_PER_KG * KWH_PER_MJ


def _format_number(value: float) -> str:
    if abs(value) >= 1000:
        return f"{value:,.0f}"
    if abs(value) >= 10:
        return f"{value:.1f}"
    return f"{value:.3g}"


def _sample_sequence(values: list[Any], *, max_items: int) -> list[Any]:
    if len(values) <= max_items:
        return values
    step = math.ceil(len(values) / max_items)
    return values[::step]


def _coerce_value(value: Any) -> object:
    if value is None:
        return ""
    if isinstance(value, (int, float)):
        return value
    text = str(value)
    numeric_value = _maybe_float(text)
    if numeric_value is None:
        return text
    return numeric_value


def _coerce_field_value(key: str, value: Any) -> object:
    """Convert numeric fields while keeping simulator identifiers unchanged."""

    if _is_string_value_field(key):
        return "" if value is None else str(value)
    return _coerce_value(value)


def _is_string_value_field(key: str) -> bool:
    return key in STRING_VALUE_FIELDS or key.endswith("_id")


def _maybe_float(value: Any) -> float | None:
    try:
        if value in {None, ""}:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _numbers(rows: list[dict[str, object]], key: str) -> list[float]:
    values = []
    for row in rows:
        number = _maybe_float(row.get(key))
        if number is not None:
            values.append(number)
    return values


def _sum_first_available(rows: list[dict[str, object]], keys: list[str]) -> float:
    for key in keys:
        values = _numbers(rows, key)
        if values:
            return sum(values)
    return 0.0


def _mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def _observed_hours(
    departures: list[float],
    arrivals: list[float],
    durations: list[float],
) -> float:
    if departures and arrivals:
        observed_seconds = max(arrivals) - min(departures)
    elif durations:
        observed_seconds = max(durations)
    else:
        observed_seconds = 0.0
    return observed_seconds / 3600.0 if observed_seconds > 0 else 0.0


def _count_stop_episodes(
    rows: list[dict[str, object]], stop_threshold: float = 0.1
) -> int:
    rows_by_actor: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        actor_id = str(row.get("actor_id", ""))
        if actor_id:
            rows_by_actor.setdefault(actor_id, []).append(row)

    stop_count = 0
    for actor_rows in rows_by_actor.values():
        actor_rows.sort(key=lambda row: _maybe_float(row.get("time_s")) or 0.0)
        was_stopped = False
        for row in actor_rows:
            speed = _maybe_float(row.get("speed_mps")) or 0.0
            is_stopped = speed <= stop_threshold
            if is_stopped and not was_stopped:
                stop_count += 1
            was_stopped = is_stopped
    return stop_count


def _fieldnames(rows: list[dict[str, object]]) -> list[str]:
    names: list[str] = []
    for row in rows:
        for key in row:
            if key not in names:
                names.append(key)
    return names or ["scenario_id"]


def _has_content(path: str | Path) -> bool:
    candidate = Path(path)
    return candidate.exists() and candidate.stat().st_size > 0


def _scenario_id_from_project(project_dir: str | Path | None) -> str:
    if project_dir is None:
        return "scenario"
    config_path = Path(project_dir) / "project_config.json"
    if not config_path.exists():
        return Path(project_dir).name
    try:
        payload = json.loads(config_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return Path(project_dir).name
    scenario = payload.get("scenario", {})
    if isinstance(scenario, dict):
        return str(scenario.get("scenario_id") or Path(project_dir).name)
    return Path(project_dir).name


def _ego_filter_from_project(project_dir: str | Path | None) -> dict[str, object]:
    ego_filter: dict[str, object] = {
        "enabled": False,
        "analyze_only_ego": False,
        "vehicle_ids": [],
        "carla_actor_ids": [],
        "carla_role_names": [],
        "carla_type_ids": [],
    }
    if project_dir is None:
        return ego_filter

    project_path = Path(project_dir)
    project_config_path = project_path / "project_config.json"
    if project_config_path.exists():
        try:
            project_config = json.loads(project_config_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            project_config = {}
        scenario = project_config.get("scenario", {})
        ego = scenario.get("ego", {}) if isinstance(scenario, dict) else {}
        if isinstance(ego, dict):
            ego_filter["enabled"] = bool(ego.get("enabled", False))
            ego_filter["analyze_only_ego"] = bool(ego.get("analyze_only_ego", True))
            ego_filter["vehicle_ids"] = _string_list(ego.get("vehicle_ids", []))
            ego_filter["carla_actor_ids"] = _string_list(ego.get("carla_actor_ids", []))
            ego_filter["carla_role_names"] = _string_list(
                ego.get("carla_role_names", [])
            )
            ego_filter["carla_type_ids"] = _string_list(ego.get("carla_type_ids", []))

    ego_metadata_path = project_path / "sumo" / "ego_vehicles.json"
    if ego_metadata_path.exists():
        try:
            ego_metadata = json.loads(ego_metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            ego_metadata = {}
        if isinstance(ego_metadata, dict):
            ego_filter["enabled"] = bool(
                ego_metadata.get("enabled", ego_filter["enabled"])
            )
            ego_filter["analyze_only_ego"] = bool(
                ego_metadata.get("analyze_only_ego", ego_filter["analyze_only_ego"])
            )
            metadata_ids = _string_list(ego_metadata.get("vehicle_ids", []))
            if metadata_ids:
                ego_filter["vehicle_ids"] = metadata_ids
            for key in ["carla_actor_ids", "carla_role_names", "carla_type_ids"]:
                metadata_values = _string_list(ego_metadata.get(key, []))
                if metadata_values:
                    ego_filter[key] = metadata_values
    return ego_filter


def _should_filter_ego(ego_filter: dict[str, object]) -> bool:
    return bool(ego_filter.get("enabled")) and bool(ego_filter.get("analyze_only_ego"))


def _filter_tripinfo_to_ego(
    rows: list[dict[str, object]],
    ego_filter: dict[str, object],
) -> list[dict[str, object]]:
    vehicle_ids = set(_string_list(ego_filter.get("vehicle_ids", [])))
    if not vehicle_ids:
        return []
    return [row for row in rows if str(row.get("id", "")) in vehicle_ids]


def _filter_carla_rows_to_ego(
    rows: list[dict[str, object]],
    ego_filter: dict[str, object],
) -> list[dict[str, object]]:
    vehicle_ids = set(_string_list(ego_filter.get("vehicle_ids", [])))
    actor_ids = set(_string_list(ego_filter.get("carla_actor_ids", [])))
    role_names = set(_string_list(ego_filter.get("carla_role_names", []))) | vehicle_ids
    type_ids = set(_string_list(ego_filter.get("carla_type_ids", [])))
    if not any([vehicle_ids, actor_ids, role_names, type_ids]):
        return []
    filtered_rows = []
    for row in rows:
        row_actor_id = str(row.get("actor_id", ""))
        row_role_name = str(row.get("role_name", ""))
        row_sumo_id = str(row.get("sumo_id", ""))
        row_type_id = str(row.get("type_id", ""))
        if (
            row_actor_id in actor_ids
            or row_role_name in role_names
            or row_sumo_id in vehicle_ids
            or row_type_id in type_ids
        ):
            filtered_rows.append(row)
    return filtered_rows


def _string_list(raw_values: object) -> list[str]:
    if raw_values is None or raw_values == "":
        return []
    if isinstance(raw_values, list):
        return [str(value) for value in raw_values]
    return [str(raw_values)]


def _unavailable_tripinfo_summary(scenario_id: str) -> dict[str, object]:
    return summarize_tripinfo([], scenario_id)
