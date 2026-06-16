#!/usr/bin/env python3
"""Guided 9D-to-6D motion calibration for the CAD Mouse MK2 firmware."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
from typing import Dict, List, Mapping, Optional, Sequence, Tuple, Union

import numpy as np


AXES = ("Tx", "Ty", "Tz", "Rx", "Ry", "Rz")
RAW_CHANNELS = (
    "mag1x",
    "mag1y",
    "mag1z",
    "mag2x",
    "mag2y",
    "mag2z",
    "mag3x",
    "mag3y",
    "mag3z",
)
EXTRA_SHORT_RANGE_LINEAR_LIMIT_MT = 50.0
RANGE_WARNING_THRESHOLD_MT = 45.0

# Human-readable guidance for each signed motion the operator is asked to hold.
# These describe the intended physical gesture; if a direction feels inverted on
# your build, the sign is recovered from the data regardless of how it is labeled.
MOVEMENT_DESCRIPTIONS = {
    "+Tx": "Translate Right",
    "-Tx": "Translate Left",
    "+Ty": "Translate Backward (toward you)",
    "-Ty": "Translate Forward (away from you)",
    "+Tz": "Translate Down (push the knob)",
    "-Tz": "Translate Up (lift the knob)",
    "+Rx": "Tilt Backward (pitch the top toward you)",
    "-Rx": "Tilt Forward (pitch the top away)",
    "+Ry": "Roll Left (tilt the left side down)",
    "-Ry": "Roll Right (tilt the right side down)",
    "+Rz": "Twist Clockwise (yaw right, viewed from above)",
    "-Rz": "Twist Counter-Clockwise (yaw left, viewed from above)",
}


@dataclass(frozen=True)
class RawSample:
    millis: int
    values: np.ndarray


SampleInput = Union[RawSample, Sequence[float]]


@dataclass
class CalibrationResult:
    rest_mean: np.ndarray
    rest_std: np.ndarray
    response_matrix: np.ndarray
    decoder_matrix: np.ndarray
    output_matrix: np.ndarray
    deadzone: np.ndarray
    rank: int
    condition_number: float
    singular_values: np.ndarray
    confusion_matrix: np.ndarray
    normalized_confusion_matrix: np.ndarray
    signal_to_noise: np.ndarray
    warnings: List[str]
    can_generate_header: bool


def parse_raw_line(line: str) -> RawSample:
    """Parse a firmware CALRAW line into a RawSample."""
    parts = line.strip().split(",")
    if len(parts) != 11 or parts[0] != "CALRAW":
        raise ValueError(f"not a CALRAW line: {line!r}")

    try:
        millis = int(parts[1])
        values = np.array([float(v) for v in parts[2:]], dtype=float)
    except ValueError as exc:
        raise ValueError(f"invalid CALRAW values: {line!r}") from exc

    if values.shape != (9,):
        raise ValueError(f"expected 9 raw values, got {values.shape[0]}")
    return RawSample(millis=millis, values=values)


def parse_status_line(line: str) -> Optional[Dict[str, int]]:
    parts = line.strip().split(",")
    if len(parts) < 3 or parts[0] != "CAL" or parts[1] != "STATUS":
        return None

    out: Dict[str, int] = {}
    for item in parts[2:]:
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        try:
            out[key] = int(value)
        except ValueError:
            continue
    return out


def sample_matrix(samples: Sequence[SampleInput]) -> np.ndarray:
    rows = []
    for sample in samples:
        if isinstance(sample, RawSample):
            rows.append(np.asarray(sample.values, dtype=float))
        else:
            rows.append(np.asarray(sample, dtype=float))

    if not rows:
        raise ValueError("no samples captured")

    matrix = np.vstack(rows)
    if matrix.shape[1] != 9:
        raise ValueError(f"expected 9 channels, got {matrix.shape[1]}")
    return matrix


def robust_mean(samples: Sequence[SampleInput],
                trim_fraction: float = 0.10) -> np.ndarray:
    matrix = sample_matrix(samples)
    if matrix.shape[0] <= 2 or trim_fraction <= 0.0:
        return matrix.mean(axis=0)

    center = np.median(matrix, axis=0)
    distances = np.linalg.norm(matrix - center, axis=1)
    keep_count = max(1, int(round(matrix.shape[0] * (1.0 - trim_fraction))))
    keep_indices = np.argsort(distances)[:keep_count]
    return matrix[keep_indices].mean(axis=0)


def build_response_matrix(
    rest_samples: Sequence[SampleInput],
    captures: Mapping[str, Sequence[SampleInput]],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    rest_matrix = sample_matrix(rest_samples)
    rest_mean = robust_mean(rest_samples)
    response = np.zeros((9, 6), dtype=float)

    for axis_index, axis in enumerate(AXES):
        plus_key = f"+{axis}"
        minus_key = f"-{axis}"
        if plus_key not in captures or minus_key not in captures:
            raise ValueError(f"missing captures for {axis}")

        plus = robust_mean(captures[plus_key]) - rest_mean
        minus = robust_mean(captures[minus_key]) - rest_mean
        response[:, axis_index] = (plus - minus) / 2.0

    return rest_matrix, rest_mean, response


def regularized_pseudoinverse(response: np.ndarray, ridge: float) -> np.ndarray:
    if ridge <= 0.0:
        return np.linalg.pinv(response)

    gram = response.T @ response
    mean_energy = float(np.trace(gram) / gram.shape[0])
    lam = ridge * mean_energy if mean_energy > 0.0 else ridge
    return np.linalg.solve(gram + lam * np.eye(gram.shape[0]), response.T)


def normalize_confusion(confusion: np.ndarray) -> np.ndarray:
    normalized = confusion.copy()
    diagonal = np.diag(confusion)
    for row, value in enumerate(diagonal):
        denom = abs(float(value))
        if denom < 1e-9:
            denom = 1.0
        normalized[row, :] /= denom
    return normalized


def compute_calibration(
    rest_samples: Sequence[SampleInput],
    captures: Mapping[str, Sequence[SampleInput]],
    *,
    target_output: float = 300.0,
    ridge: float = 0.03,
) -> CalibrationResult:
    rest_matrix, rest_mean, response = build_response_matrix(rest_samples, captures)
    rest_std = rest_matrix.std(axis=0, ddof=1) if rest_matrix.shape[0] > 1 else np.zeros(9)

    singular_values = np.linalg.svd(response, compute_uv=False)
    if singular_values.size == 0 or singular_values[0] <= 1e-12:
        rank = 0
    else:
        rank_tolerance = singular_values[0] * 1e-3
        rank = int(np.sum(singular_values > rank_tolerance))
    if singular_values.size == 0 or singular_values[-1] <= 1e-12:
        condition_number = float("inf")
    else:
        condition_number = float(singular_values[0] / singular_values[-1])

    decoder = regularized_pseudoinverse(response, ridge)
    output_matrix = target_output * decoder
    confusion = decoder @ response
    normalized_confusion = normalize_confusion(confusion)

    rest_deltas = rest_matrix - rest_mean
    decoded_rest = rest_deltas @ output_matrix.T
    decoded_std = decoded_rest.std(axis=0, ddof=1) if decoded_rest.shape[0] > 1 else np.zeros(6)
    deadzone = np.maximum(5.0, np.ceil(decoded_std * 4.0))

    rest_noise_norm = max(float(np.linalg.norm(rest_std)), 1e-9)
    signal_to_noise = np.array(
        [np.linalg.norm(response[:, i]) / rest_noise_norm for i in range(6)],
        dtype=float,
    )

    warnings: List[str] = []
    if rank < 6:
        warnings.append(f"rank is {rank}; need rank 6 to generate a calibration header")
    if condition_number > 50.0:
        warnings.append(f"condition number is high ({condition_number:.2f}); axes may be hard to separate")

    weak_axes = [AXES[i] for i, value in enumerate(signal_to_noise) if value < 6.0]
    if weak_axes:
        warnings.append("weak signal-to-noise on: " + ", ".join(weak_axes))

    off_axis = normalized_confusion.copy()
    np.fill_diagonal(off_axis, 0.0)
    max_off_axis = float(np.max(np.abs(off_axis)))
    if max_off_axis > 0.35:
        warnings.append(f"cross-axis confusion is high ({max_off_axis:.2f})")

    all_values = [sample_matrix(rest_samples)]
    for values in captures.values():
        all_values.append(sample_matrix(values))
    max_abs_raw = float(np.max(np.abs(np.vstack(all_values))))
    if max_abs_raw >= RANGE_WARNING_THRESHOLD_MT:
        warnings.append(
            f"raw magnetic field reached {max_abs_raw:.2f} mT; "
            f"extra-short-range linear limit is about {EXTRA_SHORT_RANGE_LINEAR_LIMIT_MT:.0f} mT"
        )

    can_generate_header = rank == 6

    return CalibrationResult(
        rest_mean=rest_mean,
        rest_std=rest_std,
        response_matrix=response,
        decoder_matrix=decoder,
        output_matrix=output_matrix,
        deadzone=deadzone,
        rank=rank,
        condition_number=condition_number,
        singular_values=singular_values,
        confusion_matrix=confusion,
        normalized_confusion_matrix=normalized_confusion,
        signal_to_noise=signal_to_noise,
        warnings=warnings,
        can_generate_header=can_generate_header,
    )


def cpp_float(value: float) -> str:
    value = float(value)
    if abs(value) < 5e-8:
        value = 0.0
    return f"{value:.9g}f"


def generate_header(result: CalibrationResult, generated_at: str) -> str:
    matrix_rows = []
    for row in result.output_matrix:
        matrix_rows.append("  {" + ", ".join(cpp_float(v) for v in row) + "}")

    deadzone = ", ".join(cpp_float(v) for v in result.deadzone)
    return (
        "#pragma once\n"
        "\n"
        "// Generated by firmware/calibration/calibrate_motion.py.\n"
        "// This file is specific to one physical CAD Mouse MK2 build.\n"
        f"// Generated at: {generated_at}\n"
        "\n"
        "namespace CalibrationData {\n"
        "constexpr bool AVAILABLE = true;\n"
        "constexpr float MATRIX[6][9] = {\n"
        + ",\n".join(matrix_rows)
        + "\n};\n"
        f"constexpr float DEADZONE[6] = {{{deadzone}}};\n"
        "}  // namespace CalibrationData\n"
    )


def array_to_list(value: np.ndarray) -> list:
    return value.tolist()


def diagnostics_dict(result: CalibrationResult, generated_at: str) -> dict:
    return {
        "generated_at": generated_at,
        "axes": list(AXES),
        "raw_channels": list(RAW_CHANNELS),
        "rank": result.rank,
        "condition_number": result.condition_number,
        "singular_values": array_to_list(result.singular_values),
        "rest_mean": array_to_list(result.rest_mean),
        "rest_std": array_to_list(result.rest_std),
        "response_matrix_9x6": array_to_list(result.response_matrix),
        "decoder_matrix_6x9": array_to_list(result.decoder_matrix),
        "output_matrix_6x9": array_to_list(result.output_matrix),
        "deadzone": array_to_list(result.deadzone),
        "confusion_matrix": array_to_list(result.confusion_matrix),
        "normalized_confusion_matrix": array_to_list(result.normalized_confusion_matrix),
        "signal_to_noise": array_to_list(result.signal_to_noise),
        "warnings": result.warnings,
        "can_generate_header": result.can_generate_header,
    }


def write_report(path: Path, result: CalibrationResult, generated_at: str,
                 header_path: Path) -> None:
    warning_lines = "\n".join(f"- {warning}" for warning in result.warnings) or "- None"
    snr_lines = "\n".join(
        f"- {axis}: {result.signal_to_noise[i]:.2f}" for i, axis in enumerate(AXES)
    )
    deadzone_lines = "\n".join(
        f"- {axis}: {result.deadzone[i]:.1f}" for i, axis in enumerate(AXES)
    )
    confusion_rows = ["| axis | " + " | ".join(AXES) + " |"]
    confusion_rows.append("| --- | " + " | ".join("---" for _ in AXES) + " |")
    for row_index, axis in enumerate(AXES):
        values = " | ".join(
            f"{result.normalized_confusion_matrix[row_index, col]:.3f}"
            for col in range(6)
        )
        confusion_rows.append(f"| {axis} | {values} |")

    text = f"""# CAD Mouse MK2 Motion Calibration Report

Generated: {generated_at}

## Summary

- Rank: {result.rank}
- Condition number: {result.condition_number:.2f}
- Header generated: {"yes" if result.can_generate_header else "no"}
- Header path: `{header_path}`

## Warnings

{warning_lines}

## Signal To Noise

{snr_lines}

## Recommended Dead Zones

{deadzone_lines}

## Normalized Confusion Matrix

Values near 1.0 on the diagonal and near 0.0 elsewhere are best.

{chr(10).join(confusion_rows)}
"""
    path.write_text(text, encoding="utf-8")


def write_raw_csv(path: Path, rest_samples: Sequence[RawSample],
                  captures: Mapping[str, Sequence[RawSample]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["phase", "repeat", "millis", *RAW_CHANNELS])
        for sample in rest_samples:
            writer.writerow(["rest", 0, sample.millis, *sample.values.tolist()])
        for label, samples in captures.items():
            for index, sample in enumerate(samples):
                writer.writerow([label, index, sample.millis, *sample.values.tolist()])


class CalibrationSerial:
    def __init__(self, port: str, baud: int) -> None:
        import serial

        self.serial = serial.Serial(port, baudrate=baud, timeout=0.2)

    def close(self) -> None:
        self.serial.close()

    def reset_input_buffer(self) -> None:
        self.serial.reset_input_buffer()

    def command(self, text: str) -> None:
        self.serial.write((text.strip() + "\n").encode("ascii"))
        self.serial.flush()

    def readline(self) -> str:
        return self.serial.readline().decode("utf-8", errors="replace").strip()


def list_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [port.device for port in list_ports.comports()]


def wait_for_ready(link: CalibrationSerial, timeout_s: float = 30.0) -> Dict[str, int]:
    deadline = time.monotonic() + timeout_s
    next_query = 0.0
    last_status: Dict[str, int] = {}

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_query:
            link.command("CAL?")
            next_query = now + 1.0

        line = link.readline()
        status = parse_status_line(line)
        if status is None:
            continue
        last_status = status
        if status.get("ready", 0) == 1:
            return status
        print("Waiting for firmware baseline calibration to finish...")

    raise TimeoutError(f"firmware did not become ready; last status: {last_status}")


def set_raw_mode(link: CalibrationSerial, enabled: bool, timeout_s: float = 5.0) -> None:
    command = "CAL RAW 1" if enabled else "CAL RAW 0"
    expected = "CAL,RAW,1" if enabled else "CAL,RAW,0"
    deadline = time.monotonic() + timeout_s
    link.command(command)

    while time.monotonic() < deadline:
        if link.readline() == expected:
            return

    raise TimeoutError(f"firmware did not acknowledge {command}")


def capture_window(link: CalibrationSerial, seconds: float, settle_seconds: float) -> List[RawSample]:
    link.reset_input_buffer()
    start = time.monotonic()
    deadline = start + settle_seconds + seconds
    samples: List[RawSample] = []

    while time.monotonic() < deadline:
        line = link.readline()
        if not line:
            continue
        try:
            sample = parse_raw_line(line)
        except ValueError:
            continue
        if time.monotonic() - start >= settle_seconds:
            samples.append(sample)

    if not samples:
        raise RuntimeError("captured zero CALRAW samples")
    return samples


def guided_capture(args: argparse.Namespace) -> Tuple[List[RawSample], Dict[str, List[RawSample]], Dict[str, int]]:
    link = CalibrationSerial(args.port, args.baud)
    try:
        status = wait_for_ready(link)
        set_raw_mode(link, True)
        print(f"Firmware ready. Calibrated decoder active on board: {status.get('decoder', 0)}")

        input("Release the knob and press Enter to capture rest...")
        rest_samples = capture_window(link, args.rest_seconds, 0.0)
        print(f"Captured {len(rest_samples)} rest samples.")

        captures: Dict[str, List[RawSample]] = {f"{sign}{axis}": [] for axis in AXES for sign in ("+", "-")}
        for repeat in range(args.repeats):
            print(f"\nRepeat {repeat + 1} of {args.repeats}")
            for axis in AXES:
                for sign, word in (("+", "positive"), ("-", "negative")):
                    label = f"{sign}{axis}"
                    description = MOVEMENT_DESCRIPTIONS.get(label, f"{word} {axis}")
                    input(
                        f"Move and hold {label} — {description} ({word} {axis}) "
                        "with normal force, then press Enter..."
                    )
                    samples = capture_window(link, args.hold_seconds, args.settle_seconds)
                    captures[label].extend(samples)
                    print(f"Captured {len(samples)} samples for {label}.")

        return rest_samples, captures, status
    finally:
        try:
            set_raw_mode(link, False)
        except Exception:
            pass
        link.close()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port for the CAD Mouse MK2")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--rest-seconds", type=float, default=5.0)
    parser.add_argument("--hold-seconds", type=float, default=2.5)
    parser.add_argument("--settle-seconds", type=float, default=0.75)
    parser.add_argument("--target-output", type=float, default=300.0)
    parser.add_argument("--ridge", type=float, default=0.03)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "out",
        help="Directory for raw logs and reports",
    )
    parser.add_argument(
        "--header",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "include" / "CalibrationData.h",
        help="Generated firmware calibration header path",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    if not args.port:
        ports = list_serial_ports()
        if ports:
            print("Available serial ports:")
            for port in ports:
                print(f"  {port}")
        print("Pass --port with the CAD Mouse MK2 serial port.")
        return 2

    args.out_dir.mkdir(parents=True, exist_ok=True)
    generated_at = datetime.now(timezone.utc).isoformat()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = args.out_dir / f"calibration_raw_{stamp}.csv"
    json_path = args.out_dir / f"calibration_diagnostics_{stamp}.json"
    report_path = args.out_dir / f"calibration_report_{stamp}.md"

    rest_samples, captures, _ = guided_capture(args)
    write_raw_csv(raw_path, rest_samples, captures)

    result = compute_calibration(
        rest_samples,
        captures,
        target_output=args.target_output,
        ridge=args.ridge,
    )

    json_path.write_text(
        json.dumps(diagnostics_dict(result, generated_at), indent=2),
        encoding="utf-8",
    )
    write_report(report_path, result, generated_at, args.header)

    if result.can_generate_header:
        args.header.write_text(generate_header(result, generated_at), encoding="utf-8")
        print(f"Generated calibration header: {args.header}")
    else:
        print("Calibration header was not generated because validation failed.")

    print(f"Raw log: {raw_path}")
    print(f"Diagnostics: {json_path}")
    print(f"Report: {report_path}")
    if result.warnings:
        print("\nWarnings:")
        for warning in result.warnings:
            print(f"  - {warning}")

    return 0 if result.can_generate_header else 1


if __name__ == "__main__":
    raise SystemExit(main())
